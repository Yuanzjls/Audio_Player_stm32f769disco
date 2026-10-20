/**
 * @file  player.c
 * @brief Playlist / streaming logic.
 *
 * Audio is decoded from the SD card (decoder.c: WAV, MP3) into a two-half
 * circular DMA buffer: whenever the DMA finishes playing one half, the task
 * refills it with the next stereo samples.
 *
 * End of track: once the file is exhausted the rest of the buffer is filled
 * with silence; after the half holding the last samples has been played the
 * next track is loaded. Tracks with the same sample rate follow each other
 * without restarting the DMA (only the ~0.2 s of buffered silence separates
 * them); otherwise the output is reconfigured for the new rate.
 */
#include "player.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "audio_out.h"
#include "ff.h"
#include "log.h"
#include "decoder.h"

#define TASK_STACK_WORDS   6144       /* the MP3 decoder needs ~16 KB of scratch on the stack */
#define TASK_PRIORITY      3          /* above the GUI: audio must never starve */
#define CMD_QUEUE_LENGTH   8

/* Each half holds ~93 ms of 44.1 kHz stereo audio */
#define HALF_BYTES         16384U
#define HALF_SAMPLES       (HALF_BYTES / sizeof(int16_t))
#define HALF_FRAMES        (HALF_SAMPLES / 2U)          /* stereo frames per half */

#define NOTIFY_HALF_DONE   (1UL << 0)
#define NOTIFY_FULL_DONE   (1UL << 1)
#define NOTIFY_COMMAND     (1UL << 2)
#define NOTIFY_UI_READY    (1UL << 3)

/* Start playing even without the UI signal after this long (display failure) */
#define UI_READY_TIMEOUT_MS 5000

#define IDLE_POLL_MS       100
#define CARD_RETRY_MS      1000

typedef enum
{
  CMD_TOGGLE_PLAY,
  CMD_NEXT,
  CMD_PREV,
  CMD_PLAY_INDEX,
  CMD_SEEK,
  CMD_SET_VOLUME,
  CMD_SET_REPEAT,
} command_type_t;

typedef struct
{
  command_type_t type;
  uint32_t       value;
} command_t;

/* Track currently loaded */
typedef struct
{
  decoder_info_t info;
  bool           open;
  uint64_t       frames_out;     /* stereo frames handed to the DMA buffer      */
  uint8_t        silent_halves;  /* halves filled with silence after the end    */
} track_t;

static TaskHandle_t      s_task;
static QueueHandle_t     s_cmd_queue;
static SemaphoreHandle_t s_status_mutex;
static player_status_t   s_status;      /* guarded by s_status_mutex */

static FATFS   s_fs;
static track_t s_track;
static uint32_t s_output_rate;          /* rate the output is configured for, 0 = none */

/* Restart the current track instead of going back when past this position */
#define PREV_RESTART_AFTER_S  3U

static uint16_t track_count(void);

/* Library: file names of the playable tracks, sorted. Written once by the
 * player task (scan_library) before track_count is published, read-only after. */
static char s_names[PLAYER_MAX_TRACKS][PLAYER_TITLE_MAX] __attribute__((section(".sdram")));

/* DMA buffer: both halves. Lives in SRAM that the MPU maps non-cacheable. */
static int16_t s_audio_buf[2 * HALF_SAMPLES] __attribute__((aligned(32)));

/* ---- status helpers -------------------------------------------------------- */

static void status_lock(void)   { xSemaphoreTake(s_status_mutex, portMAX_DELAY); }
static void status_unlock(void) { xSemaphoreGive(s_status_mutex); }

static void set_state(player_state_t state, const char *message)
{
  status_lock();
  s_status.state = state;
  snprintf(s_status.message, sizeof(s_status.message), "%s", message ? message : "");
  status_unlock();
}

static player_state_t get_state(void)
{
  status_lock();
  const player_state_t state = s_status.state;
  status_unlock();
  return state;
}

static void update_position(void)
{
  if (!s_track.open || s_track.info.sample_rate == 0U)
  {
    return;
  }
  /* Frames handed to the DMA minus roughly one buffered half */
  const uint64_t played = (s_track.frames_out > HALF_FRAMES) ? s_track.frames_out - HALF_FRAMES : 0U;

  status_lock();
  s_status.position_s = (uint32_t)(played / s_track.info.sample_rate);
  status_unlock();
}

/* ---- playlist -------------------------------------------------------------- */

/* Length of the name without its extension */
static size_t base_name_len(const char *name)
{
  const char *dot = strrchr(name, '.');
  return (dot != NULL) ? (size_t)(dot - name) : strlen(name);
}

static int compare_names(const void *a, const void *b)
{
  return strcasecmp((const char *)a, (const char *)b);
}

/* Read the music folder into s_names (sorted, case-insensitive). Returns the count. */
static uint16_t scan_library(void)
{
  DIR dir;
  FILINFO info;
  uint16_t count = 0;

  if (f_opendir(&dir, PLAYER_MUSIC_DIR) != FR_OK)
  {
    return 0;
  }
  while (f_readdir(&dir, &info) == FR_OK && info.fname[0] != '\0')
  {
    if ((info.fattrib & (AM_DIR | AM_HID | AM_SYS)) != 0U || !decoder_is_supported(info.fname))
    {
      continue;
    }
    if (count == PLAYER_MAX_TRACKS)
    {
      LOG_WARN("player: more than %u tracks, the rest is ignored", (unsigned)PLAYER_MAX_TRACKS);
      break;
    }
    snprintf(s_names[count], PLAYER_TITLE_MAX, "%s", info.fname);
    count++;
  }
  f_closedir(&dir);

  qsort(s_names, count, PLAYER_TITLE_MAX, compare_names);
  return count;
}

static void close_track(void)
{
  if (s_track.open)
  {
    decoder_close();
    s_track.open = false;
  }
}

/* Open and validate track @p index. On success the status is updated. */
static bool open_track(uint16_t index)
{
  char name[PLAYER_TITLE_MAX];
  char path[sizeof(PLAYER_MUSIC_DIR) + 1 + PLAYER_TITLE_MAX];

  close_track();
  if (index >= track_count())
  {
    return false;
  }
  snprintf(name, sizeof(name), "%s", s_names[index]);

  snprintf(path, sizeof(path), "%s/%s", PLAYER_MUSIC_DIR, name);
  if (!decoder_open(path, &s_track.info))
  {
    return false;
  }
  s_track.open = true;

  const decoder_info_t *info = &s_track.info;
  if (!audio_out_is_rate_supported(info->sample_rate))
  {
    LOG_WARN("player: %s: unsupported sample rate %lu Hz", name, (unsigned long)info->sample_rate);
    close_track();
    return false;
  }

  s_track.frames_out    = 0;
  s_track.silent_halves = 0;

  /* Without a title tag, show the file name without its extension */
  name[base_name_len(name)] = '\0';

  status_lock();
  s_status.track_index  = index;
  s_status.position_s   = 0;
  s_status.duration_s   = info->duration_s;
  s_status.sample_rate  = info->sample_rate;
  s_status.channels     = info->channels;
  s_status.bitrate_kbps = info->bitrate_kbps;
  s_status.vbr          = info->vbr;
  s_status.track_serial++;
  snprintf(s_status.format, sizeof(s_status.format), "%s", info->format);
  snprintf(s_status.title, sizeof(s_status.title), "%s", info->title[0] != '\0' ? info->title : name);
  snprintf(s_status.artist, sizeof(s_status.artist), "%s", info->artist);
  status_unlock();

  LOG_INFO("player: [%u] %s (%s %lu Hz, %u ch, %lu kbps%s, %lu s%s)", (unsigned)index + 1U, name,
           info->format, (unsigned long)info->sample_rate, (unsigned)info->channels,
           (unsigned long)info->bitrate_kbps, info->vbr ? " VBR" : "",
           (unsigned long)info->duration_s, info->duration_exact ? "" : " estimated");
  return true;
}

/* Open @p index, or the next playable track after it (wrapping around). */
static bool open_playable_track(uint16_t index, uint16_t count)
{
  for (uint16_t tries = 0; tries < count; tries++)
  {
    if (open_track((uint16_t)((index + tries) % count)))
    {
      return true;
    }
  }
  return false;
}

/* ---- streaming ------------------------------------------------------------- */

static void fill_half(uint32_t half)
{
  int16_t *dst = &s_audio_buf[half * HALF_SAMPLES];
  const uint32_t got = s_track.open ? decoder_read(dst, HALF_FRAMES) : 0U;

  if (got == 0U)
  {
    s_track.silent_halves++;
  }
  if (got < HALF_FRAMES)
  {
    memset(&dst[2U * got], 0, (HALF_FRAMES - got) * 2U * sizeof(int16_t));
  }
  s_track.frames_out += got;
}

static void audio_callback(audio_out_event_t event)
{
  BaseType_t woken = pdFALSE;
  xTaskNotifyFromISR(s_task, (event == AUDIO_OUT_HALF_DONE) ? NOTIFY_HALF_DONE : NOTIFY_FULL_DONE,
                     eSetBits, &woken);
  portYIELD_FROM_ISR(woken);
}

static uint8_t current_volume(void)
{
  status_lock();
  const uint8_t volume = s_status.volume;
  status_unlock();
  return volume;
}

/* Start the output for the open track from its current read position. */
static bool start_output(void)
{
  if (s_output_rate != s_track.info.sample_rate)
  {
    if (!audio_out_configure(s_track.info.sample_rate, current_volume(), audio_callback))
    {
      s_output_rate = 0;
      set_state(PLAYER_STATE_STOPPED, "Audio output error");
      return false;
    }
    s_output_rate = s_track.info.sample_rate;
  }

  fill_half(0);
  fill_half(1);
  if (!audio_out_start(s_audio_buf, sizeof(s_audio_buf)))
  {
    set_state(PLAYER_STATE_STOPPED, "Audio output error");
    return false;
  }
  set_state(PLAYER_STATE_PLAYING, NULL);
  return true;
}

static void stop_output(void)
{
  audio_out_stop();
  /* Force a full reconfiguration on the next start: the codec was muted */
  s_output_rate = 0;
}

static uint16_t track_count(void)
{
  status_lock();
  const uint16_t count = s_status.track_count;
  status_unlock();
  return count;
}

static uint16_t track_index(void)
{
  status_lock();
  const uint16_t index = s_status.track_index;
  status_unlock();
  return index;
}

/* Load @p index and keep playing. Restarts the DMA only if the rate changes. */
static void play_track(uint16_t index)
{
  const uint32_t previous_rate = s_track.open ? s_track.info.sample_rate : 0U;
  const bool was_playing = (get_state() == PLAYER_STATE_PLAYING);

  if (!open_playable_track(index, track_count()))
  {
    stop_output();
    set_state(PLAYER_STATE_NO_TRACKS, "No playable music files in /Music");
    return;
  }

  if (was_playing && s_track.info.sample_rate == previous_rate && s_output_rate == previous_rate)
  {
    /* Same rate: the DMA keeps running and picks up the new data on the next refill */
    return;
  }
  stop_output();
  start_output();
}

static void on_track_finished(void)
{
  player_repeat_t repeat;
  status_lock();
  repeat = s_status.repeat;
  status_unlock();

  if (repeat == PLAYER_REPEAT_ONE)
  {
    uint32_t actual_s;
    decoder_seek(0, &actual_s);
    s_track.frames_out    = 0;
    s_track.silent_halves = 0;
    return;
  }

  const uint16_t next = (uint16_t)(track_index() + 1U);
  if (next >= track_count())
  {
    /* End of the playlist: stop and rewind to the first track */
    stop_output();
    open_playable_track(0, track_count());
    set_state(PLAYER_STATE_STOPPED, NULL);
    return;
  }
  play_track(next);
}

/* ---- commands -------------------------------------------------------------- */

/* Switching tracks always ends up playing the new one */
static void skip_to(uint16_t index, player_state_t state)
{
  if (state == PLAYER_STATE_PLAYING)
  {
    play_track(index);
  }
  else
  {
    stop_output();
    if (open_playable_track(index, track_count()))
    {
      start_output();
    }
  }
}

static void seek(uint32_t position_s)
{
  uint32_t actual_s;
  if (!s_track.open || !decoder_seek(position_s, &actual_s))
  {
    return;
  }
  /* The half already queued still plays: count it so the position is right */
  s_track.frames_out    = (uint64_t)actual_s * s_track.info.sample_rate + HALF_FRAMES;
  s_track.silent_halves = 0;
}

static void handle_command(const command_t *cmd)
{
  const player_state_t state = get_state();
  const bool has_tracks = (state == PLAYER_STATE_STOPPED || state == PLAYER_STATE_PLAYING ||
                           state == PLAYER_STATE_PAUSED);

  switch (cmd->type)
  {
    case CMD_TOGGLE_PLAY:
      if (state == PLAYER_STATE_PLAYING)
      {
        audio_out_pause();
        set_state(PLAYER_STATE_PAUSED, NULL);
      }
      else if (state == PLAYER_STATE_PAUSED)
      {
        audio_out_resume();
        set_state(PLAYER_STATE_PLAYING, NULL);
      }
      else if (state == PLAYER_STATE_STOPPED && s_track.open)
      {
        start_output();
      }
      break;

    case CMD_NEXT:
      if (has_tracks)
      {
        skip_to((uint16_t)((track_index() + 1U) % track_count()), state);
      }
      break;

    case CMD_PREV:
      if (has_tracks)
      {
        uint32_t position_s;
        status_lock();
        position_s = s_status.position_s;
        status_unlock();

        if (position_s >= PREV_RESTART_AFTER_S && state != PLAYER_STATE_STOPPED)
        {
          seek(0);
          update_position();
        }
        else
        {
          const uint16_t count = track_count();
          skip_to((uint16_t)((track_index() + count - 1U) % count), state);
        }
      }
      break;

    case CMD_PLAY_INDEX:
      if (has_tracks && cmd->value < track_count())
      {
        skip_to((uint16_t)cmd->value, state);
      }
      break;

    case CMD_SEEK:
      seek(cmd->value);
      update_position();
      break;

    case CMD_SET_VOLUME:
      status_lock();
      s_status.volume = (uint8_t)cmd->value;
      status_unlock();
      if (s_output_rate != 0U)
      {
        audio_out_set_volume((uint8_t)cmd->value);
      }
      break;

    case CMD_SET_REPEAT:
      status_lock();
      s_status.repeat = (player_repeat_t)cmd->value;
      status_unlock();
      break;
  }
}

static void send_command(command_type_t type, uint32_t value)
{
  const command_t cmd = { .type = type, .value = value };
  if (xQueueSend(s_cmd_queue, &cmd, 0) == pdTRUE)
  {
    xTaskNotify(s_task, NOTIFY_COMMAND, eSetBits);
  }
  else
  {
    LOG_WARN("player: command queue full");
  }
}

/* ---- task ------------------------------------------------------------------ */

static void mount_card_and_scan(void)
{
  bool warned = false;

  for (;;)
  {
    if (f_mount(&s_fs, "", 1) == FR_OK)
    {
      const uint16_t count = scan_library();
      status_lock();
      s_status.track_count = count;
      s_status.library_serial++;
      status_unlock();

      if (count > 0U)
      {
        LOG_INFO("player: %u track(s) in %s", (unsigned)count, PLAYER_MUSIC_DIR);
        /* Card and library are ready (the UI can load fonts and the list) */
        set_state(PLAYER_STATE_STOPPED, NULL);
        return;
      }
      set_state(PLAYER_STATE_NO_TRACKS, "No .wav or .mp3 files in /Music");
      if (!warned)
      {
        LOG_WARN("player: no .wav or .mp3 files in %s", PLAYER_MUSIC_DIR);
      }
    }
    else
    {
      set_state(PLAYER_STATE_NO_CARD, "Insert an SD card");
      if (!warned)
      {
        LOG_WARN("player: SD card not mounted, retrying");
      }
    }
    warned = true;
    f_unmount("");
    vTaskDelay(pdMS_TO_TICKS(CARD_RETRY_MS));
  }
}

/* Block until the GUI has drawn its first frame (or the timeout expires).
 * Only the UI-ready bit is consumed; other pending events are kept. */
static void wait_for_ui(void)
{
  const TickType_t start = xTaskGetTickCount();
  const TickType_t limit = pdMS_TO_TICKS(UI_READY_TIMEOUT_MS);

  for (;;)
  {
    const TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= limit)
    {
      LOG_WARN("player: UI not ready after %u ms, starting anyway", (unsigned)UI_READY_TIMEOUT_MS);
      return;
    }
    uint32_t events = 0;
    if (xTaskNotifyWait(0, NOTIFY_UI_READY, &events, limit - elapsed) == pdTRUE &&
        (events & NOTIFY_UI_READY) != 0U)
    {
      return;
    }
  }
}

static void player_task(void *arg)
{
  (void)arg;

  mount_card_and_scan();
  wait_for_ui();

  if (open_playable_track(0, track_count()))
  {
    start_output();
  }
  else
  {
    set_state(PLAYER_STATE_NO_TRACKS, "No playable music files in /Music");
  }

  for (;;)
  {
    uint32_t events = 0;
    xTaskNotifyWait(0, UINT32_MAX, &events, pdMS_TO_TICKS(IDLE_POLL_MS));

    if (get_state() == PLAYER_STATE_PLAYING)
    {
      if (events & NOTIFY_HALF_DONE)
      {
        fill_half(0);
      }
      if (events & NOTIFY_FULL_DONE)
      {
        fill_half(1);
      }
      /* The half holding the last samples has been played once two halves of silence were queued */
      if (s_track.silent_halves >= 2U)
      {
        on_track_finished();
      }
      update_position();

      /* Background work (e.g. exact length of a VBR MP3) between refills */
      uint32_t duration_s;
      if (decoder_poll(&duration_s))
      {
        status_lock();
        s_status.duration_s = duration_s;
        status_unlock();
        LOG_INFO("player: exact length %lu s", (unsigned long)duration_s);
      }
    }

    command_t cmd;
    while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE)
    {
      handle_command(&cmd);
    }
  }
}

/* ---- public API ------------------------------------------------------------ */

void player_start(uint8_t initial_volume)
{
  s_status_mutex = xSemaphoreCreateMutex();
  s_cmd_queue    = xQueueCreate(CMD_QUEUE_LENGTH, sizeof(command_t));
  configASSERT(s_status_mutex != NULL && s_cmd_queue != NULL);

  s_status.state  = PLAYER_STATE_NO_CARD;
  s_status.repeat = PLAYER_REPEAT_OFF;
  s_status.volume = (initial_volume > 100U) ? 100U : initial_volume;

  if (xTaskCreate(player_task, "player", TASK_STACK_WORDS, NULL, TASK_PRIORITY, &s_task) != pdPASS)
  {
    configASSERT(0);
  }
}

void player_get_status(player_status_t *status)
{
  status_lock();
  *status = s_status;
  status_unlock();
}

void player_ui_ready(void)
{
  xTaskNotify(s_task, NOTIFY_UI_READY, eSetBits);
}

void player_toggle_play(void)               { send_command(CMD_TOGGLE_PLAY, 0); }
void player_next(void)                      { send_command(CMD_NEXT, 0); }
void player_prev(void)                      { send_command(CMD_PREV, 0); }
void player_play_track(uint16_t index)      { send_command(CMD_PLAY_INDEX, index); }

bool player_get_track_name(uint16_t index, char *name, size_t len)
{
  bool ok = false;

  status_lock();
  if (index < s_status.track_count && len > 0U)
  {
    /* File name without its extension */
    snprintf(name, len, "%.*s", (int)base_name_len(s_names[index]), s_names[index]);
    ok = true;
  }
  status_unlock();
  return ok;
}
void player_seek(uint32_t position_s)       { send_command(CMD_SEEK, position_s); }
void player_set_repeat(player_repeat_t r)   { send_command(CMD_SET_REPEAT, (uint32_t)r); }

void player_set_volume(uint8_t volume)
{
  send_command(CMD_SET_VOLUME, (volume > 100U) ? 100U : volume);
}
