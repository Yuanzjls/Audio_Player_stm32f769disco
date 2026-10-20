/**
 * @file  player.c
 * @brief Playlist / streaming logic.
 *
 * Audio is streamed from the SD card into a two-half circular DMA buffer:
 * whenever the DMA finishes playing one half, the task refills it from the
 * file. Mono files are expanded to stereo while refilling.
 *
 * End of track: once the file is exhausted the rest of the buffer is filled
 * with silence; after the half holding the last samples has been played the
 * next track is loaded. Tracks with the same sample rate follow each other
 * without restarting the DMA (only the ~0.2 s of buffered silence separates
 * them); otherwise the output is reconfigured for the new rate.
 */
#include "player.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "audio_out.h"
#include "ff.h"
#include "log.h"
#include "wav.h"

#define TASK_STACK_WORDS   1024
#define TASK_PRIORITY      3          /* above the GUI: audio must never starve */
#define CMD_QUEUE_LENGTH   8

/* Each half holds ~93 ms of 44.1 kHz stereo audio */
#define HALF_BYTES         16384U
#define HALF_SAMPLES       (HALF_BYTES / sizeof(int16_t))

#define NOTIFY_HALF_DONE   (1UL << 0)
#define NOTIFY_FULL_DONE   (1UL << 1)
#define NOTIFY_COMMAND     (1UL << 2)

#define IDLE_POLL_MS       100
#define CARD_RETRY_MS      1000

typedef enum
{
  CMD_TOGGLE_PLAY,
  CMD_NEXT,
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
  FIL        file;
  wav_info_t info;
  bool       open;
  uint32_t   remaining;      /* bytes of sample data not yet read          */
  uint8_t    silent_halves;  /* halves filled with silence after the end   */
} track_t;

static TaskHandle_t      s_task;
static QueueHandle_t     s_cmd_queue;
static SemaphoreHandle_t s_status_mutex;
static player_status_t   s_status;      /* guarded by s_status_mutex */

static FATFS   s_fs;
static track_t s_track;
static uint32_t s_output_rate;          /* rate the output is configured for, 0 = none */

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
  if (!s_track.open || s_track.info.byte_rate == 0U)
  {
    return;
  }
  /* Bytes handed to the DMA minus roughly one buffered half */
  uint32_t played = s_track.info.data_size - s_track.remaining;
  const uint32_t buffered = (s_track.info.channels == 2U) ? HALF_BYTES : HALF_BYTES / 2U;
  played = (played > buffered) ? played - buffered : 0U;

  status_lock();
  s_status.position_s = played / s_track.info.byte_rate;
  status_unlock();
}

/* ---- playlist -------------------------------------------------------------- */

static bool is_wav_name(const char *name)
{
  const size_t len = strlen(name);
  return len > 4U && strcasecmp(&name[len - 4U], ".wav") == 0;
}

/* Call fn for each .wav in the music folder; stops when fn returns false. */
static uint16_t for_each_track(bool (*fn)(uint16_t index, const FILINFO *info, void *ctx), void *ctx)
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
    if ((info.fattrib & (AM_DIR | AM_HID | AM_SYS)) != 0U || !is_wav_name(info.fname))
    {
      continue;
    }
    if (fn != NULL && !fn(count, &info, ctx))
    {
      count++;
      break;
    }
    count++;
  }
  f_closedir(&dir);
  return count;
}

typedef struct
{
  uint16_t wanted;
  char    *name;
  bool     found;
} find_ctx_t;

static bool find_track_cb(uint16_t index, const FILINFO *info, void *ctx)
{
  find_ctx_t *find = ctx;
  if (index != find->wanted)
  {
    return true;
  }
  snprintf(find->name, PLAYER_TITLE_MAX, "%s", info->fname);
  find->found = true;
  return false;
}

static void close_track(void)
{
  if (s_track.open)
  {
    f_close(&s_track.file);
    s_track.open = false;
  }
}

/* Open and validate track @p index. On success the status is updated. */
static bool open_track(uint16_t index)
{
  char name[PLAYER_TITLE_MAX];
  char path[sizeof(PLAYER_MUSIC_DIR) + 1 + PLAYER_TITLE_MAX];
  find_ctx_t find = { .wanted = index, .name = name, .found = false };

  close_track();
  for_each_track(find_track_cb, &find);
  if (!find.found)
  {
    return false;
  }

  snprintf(path, sizeof(path), "%s/%s", PLAYER_MUSIC_DIR, name);
  if (f_open(&s_track.file, path, FA_READ) != FR_OK)
  {
    LOG_WARN("player: cannot open %s", path);
    return false;
  }
  s_track.open = true;

  const wav_result_t res = wav_parse(&s_track.file, &s_track.info);
  if (res != WAV_OK)
  {
    LOG_WARN("player: %s: %s", name, wav_result_str(res));
    close_track();
    return false;
  }
  if (!audio_out_is_rate_supported(s_track.info.sample_rate))
  {
    LOG_WARN("player: %s: unsupported sample rate %lu Hz", name, (unsigned long)s_track.info.sample_rate);
    close_track();
    return false;
  }

  s_track.remaining     = s_track.info.data_size;
  s_track.silent_halves = 0;

  /* Show the name without the extension */
  name[strlen(name) - 4U] = '\0';

  status_lock();
  s_status.track_index = index;
  s_status.position_s  = 0;
  s_status.duration_s  = wav_duration_s(&s_track.info);
  s_status.sample_rate = s_track.info.sample_rate;
  s_status.channels    = (uint8_t)s_track.info.channels;
  s_status.track_serial++;
  snprintf(s_status.title, sizeof(s_status.title), "%s", name);
  status_unlock();

  LOG_INFO("player: [%u] %s (%lu Hz, %u ch, %lu s)", (unsigned)index + 1U, name,
           (unsigned long)s_track.info.sample_rate, (unsigned)s_track.info.channels,
           (unsigned long)wav_duration_s(&s_track.info));
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
  const bool mono = (s_track.info.channels == 1U);
  UINT got = 0;

  if (s_track.open && s_track.remaining > 0U)
  {
    /* Mono is read into the first half of the region and expanded in place */
    uint32_t want = mono ? HALF_BYTES / 2U : HALF_BYTES;
    if (want > s_track.remaining)
    {
      want = s_track.remaining;
    }
    if (f_read(&s_track.file, dst, want, &got) != FR_OK)
    {
      LOG_ERROR("player: read error, skipping rest of track");
      got = 0;
      s_track.remaining = 0;
    }
    else
    {
      got -= got % s_track.info.block_align;
      s_track.remaining = (got == 0U) ? 0U : s_track.remaining - got;
    }

    if (mono)
    {
      /* Walk backwards so samples are not overwritten before being copied */
      for (int32_t i = (int32_t)(got / sizeof(int16_t)) - 1; i >= 0; i--)
      {
        const int16_t sample = dst[i];
        dst[2 * i]     = sample;
        dst[2 * i + 1] = sample;
      }
      got *= 2U;
    }
  }
  else
  {
    s_track.silent_halves++;
  }

  if (got < HALF_BYTES)
  {
    memset((uint8_t *)dst + got, 0, HALF_BYTES - got);
  }
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
    set_state(PLAYER_STATE_NO_TRACKS, "No playable .wav files in /Music");
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
    f_lseek(&s_track.file, s_track.info.data_offset);
    s_track.remaining     = s_track.info.data_size;
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

static void seek(uint32_t position_s)
{
  if (!s_track.open)
  {
    return;
  }
  uint32_t offset = position_s * s_track.info.byte_rate;
  if (offset >= s_track.info.data_size)
  {
    offset = s_track.info.data_size - s_track.info.block_align;
  }
  offset -= offset % s_track.info.block_align;

  if (f_lseek(&s_track.file, s_track.info.data_offset + offset) == FR_OK)
  {
    s_track.remaining     = s_track.info.data_size - offset;
    s_track.silent_halves = 0;
  }
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
      /* Skipping always ends up playing the next track (wrapping to the first) */
      if (has_tracks)
      {
        const uint16_t next = (uint16_t)((track_index() + 1U) % track_count());
        if (state == PLAYER_STATE_PLAYING)
        {
          play_track(next);
        }
        else
        {
          stop_output();
          if (open_playable_track(next, track_count()))
          {
            start_output();
          }
        }
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
      const uint16_t count = for_each_track(NULL, NULL);
      status_lock();
      s_status.track_count = count;
      status_unlock();

      if (count > 0U)
      {
        LOG_INFO("player: %u track(s) in %s", (unsigned)count, PLAYER_MUSIC_DIR);
        return;
      }
      set_state(PLAYER_STATE_NO_TRACKS, "No .wav files in /Music");
      if (!warned)
      {
        LOG_WARN("player: no .wav files in %s", PLAYER_MUSIC_DIR);
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

static void player_task(void *arg)
{
  (void)arg;

  mount_card_and_scan();

  if (open_playable_track(0, track_count()))
  {
    start_output();
  }
  else
  {
    set_state(PLAYER_STATE_NO_TRACKS, "No playable .wav files in /Music");
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

void player_toggle_play(void)               { send_command(CMD_TOGGLE_PLAY, 0); }
void player_next(void)                      { send_command(CMD_NEXT, 0); }
void player_seek(uint32_t position_s)       { send_command(CMD_SEEK, position_s); }
void player_set_repeat(player_repeat_t r)   { send_command(CMD_SET_REPEAT, (uint32_t)r); }

void player_set_volume(uint8_t volume)
{
  send_command(CMD_SET_VOLUME, (volume > 100U) ? 100U : volume);
}
