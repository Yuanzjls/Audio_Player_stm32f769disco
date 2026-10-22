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

#include "main.h"
#include "settings.h"

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
  CMD_SET_SHUFFLE,
  CMD_RELEASE_AUDIO,
  CMD_PLAY_FILE,
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
static SemaphoreHandle_t s_audio_released;   /* given once the output is stopped for another user */
static player_status_t   s_status;      /* guarded by s_status_mutex */

static FATFS   s_fs;
static track_t s_track;
static uint32_t s_output_rate;          /* rate the output is configured for, 0 = none */
static char     s_current_path[PLAYER_PATH_MAX];   /* file being played; guarded by s_status_mutex */
static char     s_file_request[PLAYER_PATH_MAX];   /* player_play_file(); guarded by s_status_mutex */

/* Restart the current track instead of going back when past this position */
#define PREV_RESTART_AFTER_S  3U

static uint16_t track_count(void);

/* Library: file names of the playable tracks in directory order, one after the
 * other in a pool (where each starts: s_name_at), and the sorted order (indices:
 * sorting moves 2 bytes per name). Written once by the player task
 * (scan_library) before track_count is published, read-only after.
 * NAME(i) is the i-th track in sorted order. */
static char     s_name_pool[PLAYER_NAMES_POOL] __attribute__((section(".sdram")));
static uint32_t s_name_at[PLAYER_MAX_TRACKS] __attribute__((section(".sdram")));
static uint16_t s_sorted[PLAYER_MAX_TRACKS] __attribute__((section(".sdram")));
#define NAME_AT(n) (&s_name_pool[s_name_at[n]])
#define NAME(i)    NAME_AT(s_sorted[i])

/* Shuffle: play order (a permutation of the track indices) and the position in
 * it of the current track. Player task only. */
static uint16_t s_order[PLAYER_MAX_TRACKS];
static uint16_t s_order_len;
static uint16_t s_order_pos;
static uint32_t s_random;               /* xorshift32 state, 0 = not seeded yet */

/* DMA buffer: both halves, in uncached DTCM */
static int16_t s_audio_buf[2 * HALF_SAMPLES] DMA_BUFFER;

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
  return strcasecmp(NAME_AT(*(const uint16_t *)a), NAME_AT(*(const uint16_t *)b));
}

/* Read the music folder into the name pool (sorted, case-insensitive). Returns the count. */
static uint16_t scan_library(void)
{
  DIR dir;
  FILINFO info;
  uint16_t count = 0;
  uint32_t used = 0;     /* bytes of the name pool */

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
    const size_t len = strlen(info.fname) + 1U;
    if (used + len > PLAYER_NAMES_POOL)
    {
      LOG_WARN("player: the names of more than %u tracks do not fit, the rest is ignored", (unsigned)count);
      break;
    }
    memcpy(&s_name_pool[used], info.fname, len);
    s_name_at[count] = used;
    used += (uint32_t)len;
    s_sorted[count] = count;
    count++;
  }
  f_closedir(&dir);

  qsort(s_sorted, count, sizeof(s_sorted[0]), compare_names);
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

/* Open and validate the file at @p path, named @p file_name; it counts as
 * library entry @p index (the current one for a file from elsewhere). On
 * success the status is updated. */
static bool open_path(const char *path, const char *file_name, uint16_t index, bool from_file)
{
  char name[PLAYER_TITLE_MAX];

  close_track();
  snprintf(name, sizeof(name), "%s", file_name);
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
  s_status.from_file    = from_file;
  snprintf(s_current_path, sizeof(s_current_path), "%s", path);
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
  s_status.art_format = info->art_format;
  s_status.art_offset = info->art_offset;
  s_status.art_size   = info->art_size;
  status_unlock();

  LOG_INFO("player: [%u] %s (%s %lu Hz, %u ch, %lu kbps%s, %lu s%s)", (unsigned)index + 1U, name,
           info->format, (unsigned long)info->sample_rate, (unsigned)info->channels,
           (unsigned long)info->bitrate_kbps, info->vbr ? " VBR" : "",
           (unsigned long)info->duration_s, info->duration_exact ? "" : " estimated");
  return true;
}

/* Open and validate library track @p index */
static bool open_track(uint16_t index)
{
  char path[sizeof(PLAYER_MUSIC_DIR) + 1 + PLAYER_TITLE_MAX];

  close_track();
  if (index >= track_count())
  {
    return false;
  }
  snprintf(path, sizeof(path), "%s/%.*s", PLAYER_MUSIC_DIR, PLAYER_TITLE_MAX - 1, NAME(index));   /* names are shorter */
  return open_path(path, NAME(index), index, false);
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

/* Log errors of the audio output (an SAI underrun garbles the sound until the
 * output restarts), at most once a second, with the totals since it started */
static void report_output_errors(void)
{
  static audio_out_errors_t logged;
  static TickType_t         logged_at;
  audio_out_errors_t now;

  audio_out_errors(&now);
  if (now.underruns < logged.underruns || now.dma_errors < logged.dma_errors)
  {
    logged.underruns = logged.dma_errors = 0;   /* the output was restarted */
  }
  if ((now.underruns != logged.underruns || now.dma_errors != logged.dma_errors) &&
      (xTaskGetTickCount() - logged_at) >= pdMS_TO_TICKS(1000))
  {
    LOG_WARN("audio: %lu SAI underrun(s), %lu DMA error(s) since the output started",
             (unsigned long)now.underruns, (unsigned long)now.dma_errors);
    logged    = now;
    logged_at = xTaskGetTickCount();
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

/* ---- play order -------------------------------------------------------------- */

static bool shuffle_enabled(void)
{
  status_lock();
  const bool shuffle = s_status.shuffle;
  status_unlock();
  return shuffle;
}

/* Seed from the hardware RNG (clocked by the 48 MHz PLLQ output) */
static void random_seed(void)
{
  __HAL_RCC_RNG_CLK_ENABLE();
  RNG->CR |= RNG_CR_RNGEN;
  for (uint32_t tries = 0; tries < 10000U && s_random == 0U; tries++)
  {
    if ((RNG->SR & RNG_SR_DRDY) != 0U)
    {
      s_random = RNG->DR;
    }
  }
  RNG->CR &= ~RNG_CR_RNGEN;
  __HAL_RCC_RNG_CLK_DISABLE();

  if (s_random == 0U)
  {
    s_random = (uint32_t)xTaskGetTickCount() | 1U;   /* RNG failed: any non-zero seed */
  }
}

/* Uniform enough for n up to PLAYER_MAX_TRACKS */
static uint32_t random_below(uint32_t n)
{
  if (s_random == 0U)
  {
    random_seed();
  }
  s_random ^= s_random << 13;
  s_random ^= s_random >> 17;
  s_random ^= s_random << 5;
  return s_random % n;
}

/* New shuffled order starting with @p first, or, if @p first is not a track,
 * with any track but the current one (so a track never plays twice in a row). */
static void shuffle_order(uint16_t first)
{
  const uint16_t count = track_count();
  for (uint16_t i = 0; i < count; i++)
  {
    s_order[i] = i;
  }
  for (uint16_t i = count; i > 1U; i--)
  {
    const uint16_t j   = (uint16_t)random_below(i);
    const uint16_t tmp = s_order[i - 1U];
    s_order[i - 1U] = s_order[j];
    s_order[j]      = tmp;
  }

  if (first < count)
  {
    for (uint16_t i = 0; i < count; i++)
    {
      if (s_order[i] == first)
      {
        s_order[i] = s_order[0];
        s_order[0] = first;
        break;
      }
    }
  }
  else if (count > 1U && s_order[0] == track_index())
  {
    s_order[0]         = s_order[count - 1U];
    s_order[count - 1U] = track_index();
  }
  s_order_len = count;
  s_order_pos = 0;
}

/* The library may have been re-read since the order was drawn */
static void check_order(void)
{
  if (s_order_len != track_count())
  {
    shuffle_order(track_index());
  }
}

/* Track after the current one in play order. @p wrapped tells whether the order
 * starts over (the end of the playlist). */
static uint16_t following_track(bool *wrapped)
{
  const uint16_t count = track_count();
  if (!shuffle_enabled())
  {
    const uint16_t next = (uint16_t)(track_index() + 1U);
    *wrapped = (next >= count);
    return *wrapped ? 0U : next;
  }

  check_order();
  if ((uint32_t)s_order_pos + 1U < s_order_len)
  {
    *wrapped = false;
    return s_order[++s_order_pos];
  }
  *wrapped = true;
  shuffle_order(UINT16_MAX);
  return s_order[0];
}

/* Track before the current one in play order (wrapping around) */
static uint16_t preceding_track(void)
{
  const uint16_t count = track_count();
  if (!shuffle_enabled())
  {
    return (uint16_t)((track_index() + count - 1U) % count);
  }

  check_order();
  s_order_pos = (s_order_pos == 0U) ? (uint16_t)(s_order_len - 1U) : (uint16_t)(s_order_pos - 1U);
  return s_order[s_order_pos];
}

/* ---- track switching ------------------------------------------------------- */

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

/* Play the file at @p path, from outside the library */
static void play_file(const char *path)
{
  const uint32_t previous_rate = s_track.open ? s_track.info.sample_rate : 0U;
  const bool was_playing = (get_state() == PLAYER_STATE_PLAYING);
  const char *slash = strrchr(path, '/');

  if (!open_path(path, (slash != NULL) ? slash + 1 : path, track_index(), true))
  {
    /* Back to the library's track, stopped */
    LOG_WARN("player: %s cannot be played", path);
    stop_output();
    const uint16_t count = track_count();
    if (count != 0U && open_playable_track(track_index(), count))
    {
      set_state(PLAYER_STATE_STOPPED, "This file cannot be played");
    }
    else
    {
      set_state(PLAYER_STATE_NO_TRACKS, "This file cannot be played");
    }
    return;
  }
  if (was_playing && s_track.info.sample_rate == previous_rate && s_output_rate == previous_rate)
  {
    return;
  }
  stop_output();
  start_output();
}

static void on_track_finished(void)
{
  player_repeat_t repeat;
  bool from_file;
  status_lock();
  repeat    = s_status.repeat;
  from_file = s_status.from_file;
  status_unlock();

  if (repeat == PLAYER_REPEAT_ONE || from_file)
  {
    uint32_t actual_s;
    decoder_seek(0, &actual_s);
    s_track.frames_out    = 0;
    s_track.silent_halves = 0;
    if (from_file && repeat != PLAYER_REPEAT_ONE)
    {
      /* A file from elsewhere: stays loaded, stopped at its start */
      stop_output();
      update_position();
      set_state(PLAYER_STATE_STOPPED, NULL);
    }
    return;
  }

  bool wrapped;
  const uint16_t next = following_track(&wrapped);
  if (wrapped)
  {
    /* End of the playlist: stop at the first track (of a new order when shuffling) */
    stop_output();
    open_playable_track(next, track_count());
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
  /* A file from elsewhere may play while the library is empty */
  const bool has_tracks = (state == PLAYER_STATE_STOPPED || state == PLAYER_STATE_PLAYING ||
                           state == PLAYER_STATE_PAUSED) && track_count() != 0U;

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
        bool wrapped;
        skip_to(following_track(&wrapped), state);
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
          skip_to(preceding_track(), state);
        }
      }
      break;

    case CMD_PLAY_INDEX:
      if (has_tracks && cmd->value < track_count())
      {
        if (shuffle_enabled())
        {
          shuffle_order((uint16_t)cmd->value);   /* a new order from the chosen track */
        }
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

    case CMD_SET_SHUFFLE:
      status_lock();
      s_status.shuffle = (cmd->value != 0U);
      status_unlock();
      if (cmd->value != 0U && has_tracks)
      {
        shuffle_order(track_index());   /* the current track stays first */
      }
      break;

    case CMD_RELEASE_AUDIO:
      /* Another player (video) takes the audio output: stop, keep the track
       * open (play continues from here), and tell the caller */
      if (s_output_rate != 0U)
      {
        stop_output();
      }
      if (state == PLAYER_STATE_PLAYING || state == PLAYER_STATE_PAUSED)
      {
        set_state(PLAYER_STATE_STOPPED, NULL);
      }
      xSemaphoreGive(s_audio_released);
      break;

    case CMD_PLAY_FILE:
      if (state != PLAYER_STATE_NO_CARD)
      {
        static char path[PLAYER_PATH_MAX];
        status_lock();
        snprintf(path, sizeof(path), "%s", s_file_request);
        status_unlock();
        play_file(path);
      }
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

/* Wait for a card and mount it (retrying every CARD_RETRY_MS). */
static void mount_card(void)
{
  bool warned = false;

  while (f_mount(&s_fs, "", 1) != FR_OK)
  {
    set_state(PLAYER_STATE_NO_CARD, "Insert an SD card");
    if (!warned)
    {
      LOG_WARN("player: SD card not mounted, retrying");
      warned = true;
    }
    f_unmount("");
    vTaskDelay(pdMS_TO_TICKS(CARD_RETRY_MS));
  }

  static const char *const k_fs_names[] = { "?", "FAT12", "FAT16", "FAT32", "exFAT" };
  status_lock();
  s_status.card_mb = (uint32_t)(((uint64_t)(s_fs.n_fatent - 2U) * s_fs.csize) / 2048U);
  snprintf(s_status.card_fs, sizeof(s_status.card_fs), "%s",
           k_fs_names[(s_fs.fs_type <= FS_EXFAT) ? s_fs.fs_type : 0]);
  status_unlock();
}

/* Read the library of the mounted card. The card stays mounted without songs
 * too: the other apps (files, video, pictures, Doom), the settings and the
 * fonts use it. */
static uint16_t load_library(void)
{
  const uint16_t count = scan_library();
  status_lock();
  s_status.track_count = count;
  s_status.library_serial++;
  status_unlock();

  if (count > 0U)
  {
    LOG_INFO("player: %u track(s) in %s", (unsigned)count, PLAYER_MUSIC_DIR);
    set_state(PLAYER_STATE_STOPPED, NULL);
  }
  else
  {
    LOG_WARN("player: no .wav or .mp3 files in %s", PLAYER_MUSIC_DIR);
    set_state(PLAYER_STATE_NO_TRACKS, "No .wav or .mp3 files in /Music");
  }
  return count;
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

  mount_card();

  /* The saved settings decide the volume and whether to start in shuffle mode
   * (loaded with the card, songs or not) */
  settings_load();
  const uint16_t count = load_library();
  const settings_t settings = settings_get();
  status_lock();
  s_status.volume  = settings.volume;
  s_status.shuffle = settings.shuffle_on_start;
  status_unlock();

  wait_for_ui();

  /* Shuffle (the default) also picks the first song at random */
  const uint16_t first = (shuffle_enabled() && count > 0U) ? (uint16_t)random_below(count) : 0U;
  if (count > 0U && open_playable_track(first, count))
  {
    if (shuffle_enabled())
    {
      shuffle_order(track_index());
    }
    start_output();
  }
  else if (count > 0U)
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
      if (events & (NOTIFY_HALF_DONE | NOTIFY_FULL_DONE))
      {
        report_output_errors();
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
  s_audio_released = xSemaphoreCreateBinary();
  configASSERT(s_audio_released != NULL);
  configASSERT(s_status_mutex != NULL && s_cmd_queue != NULL);

  s_status.state  = PLAYER_STATE_NO_CARD;
  s_status.repeat = PLAYER_REPEAT_OFF;
  s_status.shuffle = true;
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
    snprintf(name, len, "%.*s", (int)base_name_len(NAME(index)), NAME(index));
    ok = true;
  }
  status_unlock();
  return ok;
}

bool player_get_track_path(uint16_t index, char *path, size_t len)
{
  bool ok = false;

  status_lock();
  if (index < s_status.track_count && len > 0U)
  {
    snprintf(path, len, "%s/%s", PLAYER_MUSIC_DIR, NAME(index));
    ok = true;
  }
  status_unlock();
  return ok;
}
bool player_get_current_path(char *path, size_t len)
{
  status_lock();
  const bool ok = s_current_path[0] != '\0' && len > 0U;
  if (ok)
  {
    snprintf(path, len, "%s", s_current_path);
  }
  status_unlock();
  return ok;
}

void player_play_file(const char *path)
{
  status_lock();
  snprintf(s_file_request, sizeof(s_file_request), "%s", path);
  status_unlock();
  send_command(CMD_PLAY_FILE, 0);
}

void player_seek(uint32_t position_s)       { send_command(CMD_SEEK, position_s); }
void player_set_repeat(player_repeat_t r)   { send_command(CMD_SET_REPEAT, (uint32_t)r); }
void player_set_shuffle(bool shuffle)       { send_command(CMD_SET_SHUFFLE, shuffle ? 1U : 0U); }

bool player_release_audio(void)
{
  (void)xSemaphoreTake(s_audio_released, 0);
  send_command(CMD_RELEASE_AUDIO, 0);
  return xSemaphoreTake(s_audio_released, pdMS_TO_TICKS(1000)) == pdTRUE;
}

void player_set_volume(uint8_t volume)
{
  send_command(CMD_SET_VOLUME, (volume > 100U) ? 100U : volume);
}
