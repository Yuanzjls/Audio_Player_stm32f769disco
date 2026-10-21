/**
 * @file  video_player.c
 * @brief AVI playback, see video_player.h.
 *
 * Two tasks:
 *  - reader: reads the "movi" chunks in file order; audio is decoded (16-bit
 *    PCM, or MP3 with minimp3) into a ring buffer, compressed video frames go
 *    into a queue of slots. It runs ahead of playback as far as the ring and
 *    the slots allow, and starts the audio output once some audio is buffered.
 *  - render: takes the frames in order, waits until the clock reaches each
 *    one's time, decodes it with the hardware JPEG codec and converts it
 *    (yuv2rgb) straight into the display's free frame buffer, then shows it.
 *    A frame whose time has passed is dropped if the next one is already
 *    there. An empty frame means "show the previous one again".
 * The audio output's DMA interrupt refills each half of its buffer from the
 * ring and counts the halves played: that, plus the DMA position, is the clock.
 *
 * The control functions (GUI task) stop the pipeline by clearing s_run and
 * taking the locks the tasks hold while they work, so no frame is half drawn
 * and no chunk half read when they change things.
 */
#include "video_player.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "audio_out.h"
#include "avi.h"
#include "display.h"
#include "ff.h"
#include "jpeg_hw.h"
#include "log.h"
#include "main.h"
#include "minimp3.h"
#include "player.h"
#include "yuv2rgb.h"

#define SLOTS              6
#define SLOT_SIZE          (192U * 1024U)   /* largest compressed frame */
#define RING_FRAMES        32768U           /* audio buffered ahead: 0.74 s at 44.1 kHz */
#define PRIME_MS           150U             /* audio buffered before the output starts */
#define HALF_FRAMES        1024U            /* DMA buffer half: 23 ms at 44.1 kHz */
#define READ_PIECE         4096U
#define MP3_IN_SIZE        16384U
#define READER_STACK_WORDS 6144             /* minimp3 needs ~16 KB of scratch on the stack */
#define RENDER_STACK_WORDS 1024
#define READER_PRIORITY    3
#define RENDER_PRIORITY    1                /* the GUI's: a slow decode must not starve the touch input */

typedef struct
{
  uint8_t *data;
  uint32_t size;            /* 0: show the previous frame again */
  uint32_t frame;           /* number in the file */
} slot_t;

/* ---- state ------------------------------------------------------------------ */

static SemaphoreHandle_t s_ctrl;            /* control functions, one at a time */
static SemaphoreHandle_t s_reader_lock;     /* held by the reader while it works */
static SemaphoreHandle_t s_render_lock;     /* held by the renderer while it works */
static QueueHandle_t     s_free;            /* slot numbers */
static QueueHandle_t     s_ready;
static TaskHandle_t      s_reader;
static TaskHandle_t      s_render;

static uint8_t s_slot_mem[SLOTS][SLOT_SIZE] __attribute__((section(".sdram"), aligned(32)));
static slot_t  s_slots[SLOTS];

static volatile video_state_t s_state = VIDEO_IDLE;
static volatile bool s_run;                 /* tasks may work */
static FIL           s_file;
static avi_info_t    s_avi;
static uint32_t      s_pos;                 /* next chunk header (reader) */
static uint32_t      s_next_frame;          /* number of the next video chunk */
static volatile bool s_eof;
static bool          s_audio_ok;            /* the audio can be played */
static uint32_t      s_shown, s_dropped;
static uint32_t      s_decode_ms;           /* total time spent decoding the shown frames */

/* Audio ring: stereo frames, written by the reader, read by the DMA interrupt */
static int16_t           s_ring[RING_FRAMES * 2U] __attribute__((section(".sdram"), aligned(32)));
static volatile uint32_t s_ring_w, s_ring_r;   /* frames, counting up */
static int16_t           s_dma[2U * HALF_FRAMES * 2U] DMA_BUFFER;
static volatile uint32_t s_halves;          /* DMA buffer halves played */
static volatile bool     s_audio_started;

/* Clock */
static volatile bool s_clock_running;
static uint32_t      s_clock_base_ms;       /* time at the start (or last seek / pause) */
static TickType_t    s_tick_start;          /* without audio: tick at the base */

/* MP3 audio */
static mp3dec_t s_mp3;
static uint8_t  s_mp3_in[MP3_IN_SIZE];
static uint32_t s_mp3_len;
static uint8_t  s_piece[READ_PIECE];

/* Frame being decoded (render task) */
static yuv2rgb_t     s_target;
static const slot_t *s_job_slot;
static uint32_t      s_job_read;
static const uint16_t *s_cleared[2];        /* frame buffers with black borders */

/* Layers over the video (GUI task sets them, render task blends them onto every
 * frame). What they cover is kept, for a clean copy of the frame on screen. */
#define UNDER_MAX (DISPLAY_WIDTH * 128U)    /* pixels of the largest layer */
static SemaphoreHandle_t    s_layer_lock;
static const video_layer_t *s_layers[VIDEO_LAYERS];
static bool                 s_layers_changed;
static video_layer_t        s_drawn[VIDEO_LAYERS];     /* blended onto s_last_fb */
static bool                 s_drawn_on[VIDEO_LAYERS];
static uint16_t             s_under[VIDEO_LAYERS][UNDER_MAX] __attribute__((section(".sdram"), aligned(32)));
static uint16_t            *s_last_fb;                 /* last frame shown */

/* ---- audio ------------------------------------------------------------------ */

static uint32_t ring_level(void) { return s_ring_w - s_ring_r; }

/* Copy @p frames frames from the ring into @p out (silence where it runs dry) */
static void ring_take(int16_t *out, uint32_t frames)
{
  uint32_t avail = ring_level();
  uint32_t n = (avail < frames) ? avail : frames;
  uint32_t r = s_ring_r % RING_FRAMES;
  uint32_t first = (n < RING_FRAMES - r) ? n : RING_FRAMES - r;
  memcpy(out, &s_ring[r * 2U], first * 4U);
  memcpy(out + first * 2U, s_ring, (n - first) * 4U);
  memset(out + n * 2U, 0, (frames - n) * 4U);
  s_ring_r += n;
}

/* DMA interrupt: half @p event played, refill it */
static void audio_callback(audio_out_event_t event)
{
  int16_t *half = (event == AUDIO_OUT_HALF_DONE) ? s_dma : &s_dma[HALF_FRAMES * 2U];
  ring_take(half, HALF_FRAMES);
  s_halves++;
}

/* Stereo frames played since the output started */
static uint32_t audio_frames_played(void)
{
  __disable_irq();                   /* the audio interrupt is above the RTOS critical sections */
  const uint32_t halves = s_halves;
  const uint32_t pos    = audio_out_position();
  __enable_irq();
  /* Offset into the half playing now; a wrap whose interrupt is still pending
   * shows as a position in the other half */
  int32_t offset = (int32_t)pos - (int32_t)((halves & 1U) ? HALF_FRAMES : 0U);
  if (offset < 0)
  {
    offset += (int32_t)(2U * HALF_FRAMES);
  }
  return halves * HALF_FRAMES + (uint32_t)offset;
}

static uint32_t clock_ms(void)
{
  if (!s_clock_running)
  {
    return s_clock_base_ms;
  }
  if (s_audio_started)
  {
    return s_clock_base_ms + (uint32_t)(((uint64_t)audio_frames_played() * 1000U) / s_avi.sample_rate);
  }
  return s_clock_base_ms + (uint32_t)((xTaskGetTickCount() - s_tick_start) * portTICK_PERIOD_MS);
}

/* Write @p frames stereo frames into the ring, waiting for room; false if stopped */
static bool ring_put(const int16_t *pcm, uint32_t frames, uint32_t channels)
{
  for (uint32_t i = 0; i < frames;)
  {
    while (RING_FRAMES - ring_level() == 0U)
    {
      if (!s_run)
      {
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    uint32_t room = RING_FRAMES - ring_level();
    for (; i < frames && room > 0U; i++, room--)
    {
      const uint32_t w = s_ring_w % RING_FRAMES;
      const int16_t l = pcm[i * channels];
      s_ring[w * 2U]      = l;
      s_ring[w * 2U + 1U] = (channels == 2U) ? pcm[i * 2U + 1U] : l;
      s_ring_w++;
    }
  }
  return true;
}

/* Decode the MP3 frames complete in the input buffer into the ring */
static bool mp3_drain(void)
{
  static int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  uint32_t used = 0;
  for (;;)
  {
    mp3dec_frame_info_t fi;
    memset(&fi, 0, sizeof(fi));
    const int samples = mp3dec_decode_frame(&s_mp3, &s_mp3_in[used], (int)(s_mp3_len - used), pcm, &fi);
    if (fi.frame_bytes == 0)
    {
      break;                       /* needs more data */
    }
    used += (uint32_t)fi.frame_bytes;
    if (samples > 0 && !ring_put(pcm, (uint32_t)samples, (uint32_t)fi.channels))
    {
      return false;
    }
  }
  memmove(s_mp3_in, &s_mp3_in[used], s_mp3_len - used);
  s_mp3_len -= used;
  return true;
}

static bool feed_audio(const avi_chunk_t *c)
{
  if (!s_audio_ok)
  {
    return true;
  }
  for (uint32_t done = 0; done < c->size;)
  {
    UINT got = 0;
    const UINT want = (c->size - done < READ_PIECE) ? (UINT)(c->size - done) : (UINT)READ_PIECE;
    if (f_lseek(&s_file, c->offset + done) != FR_OK || f_read(&s_file, s_piece, want, &got) != FR_OK || got == 0U)
    {
      return false;
    }
    done += got;
    if (s_avi.audio_format == AVI_AUDIO_PCM)
    {
      if (!ring_put((const int16_t *)s_piece, got / (s_avi.channels * 2U), s_avi.channels))
      {
        return false;
      }
    }
    else
    {
      /* MP3: frames may be split across chunks; the input buffer joins them */
      const uint32_t n = (got < MP3_IN_SIZE - s_mp3_len) ? got : MP3_IN_SIZE - s_mp3_len;
      memcpy(&s_mp3_in[s_mp3_len], s_piece, n);
      s_mp3_len += n;
      if (!mp3_drain())
      {
        return false;
      }
    }
  }
  return true;
}

/* Start the audio output once enough is buffered (or the file has no more) */
static void maybe_start_audio(void)
{
  if (s_audio_started || !s_run)
  {
    return;
  }
  if (!s_audio_ok)
  {
    if (!s_clock_running)
    {
      s_tick_start    = xTaskGetTickCount();
      s_clock_running = true;
    }
    return;
  }
  const uint32_t prime = s_avi.sample_rate * PRIME_MS / 1000U;
  if (ring_level() < prime && !s_eof)
  {
    return;
  }
  player_status_t st;
  player_get_status(&st);
  ring_take(s_dma, 2U * HALF_FRAMES);
  s_halves = 0;
  if (audio_out_configure(s_avi.sample_rate, st.volume, audio_callback) &&
      audio_out_start(s_dma, sizeof(s_dma)))
  {
    s_audio_started = true;
  }
  else
  {
    LOG_WARN("video: audio output failed, playing without sound");
    s_audio_ok = false;
    s_tick_start = xTaskGetTickCount();
  }
  s_clock_running = true;
}

/* ---- reader ------------------------------------------------------------------- */

static bool queue_frame(const avi_chunk_t *c)
{
  uint32_t idx;
  while (xQueueReceive(s_free, &idx, pdMS_TO_TICKS(20)) != pdTRUE)
  {
    maybe_start_audio();
    if (!s_run)
    {
      return false;
    }
  }
  slot_t *slot = &s_slots[idx];
  slot->frame = s_next_frame++;
  slot->size  = 0;
  if (c->size > SLOT_SIZE)
  {
    LOG_WARN("video: frame %lu too large (%lu bytes)", (unsigned long)slot->frame, (unsigned long)c->size);
  }
  else if (c->size > 0U)
  {
    UINT got = 0;
    if (f_lseek(&s_file, c->offset) == FR_OK && f_read(&s_file, slot->data, c->size, &got) == FR_OK &&
        got == c->size)
    {
      slot->size = c->size;
    }
  }
  xQueueSend(s_ready, &idx, portMAX_DELAY);
  return true;
}

static void reader_task(void *arg)
{
  (void)arg;
  for (;;)
  {
    xSemaphoreTake(s_reader_lock, portMAX_DELAY);
    bool idle = !s_run || s_eof;
    if (!idle)
    {
      avi_chunk_t c;
      if (!avi_next_chunk(&s_file, &s_avi, &s_pos, &c))
      {
        s_eof = true;
      }
      else if (c.type == AVI_CHUNK_VIDEO)
      {
        queue_frame(&c);
      }
      else
      {
        feed_audio(&c);
      }
    }
    maybe_start_audio();
    xSemaphoreGive(s_reader_lock);
    if (idle)
    {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    }
  }
}

/* ---- render ------------------------------------------------------------------- */

static uint32_t read_frame(void *ctx, uint8_t *buf, uint32_t max)
{
  (void)ctx;
  const uint32_t n = (s_job_slot->size - s_job_read < max) ? s_job_slot->size - s_job_read : max;
  memcpy(buf, &s_job_slot->data[s_job_read], n);
  s_job_read += n;
  return n;
}

static bool begin_frame(void *ctx, const jpeg_hw_layout_t *layout)
{
  (void)ctx;
  s_target.layout    = *layout;
  s_target.mcu_index = 0;
  /* Centred; a picture larger than the screen is cut to its middle */
  s_target.x0 = ((int32_t)DISPLAY_WIDTH - (int32_t)layout->width) / 2;
  s_target.y0 = ((int32_t)DISPLAY_HEIGHT - (int32_t)layout->height) / 2;
  return true;
}

static void frame_mcus(void *ctx, const uint8_t *data, uint32_t len)
{
  (void)ctx;
  yuv2rgb_mcus(&s_target, data, len);
}

/* Blend layer @p l onto @p fb, keeping what it covers in @p under. The layer's
 * colours are premultiplied by its alpha: out = rgb + fb * (255 - alpha). */
__attribute__((optimize("O3")))
static void blend_layer(uint16_t *fb, const video_layer_t *l, uint16_t *under)
{
  for (uint32_t y = 0; y < l->h; y++)
  {
    uint16_t       *d = &fb[(uint32_t)(l->y + (int32_t)y) * DISPLAY_WIDTH + (uint32_t)l->x];
    const uint16_t *s = &l->rgb[y * l->w];
    memcpy(&under[y * l->w], d, l->w * 2U);
    if (l->alpha == NULL)
    {
      memcpy(d, s, l->w * 2U);
      continue;
    }
    const uint8_t *a = &l->alpha[y * l->w];
    for (uint32_t x = 0; x < l->w; x++)
    {
      const uint32_t k = a[x];
      if (k == 0U)
      {
        continue;
      }
      if (k == 255U)
      {
        d[x] = s[x];
        continue;
      }
      const uint32_t inv = 255U - k;
      const uint32_t p   = d[x];
      const uint32_t r   = ((p >> 11) * inv) >> 8;
      const uint32_t g   = (((p >> 5) & 0x3FU) * inv) >> 8;
      const uint32_t b   = ((p & 0x1FU) * inv) >> 8;
      d[x] = (uint16_t)(s[x] + ((r << 11) | (g << 5) | b));
    }
  }
}

static void show_frame(const slot_t *slot)
{
  uint16_t *fb = display_video_frame();
  const bool small = s_avi.width < DISPLAY_WIDTH || s_avi.height < DISPLAY_HEIGHT;

  xSemaphoreTake(s_layer_lock, portMAX_DELAY);
  if (s_layers_changed)
  {
    s_layers_changed = false;
    s_cleared[0] = s_cleared[1] = NULL;   /* a layer taken away from a border goes with it */
  }
  xSemaphoreGive(s_layer_lock);
  /* A picture smaller than the screen leaves borders: black, once per buffer;
   * where layers are blended, every time (the picture does not cover them) */
  if (small && s_cleared[0] != fb && s_cleared[1] != fb)
  {
    memset(fb, 0, (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * 2U);
    s_cleared[(s_cleared[0] == NULL) ? 0 : 1] = fb;
  }
  else if (small)
  {
    for (uint32_t i = 0; i < VIDEO_LAYERS; i++)
    {
      if (s_drawn_on[i])
      {
        for (uint32_t y = 0; y < s_drawn[i].h; y++)
        {
          memset(&fb[(uint32_t)(s_drawn[i].y + (int32_t)y) * DISPLAY_WIDTH + (uint32_t)s_drawn[i].x], 0,
                 s_drawn[i].w * 2U);
        }
      }
    }
  }
  s_target.dst   = fb;
  s_target.dst_w = DISPLAY_WIDTH;
  s_target.dst_h = DISPLAY_HEIGHT;
  s_job_slot = slot;
  s_job_read = 0;
  const jpeg_hw_job_t job = { read_frame, begin_frame, frame_mcus, NULL };
  const TickType_t start = xTaskGetTickCount();
  if (jpeg_hw_decode(&job) == JPEG_HW_OK)
  {
    s_decode_ms += (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
    xSemaphoreTake(s_layer_lock, portMAX_DELAY);
    for (uint32_t i = 0; i < VIDEO_LAYERS; i++)
    {
      s_drawn_on[i] = (s_layers[i] != NULL);
      if (s_drawn_on[i])
      {
        s_drawn[i] = *s_layers[i];
        blend_layer(fb, &s_drawn[i], s_under[i]);
      }
    }
    xSemaphoreGive(s_layer_lock);
    display_video_present(fb);
    s_last_fb = fb;
    s_shown++;
  }
}

static void render_task(void *arg)
{
  (void)arg;
  for (;;)
  {
    uint32_t idx;
    if (xQueueReceive(s_ready, &idx, pdMS_TO_TICKS(50)) != pdTRUE)
    {
      continue;
    }
    xSemaphoreTake(s_render_lock, portMAX_DELAY);
    const slot_t *slot = &s_slots[idx];
    const uint32_t frame_ms = s_avi.frame_us / 1000U;
    const uint32_t due = (uint32_t)(((uint64_t)slot->frame * s_avi.frame_us) / 1000U);

    /* Wait for the frame's time (the clock starts with the audio) */
    bool put_back = false;
    for (;;)
    {
      if (!s_run)
      {
        put_back = true;
        break;
      }
      const uint32_t now = clock_ms();
      if (s_clock_running && now + 2U >= due)
      {
        break;
      }
      const uint32_t wait = (s_clock_running && due - now < 10U) ? due - now : 10U;
      vTaskDelay(pdMS_TO_TICKS(wait > 0U ? wait : 1U));
    }
    if (put_back)
    {
      xQueueSendToFront(s_ready, &idx, 0);   /* shown when playing again */
    }
    else
    {
      const bool late = clock_ms() > due + frame_ms && uxQueueMessagesWaiting(s_ready) > 0U;
      if (late)
      {
        s_dropped++;
      }
      else if (slot->size > 0U)
      {
        show_frame(slot);
      }
      xQueueSend(s_free, &idx, 0);
    }
    xSemaphoreGive(s_render_lock);
    if (put_back)
    {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    }
  }
}

/* ---- control (GUI task) ---------------------------------------------------- */

/* Stop both tasks between two steps; they stay stopped until s_run is set */
static void halt_pipeline(void)
{
  s_run = false;
  xSemaphoreTake(s_reader_lock, portMAX_DELAY);
  xSemaphoreTake(s_render_lock, portMAX_DELAY);
}

static void release_pipeline(void)
{
  xSemaphoreGive(s_render_lock);
  xSemaphoreGive(s_reader_lock);
}

/* Empty the frame queue and the audio ring, stop the audio output */
static void flush(void)
{
  uint32_t idx;
  while (xQueueReceive(s_ready, &idx, 0) == pdTRUE)
  {
    xQueueSend(s_free, &idx, 0);
  }
  if (s_audio_started)
  {
    audio_out_stop();
    s_audio_started = false;
  }
  s_ring_w = s_ring_r = 0;
  s_mp3_len = 0;
  mp3dec_init(&s_mp3);
  s_clock_running = false;
}

/* Continue from video frame @p frame */
static void restart_at(uint32_t frame)
{
  uint32_t pos = s_avi.movi_start;
  uint32_t audio_bytes;
  if (frame > 0U && !avi_seek(&s_file, &s_avi, frame, &pos, &audio_bytes))
  {
    frame = 0;                       /* no index: from the start */
    pos = s_avi.movi_start;
  }
  s_pos           = pos;
  s_next_frame    = frame;
  s_eof           = false;
  s_clock_base_ms = (uint32_t)(((uint64_t)frame * s_avi.frame_us) / 1000U);
}

void video_init(void)
{
  s_ctrl        = xSemaphoreCreateMutex();
  s_reader_lock = xSemaphoreCreateMutex();
  s_render_lock = xSemaphoreCreateMutex();
  s_layer_lock  = xSemaphoreCreateMutex();
  s_free        = xQueueCreate(SLOTS, sizeof(uint32_t));
  s_ready       = xQueueCreate(SLOTS, sizeof(uint32_t));
  configASSERT(s_ctrl && s_reader_lock && s_render_lock && s_layer_lock && s_free && s_ready);
  for (uint32_t i = 0; i < SLOTS; i++)
  {
    s_slots[i].data = s_slot_mem[i];
    xQueueSend(s_free, &i, 0);
  }
  yuv2rgb_init();
  mp3dec_init(&s_mp3);
  if (xTaskCreate(reader_task, "vread", READER_STACK_WORDS, NULL, READER_PRIORITY, &s_reader) != pdPASS ||
      xTaskCreate(render_task, "vshow", RENDER_STACK_WORDS, NULL, RENDER_PRIORITY, &s_render) != pdPASS)
  {
    configASSERT(0);
  }
}

static void close_locked(void)
{
  halt_pipeline();
  flush();
  if (s_state != VIDEO_IDLE)
  {
    f_close(&s_file);
  }
  s_state = VIDEO_IDLE;
  release_pipeline();
}

bool video_open(const char *path, video_info_t *info)
{
  xSemaphoreTake(s_ctrl, portMAX_DELAY);
  close_locked();
  memset(info, 0, sizeof(*info));

  bool ok = false;
  if (f_open(&s_file, path, FA_READ) != FR_OK)
  {
    LOG_WARN("video: cannot open %s", path);
  }
  else
  {
    const avi_result_t r = avi_open(&s_file, &s_avi);
    if (r != AVI_OK)
    {
      LOG_WARN("video: %s: %s", path, avi_result_str(r));
      f_close(&s_file);
    }
    else
    {
      ok = true;
      s_audio_ok = s_avi.has_audio && audio_out_is_rate_supported(s_avi.sample_rate);
      if (s_avi.has_audio && !s_audio_ok)
      {
        LOG_WARN("video: %lu Hz audio not supported, playing without sound", (unsigned long)s_avi.sample_rate);
      }
      info->width       = s_avi.width;
      info->height      = s_avi.height;
      info->fps_x100    = (uint32_t)(100000000ULL / s_avi.frame_us);
      info->duration_ms = (uint32_t)(((uint64_t)s_avi.frames * s_avi.frame_us) / 1000U);
      info->has_audio   = s_audio_ok;
      if (!s_avi.has_audio)
      {
        snprintf(info->audio, sizeof(info->audio), "no sound");
      }
      else
      {
        snprintf(info->audio, sizeof(info->audio), "%s %lu.%lu kHz %s",
                 (s_avi.audio_format == AVI_AUDIO_MP3) ? "MP3" : "PCM",
                 (unsigned long)(s_avi.sample_rate / 1000U), (unsigned long)(s_avi.sample_rate % 1000U / 100U),
                 (s_avi.channels == 2U) ? "stereo" : "mono");
      }
      s_shown = s_dropped = s_decode_ms = 0;
      s_last_fb = NULL;
      memset(s_drawn_on, 0, sizeof(s_drawn_on));
      s_cleared[0] = s_cleared[1] = NULL;
      restart_at(0);
      s_state = VIDEO_PAUSED;
      LOG_INFO("video: %s %lux%lu %lu.%02lu fps, %s", path, (unsigned long)s_avi.width, (unsigned long)s_avi.height,
               (unsigned long)(info->fps_x100 / 100U), (unsigned long)(info->fps_x100 % 100U), info->audio);
    }
  }
  xSemaphoreGive(s_ctrl);
  return ok;
}

void video_play(void)
{
  xSemaphoreTake(s_ctrl, portMAX_DELAY);
  if (s_state == VIDEO_PAUSED)
  {
    s_cleared[0] = s_cleared[1] = NULL;   /* LVGL drew into the buffers meanwhile */
    if (s_audio_started)
    {
      audio_out_resume();
    }
    else if (s_clock_running)
    {
      s_tick_start = xTaskGetTickCount();
    }
    if (!s_audio_started && !s_audio_ok)
    {
      s_clock_running = false;           /* restarted by the reader from the base */
    }
    s_state = VIDEO_PLAYING;
    s_run   = true;
    xTaskNotifyGive(s_reader);
    xTaskNotifyGive(s_render);
  }
  xSemaphoreGive(s_ctrl);
}

void video_pause(void)
{
  xSemaphoreTake(s_ctrl, portMAX_DELAY);
  if (s_state == VIDEO_PLAYING)
  {
    halt_pipeline();
    if (s_audio_started)
    {
      audio_out_pause();
    }
    else
    {
      s_clock_base_ms = clock_ms();
    }
    s_state = VIDEO_PAUSED;
    release_pipeline();
  }
  xSemaphoreGive(s_ctrl);
}

void video_seek(uint32_t position_ms)
{
  xSemaphoreTake(s_ctrl, portMAX_DELAY);
  if (s_state != VIDEO_IDLE)
  {
    const bool was_playing = (s_state == VIDEO_PLAYING);
    halt_pipeline();
    flush();
    restart_at((uint32_t)(((uint64_t)position_ms * 1000U) / s_avi.frame_us));
    s_state = VIDEO_PAUSED;
    release_pipeline();
    if (was_playing)
    {
      s_state = VIDEO_PLAYING;
      s_run   = true;
      xTaskNotifyGive(s_reader);
      xTaskNotifyGive(s_render);
    }
  }
  xSemaphoreGive(s_ctrl);
}

void video_close(void)
{
  xSemaphoreTake(s_ctrl, portMAX_DELAY);
  close_locked();
  xSemaphoreGive(s_ctrl);
}

video_state_t video_state(void)
{
  /* Played to the end: nothing more to read, show or hear */
  if (s_state == VIDEO_PLAYING && s_eof && uxQueueMessagesWaiting(s_ready) == 0U &&
      uxQueueMessagesWaiting(s_free) == SLOTS && (!s_audio_started || ring_level() == 0U))
  {
    return VIDEO_ENDED;
  }
  return s_state;
}

uint32_t video_position_ms(void)
{
  return (s_state == VIDEO_IDLE) ? 0U : clock_ms();
}

void video_stats(uint32_t *shown, uint32_t *dropped, uint32_t *decode_ms)
{
  *shown     = s_shown;
  *dropped   = s_dropped;
  *decode_ms = s_decode_ms;
}

void video_set_layer(uint32_t slot, const video_layer_t *layer)
{
  if (slot >= VIDEO_LAYERS)
  {
    return;
  }
  if (layer != NULL &&
      (layer->x < 0 || layer->y < 0 || (uint32_t)layer->x + layer->w > DISPLAY_WIDTH ||
       (uint32_t)layer->y + layer->h > DISPLAY_HEIGHT || layer->w * layer->h > UNDER_MAX || layer->rgb == NULL))
  {
    LOG_WARN("video: layer %lu does not fit", (unsigned long)slot);
    return;
  }
  xSemaphoreTake(s_layer_lock, portMAX_DELAY);
  if ((s_layers[slot] == NULL) != (layer == NULL))
  {
    s_layers_changed = true;
  }
  s_layers[slot] = layer;
  xSemaphoreGive(s_layer_lock);
}

void video_copy_frame(uint16_t *dst)
{
  if (s_last_fb == NULL)
  {
    memset(dst, 0, (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * 2U);
    return;
  }
  memcpy(dst, s_last_fb, (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * 2U);
  for (uint32_t i = 0; i < VIDEO_LAYERS; i++)
  {
    if (s_drawn_on[i])
    {
      for (uint32_t y = 0; y < s_drawn[i].h; y++)
      {
        memcpy(&dst[(uint32_t)(s_drawn[i].y + (int32_t)y) * DISPLAY_WIDTH + (uint32_t)s_drawn[i].x],
               &s_under[i][y * s_drawn[i].w], s_drawn[i].w * 2U);
      }
    }
  }
}
