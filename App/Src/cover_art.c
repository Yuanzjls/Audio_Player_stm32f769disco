/**
 * @file  cover_art.c
 * @brief Cover pictures decoded with the STM32F769's hardware JPEG codec
 *        (jpeg_hw.c), in a task of its own.
 *
 * The JPEG data is read from the music file as the codec needs it; the decoded
 * YCbCr samples of the centred square crop are averaged straight into a
 * COVER_ART_SIZE grid, so a picture of any size needs only that grid in memory.
 * The averages are converted to RGB565 (averaging before the conversion gives
 * the same result, the conversion being linear) with an alpha channel that
 * rounds the corners: an LVGL RGB565A8 image.
 * Baseline JPEG only, like the hardware: progressive JPEG and PNG pictures get
 * no cover.
 */
#include "cover_art.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "ff.h"
#include "jpeg_hw.h"
#include "log.h"
#include "main.h"
#include "src/misc/cache/instance/lv_image_cache.h"

#define SIZE              COVER_ART_SIZE
#define CORNER_RADIUS     16
#define MAX_SIDE          4096U        /* larger pictures are not shown */
#define TASK_STACK_WORDS  768
#define TASK_PRIORITY     2            /* above the GUI: it mostly sleeps, and wakes for each chunk */
#define PATH_MAX_LEN      300
#define NO_CELL           0xFFFFU
#define BACKDROP_W        COVER_ART_BACKDROP_W
#define BACKDROP_H        COVER_ART_BACKDROP_H
#define BLUR_GRID         24           /* the backdrop is this coarse a copy, smoothed */
#define BACKDROP_MIX      90           /* share of the cover in the backdrop, of 256 */
#define BACKDROP_PRIORITY 1            /* as the GUI, while the backdrop is drawn */

typedef struct
{
  uint32_t id;
  uint8_t  format;
  uint32_t offset;
  uint32_t size;
  char     path[PATH_MAX_LEN];
} request_t;

/* Sums of the samples that fall into one cell of the output grid */
typedef struct
{
  uint32_t y, cb, cr, n;
} cell_t;

/* The picture being averaged */
typedef struct
{
  jpeg_hw_layout_t jpeg;
  uint32_t mcu_index;   /* next MCU in the output stream */
  uint32_t grid;        /* cells per side of the averaging grid */
} layout_t;

static QueueHandle_t      s_requests;
/* Task only */
static FIL      s_file;
static uint32_t s_in_left;               /* bytes of the picture not read yet */
static layout_t s_layout;
static cell_t   s_cells[SIZE * SIZE] __attribute__((section(".sdram"), aligned(4)));
static uint16_t s_map_x[MAX_SIDE] __attribute__((section(".sdram")));   /* used once per cover: SDRAM */
static uint16_t s_map_y[MAX_SIDE] __attribute__((section(".sdram")));

/* Two images: one is shown while the other is decoded */
static uint8_t        s_pixels[2][SIZE * SIZE * 3] __attribute__((section(".sdram"), aligned(4)));
static lv_image_dsc_t s_images[2];
static uint16_t       s_backdrop_px[2][BACKDROP_W * BACKDROP_H] __attribute__((section(".sdram"), aligned(4)));
static lv_image_dsc_t s_backdrops[2];
static int32_t        s_tint[3];           /* colour the backdrop is washed with */
static uint8_t        s_alpha[SIZE * SIZE] __attribute__((section(".sdram")));
static uint8_t        s_next;            /* image the next decode writes */

/* Result of the latest request (task writes, GUI reads) */
static volatile uint32_t          s_done_id = UINT32_MAX;
static volatile cover_art_state_t s_done_state;
static volatile uint8_t           s_done_image;
static uint32_t                   s_dropped_id = UINT32_MAX;   /* GUI only */

/* ---- set-up ------------------------------------------------------------------ */

/* Alpha of the rounded corners, anti-aliased */
static void build_alpha(void)
{
  const float r = (float)CORNER_RADIUS;
  for (uint32_t y = 0; y < SIZE; y++)
  {
    for (uint32_t x = 0; x < SIZE; x++)
    {
      const float px = (float)x + 0.5f;
      const float py = (float)y + 0.5f;
      float dx = 0.0f;
      float dy = 0.0f;
      if (px < r)                    dx = r - px;
      else if (px > (float)SIZE - r) dx = px - ((float)SIZE - r);
      if (py < r)                    dy = r - py;
      else if (py > (float)SIZE - r) dy = py - ((float)SIZE - r);

      float a = 1.0f;
      if (dx > 0.0f && dy > 0.0f)
      {
        /* Coverage of the pixel: r - distance from the corner centre, over one pixel */
        a = r - sqrtf(dx * dx + dy * dy) + 0.5f;
        a = (a < 0.0f) ? 0.0f : (a > 1.0f ? 1.0f : a);
      }
      s_alpha[y * SIZE + x] = (uint8_t)(a * 255.0f + 0.5f);
    }
  }
}

/* ---- task side ----------------------------------------------------------------- */

/* jpeg_hw: the next piece of the picture's data from the music file */
static uint32_t read_picture(void *ctx, uint8_t *buf, uint32_t max)
{
  (void)ctx;
  UINT got = 0;
  const UINT want = (s_in_left < max) ? (UINT)s_in_left : (UINT)max;
  if (want == 0U || f_read(&s_file, buf, want, &got) != FR_OK)
  {
    s_in_left = 0;
    return 0;
  }
  s_in_left -= got;
  return got;
}

/* jpeg_hw: header parsed: set up the averaging grid */
static bool begin_picture(void *ctx, const jpeg_hw_layout_t *jl)
{
  (void)ctx;
  layout_t *l = &s_layout;
  memset(l, 0, sizeof(*l));
  l->jpeg = *jl;

  const uint32_t pad_w = jl->mcus_per_row * jl->mcu_w;
  const uint32_t pad_h = (jl->height + jl->mcu_h - 1U) / jl->mcu_h * jl->mcu_h;
  if (pad_w > MAX_SIDE || pad_h > MAX_SIDE)
  {
    LOG_WARN("cover: %lux%lu picture not supported", (unsigned long)jl->width, (unsigned long)jl->height);
    return false;
  }
  const uint32_t width  = jl->width;
  const uint32_t height = jl->height;

  /* Centred square crop, averaged into grid x grid cells */
  const uint32_t side = (width < height) ? width : height;
  const uint32_t x0   = (width - side) / 2U;
  const uint32_t y0   = (height - side) / 2U;
  l->grid = (side < SIZE) ? side : SIZE;

  for (uint32_t x = 0; x < pad_w; x++)
  {
    s_map_x[x] = (x >= x0 && x < x0 + side) ? (uint16_t)((x - x0) * l->grid / side) : NO_CELL;
  }
  for (uint32_t y = 0; y < pad_h; y++)
  {
    s_map_y[y] = (y >= y0 && y < y0 + side) ? (uint16_t)((y - y0) * l->grid / side) : NO_CELL;
  }
  memset(s_cells, 0, sizeof(cell_t) * l->grid * l->grid);
  return true;
}

/* Add the samples of one MCU to the cells they fall into */
static void collect_mcu(const uint8_t *mcu)
{
  const layout_t *l = &s_layout;
  const jpeg_hw_layout_t *j = &l->jpeg;
  const uint32_t mx = (l->mcu_index % j->mcus_per_row) * j->mcu_w;
  const uint32_t my = (l->mcu_index / j->mcus_per_row) * j->mcu_h;
  const uint8_t *cb = mcu + j->y_blocks * 64U;
  const uint8_t *cr = cb + 64U;
  const uint32_t blocks_per_row = j->mcu_w / 8U;

  if (my >= MAX_SIDE)
  {
    return;
  }
  for (uint32_t py = 0; py < j->mcu_h; py++)
  {
    const uint16_t cy = s_map_y[my + py];
    if (cy == NO_CELL)
    {
      continue;
    }
    cell_t *row = &s_cells[cy * l->grid];
    for (uint32_t px = 0; px < j->mcu_w; px++)
    {
      const uint16_t cx = s_map_x[mx + px];
      if (cx == NO_CELL)
      {
        continue;
      }
      const uint32_t block = (py / 8U) * blocks_per_row + px / 8U;
      cell_t *c = &row[cx];
      c->y += mcu[block * 64U + (py % 8U) * 8U + (px % 8U)];
      if (!j->gray)
      {
        /* Chroma: one 8x8 block covering the whole MCU */
        const uint32_t ci = (py * 8U / j->mcu_h) * 8U + (px * 8U / j->mcu_w);
        c->cb += cb[ci];
        c->cr += cr[ci];
      }
      c->n++;
    }
  }
}

/* jpeg_hw: decoded MCUs: add them to the grid */
static void picture_mcus(void *ctx, const uint8_t *data, uint32_t len)
{
  (void)ctx;
  for (uint32_t i = 0; i + s_layout.jpeg.mcu_bytes <= len; i += s_layout.jpeg.mcu_bytes)
  {
    collect_mcu(&data[i]);
    s_layout.mcu_index++;
  }
}

static inline uint8_t clamp8(int32_t v)
{
  return (uint8_t)((v < 0) ? 0 : (v > 255 ? 255 : v));
}

/* Cells -> RGB565A8 image @p index */
static void render(uint8_t index)
{
  const layout_t *l = &s_layout;
  uint16_t *rgb   = (uint16_t *)s_pixels[index];
  uint8_t  *alpha = s_pixels[index] + SIZE * SIZE * 2U;

  for (uint32_t ty = 0; ty < SIZE; ty++)
  {
    const cell_t *row = &s_cells[(ty * l->grid / SIZE) * l->grid];
    for (uint32_t tx = 0; tx < SIZE; tx++)
    {
      const cell_t *c = &row[tx * l->grid / SIZE];
      const int32_t n  = (c->n != 0U) ? (int32_t)c->n : 1;
      const int32_t y  = (int32_t)c->y / n;
      const int32_t cb = l->jpeg.gray ? 0 : (int32_t)c->cb / n - 128;
      const int32_t cr = l->jpeg.gray ? 0 : (int32_t)c->cr / n - 128;
      /* BT.601 full range, 16.16 fixed point */
      const uint8_t r = clamp8(y + ((91881 * cr) >> 16));
      const uint8_t g = clamp8(y - ((22554 * cb + 46802 * cr) >> 16));
      const uint8_t b = clamp8(y + ((116130 * cb) >> 16));
      rgb[ty * SIZE + tx] = (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
    }
  }
  memcpy(alpha, s_alpha, sizeof(s_alpha));
}

/* 4x4 ordered dither: smooth gradients without RGB565 banding */
static const uint8_t k_bayer[4][4] = { { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 } };

/* Blurred, washed-out copy of cover @p index filling the backdrop: the cover
 * averaged into BLUR_GRID x BLUR_GRID blocks, scaled up with bilinear
 * interpolation (which makes the blur), and mixed with the tint colour. */
static void render_backdrop(uint8_t index)
{
  static int32_t blocks[BLUR_GRID][BLUR_GRID][3];
  const uint16_t *cover = (const uint16_t *)s_pixels[index];

  for (uint32_t by = 0; by < BLUR_GRID; by++)
  {
    for (uint32_t bx = 0; bx < BLUR_GRID; bx++)
    {
      int32_t sum[3] = { 0, 0, 0 };
      int32_t n = 0;
      for (uint32_t y = by * SIZE / BLUR_GRID; y < (by + 1U) * SIZE / BLUR_GRID; y++)
      {
        for (uint32_t x = bx * SIZE / BLUR_GRID; x < (bx + 1U) * SIZE / BLUR_GRID; x++)
        {
          const uint16_t p = cover[y * SIZE + x];
          sum[0] += (p >> 8) & 0xF8;
          sum[1] += (p >> 3) & 0xFC;
          sum[2] += (p << 3) & 0xF8;
          n++;
        }
      }
      for (uint32_t c = 0; c < 3U; c++)
      {
        /* Already mixed with the tint, so the per-pixel loop only interpolates */
        const int32_t avg = sum[c] / (n ? n : 1);
        blocks[by][bx][c] = (avg * BACKDROP_MIX + s_tint[c] * (256 - BACKDROP_MIX)) >> 8;
      }
    }
  }

  /* The square cover is stretched to the backdrop width, centred vertically */
  uint16_t *out = s_backdrop_px[index];
  const int32_t max = (BLUR_GRID - 1) * 256;
  for (uint32_t y = 0; y < BACKDROP_H; y++)
  {
    int32_t v = (int32_t)(((y + (BACKDROP_W - BACKDROP_H) / 2U) * BLUR_GRID * 256U) / BACKDROP_W) - 128;
    v = LV_CLAMP(0, v, max);
    const int32_t y0 = v >> 8;
    const int32_t y1 = (y0 < BLUR_GRID - 1) ? y0 + 1 : y0;
    const int32_t fy = v & 255;

    int32_t row[BLUR_GRID][3];
    for (int32_t bx = 0; bx < BLUR_GRID; bx++)
    {
      for (uint32_t c = 0; c < 3U; c++)
      {
        row[bx][c] = (blocks[y0][bx][c] * (256 - fy) + blocks[y1][bx][c] * fy) >> 8;
      }
    }
    for (uint32_t x = 0; x < BACKDROP_W; x++)
    {
      int32_t u = (int32_t)((x * BLUR_GRID * 256U) / BACKDROP_W) - 128;
      u = LV_CLAMP(0, u, max);
      const int32_t x0 = u >> 8;
      const int32_t x1 = (x0 < BLUR_GRID - 1) ? x0 + 1 : x0;
      const int32_t fx = u & 255;
      const int32_t d  = k_bayer[y & 3U][x & 3U];
      int32_t rgb[3];
      for (uint32_t c = 0; c < 3U; c++)
      {
        rgb[c] = ((row[x0][c] * (256 - fx) + row[x1][c] * fx) >> 8) + (c == 1U ? d / 4 : d / 2);
      }
      const uint32_t r = (uint32_t)LV_MIN(rgb[0], 255);
      const uint32_t g = (uint32_t)LV_MIN(rgb[1], 255);
      const uint32_t b = (uint32_t)LV_MIN(rgb[2], 255);
      out[y * BACKDROP_W + x] = (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
    }
  }
}

/* Decode the picture of @p req into image @p index */
static bool decode(const request_t *req, uint8_t index)
{
  if (req->format != 1U || f_open(&s_file, req->path, FA_READ) != FR_OK)
  {
    return false;   /* PNG, none, or no file */
  }
  bool ok = false;
  if (f_lseek(&s_file, req->offset) == FR_OK)
  {
    s_in_left = req->size;
    memset(&s_layout, 0, sizeof(s_layout));
    const jpeg_hw_job_t job = { read_picture, begin_picture, picture_mcus, NULL };
    const TickType_t start = xTaskGetTickCount();
    if (jpeg_hw_decode(&job) == JPEG_HW_OK && s_layout.mcu_index > 0U)
    {
      render(index);
      /* The backdrop takes a while: let the GUI run meanwhile */
      vTaskPrioritySet(NULL, BACKDROP_PRIORITY);
      render_backdrop(index);
      vTaskPrioritySet(NULL, TASK_PRIORITY);
      ok = true;
      LOG_INFO("cover: %lux%lu JPEG in %lu ms", (unsigned long)s_layout.jpeg.width,
               (unsigned long)s_layout.jpeg.height, (unsigned long)(xTaskGetTickCount() - start));
    }
  }
  f_close(&s_file);
  return ok;
}

static void cover_task(void *arg)
{
  (void)arg;
  for (;;)
  {
    request_t req;
    xQueueReceive(s_requests, &req, portMAX_DELAY);

    const uint8_t index = s_next;
    const bool ok = decode(&req, index);
    if (ok)
    {
      s_next ^= 1U;
    }

    taskENTER_CRITICAL();
    s_done_image = index;
    s_done_state = ok ? COVER_ART_READY : COVER_ART_NONE;
    s_done_id    = req.id;
    taskEXIT_CRITICAL();
  }
}

/* ---- public API -------------------------------------------------------------- */

void cover_art_init(uint32_t backdrop_tint)
{
  build_alpha();
  s_tint[0] = (int32_t)((backdrop_tint >> 16) & 0xFFU);
  s_tint[1] = (int32_t)((backdrop_tint >> 8) & 0xFFU);
  s_tint[2] = (int32_t)(backdrop_tint & 0xFFU);
  for (uint32_t i = 0; i < 2U; i++)
  {
    lv_image_dsc_t *img = &s_images[i];
    memset(img, 0, sizeof(*img));
    img->header.magic  = LV_IMAGE_HEADER_MAGIC;
    img->header.cf     = LV_COLOR_FORMAT_RGB565A8;
    img->header.w      = SIZE;
    img->header.h      = SIZE;
    img->header.stride = SIZE * 2U;
    img->data          = s_pixels[i];
    img->data_size     = sizeof(s_pixels[i]);

    lv_image_dsc_t *bd = &s_backdrops[i];
    memset(bd, 0, sizeof(*bd));
    bd->header.magic  = LV_IMAGE_HEADER_MAGIC;
    bd->header.cf     = LV_COLOR_FORMAT_RGB565;
    bd->header.w      = BACKDROP_W;
    bd->header.h      = BACKDROP_H;
    bd->header.stride = BACKDROP_W * 2U;
    bd->data          = (const uint8_t *)s_backdrop_px[i];
    bd->data_size     = sizeof(s_backdrop_px[i]);
  }

  s_requests = xQueueCreate(1, sizeof(request_t));
  configASSERT(s_requests != NULL);
  if (xTaskCreate(cover_task, "cover", TASK_STACK_WORDS, NULL, TASK_PRIORITY, NULL) != pdPASS)
  {
    configASSERT(0);
  }
}

void cover_art_request(uint32_t id, const char *path, uint8_t format, uint32_t offset, uint32_t size)
{
  static request_t req;   /* large: not on the caller's stack */
  req.id     = id;
  req.format = format;
  req.offset = offset;
  req.size   = size;
  strncpy(req.path, path, sizeof(req.path) - 1U);
  req.path[sizeof(req.path) - 1U] = '\0';
  /* Only the latest request matters */
  xQueueOverwrite(s_requests, &req);
}

cover_art_state_t cover_art_poll(uint32_t id, const lv_image_dsc_t **image, const lv_image_dsc_t **backdrop)
{
  taskENTER_CRITICAL();
  const uint32_t          done_id = s_done_id;
  const cover_art_state_t state   = s_done_state;
  const uint8_t           index   = s_done_image;
  taskEXIT_CRITICAL();

  if (done_id != id)
  {
    return COVER_ART_PENDING;
  }
  if (state == COVER_ART_READY)
  {
    /* Same descriptor, new pixels: drop what LVGL may have cached for it */
    if (s_dropped_id != id)
    {
      lv_image_cache_drop(&s_images[index]);
      lv_image_cache_drop(&s_backdrops[index]);
      s_dropped_id = id;
    }
    *image    = &s_images[index];
    *backdrop = &s_backdrops[index];
  }
  return state;
}
