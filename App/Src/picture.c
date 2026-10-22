/**
 * @file  picture.c
 * @brief Background decoding of pictures: JPEG with the hardware codec, PNG
 *        and BMP in software, all shrunk into one RGB565 buffer.
 *
 * Shrinking a JPEG: the codec delivers YCbCr MCUs, whose samples are summed
 * per output pixel as they are (the conversion to RGB is linear, so the average
 * of the converted pixels is the converted average) and converted once per
 * output pixel. Four samples falling on the same output pixel are summed by one
 * SIMD instruction (USAD8). A picture that fits unshrunk and unturned goes
 * straight through the SIMD YCbCr converter of the video player instead.
 */
#include "picture.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "ff.h"
#include "image_decode.h"
#include "jpeg_hw.h"
#include "log.h"
#include "simd8.h"
#include "theme.h"
#include "yuv2rgb.h"

#define TASK_STACK_WORDS  768
#define TASK_PRIORITY     1            /* the GUI's: it keeps drawing (time slices) while a PNG decodes */
#define PATH_LEN          512
#define EXIF_READ         4096U        /* bytes of the EXIF block searched for the orientation */
#define IN_CHUNK          4096U

/* Memory lent by the Files app (picture_attach) */
typedef struct
{
  uint16_t     out[PICTURE_MAX_W * PICTURE_MAX_H] __attribute__((aligned(32)));
  image_work_t work __attribute__((aligned(32)));
  uint8_t      exif[EXIF_READ];
} picture_mem_t;

static picture_mem_t *s_mem;            /* NULL: none; changed only while s_busy is held */
static SemaphoreHandle_t s_busy;        /* held by the task while it decodes */
static lv_image_dsc_t s_dsc;
static FIL      s_file;

static SemaphoreHandle_t s_lock;       /* request and result */
static SemaphoreHandle_t s_wake;
static char     s_req_path[PATH_LEN];
static uint32_t s_req_id;              /* last request */
static volatile bool s_abort;          /* stop the decoding under way */

static uint32_t s_done_id;             /* request the result is for */
static picture_result_t s_result;

/* ---- JPEG ------------------------------------------------------------------ */

typedef struct
{
  scale_t         *scale;      /* NULL: straight conversion (yuv) */
  yuv2rgb_t        yuv;
  jpeg_hw_layout_t layout;
  uint32_t         transform;
  uint32_t         mcu_index;
  uint32_t         width, height;
  bool             too_large;
} jpeg_job_t;

static jpeg_job_t s_jpeg;

static uint16_t rd16(const uint8_t *p, bool le) { return le ? (uint16_t)(p[0] | (p[1] << 8)) : (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t rd32(const uint8_t *p, bool le)
{
  return le ? ((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24))
            : (((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}

/* EXIF orientation (1..8) in an APP1 block of @p len bytes; 1 if none */
static uint32_t exif_orientation(const uint8_t *d, uint32_t len)
{
  if (len < 14U || memcmp(d, "Exif\0\0", 6) != 0)
  {
    return 1;
  }
  const uint8_t *t = d + 6;           /* TIFF header */
  const uint32_t tlen = len - 6U;
  const bool le = t[0] == 'I' && t[1] == 'I';
  if (!le && !(t[0] == 'M' && t[1] == 'M'))
  {
    return 1;
  }
  const uint32_t ifd = rd32(&t[4], le);
  if (ifd + 2U > tlen)
  {
    return 1;
  }
  const uint32_t count = rd16(&t[ifd], le);
  for (uint32_t i = 0; i < count && ifd + 2U + (i + 1U) * 12U <= tlen; i++)
  {
    const uint8_t *e = &t[ifd + 2U + i * 12U];
    if (rd16(e, le) == 0x0112U)
    {
      const uint32_t o = rd16(&e[8], le);
      return (o >= 1U && o <= 8U) ? o : 1U;
    }
  }
  return 1;
}

/* Walk the JPEG's segments up to its frame header: baseline? orientation? */
static const char *jpeg_scan(uint32_t *orientation)
{
  uint8_t m[4];
  UINT got;
  *orientation = 1;
  if (f_lseek(&s_file, 2) != FR_OK)
  {
    return "The file cannot be read.";
  }
  for (;;)
  {
    if (f_read(&s_file, m, 4, &got) != FR_OK || got != 4U || m[0] != 0xFFU)
    {
      return "This JPEG file is damaged.";
    }
    const uint8_t marker = m[1];
    const uint32_t len = ((uint32_t)m[2] << 8) | m[3];
    if (len < 2U)
    {
      return "This JPEG file is damaged.";
    }
    if (marker == 0xC0U || marker == 0xC1U)
    {
      return NULL;   /* baseline / extended sequential */
    }
    if (marker == 0xC2U || marker == 0xC6U || marker == 0xCAU || marker == 0xCEU)
    {
      return "Progressive JPEG is not supported.";
    }
    if ((marker >= 0xC3U && marker <= 0xCFU) && marker != 0xC4U && marker != 0xC8U && marker != 0xCCU)
    {
      return "This kind of JPEG is not supported.";
    }
    if (marker == 0xDAU)
    {
      return "This JPEG file is damaged.";
    }
    const FSIZE_t next = f_tell(&s_file) + len - 2U;
    if (marker == 0xE1U && *orientation == 1U)
    {
      const uint32_t n = (len - 2U < EXIF_READ) ? len - 2U : EXIF_READ;
      if (f_read(&s_file, s_mem->exif, n, &got) == FR_OK && got == n)
      {
        *orientation = exif_orientation(s_mem->exif, n);
      }
    }
    if (f_lseek(&s_file, next) != FR_OK)
    {
      return "This JPEG file is damaged.";
    }
  }
}

/* EXIF orientation -> how the scaler turns the picture */
static uint32_t orientation_transform(uint32_t o)
{
  static const uint8_t k_map[9] = {
    0, 0, SCALE_FLIP_X, SCALE_FLIP_X | SCALE_FLIP_Y, SCALE_FLIP_Y, SCALE_TRANSPOSE,
    SCALE_TRANSPOSE | SCALE_FLIP_X, SCALE_TRANSPOSE | SCALE_FLIP_X | SCALE_FLIP_Y, SCALE_TRANSPOSE | SCALE_FLIP_Y,
  };
  return k_map[o];
}

/* jpeg_hw: the file's data; after an abort too (the codec must see the end) */
static uint32_t jpeg_read(void *ctx, uint8_t *buf, uint32_t max)
{
  (void)ctx;
  UINT got = 0;
  if (f_read(&s_file, buf, max, &got) != FR_OK)
  {
    return 0;
  }
  return got;
}

static bool jpeg_begin(void *ctx, const jpeg_hw_layout_t *l)
{
  jpeg_job_t *j = ctx;
  j->layout    = *l;
  j->width     = l->width;
  j->height    = l->height;
  j->mcu_index = 0;
  if (j->transform == 0U && l->width <= PICTURE_MAX_W && l->height <= PICTURE_MAX_H)
  {
    j->scale = NULL;
    memset(&j->yuv, 0, sizeof(j->yuv));
    j->yuv.dst    = s_mem->out;
    j->yuv.dst_w  = l->width;
    j->yuv.dst_h  = l->height;
    j->yuv.layout = *l;
    return true;
  }
  j->scale = &s_mem->work.scale;
  if (!scale_begin(j->scale, l->width, l->height, j->transform, SCALE_YCBCR, s_mem->out, PICTURE_MAX_W, PICTURE_MAX_H))
  {
    j->too_large = true;
    return false;
  }
  return true;
}

/* Add the samples of one MCU to the output pixels they fall on */
static void jpeg_collect(jpeg_job_t *j, const uint8_t *mcu)
{
  scale_t *s = j->scale;
  const jpeg_hw_layout_t *l = &j->layout;
  const uint32_t mx = (j->mcu_index % l->mcus_per_row) * l->mcu_w;
  const uint32_t my = (j->mcu_index / l->mcus_per_row) * l->mcu_h;
  const uint8_t *cb = mcu + l->y_blocks * 64U;
  const uint8_t *cr = cb + 64U;
  const uint32_t blocks_per_row = l->mcu_w / 8U;
  const uint32_t chroma_step = l->mcu_w / 8U;        /* pixels per chroma sample across: 1 or 2 */

  for (uint32_t py = 0; py < l->mcu_h && my + py < j->height; py++)
  {
    uint32_t *a = scale_acc(s, s->map_y[my + py]);
    const uint8_t *cb_row = &cb[(py * 8U / l->mcu_h) * 8U];
    const uint8_t *cr_row = &cr[(py * 8U / l->mcu_h) * 8U];
    for (uint32_t bx = 0; bx < blocks_per_row; bx++)
    {
      const uint8_t *luma = &mcu[((py / 8U) * blocks_per_row + bx) * 64U + (py % 8U) * 8U];
      for (uint32_t k = 0; k < 8U; k += 4U)
      {
        const uint32_t px = bx * 8U + k;              /* in the MCU */
        const uint32_t x  = mx + px;
        if (x >= j->width)
        {
          break;
        }
        if (x + 3U < j->width && s->map_x[x] == s->map_x[x + 3U])
        {
          /* Four pixels on one output pixel: one SIMD sum each */
          uint32_t *c = &a[s->map_x[x] * 3U];
          c[0] += sum8x4(load32(&luma[k]));
          if (l->gray)
          {
            c[1] += 4U * 128U;
            c[2] += 4U * 128U;
          }
          else if (chroma_step == 1U)
          {
            c[1] += sum8x4(load32(&cb_row[px]));
            c[2] += sum8x4(load32(&cr_row[px]));
          }
          else
          {
            const uint32_t ci = px / 2U;
            c[1] += 2U * ((uint32_t)cb_row[ci] + cb_row[ci + 1U]);
            c[2] += 2U * ((uint32_t)cr_row[ci] + cr_row[ci + 1U]);
          }
          continue;
        }
        for (uint32_t i = 0; i < 4U && x + i < j->width; i++)
        {
          uint32_t *c = &a[s->map_x[x + i] * 3U];
          c[0] += luma[k + i];
          if (l->gray)
          {
            c[1] += 128U;
            c[2] += 128U;
          }
          else
          {
            const uint32_t ci = (px + i) / chroma_step;
            c[1] += cb_row[ci];
            c[2] += cr_row[ci];
          }
        }
      }
    }
  }
}

static void jpeg_mcus(void *ctx, const uint8_t *data, uint32_t len)
{
  jpeg_job_t *j = ctx;
  if (s_abort)
  {
    return;   /* the codec runs to the end of the file; nothing more to do */
  }
  if (j->scale == NULL)
  {
    yuv2rgb_mcus(&j->yuv, data, len);
    return;
  }
  const jpeg_hw_layout_t *l = &j->layout;
  for (uint32_t i = 0; i + l->mcu_bytes <= len; i += l->mcu_bytes)
  {
    jpeg_collect(j, &data[i]);
    if (++j->mcu_index % l->mcus_per_row == 0U)
    {
      /* An MCU row is complete: its output rows can be written */
      const uint32_t end = (j->mcu_index / l->mcus_per_row) * l->mcu_h;
      scale_rows_done(j->scale, ((end < j->height) ? end : j->height) - 1U);
    }
  }
}

static const char *decode_jpeg(uint32_t *out_w, uint32_t *out_h, uint32_t *src_w, uint32_t *src_h)
{
  uint32_t orientation;
  const char *err = jpeg_scan(&orientation);
  if (err != NULL)
  {
    return err;
  }
  if (f_lseek(&s_file, 0) != FR_OK)
  {
    return "The file cannot be read.";
  }

  jpeg_job_t *j = &s_jpeg;
  memset(j, 0, sizeof(*j));
  j->transform = orientation_transform(orientation);
  const jpeg_hw_job_t job = { jpeg_read, jpeg_begin, jpeg_mcus, j };
  const jpeg_hw_result_t res = jpeg_hw_decode(&job);
  *src_w = j->width;
  *src_h = j->height;
  if (res == JPEG_HW_REJECTED)
  {
    return j->too_large || j->width > 8192U || j->height > 8192U ? "The picture is too large (8192 x 8192 at most)."
                                                                 : "This kind of JPEG is not supported.";
  }
  if (j->scale != NULL)
  {
    scale_end(j->scale);
    *out_w = j->scale->out_w;
    *out_h = j->scale->out_h;
  }
  else
  {
    *out_w = j->width;
    *out_h = j->height;
  }
  return (res == JPEG_HW_OK) ? NULL : "This JPEG file is damaged.";
}

/* ---- task ------------------------------------------------------------------ */

static const char *image_message(image_result_t r)
{
  switch (r)
  {
    case IMAGE_UNSUPPORTED: return "This kind of picture is not supported.";
    case IMAGE_TOO_LARGE:   return "The picture is too large.";
    case IMAGE_CORRUPT:     return "The picture is damaged.";
    default:                return NULL;
  }
}

/* Decode @p path; the message is NULL on success */
static const char *decode(const char *path, picture_result_t *r)
{
  uint32_t out_w = 0, out_h = 0;
  const char *err = NULL;
  r->width = r->height = 0;

  if (f_open(&s_file, path, FA_READ) != FR_OK)
  {
    return "The file cannot be opened.";
  }
  uint8_t head[8] = { 0 };
  UINT got = 0;
  (void)f_read(&s_file, head, sizeof(head), &got);
  (void)f_lseek(&s_file, 0);

  const image_target_t target = { s_mem->out, PICTURE_MAX_W, PICTURE_MAX_H, THEME_BG_RGB, &s_abort, &s_mem->work };
  switch (image_kind(head, got))
  {
    case IMAGE_KIND_JPEG:
      err = decode_jpeg(&out_w, &out_h, &r->width, &r->height);
      break;
    case IMAGE_KIND_PNG:
    case IMAGE_KIND_BMP:
    {
      const image_result_t res = (image_kind(head, got) == IMAGE_KIND_PNG)
                                   ? image_decode_png(&s_file, &target, &out_w, &out_h)
                                   : image_decode_bmp(&s_file, &target, &out_w, &out_h);
      r->width  = s_mem->work.scale.src_w;
      r->height = s_mem->work.scale.src_h;
      err = image_message(res);
      break;
    }
    default:
      err = "This kind of picture is not supported.";
      break;
  }
  f_close(&s_file);

  if (out_w != 0U && out_h != 0U)
  {
    memset(&s_dsc, 0, sizeof(s_dsc));
    s_dsc.header.magic  = LV_IMAGE_HEADER_MAGIC;
    s_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    s_dsc.header.w      = out_w;
    s_dsc.header.h      = out_h;
    s_dsc.header.stride = out_w * 2U;
    s_dsc.data_size     = out_w * out_h * 2U;
    s_dsc.data          = (const uint8_t *)s_mem->out;
    r->image = &s_dsc;
  }
  return err;
}

static void picture_task(void *arg)
{
  (void)arg;
  static char path[PATH_LEN];
  for (;;)
  {
    xSemaphoreTake(s_wake, portMAX_DELAY);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const uint32_t id = s_req_id;
    snprintf(path, sizeof(path), "%s", s_req_path);
    s_abort = false;
    xSemaphoreGive(s_lock);
    if (path[0] == '\0')
    {
      continue;   /* cancelled */
    }

    picture_result_t r = { PICTURE_PENDING, NULL, 0, 0, NULL };
    const TickType_t t0 = xTaskGetTickCount();
    xSemaphoreTake(s_busy, portMAX_DELAY);
    if (s_mem == NULL)
    {
      xSemaphoreGive(s_busy);
      continue;   /* no memory lent (any more) */
    }
    r.message = decode(path, &r);
    xSemaphoreGive(s_busy);
    r.state = (r.message == NULL) ? PICTURE_READY : PICTURE_FAILED;
    LOG_INFO("picture: %s %lux%lu in %lu ms%s%s", path, (unsigned long)r.width, (unsigned long)r.height,
             (unsigned long)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS), r.message ? ": " : "",
             r.message ? r.message : "");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (id == s_req_id && !s_abort)
    {
      s_result  = r;
      s_done_id = id;
    }
    xSemaphoreGive(s_lock);
  }
}

void picture_init(void)
{
  s_lock = xSemaphoreCreateMutex();
  s_busy = xSemaphoreCreateMutex();
  s_wake = xSemaphoreCreateBinary();
  if (s_lock == NULL || s_busy == NULL || s_wake == NULL ||
      xTaskCreate(picture_task, "picture", TASK_STACK_WORDS, NULL, TASK_PRIORITY, NULL) != pdPASS)
  {
    LOG_ERROR("picture: task not created");
  }
}

static void set_request(const char *path)
{
  xSemaphoreTake(s_lock, portMAX_DELAY);
  snprintf(s_req_path, sizeof(s_req_path), "%s", path);
  s_req_id++;
  s_abort = true;    /* the decoding under way, if any, stops */
  xSemaphoreGive(s_lock);
  xSemaphoreGive(s_wake);
}

void picture_request(const char *path) { set_request(path); }
void picture_cancel(void)              { set_request(""); }

size_t picture_mem_size(void) { return sizeof(picture_mem_t); }

void picture_attach(void *mem)
{
  xSemaphoreTake(s_busy, portMAX_DELAY);
  s_mem = mem;
  xSemaphoreGive(s_busy);
}

void picture_detach(void)
{
  set_request("");
  xSemaphoreTake(s_busy, portMAX_DELAY);   /* waits for a decoding to stop */
  s_mem = NULL;
  xSemaphoreGive(s_busy);
}

picture_result_t picture_poll(void)
{
  picture_result_t r = { PICTURE_IDLE, NULL, 0, 0, NULL };
  xSemaphoreTake(s_lock, portMAX_DELAY);
  if (s_req_path[0] != '\0')
  {
    if (s_done_id == s_req_id)
    {
      r = s_result;
    }
    else
    {
      r.state = PICTURE_PENDING;
    }
  }
  xSemaphoreGive(s_lock);
  return r;
}
