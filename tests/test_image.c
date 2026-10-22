/**
 * @file  test_image.c
 * @brief Picture decoding: PNG of every colour type and bit depth (with the
 *        streaming inflate), interlaced PNG, transparency, BMP of every pixel
 *        format, shrinking (box filter) and turning, and damaged files.
 *        Compares with the pixels FFmpeg decodes from the files in
 *        tests/fixtures/images, or with pixels the test computes.
 */
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "fakes.h"
#include "ff.h"
#include "image_decode.h"
#include "image_scale.h"

#ifndef TEST_FIXTURES_DIR
#error "TEST_FIXTURES_DIR must point to tests/fixtures"
#endif

#define BG      0x204060U        /* behind transparent pixels */
#define OUT_MAX 800U

static uint16_t     s_out[OUT_MAX * OUT_MAX];
static image_work_t s_work;
static FIL      s_file;

/* ---- helpers --------------------------------------------------------------- */

static uint8_t *read_host(const char *name, size_t *size)
{
  char path[512];
  snprintf(path, sizeof(path), "%s/images/%s", TEST_FIXTURES_DIR, name);
  FILE *f = fopen(path, "rb");
  if (f == NULL)
  {
    return NULL;
  }
  fseek(f, 0, SEEK_END);
  *size = (size_t)ftell(f);
  fseek(f, 0, SEEK_SET);
  uint8_t *data = malloc(*size);
  if (data != NULL && fread(data, 1, *size, f) != *size)
  {
    free(data);
    data = NULL;
  }
  fclose(f);
  return data;
}

static image_result_t decode_data(const uint8_t *data, size_t size, bool png, uint32_t max_w, uint32_t max_h,
                                  uint32_t *w, uint32_t *h, volatile const bool *abort)
{
  fakefs_reset();
  fakefs_add("/p", data, size);
  *w = *h = 0;
  if (f_open(&s_file, "/p", FA_READ) != FR_OK)
  {
    return IMAGE_UNSUPPORTED;
  }
  const image_target_t t = { s_out, max_w, max_h, BG, abort, &s_work };
  const image_result_t r = png ? image_decode_png(&s_file, &t, w, h) : image_decode_bmp(&s_file, &t, w, h);
  f_close(&s_file);
  return r;
}

static uint8_t blend(uint32_t c, uint32_t a, uint32_t bg)
{
  return (uint8_t)((c * a + bg * (255U - a) + 127U) / 255U);
}

/* RGBA reference pixel blended over BG */
static void ref_rgb(const uint8_t *rgba, uint8_t rgb[3])
{
  const uint32_t a = rgba[3];
  rgb[0] = blend(rgba[0], a, (BG >> 16) & 0xFFU);
  rgb[1] = blend(rgba[1], a, (BG >> 8) & 0xFFU);
  rgb[2] = blend(rgba[2], a, BG & 0xFFU);
}

/* Output pixel against an RGB colour: the RGB565 fields within @p tol */
static bool near565(uint16_t px, const uint8_t rgb[3], int tol)
{
  const int dr = (int)(px >> 11) - (rgb[0] >> 3);
  const int dg = (int)((px >> 5) & 63U) - (rgb[1] >> 2);
  const int db = (int)(px & 31U) - (rgb[2] >> 3);
  return abs(dr) <= tol && abs(dg) <= tol && abs(db) <= tol;
}

/* Decode fixture @p name unscaled and compare it with @p name's .rgba */
static void check_fixture(const char *name, bool png, int tol)
{
  char ref_name[128];
  snprintf(ref_name, sizeof(ref_name), "%.*s.rgba", (int)(strrchr(name, '.') - name), name);
  size_t size = 0, ref_size = 0;
  uint8_t *data = read_host(name, &size);
  uint8_t *ref  = read_host(ref_name, &ref_size);
  CHECK(data != NULL && ref != NULL);
  if (data == NULL || ref == NULL)
  {
    free(data);
    free(ref);
    return;
  }

  uint32_t w, h;
  const image_result_t r = decode_data(data, size, png, OUT_MAX, OUT_MAX, &w, &h, NULL);
  if (r != IMAGE_OK) fprintf(stderr, "  %s: result %d\n", name, (int)r);
  CHECK_EQ_INT(r, IMAGE_OK);
  CHECK_EQ_INT(w, 32);
  CHECK_EQ_INT(h, 24);
  int bad = 0;
  for (uint32_t i = 0; i < 32U * 24U && w == 32U && h == 24U; i++)
  {
    uint8_t rgb[3];
    ref_rgb(&ref[i * 4U], rgb);
    if (!near565(s_out[i], rgb, tol) && bad++ == 0)
    {
      fprintf(stderr, "  %s: pixel %u,%u is %04x, expected %02x%02x%02x\n", name, i % 32U, i / 32U, s_out[i],
              rgb[0], rgb[1], rgb[2]);
    }
  }
  CHECK_EQ_INT(bad, 0);
  free(data);
  free(ref);
}

/* ---- PNG builder (stored deflate blocks, IDAT split into small chunks) ----- */

typedef struct
{
  uint8_t data[200000];
  size_t  len;
} buf_t;

static buf_t s_png, s_raw;

static void put8(buf_t *b, uint32_t v)  { b->data[b->len++] = (uint8_t)v; }
static void put32(buf_t *b, uint32_t v) { put8(b, v >> 24); put8(b, v >> 16); put8(b, v >> 8); put8(b, v); }
static void put_bytes(buf_t *b, const void *d, size_t n) { if (n != 0U) { memcpy(&b->data[b->len], d, n); b->len += n; } }   /* d may be NULL when n is 0 */

static void chunk(buf_t *b, const char *type, const uint8_t *d, size_t n)
{
  put32(b, (uint32_t)n);
  put_bytes(b, type, 4);
  put_bytes(b, d, n);
  put32(b, 0);   /* CRC: not checked */
}

/* PNG with the raw (filtered) rows in s_raw, as zlib data in stored blocks of
 * @p block bytes, split into IDAT chunks of @p idat bytes */
static void build_png(uint32_t w, uint32_t h, uint32_t depth, uint32_t ctype, bool interlaced,
                      const uint8_t *trns, size_t trns_len, size_t block, size_t idat)
{
  static buf_t z;
  z.len = 0;
  put8(&z, 0x78);
  put8(&z, 0x01);
  for (size_t off = 0; off < s_raw.len || off == 0; off += block)
  {
    const size_t n = (s_raw.len - off < block) ? s_raw.len - off : block;
    put8(&z, (off + n >= s_raw.len) ? 1U : 0U);
    put8(&z, n & 0xFFU); put8(&z, n >> 8);
    put8(&z, ~n & 0xFFU); put8(&z, (~n >> 8) & 0xFFU);
    put_bytes(&z, &s_raw.data[off], n);
    if (s_raw.len == 0U) break;
  }
  put32(&z, 0);  /* Adler-32: not checked */

  s_png.len = 0;
  static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
  put_bytes(&s_png, sig, 8);
  const uint8_t ihdr[13] = { w >> 24, w >> 16, w >> 8, w, h >> 24, h >> 16, h >> 8, h,
                             (uint8_t)depth, (uint8_t)ctype, 0, 0, interlaced ? 1 : 0 };
  chunk(&s_png, "IHDR", ihdr, 13);
  if (trns != NULL)
  {
    chunk(&s_png, "tRNS", trns, trns_len);
  }
  for (size_t off = 0; off < z.len; off += idat)
  {
    chunk(&s_png, "IDAT", &z.data[off], (z.len - off < idat) ? z.len - off : idat);
  }
  chunk(&s_png, "IEND", NULL, 0);
}

static const uint8_t k_adam7[7][4] = {
  { 0, 0, 8, 8 }, { 4, 0, 8, 8 }, { 0, 4, 4, 8 }, { 2, 0, 4, 4 }, { 0, 2, 2, 4 }, { 1, 0, 2, 2 }, { 0, 1, 1, 2 },
};

static void pattern_rgb(uint32_t x, uint32_t y, uint8_t rgb[3])
{
  rgb[0] = (uint8_t)(x * 20U);
  rgb[1] = (uint8_t)(y * 20U);
  rgb[2] = (uint8_t)((x + y) * 10U);
}

/* Rows (filter 0) of a w x h picture, interlaced or not: RGB 8 bits or grey of @p gray_depth bits */
static void build_rows(uint32_t w, uint32_t h, bool interlaced, uint32_t gray_depth)
{
  s_raw.len = 0;
  for (uint32_t p = 0; p < (interlaced ? 7U : 1U); p++)
  {
    const uint32_t sx = interlaced ? k_adam7[p][0] : 0U, sy = interlaced ? k_adam7[p][1] : 0U;
    const uint32_t dx = interlaced ? k_adam7[p][2] : 1U, dy = interlaced ? k_adam7[p][3] : 1U;
    for (uint32_t y = sy; y < h; y += dy)
    {
      if (sx >= w)
      {
        break;
      }
      put8(&s_raw, 0);
      uint32_t bits = 0, acc = 0;
      for (uint32_t x = sx; x < w; x += dx)
      {
        if (gray_depth == 0U)
        {
          uint8_t rgb[3];
          pattern_rgb(x, y, rgb);
          put_bytes(&s_raw, rgb, 3);
        }
        else
        {
          acc = (acc << gray_depth) | ((x + y) % (1U << gray_depth));
          bits += gray_depth;
          if (bits == 8U)
          {
            put8(&s_raw, acc);
            bits = acc = 0;
          }
        }
      }
      if (bits != 0U)
      {
        put8(&s_raw, acc << (8U - bits));
      }
    }
  }
}

/* ---- tests ----------------------------------------------------------------- */

static void test_kind(void)
{
  static const uint8_t jpg[8] = { 0xFF, 0xD8, 0xFF, 0xE0, 0, 0, 0, 0 };
  static const uint8_t png[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
  CHECK_EQ_INT(image_kind(jpg, 8), IMAGE_KIND_JPEG);
  CHECK_EQ_INT(image_kind(png, 8), IMAGE_KIND_PNG);
  CHECK_EQ_INT(image_kind((const uint8_t *)"GIF89a..", 8), IMAGE_KIND_GIF);
  CHECK_EQ_INT(image_kind((const uint8_t *)"BM......", 8), IMAGE_KIND_BMP);
  CHECK_EQ_INT(image_kind((const uint8_t *)"hello!!!", 8), IMAGE_KIND_UNKNOWN);
}

static void test_png_fixtures(void)
{
  static const char *const names[] = { "png_rgb24.png", "png_rgba.png", "png_gray.png", "png_ya8.png",
                                       "png_gray16be.png", "png_rgb48be.png", "png_rgba64be.png",
                                       "png_pal8.png", "png_monob.png" };
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
  {
    check_fixture(names[i], true, 1);
  }
}

/* 256 x 192 with computed pixels: more output than the inflate window */
static void test_png_large(void)
{
  size_t size = 0;
  uint8_t *data = read_host("png_pattern.png", &size);
  CHECK(data != NULL);
  if (data == NULL) return;
  uint32_t w, h;
  CHECK_EQ_INT(decode_data(data, size, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  CHECK_EQ_INT(w, 256);
  CHECK_EQ_INT(h, 192);
  int bad = 0;
  for (uint32_t y = 0; y < 192U; y++)
  {
    for (uint32_t x = 0; x < 256U; x++)
    {
      const uint8_t rgb[3] = { (uint8_t)((x * 7U + y * 3U) & 255U), (uint8_t)((x * x + y) & 255U), (uint8_t)((x * y) & 255U) };
      bad += !near565(s_out[y * 256U + x], rgb, 0);
    }
  }
  CHECK_EQ_INT(bad, 0);
  free(data);
}

static void check_pattern(uint32_t w, uint32_t h, uint32_t gray_depth)
{
  int bad = 0;
  for (uint32_t y = 0; y < h; y++)
  {
    for (uint32_t x = 0; x < w; x++)
    {
      uint8_t rgb[3];
      if (gray_depth == 0U)
      {
        pattern_rgb(x, y, rgb);
      }
      else
      {
        const uint32_t max = (1U << gray_depth) - 1U;
        rgb[0] = rgb[1] = rgb[2] = (uint8_t)(((x + y) % (max + 1U)) * 255U / max);
      }
      bad += !near565(s_out[y * w + x], rgb, 0);
    }
  }
  CHECK_EQ_INT(bad, 0);
}

static void test_png_interlaced(void)
{
  uint32_t w, h;
  /* Odd sizes: some passes are narrow or empty */
  build_rows(13, 11, true, 0);
  build_png(13, 11, 8, 2, true, NULL, 0, 1000, 37);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  CHECK_EQ_INT(w, 13);
  check_pattern(13, 11, 0);

  /* 2-bit grey: pass rows end in part of a byte */
  build_rows(13, 11, true, 2);
  build_png(13, 11, 2, 0, true, NULL, 0, 1000, 37);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  check_pattern(13, 11, 2);

  /* Too small for some passes altogether */
  build_rows(1, 1, true, 0);
  build_png(1, 1, 8, 2, true, NULL, 0, 1000, 37);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  check_pattern(1, 1, 0);
}

/* Stored blocks and IDAT chunks of a few bytes: data split everywhere */
static void test_png_split(void)
{
  uint32_t w, h;
  build_rows(40, 30, false, 0);
  build_png(40, 30, 8, 2, false, NULL, 0, 333, 7);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  check_pattern(40, 30, 0);

  build_rows(9, 5, false, 4);
  build_png(9, 5, 4, 0, false, NULL, 0, 2, 1);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  check_pattern(9, 5, 4);
}

/* tRNS colour key of an RGB picture: that colour shows the background */
static void test_png_colour_key(void)
{
  uint32_t w, h;
  uint8_t key_rgb[3];
  pattern_rgb(2, 1, key_rgb);
  const uint8_t trns[6] = { 0, key_rgb[0], 0, key_rgb[1], 0, key_rgb[2] };
  build_rows(4, 3, false, 0);
  build_png(4, 3, 8, 2, false, trns, 6, 1000, 1000);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_OK);
  const uint8_t bg[3] = { (BG >> 16) & 0xFFU, (BG >> 8) & 0xFFU, BG & 0xFFU };
  CHECK(near565(s_out[1 * 4 + 2], bg, 0));
  uint8_t other[3];
  pattern_rgb(1, 1, other);
  CHECK(near565(s_out[1 * 4 + 1], other, 0));
}

static void test_png_errors(void)
{
  uint32_t w, h;
  size_t size = 0;
  uint8_t *data = read_host("png_pattern.png", &size);
  CHECK(data != NULL);
  if (data == NULL) return;

  /* Cut short: what was decoded is kept, the rest is the fill */
  CHECK_EQ_INT(decode_data(data, size / 2U, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_CORRUPT);
  CHECK_EQ_INT(w, 256);
  const uint8_t first[3] = { 0, 0, 0 };
  CHECK(near565(s_out[0], first, 0));

  /* Stopped */
  volatile bool abort = true;
  CHECK_EQ_INT(decode_data(data, size, true, OUT_MAX, OUT_MAX, &w, &h, &abort), IMAGE_ABORTED);

  /* Not zlib data (compression method 0 in the first IDAT byte) */
  for (size_t i = 8; i + 4U < size; i++)
  {
    if (memcmp(&data[i], "IDAT", 4) == 0)
    {
      data[i + 4U] = 0;
      break;
    }
  }
  CHECK_EQ_INT(decode_data(data, size, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_CORRUPT);
  free(data);

  /* Not a PNG; too large; unknown colour type */
  CHECK_EQ_INT(decode_data((const uint8_t *)"not a picture at all", 20, true, OUT_MAX, OUT_MAX, &w, &h, NULL),
               IMAGE_UNSUPPORTED);
  s_raw.len = 0;
  build_png(SCALE_MAX_SRC + 1U, 10, 8, 2, false, NULL, 0, 1000, 1000);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_TOO_LARGE);
  build_png(10, 10, 8, 5, false, NULL, 0, 1000, 1000);
  CHECK_EQ_INT(decode_data(s_png.data, s_png.len, true, OUT_MAX, OUT_MAX, &w, &h, NULL), IMAGE_UNSUPPORTED);
}

static void test_bmp_fixtures(void)
{
  static const char *const names[] = { "bmp_bgr24.bmp", "bmp_bgra.bmp", "bmp_rgb565le.bmp", "bmp_rgb555le.bmp",
                                       "bmp_pal8.bmp", "bmp_monob.bmp" };
  for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
  {
    check_fixture(names[i], false, 1);
  }
}

/* 4 bits per pixel, top-down (negative height), odd width (padded rows) */
static void test_bmp_4bit_top_down(void)
{
  static buf_t b;
  const uint32_t w = 5, h = 3, stride = 4;   /* 5 pixels = 3 bytes, padded to 4 */
  const uint32_t offset = 14 + 40 + 16 * 4;
  b.len = 0;
  uint8_t hdr[54] = { 'B', 'M' };
  hdr[10] = (uint8_t)offset;
  hdr[14] = 40;
  hdr[18] = (uint8_t)w;
  const int32_t neg = -(int32_t)h;
  memcpy(&hdr[22], &neg, 4);
  hdr[26] = 1;
  hdr[28] = 4;
  hdr[46] = 16;
  put_bytes(&b, hdr, 54);
  for (uint32_t i = 0; i < 16U; i++)
  {
    const uint8_t bgrx[4] = { (uint8_t)(i * 16U), (uint8_t)(255U - i * 16U), (uint8_t)(i * 8U), 0 };
    put_bytes(&b, bgrx, 4);
  }
  for (uint32_t y = 0; y < h; y++)
  {
    uint8_t row[4] = { 0 };
    for (uint32_t x = 0; x < w; x++)
    {
      const uint32_t v = (x + y * 5U) & 15U;
      row[x / 2U] |= (uint8_t)(v << ((x & 1U) ? 0 : 4));
    }
    put_bytes(&b, row, stride);
  }

  uint32_t ow, oh;
  CHECK_EQ_INT(decode_data(b.data, b.len, false, OUT_MAX, OUT_MAX, &ow, &oh, NULL), IMAGE_OK);
  CHECK_EQ_INT(ow, 5);
  CHECK_EQ_INT(oh, 3);
  int bad = 0;
  for (uint32_t y = 0; y < h; y++)
  {
    for (uint32_t x = 0; x < w; x++)
    {
      const uint32_t v = (x + y * 5U) & 15U;
      const uint8_t rgb[3] = { (uint8_t)(v * 8U), (uint8_t)(255U - v * 16U), (uint8_t)(v * 16U) };
      bad += !near565(s_out[y * w + x], rgb, 0);
    }
  }
  CHECK_EQ_INT(bad, 0);

  /* RLE compression is not supported */
  b.data[30] = 2;
  CHECK_EQ_INT(decode_data(b.data, b.len, false, OUT_MAX, OUT_MAX, &ow, &oh, NULL), IMAGE_UNSUPPORTED);
}

/* Shrinking: each output pixel is the rounded mean of the source pixels mapped to it */
static void check_shrunk(uint32_t max_w, uint32_t max_h, uint32_t exp_w, uint32_t exp_h)
{
  size_t size = 0, ref_size = 0;
  uint8_t *data = read_host("png_rgb24.png", &size);
  uint8_t *ref  = read_host("png_rgb24.rgba", &ref_size);
  CHECK(data != NULL && ref != NULL);
  if (data == NULL || ref == NULL) { free(data); free(ref); return; }

  uint32_t w, h;
  CHECK_EQ_INT(decode_data(data, size, true, max_w, max_h, &w, &h, NULL), IMAGE_OK);
  CHECK_EQ_INT(w, exp_w);
  CHECK_EQ_INT(h, exp_h);
  int bad = 0;
  for (uint32_t ty = 0; ty < h && w == exp_w && h == exp_h; ty++)
  {
    for (uint32_t tx = 0; tx < w; tx++)
    {
      uint32_t sum[3] = { 0, 0, 0 }, n = 0;
      for (uint32_t y = 0; y < 24U; y++)
      {
        for (uint32_t x = 0; x < 32U; x++)
        {
          if (x * w / 32U == tx && y * h / 24U == ty)
          {
            for (int c = 0; c < 3; c++) sum[c] += ref[(y * 32U + x) * 4U + (uint32_t)c];
            n++;
          }
        }
      }
      const uint8_t rgb[3] = { (uint8_t)((sum[0] + n / 2U) / n), (uint8_t)((sum[1] + n / 2U) / n),
                               (uint8_t)((sum[2] + n / 2U) / n) };
      bad += !near565(s_out[ty * w + tx], rgb, 0);
    }
  }
  CHECK_EQ_INT(bad, 0);
  free(data);
  free(ref);
}

static void test_shrink(void)
{
  check_shrunk(16, 12, 16, 12);    /* 2 x 2 boxes */
  check_shrunk(10, 100, 10, 8);    /* uneven boxes; height follows the shape (7.5 -> 8) */
  check_shrunk(800, 6, 8, 6);      /* limited by the height */
  check_shrunk(1, 1, 1, 1);
}

/* Turning: a 3 x 2 picture in every orientation */
static void test_transform(void)
{
  scale_t *s = &s_work.scale;
  /* Source pixel (x, y) has red = 10 * (x + 1) * 8, green = (y + 1) * 64 */
  uint8_t rows[2][9];
  for (uint32_t y = 0; y < 2U; y++)
  {
    for (uint32_t x = 0; x < 3U; x++)
    {
      rows[y][x * 3U] = (uint8_t)((x + 1U) * 80U);
      rows[y][x * 3U + 1U] = (uint8_t)((y + 1U) * 64U);
      rows[y][x * 3U + 2U] = 0;
    }
  }
  for (uint32_t t = 0; t < 8U; t++)
  {
    CHECK(scale_begin(s, 3, 2, t, SCALE_RGB, s_out, 100, 100));
    scale_row(s, 0, rows[0]);
    scale_row(s, 1, rows[1]);
    scale_end(s);
    const bool tr = (t & SCALE_TRANSPOSE) != 0U;
    CHECK_EQ_INT(s->out_w, tr ? 2 : 3);
    CHECK_EQ_INT(s->out_h, tr ? 3 : 2);
    int bad = 0;
    for (uint32_t y = 0; y < 2U; y++)
    {
      for (uint32_t x = 0; x < 3U; x++)
      {
        uint32_t ox = tr ? y : x, oy = tr ? x : y;
        if (t & SCALE_FLIP_X) ox = s->out_w - 1U - ox;
        if (t & SCALE_FLIP_Y) oy = s->out_h - 1U - oy;
        bad += !near565(s_out[oy * s->out_w + ox], &rows[y][x * 3U], 0);
      }
    }
    if (bad) fprintf(stderr, "  transform %u wrong\n", t);
    CHECK_EQ_INT(bad, 0);
  }
  /* Turned before fitting: a tall picture turned to lie across */
  CHECK(scale_begin(s, 100, 400, SCALE_TRANSPOSE, SCALE_RGB, s_out, 200, 200));
  CHECK_EQ_INT(s->out_w, 200);
  CHECK_EQ_INT(s->out_h, 50);
}

/* JPEG YCbCr averaging: grey and pure colours convert as expected */
static void test_ycbcr(void)
{
  scale_t *s = &s_work.scale;
  const uint8_t px[6] = { 128, 128, 128,  76, 85, 255 };   /* grey; red (Y 76, Cb 85, Cr 255) */
  CHECK(scale_begin(s, 2, 1, 0, SCALE_YCBCR, s_out, 100, 100));
  scale_row(s, 0, px);
  const uint8_t grey[3] = { 128, 128, 128 };
  const uint8_t red[3]  = { 254, 0, 0 };
  CHECK(near565(s_out[0], grey, 0));
  CHECK(near565(s_out[1], red, 1));
}

int main(void)
{
  RUN(test_kind);
  RUN(test_png_fixtures);
  RUN(test_png_large);
  RUN(test_png_interlaced);
  RUN(test_png_split);
  RUN(test_png_colour_key);
  RUN(test_png_errors);
  RUN(test_bmp_fixtures);
  RUN(test_bmp_4bit_top_down);
  RUN(test_shrink);
  RUN(test_transform);
  RUN(test_ycbcr);
  return g_check_failures != 0;
}
