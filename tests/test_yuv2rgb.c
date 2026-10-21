/**
 * @file  test_yuv2rgb.c
 * @brief YCbCr MCUs to RGB565: every layout (4:2:0 fast path and the generic
 *        one, 4:2:2, 4:4:4, gray), pictures with partial edge MCUs, placed
 *        inside a frame buffer or cut by its edges; compared pixel by pixel
 *        with a floating-point reference. Pixels outside the picture must not
 *        be touched.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "yuv2rgb.h"

#define SENTINEL 0xDEADU

/* Test picture: sample values as functions of the pixel position */
static uint8_t luma(uint32_t x, uint32_t y)   { return (uint8_t)((x * 7U + y * 13U) & 0xFFU); }
static uint8_t chroma_b(uint32_t x, uint32_t y) { return (uint8_t)(40U + (x * 5U + y * 3U) % 180U); }
static uint8_t chroma_r(uint32_t x, uint32_t y) { return (uint8_t)(220U - (x * 3U + y * 7U) % 190U); }

static jpeg_hw_layout_t make_layout(uint32_t w, uint32_t h, uint32_t mcu_w, uint32_t mcu_h, bool gray)
{
  jpeg_hw_layout_t l;
  memset(&l, 0, sizeof(l));
  l.width = w;
  l.height = h;
  l.gray = gray;
  l.mcu_w = mcu_w;
  l.mcu_h = mcu_h;
  l.y_blocks = (mcu_w / 8U) * (mcu_h / 8U);
  l.mcu_bytes = (l.y_blocks + (gray ? 0U : 2U)) * 64U;
  l.mcus_per_row = (w + mcu_w - 1U) / mcu_w;
  return l;
}

/* The MCU stream of the test picture, as the codec outputs it */
static uint8_t *make_mcus(const jpeg_hw_layout_t *l, uint32_t *len)
{
  const uint32_t rows = (l->height + l->mcu_h - 1U) / l->mcu_h;
  const uint32_t count = rows * l->mcus_per_row;
  uint8_t *data = calloc(count, l->mcu_bytes);
  for (uint32_t m = 0; m < count; m++)
  {
    uint8_t *mcu = &data[m * l->mcu_bytes];
    const uint32_t mx = (m % l->mcus_per_row) * l->mcu_w;
    const uint32_t my = (m / l->mcus_per_row) * l->mcu_h;
    for (uint32_t py = 0; py < l->mcu_h; py++)
    {
      for (uint32_t px = 0; px < l->mcu_w; px++)
      {
        const uint32_t block = (py / 8U) * (l->mcu_w / 8U) + px / 8U;
        mcu[block * 64U + (py % 8U) * 8U + (px % 8U)] = luma(mx + px, my + py);
      }
    }
    if (!l->gray)
    {
      /* One 8x8 chroma block per MCU, each sample covering (mcu_w/8) x (mcu_h/8) pixels */
      for (uint32_t cy = 0; cy < 8U; cy++)
      {
        for (uint32_t cx = 0; cx < 8U; cx++)
        {
          const uint32_t x = mx + cx * (l->mcu_w / 8U);
          const uint32_t y = my + cy * (l->mcu_h / 8U);
          mcu[l->y_blocks * 64U + cy * 8U + cx]       = chroma_b(x, y);
          mcu[l->y_blocks * 64U + 64U + cy * 8U + cx] = chroma_r(x, y);
        }
      }
    }
  }
  *len = count * l->mcu_bytes;
  return data;
}

static int clamp(double v) { return (int)fmin(255.0, fmax(0.0, floor(v + 0.5))); }

/* Expected RGB of picture pixel (x, y) */
static void reference(const jpeg_hw_layout_t *l, uint32_t x, uint32_t y, int *r, int *g, int *b)
{
  const double Y = luma(x, y);
  double cb = 0.0, cr = 0.0;
  if (!l->gray)
  {
    /* The chroma sample of the 8x8 grid this pixel falls in */
    const uint32_t mx = x / l->mcu_w * l->mcu_w, my = y / l->mcu_h * l->mcu_h;
    const uint32_t sx = mx + ((x - mx) * 8U / l->mcu_w) * (l->mcu_w / 8U);
    const uint32_t sy = my + ((y - my) * 8U / l->mcu_h) * (l->mcu_h / 8U);
    cb = chroma_b(sx, sy) - 128.0;
    cr = chroma_r(sx, sy) - 128.0;
  }
  *r = clamp(Y + 1.402 * cr);
  *g = clamp(Y - 0.344136 * cb - 0.714136 * cr);
  *b = clamp(Y + 1.772 * cb);
}

/* Convert the test picture at (x0, y0) into a w x h buffer and check every pixel */
static void check_conversion(const jpeg_hw_layout_t *l, uint32_t w, uint32_t h, int32_t x0, int32_t y0)
{
  uint32_t len;
  uint8_t *mcus = make_mcus(l, &len);
  uint16_t *fb = malloc(w * h * sizeof(uint16_t));
  for (uint32_t i = 0; i < w * h; i++)
  {
    fb[i] = SENTINEL;
  }

  yuv2rgb_t t = { fb, w, h, x0, y0, *l, 0 };
  /* In two pieces, as the codec hands over its output */
  const uint32_t half = (len / l->mcu_bytes / 2U) * l->mcu_bytes;
  yuv2rgb_mcus(&t, mcus, half);
  yuv2rgb_mcus(&t, mcus + half, len - half);

  int worst = 0;
  uint32_t wrong_untouched = 0;
  for (uint32_t dy = 0; dy < h; dy++)
  {
    for (uint32_t dx = 0; dx < w; dx++)
    {
      const int32_t x = (int32_t)dx - x0, y = (int32_t)dy - y0;
      const uint16_t p = fb[dy * w + dx];
      if (x < 0 || y < 0 || x >= (int32_t)l->width || y >= (int32_t)l->height)
      {
        wrong_untouched += (p != SENTINEL);
        continue;
      }
      int r, g, b;
      reference(l, (uint32_t)x, (uint32_t)y, &r, &g, &b);
      /* In RGB565 units: rounding may differ by one step */
      const int dr = abs((p >> 11) - (r >> 3));
      const int dg = abs(((p >> 5) & 0x3F) - (g >> 2));
      const int db = abs((p & 0x1F) - (b >> 3));
      const int d = dr > dg ? (dr > db ? dr : db) : (dg > db ? dg : db);
      worst = d > worst ? d : worst;
    }
  }
  CHECK(worst <= 1);
  CHECK_EQ_INT(wrong_untouched, 0);
  free(fb);
  free(mcus);
}

static void test_420_inside(void)
{
  /* 40 x 24: three MCUs wide (the last half used), two high (the last half) */
  const jpeg_hw_layout_t l = make_layout(40, 24, 16, 16, false);
  check_conversion(&l, 64, 40, 16, 8);   /* MCU-aligned: fast path */
  check_conversion(&l, 64, 40, 5, 3);    /* not aligned            */
}

static void test_420_cut_by_the_buffer(void)
{
  const jpeg_hw_layout_t l = make_layout(64, 48, 16, 16, false);
  check_conversion(&l, 32, 16, -10, -4);   /* cut on all sides */
  check_conversion(&l, 32, 32, 20, 20);    /* partly off the bottom right */
  check_conversion(&l, 16, 16, 100, 100);  /* wholly off: nothing drawn */
}

static void test_other_layouts(void)
{
  const jpeg_hw_layout_t l422 = make_layout(32, 16, 16, 8, false);
  check_conversion(&l422, 40, 20, 4, 2);
  const jpeg_hw_layout_t l444 = make_layout(24, 16, 8, 8, false);
  check_conversion(&l444, 30, 20, 3, 1);
  const jpeg_hw_layout_t gray = make_layout(20, 12, 8, 8, true);
  check_conversion(&gray, 24, 16, 2, 2);
}

/* Known colours: white, black, pure red */
static void test_known_colours(void)
{
  const jpeg_hw_layout_t l = make_layout(8, 8, 8, 8, false);
  uint8_t mcu[192];
  uint16_t fb[64];
  yuv2rgb_t t = { fb, 8, 8, 0, 0, l, 0 };

  memset(mcu, 255, 64); memset(mcu + 64, 128, 128);
  yuv2rgb_mcus(&t, mcu, sizeof(mcu));
  CHECK_EQ_INT(fb[0], 0xFFFF);

  t.mcu_index = 0;
  memset(mcu, 0, 64);
  yuv2rgb_mcus(&t, mcu, sizeof(mcu));
  CHECK_EQ_INT(fb[63], 0x0000);

  /* Red: Y 76, Cb 85, Cr 255 (BT.601 full range) */
  t.mcu_index = 0;
  memset(mcu, 76, 64); memset(mcu + 64, 85, 64); memset(mcu + 128, 255, 64);
  yuv2rgb_mcus(&t, mcu, sizeof(mcu));
  CHECK((fb[0] >> 11) >= 30 && ((fb[0] >> 5) & 0x3F) <= 1 && (fb[0] & 0x1F) <= 1);
}

int main(void)
{
  yuv2rgb_init();
  RUN(test_420_inside);
  RUN(test_420_cut_by_the_buffer);
  RUN(test_other_layouts);
  RUN(test_known_colours);
  return CHECK_RESULT();
}
