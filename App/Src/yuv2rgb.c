/**
 * @file  yuv2rgb.c
 * @brief YCbCr (JPEG, BT.601 full range) to RGB565, see yuv2rgb.h.
 *
 * Per pixel: R = Y + 1.402 Cr', G = Y - 0.344 Cb' - 0.714 Cr', B = Y + 1.772 Cb'
 * (Cb' = Cb - 128, Cr' = Cr - 128). The chroma terms come from tables, and the
 * clamping to 0..255 and the packing into RGB565 from three more, so a pixel
 * costs three lookups. In a 4:2:0 MCU one chroma sample serves 2x2 pixels.
 */
#include "yuv2rgb.h"

#include <stdbool.h>

#define OFS 320            /* the packing tables cover -OFS .. 255 + OFS */
#define PACK_LEN (256 + 2 * OFS)

static int16_t  s_cr_r[256], s_cb_b[256], s_cr_g[256], s_cb_g[256];
/* The same terms two by two, for the 4:2:0 fast path: R | G << 16 of a Cr value,
 * B | G << 16 of a Cb value (16-bit lanes) */
static uint32_t s_cr_rg[256], s_cb_bg[256];
static uint16_t s_pack_r[PACK_LEN], s_pack_g[PACK_LEN], s_pack_b[PACK_LEN];

void yuv2rgb_init(void)
{
  for (int32_t i = 0; i < 256; i++)
  {
    const int32_t c = i - 128;
    /* 16.16 fixed point, rounded */
    s_cr_r[i] = (int16_t)((91881 * c + 32768) >> 16);
    s_cb_b[i] = (int16_t)((116130 * c + 32768) >> 16);
    s_cr_g[i] = (int16_t)((-46802 * c + 32768) >> 16);
    s_cb_g[i] = (int16_t)((-22554 * c + 32768) >> 16);
    s_cr_rg[i] = ((uint32_t)s_cr_r[i] & 0xFFFFU) | ((uint32_t)s_cr_g[i] << 16);
    s_cb_bg[i] = ((uint32_t)s_cb_b[i] & 0xFFFFU) | ((uint32_t)s_cb_g[i] << 16);
  }
  for (int32_t i = 0; i < PACK_LEN; i++)
  {
    int32_t v = i - OFS;
    v = (v < 0) ? 0 : (v > 255 ? 255 : v);
    s_pack_r[i] = (uint16_t)((v >> 3) << 11);
    s_pack_g[i] = (uint16_t)((v >> 2) << 5);
    s_pack_b[i] = (uint16_t)(v >> 3);
  }
}

static inline uint16_t pixel(int32_t y, int32_t r, int32_t g, int32_t b)
{
  return (uint16_t)(s_pack_r[y + r] | s_pack_g[y + g] | s_pack_b[y + b]);
}

/* ---- two pixels at a time: 16-bit lanes of a 32-bit word --------------------
 * On the Cortex-M7 these are single SIMD instructions; elsewhere (host tests)
 * plain C with the same results. */
typedef uint32_t __attribute__((may_alias)) word_t;

#if defined(__ARM_FEATURE_SIMD32)
#include <arm_acle.h>
static inline uint32_t add16(uint32_t a, uint32_t b)  { return (uint32_t)__sadd16((int16x2_t)a, (int16x2_t)b); }
static inline uint32_t sat16(uint32_t a)              { return (uint32_t)__usat16((int16x2_t)a, 8); }
static inline uint32_t lanes_even(uint32_t w)         { return (uint32_t)__uxtb16(w); }
static inline uint32_t lanes_odd(uint32_t w)          { return (uint32_t)__uxtb16(w >> 8); }
/* low lane of a, low lane of b / high lane of a, high lane of b */
static inline uint32_t low_lanes(uint32_t a, uint32_t b)
{
  uint32_t r;
  __asm__("pkhbt %0, %1, %2, lsl #16" : "=r"(r) : "r"(a), "r"(b));
  return r;
}
static inline uint32_t high_lanes(uint32_t a, uint32_t b)
{
  uint32_t r;
  __asm__("pkhtb %0, %2, %1, asr #16" : "=r"(r) : "r"(a), "r"(b));
  return r;
}
#else
static inline uint32_t low_lanes(uint32_t a, uint32_t b)  { return (a & 0xFFFFU) | (b << 16); }
static inline uint32_t high_lanes(uint32_t a, uint32_t b) { return (a >> 16) | (b & 0xFFFF0000U); }
static inline uint32_t add16(uint32_t a, uint32_t b)
{
  return ((a + b) & 0xFFFFU) | ((((a >> 16) + (b >> 16)) & 0xFFFFU) << 16);
}
static inline uint32_t sat_lane(int16_t v)            { return (v < 0) ? 0U : (v > 255 ? 255U : (uint32_t)v); }
static inline uint32_t sat16(uint32_t a)              { return sat_lane((int16_t)a) | (sat_lane((int16_t)(a >> 16)) << 16); }
static inline uint32_t lanes_even(uint32_t w)         { return w & 0x00FF00FFU; }
static inline uint32_t lanes_odd(uint32_t w)          { return (w >> 8) & 0x00FF00FFU; }
#endif

/* Two pixels: luma in the lanes of @p y, chroma terms in those of r, g, b */
static inline uint32_t two_pixels(uint32_t y, uint32_t r, uint32_t g, uint32_t b)
{
  const uint32_t rs = sat16(add16(y, r));
  const uint32_t gs = sat16(add16(y, g));
  const uint32_t bs = sat16(add16(y, b));
  return ((rs << 8) & 0xF800F800U) | ((gs << 3) & 0x07E007E0U) | ((bs >> 3) & 0x001F001FU);
}


/* 4:2:0 MCU (16x16) wholly inside the frame buffer and the picture, at an even
 * x (two pixels per 32-bit store) */
static void mcu_420_fast(const yuv2rgb_t *t, const uint8_t *mcu, int32_t px, int32_t py)
{
  const uint8_t *cb_all = mcu + 256;
  const uint8_t *cr_all = mcu + 320;
  const uint32_t stride = t->dst_w;
  uint16_t *row0 = t->dst + (uint32_t)py * stride + (uint32_t)px;

  for (uint32_t cy = 0; cy < 8U; cy++)
  {
    /* Luma rows 2cy and 2cy+1: blocks 0/1 (top half) or 2/3 (bottom half) */
    const uint8_t *yb   = mcu + ((cy >= 4U) ? 128U : 0U) + ((cy * 2U) & 7U) * 8U;
    uint16_t      *out0 = row0 + (cy * 2U) * stride;
    const uint8_t *cb   = cb_all + cy * 8U;
    const uint8_t *cr   = cr_all + cy * 8U;
    /* Four pixels per step: two chroma samples, one word of luma per row */
    for (uint32_t cx = 0; cx < 8U; cx += 2U)
    {
      const uint32_t t0 = s_cr_rg[cr[cx]], t1 = s_cr_rg[cr[cx + 1U]];
      const uint32_t u0 = s_cb_bg[cb[cx]], u1 = s_cb_bg[cb[cx + 1U]];
      const uint32_t r  = low_lanes(t0, t1);
      const uint32_t g  = add16(high_lanes(t0, t1), high_lanes(u0, u1));
      const uint32_t b  = low_lanes(u0, u1);
      /* Luma columns 2cx .. 2cx+3: block 0 (left) or 1 (right) */
      const uint8_t *y = yb + ((cx >= 4U) ? 64U : 0U) + ((cx * 2U) & 7U);
      word_t *o0 = (word_t *)&out0[cx * 2U];
      word_t *o1 = (word_t *)&out0[cx * 2U + stride];
      for (uint32_t row = 0; row < 2U; row++)
      {
        const uint32_t yw = *(const word_t *)&y[row * 8U];
        /* Lanes: pixels 0 and 2 (even), 1 and 3 (odd); chroma 0 and 1 */
        const uint32_t pe = two_pixels(lanes_even(yw), r, g, b);
        const uint32_t po = two_pixels(lanes_odd(yw), r, g, b);
        word_t *o = (row == 0U) ? o0 : o1;
        o[0] = low_lanes(pe, po);
        o[1] = high_lanes(pe, po);
      }
    }
  }
}

/* Any layout, cut to the frame buffer and the picture */
static void mcu_generic(const yuv2rgb_t *t, const uint8_t *mcu, uint32_t mx, uint32_t my)
{
  const jpeg_hw_layout_t *l = &t->layout;
  const uint8_t *cb = mcu + l->y_blocks * 64U;
  const uint8_t *cr = cb + 64U;
  const uint32_t blocks_per_row = l->mcu_w / 8U;

  for (uint32_t py = 0; py < l->mcu_h; py++)
  {
    const uint32_t sy = my + py;                       /* row in the picture */
    const int32_t  dy = t->y0 + (int32_t)sy;           /* row in the buffer  */
    if (sy >= l->height || dy < 0 || dy >= (int32_t)t->dst_h)
    {
      continue;
    }
    uint16_t *out = t->dst + (uint32_t)dy * t->dst_w;
    for (uint32_t px = 0; px < l->mcu_w; px++)
    {
      const uint32_t sx = mx + px;
      const int32_t  dx = t->x0 + (int32_t)sx;
      if (sx >= l->width || dx < 0 || dx >= (int32_t)t->dst_w)
      {
        continue;
      }
      const uint32_t block = (py / 8U) * blocks_per_row + px / 8U;
      const int32_t  y = mcu[block * 64U + (py % 8U) * 8U + (px % 8U)];
      int32_t r = OFS, g = OFS, b = OFS;
      if (!l->gray)
      {
        /* One 8x8 chroma block covers the whole MCU */
        const uint32_t ci = (py * 8U / l->mcu_h) * 8U + (px * 8U / l->mcu_w);
        r += s_cr_r[cr[ci]];
        g += s_cr_g[cr[ci]] + s_cb_g[cb[ci]];
        b += s_cb_b[cb[ci]];
      }
      out[dx] = pixel(y, r, g, b);
    }
  }
}

void yuv2rgb_mcus(yuv2rgb_t *t, const uint8_t *data, uint32_t len)
{
  const jpeg_hw_layout_t *l = &t->layout;
  const bool is_420 = !l->gray && l->mcu_w == 16U && l->mcu_h == 16U;

  for (uint32_t i = 0; i + l->mcu_bytes <= len; i += l->mcu_bytes, t->mcu_index++)
  {
    const uint32_t mx = (t->mcu_index % l->mcus_per_row) * l->mcu_w;
    const uint32_t my = (t->mcu_index / l->mcus_per_row) * l->mcu_h;
    const int32_t  px = t->x0 + (int32_t)mx;
    const int32_t  py = t->y0 + (int32_t)my;

    /* Wholly outside the buffer: nothing to draw */
    if (px >= (int32_t)t->dst_w || py >= (int32_t)t->dst_h ||
        px + (int32_t)l->mcu_w <= 0 || py + (int32_t)l->mcu_h <= 0)
    {
      continue;
    }
    const bool inside = px >= 0 && py >= 0 &&
                        px + (int32_t)l->mcu_w <= (int32_t)t->dst_w &&
                        py + (int32_t)l->mcu_h <= (int32_t)t->dst_h &&
                        mx + l->mcu_w <= l->width && my + l->mcu_h <= l->height;
    if (is_420 && inside && (px & 1) == 0)
    {
      mcu_420_fast(t, &data[i], px, py);
    }
    else
    {
      mcu_generic(t, &data[i], mx, my);
    }
  }
}
