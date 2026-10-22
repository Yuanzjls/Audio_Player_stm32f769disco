/**
 * @file  image_scale.c
 * @brief Box-filter shrinking (and turning) of a picture as it is decoded.
 */
#include "image_scale.h"

#include <string.h>

static inline uint16_t rgb565(uint32_t r, uint32_t g, uint32_t b)
{
  return (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
}

static inline uint32_t clamp8(int32_t v)
{
  return (v < 0) ? 0U : (v > 255) ? 255U : (uint32_t)v;
}

/* JPEG (full range) YCbCr to RGB565 */
static inline uint16_t ycbcr565(int32_t y, int32_t cb, int32_t cr)
{
  cb -= 128;
  cr -= 128;
  const int32_t r = y + ((91881 * cr) >> 16);
  const int32_t g = y - ((22554 * cb + 46802 * cr) >> 16);
  const int32_t b = y + ((116130 * cb) >> 16);
  return rgb565(clamp8(r), clamp8(g), clamp8(b));
}

/* Output pixel (tx, ty) before the transform */
static inline void store(scale_t *s, uint32_t tx, uint32_t ty, uint16_t v)
{
  uint32_t x = tx, y = ty;
  if ((s->transform & SCALE_TRANSPOSE) != 0U)
  {
    x = ty;
    y = tx;
  }
  if ((s->transform & SCALE_FLIP_X) != 0U)
  {
    x = s->out_w - 1U - x;
  }
  if ((s->transform & SCALE_FLIP_Y) != 0U)
  {
    y = s->out_h - 1U - y;
  }
  s->out[y * s->out_w + x] = v;
}

bool scale_begin(scale_t *s, uint32_t src_w, uint32_t src_h, uint32_t transform, scale_space_t space,
                 uint16_t *out, uint32_t max_w, uint32_t max_h)
{
  if (src_w == 0U || src_h == 0U || src_w > SCALE_MAX_SRC || src_h > SCALE_MAX_SRC ||
      max_w == 0U || max_h == 0U || max_w > SCALE_MAX_OUT || max_h > SCALE_MAX_OUT)
  {
    return false;
  }
  const bool transpose = (transform & SCALE_TRANSPOSE) != 0U;
  const uint32_t dw = transpose ? src_h : src_w;   /* as shown */
  const uint32_t dh = transpose ? src_w : src_h;
  uint32_t ow = dw, oh = dh;
  if (dw > max_w || dh > max_h)
  {
    if ((uint64_t)dw * max_h > (uint64_t)dh * max_w)
    {
      ow = max_w;
      oh = (uint32_t)(((uint64_t)dh * max_w + dw / 2U) / dw);
    }
    else
    {
      oh = max_h;
      ow = (uint32_t)(((uint64_t)dw * max_h + dh / 2U) / dh);
    }
    ow = (ow == 0U) ? 1U : ow;
    oh = (oh == 0U) ? 1U : oh;
  }

  s->src_w     = src_w;
  s->src_h     = src_h;
  s->out_w     = ow;
  s->out_h     = oh;
  s->tw        = transpose ? oh : ow;
  s->th        = transpose ? ow : oh;
  s->transform = transform;
  s->space     = space;
  s->out       = out;
  s->next_row  = 0;
  s->pointwise = false;

  memset(s->n_x, 0, sizeof(s->n_x));
  memset(s->n_y, 0, sizeof(s->n_y));
  for (uint32_t x = 0; x < src_w; x++)
  {
    s->map_x[x] = (uint16_t)((uint64_t)x * s->tw / src_w);
    s->n_x[s->map_x[x]]++;
  }
  for (uint32_t y = 0; y < src_h; y++)
  {
    s->map_y[y] = (uint16_t)((uint64_t)y * s->th / src_h);
    s->n_y[s->map_y[y]]++;
    s->last_y[s->map_y[y]] = (uint16_t)y;
  }
  for (uint32_t t = 0; t < s->tw; t++)
  {
    s->pick_x[t] = (uint16_t)((uint64_t)(2U * t + 1U) * src_w / (2U * s->tw));
  }
  for (uint32_t t = 0; t < s->th; t++)
  {
    s->pick_y[t] = (uint16_t)((uint64_t)(2U * t + 1U) * src_h / (2U * s->th));
  }
  memset(s->acc, 0, sizeof(s->acc));
  return true;
}

static void write_row(scale_t *s, uint32_t ty)
{
  uint32_t *a = scale_acc(s, ty);
  const uint32_t ny = s->n_y[ty];
  for (uint32_t tx = 0; tx < s->tw; tx++, a += 3)
  {
    const uint32_t n = s->n_x[tx] * ny;
    const uint32_t c0 = (a[0] + n / 2U) / n;
    const uint32_t c1 = (a[1] + n / 2U) / n;
    const uint32_t c2 = (a[2] + n / 2U) / n;
    store(s, tx, ty, (s->space == SCALE_YCBCR) ? ycbcr565((int32_t)c0, (int32_t)c1, (int32_t)c2) : rgb565(c0, c1, c2));
  }
  memset(scale_acc(s, ty), 0, s->tw * 3U * sizeof(uint32_t));
}

void scale_rows_done(scale_t *s, uint32_t y)
{
  while (s->next_row < s->th && s->last_y[s->next_row] <= y)
  {
    write_row(s, s->next_row++);
  }
}

void scale_row(scale_t *s, uint32_t y, const uint8_t *px)
{
  if (y >= s->src_h)
  {
    return;
  }
  uint32_t *a = scale_acc(s, s->map_y[y]);
  for (uint32_t x = 0; x < s->src_w; x++, px += 3)
  {
    uint32_t *c = &a[s->map_x[x] * 3U];
    c[0] += px[0];
    c[1] += px[1];
    c[2] += px[2];
  }
  scale_rows_done(s, y);
}

void scale_point(scale_t *s, uint32_t y, uint32_t x0, uint32_t step, const uint8_t *px, uint32_t n)
{
  s->pointwise = true;
  if (y >= s->src_h)
  {
    return;
  }
  const uint32_t ty = s->map_y[y];
  if (s->pick_y[ty] != y)
  {
    return;
  }
  for (uint32_t i = 0, x = x0; i < n && x < s->src_w; i++, x += step, px += 3)
  {
    const uint32_t tx = s->map_x[x];
    if (s->pick_x[tx] == x)
    {
      store(s, tx, ty, rgb565(px[0], px[1], px[2]));
    }
  }
}

void scale_end(scale_t *s)
{
  if (!s->pointwise)
  {
    while (s->next_row < s->th)
    {
      write_row(s, s->next_row++);
    }
  }
}
