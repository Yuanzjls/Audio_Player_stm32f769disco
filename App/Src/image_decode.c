/**
 * @file  image_decode.c
 * @brief PNG (with the streaming inflate) and BMP decoding into the scaler.
 */
#include "image_decode.h"

#include <string.h>

#include "inflate.h"
#include "simd8.h"

#define MAX_ROW_BYTES  IMAGE_MAX_ROW_BYTES
#define IN_CHUNK       IMAGE_IN_CHUNK

static image_work_t *s_w;              /* working memory of the decode under way */

static uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static uint32_t le32(const uint8_t *p) { return ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) | ((uint32_t)p[1] << 8) | p[0]; }
static uint32_t le16(const uint8_t *p) { return ((uint32_t)p[1] << 8) | p[0]; }

static bool read_exact(FIL *f, void *buf, uint32_t len)
{
  UINT got = 0;
  return f_read(f, buf, len, &got) == FR_OK && got == len;
}

static bool aborted(const image_target_t *t)
{
  return t->abort != NULL && *t->abort;
}

/* Colour @p c with alpha @p a over the background colour @p bg */
static inline uint8_t blend(uint32_t c, uint32_t a, uint32_t bg)
{
  return (uint8_t)((c * a + bg * (255U - a) + 127U) / 255U);
}

image_kind_t image_kind(const uint8_t *head, uint32_t len)
{
  static const uint8_t k_png[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
  if (len >= 3U && head[0] == 0xFFU && head[1] == 0xD8U && head[2] == 0xFFU)
  {
    return IMAGE_KIND_JPEG;
  }
  if (len >= 8U && memcmp(head, k_png, 8) == 0)
  {
    return IMAGE_KIND_PNG;
  }
  if (len >= 6U && (memcmp(head, "GIF87a", 6) == 0 || memcmp(head, "GIF89a", 6) == 0))
  {
    return IMAGE_KIND_GIF;
  }
  if (len >= 2U && head[0] == 'B' && head[1] == 'M')
  {
    return IMAGE_KIND_BMP;
  }
  return IMAGE_KIND_UNKNOWN;
}

/* ---- PNG ------------------------------------------------------------------- */

static const uint8_t k_adam7[7][4] = {   /* start x, start y, step x, step y */
  { 0, 0, 8, 8 }, { 4, 0, 8, 8 }, { 0, 4, 4, 8 }, { 2, 0, 4, 4 },
  { 0, 2, 2, 4 }, { 1, 0, 2, 2 }, { 0, 1, 1, 2 },
};

typedef struct
{
  FIL *f;
  const image_target_t *target;
  uint32_t w, h;
  uint32_t depth, ctype, channels;
  uint32_t pixel_bits;           /* bits per pixel */
  uint32_t filter_bpp;           /* bytes per pixel for the filters, at least 1 */
  bool     interlaced;
  uint8_t  palette[256][3];
  uint8_t  palette_alpha[256];
  bool     has_key;              /* tRNS of a grey or RGB picture: the transparent colour */
  uint16_t key[3];
  uint8_t  bg[3];

  uint32_t chunk_left;           /* IDAT bytes still to read in the current chunk */
  bool     idat_end;

  uint32_t pass;                 /* 0..6, or 0 for a picture without interlacing */
  uint32_t pass_w, pass_h;
  uint32_t row_bytes;            /* of the current pass, without the filter byte */
  uint32_t row;                  /* in the pass */
  uint32_t fill;                 /* bytes of the current row received, with the filter byte */
  uint8_t *cur, *prev;
  bool     done;                 /* all rows are in */
} png_t;

static png_t s_png;

/* Up to @p max bytes of the compressed data, across IDAT chunks; 0 at the end */
static uint32_t idat_read(png_t *p, uint8_t *buf, uint32_t max)
{
  uint32_t total = 0;
  while (total < max && !p->idat_end)
  {
    if (aborted(p->target))
    {
      p->idat_end = true;
      break;
    }
    if (p->chunk_left == 0U)
    {
      /* CRC of this chunk, then the next one's length and type */
      uint8_t hdr[12];
      if (!read_exact(p->f, hdr, sizeof(hdr)) || memcmp(&hdr[8], "IDAT", 4) != 0)
      {
        p->idat_end = true;
        break;
      }
      p->chunk_left = be32(&hdr[4]);
      continue;
    }
    const uint32_t want = (max - total < p->chunk_left) ? max - total : p->chunk_left;
    UINT got = 0;
    if (f_read(p->f, &buf[total], want, &got) != FR_OK || got == 0U)
    {
      p->idat_end = true;
      break;
    }
    p->chunk_left -= got;
    total += got;
  }
  return total;
}

static uint32_t png_inflate_read(void *ctx, const uint8_t **data)
{
  *data = s_w->in;
  return idat_read((png_t *)ctx, s_w->in, IN_CHUNK);
}

/* Set up pass @p pass (skipping empty ones); false when there are no more */
static bool start_pass(png_t *p, uint32_t pass)
{
  for (; pass < (p->interlaced ? 7U : 1U); pass++)
  {
    const uint32_t sx = p->interlaced ? k_adam7[pass][0] : 0U, sy = p->interlaced ? k_adam7[pass][1] : 0U;
    const uint32_t dx = p->interlaced ? k_adam7[pass][2] : 1U, dy = p->interlaced ? k_adam7[pass][3] : 1U;
    p->pass_w = (p->w > sx) ? (p->w - sx + dx - 1U) / dx : 0U;
    p->pass_h = (p->h > sy) ? (p->h - sy + dy - 1U) / dy : 0U;
    if (p->pass_w != 0U && p->pass_h != 0U)
    {
      p->pass      = pass;
      p->row_bytes = (p->pass_w * p->pixel_bits + 7U) / 8U;
      p->row       = 0;
      p->fill      = 0;
      memset(p->prev, 0, p->row_bytes + 1U);
      return true;
    }
  }
  return false;
}

static inline uint8_t paeth(uint8_t a, uint8_t b, uint8_t c)
{
  const int pa = (b > c) ? b - c : c - b;            /* |p - a| with p = a + b - c */
  const int pb = (a > c) ? a - c : c - a;
  const int pc = (a + b > 2 * c) ? a + b - 2 * c : 2 * c - a - b;
  return (pa <= pb && pa <= pc) ? a : (pb <= pc) ? b : c;
}

/* Undo the row's filter. Up, and Sub and Average for pixels of 4 or 8 bytes,
 * work on four bytes at a time (SIMD). */
static bool unfilter(png_t *p)
{
  uint8_t *r = &p->cur[1];
  const uint8_t *u = &p->prev[1];
  const uint32_t n = p->row_bytes, bpp = p->filter_bpp;
  const bool words = (bpp % 4U) == 0U;
  uint32_t i = 0;
  switch (p->cur[0])
  {
    case 0:
      break;
    case 1:   /* Sub: + the byte one pixel to the left */
      if (words)
      {
        for (i = bpp; i < n; i += 4U) store32(&r[i], add8(load32(&r[i]), load32(&r[i - bpp])));
      }
      else
      {
        for (i = bpp; i < n; i++) r[i] = (uint8_t)(r[i] + r[i - bpp]);
      }
      break;
    case 2:   /* Up: + the byte above */
      for (; i + 4U <= n; i += 4U) store32(&r[i], add8(load32(&r[i]), load32(&u[i])));
      for (; i < n; i++) r[i] = (uint8_t)(r[i] + u[i]);
      break;
    case 3:   /* Average: + (left + above) / 2 */
      if (words)
      {
        for (; i < bpp; i++) r[i] = (uint8_t)(r[i] + u[i] / 2U);
        for (; i < n; i += 4U) store32(&r[i], add8(load32(&r[i]), havg8(load32(&r[i - bpp]), load32(&u[i]))));
      }
      else
      {
        for (; i < n; i++) r[i] = (uint8_t)(r[i] + (((i >= bpp) ? r[i - bpp] : 0U) + u[i]) / 2U);
      }
      break;
    case 4:
      for (uint32_t i = 0; i < n; i++)
      {
        r[i] = (uint8_t)(r[i] + paeth((i >= bpp) ? r[i - bpp] : 0U, u[i], (i >= bpp) ? u[i - bpp] : 0U));
      }
      break;
    default:
      return false;
  }
  return true;
}

/* The unfiltered row (pass_w pixels) as RGB in s_w->line, transparency blended */
static void png_to_rgb(const png_t *p)
{
  const uint8_t *r = &p->cur[1];
  uint8_t *o = s_w->line;
  const uint32_t d = p->depth;

  for (uint32_t i = 0; i < p->pass_w; i++, o += 3)
  {
    uint32_t c[3], a = 255;
    if (d < 8U)
    {
      /* Grey or palette, several pixels per byte, the first in the high bits */
      const uint32_t bit = i * d;
      const uint32_t v = (r[bit / 8U] >> (8U - d - bit % 8U)) & ((1U << d) - 1U);
      if (p->ctype == 3U)
      {
        c[0] = p->palette[v][0]; c[1] = p->palette[v][1]; c[2] = p->palette[v][2];
        a = p->palette_alpha[v];
      }
      else
      {
        c[0] = c[1] = c[2] = v * 255U / ((1U << d) - 1U);
        if (p->has_key && v == p->key[0])
        {
          a = 0;
        }
      }
    }
    else
    {
      /* 8 or 16 bits per sample: the high byte; the key compares the whole sample */
      const uint32_t bytes = d / 8U;
      const uint8_t *s = &r[i * p->channels * bytes];
      uint32_t full[4];
      for (uint32_t ch = 0; ch < p->channels; ch++)
      {
        full[ch] = (bytes == 2U) ? ((uint32_t)s[ch * 2U] << 8) | s[ch * 2U + 1U] : s[ch];
      }
      const uint32_t shift = (bytes == 2U) ? 8U : 0U;
      switch (p->ctype)
      {
        case 0:   /* grey */
          c[0] = c[1] = c[2] = full[0] >> shift;
          a = (p->has_key && full[0] == p->key[0]) ? 0U : 255U;
          break;
        case 2:   /* RGB */
          c[0] = full[0] >> shift; c[1] = full[1] >> shift; c[2] = full[2] >> shift;
          a = (p->has_key && full[0] == p->key[0] && full[1] == p->key[1] && full[2] == p->key[2]) ? 0U : 255U;
          break;
        case 3:   /* palette, 8 bits */
          c[0] = p->palette[full[0]][0]; c[1] = p->palette[full[0]][1]; c[2] = p->palette[full[0]][2];
          a = p->palette_alpha[full[0]];
          break;
        case 4:   /* grey + alpha */
          c[0] = c[1] = c[2] = full[0] >> shift;
          a = full[1] >> shift;
          break;
        default:  /* RGBA */
          c[0] = full[0] >> shift; c[1] = full[1] >> shift; c[2] = full[2] >> shift;
          a = full[3] >> shift;
          break;
      }
    }
    if (a == 255U)
    {
      o[0] = (uint8_t)c[0]; o[1] = (uint8_t)c[1]; o[2] = (uint8_t)c[2];
    }
    else
    {
      o[0] = blend(c[0], a, p->bg[0]); o[1] = blend(c[1], a, p->bg[1]); o[2] = blend(c[2], a, p->bg[2]);
    }
  }
}

/* Decompressed data: rows with their filter bytes, pass after pass */
static bool png_inflate_write(void *ctx, const uint8_t *data, uint32_t len)
{
  png_t *p = ctx;
  while (len != 0U && !p->done)
  {
    const uint32_t need = p->row_bytes + 1U - p->fill;
    const uint32_t n = (len < need) ? len : need;
    memcpy(&p->cur[p->fill], data, n);
    p->fill += n;
    data += n;
    len -= n;
    if (p->fill < p->row_bytes + 1U)
    {
      break;
    }

    if (!unfilter(p))
    {
      return false;
    }
    png_to_rgb(p);
    if (p->interlaced)
    {
      const uint8_t *a = k_adam7[p->pass];
      scale_point(&s_w->scale, a[1] + p->row * a[3], a[0], a[2], s_w->line, p->pass_w);
    }
    else
    {
      scale_row(&s_w->scale, p->row, s_w->line);
    }

    uint8_t *t = p->prev;
    p->prev = p->cur;
    p->cur  = t;
    p->fill = 0;
    if (++p->row == p->pass_h && !start_pass(p, p->pass + 1U))
    {
      p->done = true;
    }
  }
  return !p->done && !aborted(p->target);
}

/* The chunks before the image data: IHDR, PLTE, tRNS */
static image_result_t png_header(png_t *p)
{
  uint8_t sig[8];
  uint8_t hdr[8];
  if (!read_exact(p->f, sig, 8) || image_kind(sig, 8) != IMAGE_KIND_PNG)
  {
    return IMAGE_UNSUPPORTED;
  }
  bool have_ihdr = false;
  for (;;)
  {
    if (!read_exact(p->f, hdr, 8))
    {
      return IMAGE_CORRUPT;
    }
    const uint32_t len = be32(hdr);
    const uint8_t *type = &hdr[4];

    if (memcmp(type, "IHDR", 4) == 0 && len == 13U)
    {
      uint8_t d[13];
      if (!read_exact(p->f, d, 13))
      {
        return IMAGE_CORRUPT;
      }
      p->w = be32(d);
      p->h = be32(&d[4]);
      p->depth = d[8];
      p->ctype = d[9];
      p->interlaced = d[12] == 1U;
      static const uint8_t k_channels[7] = { 1, 0, 3, 1, 2, 0, 4 };
      const bool depth_ok =
        (p->ctype == 0U && (p->depth == 1U || p->depth == 2U || p->depth == 4U || p->depth == 8U || p->depth == 16U)) ||
        (p->ctype == 3U && (p->depth == 1U || p->depth == 2U || p->depth == 4U || p->depth == 8U)) ||
        ((p->ctype == 2U || p->ctype == 4U || p->ctype == 6U) && (p->depth == 8U || p->depth == 16U));
      if (!depth_ok || d[10] != 0U || d[11] != 0U || d[12] > 1U)
      {
        return IMAGE_UNSUPPORTED;
      }
      p->channels   = k_channels[p->ctype];
      p->pixel_bits = p->depth * p->channels;
      p->filter_bpp = (p->pixel_bits >= 8U) ? p->pixel_bits / 8U : 1U;
      if (p->w == 0U || p->h == 0U)
      {
        return IMAGE_CORRUPT;
      }
      if (p->w > SCALE_MAX_SRC || p->h > SCALE_MAX_SRC || (p->w * p->pixel_bits + 7U) / 8U > MAX_ROW_BYTES)
      {
        return IMAGE_TOO_LARGE;
      }
      have_ihdr = true;
    }
    else if (!have_ihdr)
    {
      return IMAGE_CORRUPT;
    }
    else if (memcmp(type, "PLTE", 4) == 0 && len <= 768U && len % 3U == 0U)
    {
      if (!read_exact(p->f, p->palette, len))
      {
        return IMAGE_CORRUPT;
      }
    }
    else if (memcmp(type, "tRNS", 4) == 0 && len <= 256U)
    {
      uint8_t t[256];
      if (!read_exact(p->f, t, len))
      {
        return IMAGE_CORRUPT;
      }
      if (p->ctype == 3U)
      {
        memcpy(p->palette_alpha, t, len);
      }
      else if ((p->ctype == 0U && len >= 2U) || (p->ctype == 2U && len >= 6U))
      {
        p->has_key = true;
        for (uint32_t i = 0; i < len / 2U && i < 3U; i++)
        {
          p->key[i] = (uint16_t)((t[i * 2U] << 8) | t[i * 2U + 1U]);
        }
      }
    }
    else if (memcmp(type, "IDAT", 4) == 0)
    {
      p->chunk_left = len;
      return IMAGE_OK;
    }
    else if (memcmp(type, "IEND", 4) == 0)
    {
      return IMAGE_CORRUPT;
    }
    else if (f_lseek(p->f, f_tell(p->f) + len) != FR_OK)
    {
      return IMAGE_CORRUPT;
    }
    /* The chunk's CRC (not checked) */
    if (f_lseek(p->f, f_tell(p->f) + 4U) != FR_OK)
    {
      return IMAGE_CORRUPT;
    }
  }
}

image_result_t image_decode_png(FIL *file, const image_target_t *target, uint32_t *out_w, uint32_t *out_h)
{
  s_w = target->work;
  png_t *p = &s_png;
  memset(p, 0, sizeof(*p));
  memset(p->palette_alpha, 255, sizeof(p->palette_alpha));
  p->f      = file;
  p->target = target;
  p->bg[0]  = (uint8_t)(target->background >> 16);
  p->bg[1]  = (uint8_t)(target->background >> 8);
  p->bg[2]  = (uint8_t)target->background;
  p->cur    = s_w->rows[0];
  p->prev   = s_w->rows[1];

  image_result_t res = png_header(p);
  s_w->scale.src_w = p->w;   /* the size, also for a picture that is not decoded */
  s_w->scale.src_h = p->h;
  if (res != IMAGE_OK)
  {
    return res;
  }
  if (!scale_begin(&s_w->scale, p->w, p->h, 0, SCALE_RGB, target->out, target->max_w, target->max_h))
  {
    return IMAGE_TOO_LARGE;
  }
  *out_w = s_w->scale.out_w;
  *out_h = s_w->scale.out_h;
  memset(target->out, 0, (size_t)s_w->scale.out_w * s_w->scale.out_h * 2U);
  start_pass(p, 0);

  /* zlib header: deflate, no preset dictionary */
  uint8_t zlib[2];
  if (idat_read(p, zlib, 2) != 2U || (zlib[0] & 0x0FU) != 8U || (zlib[1] & 0x20U) != 0U ||
      ((zlib[0] << 8) | zlib[1]) % 31U != 0U)
  {
    res = IMAGE_CORRUPT;
  }
  else
  {
    /* Stops by itself once all rows are in; anything else is a damaged file */
    (void)inflate_run(&s_w->inflate, png_inflate_read, png_inflate_write, p);
    res = p->done ? IMAGE_OK : IMAGE_CORRUPT;
  }
  if (aborted(target))
  {
    return IMAGE_ABORTED;
  }
  scale_end(&s_w->scale);
  return res;
}

/* ---- BMP ------------------------------------------------------------------- */

typedef struct
{
  uint32_t mask, shift, max;
} field_t;

static field_t make_field(uint32_t mask)
{
  field_t f = { mask, 0, 0 };
  if (mask != 0U)
  {
    while (((mask >> f.shift) & 1U) == 0U)
    {
      f.shift++;
    }
    f.max = mask >> f.shift;
  }
  return f;
}

static inline uint8_t field_value(const field_t *f, uint32_t v)
{
  if (f->max == 0U)
  {
    return 0;
  }
  const uint32_t x = (v & f->mask) >> f->shift;
  return (uint8_t)((x * 255U + f->max / 2U) / f->max);
}

image_result_t image_decode_bmp(FIL *file, const image_target_t *target, uint32_t *out_w, uint32_t *out_h)
{
  uint8_t h[14 + 124];
  s_w = target->work;
  s_w->scale.src_w = s_w->scale.src_h = 0;
  memset(h, 0, sizeof(h));
  UINT got = 0;
  if (f_read(file, h, sizeof(h), &got) != FR_OK || got < 26U || h[0] != 'B' || h[1] != 'M')
  {
    return IMAGE_UNSUPPORTED;
  }
  const uint32_t data_offset = le32(&h[10]);
  const uint32_t dib = le32(&h[14]);
  int32_t w, height;
  uint32_t bpp, compression = 0, colours = 0, palette_offset, entry_size = 4;

  if (dib == 12U)
  {
    w = (int32_t)le16(&h[18]);
    height = (int32_t)le16(&h[20]);
    bpp = le16(&h[24]);
    palette_offset = 14U + 12U;
    entry_size = 3;
  }
  else if (dib >= 40U && got >= 54U)
  {
    w = (int32_t)le32(&h[18]);
    height = (int32_t)le32(&h[22]);
    bpp = le16(&h[28]);
    compression = le32(&h[30]);
    colours = le32(&h[46]);
    palette_offset = 14U + dib;
    if (dib == 40U && compression == 3U)
    {
      palette_offset += 12U;   /* the bit fields follow the header */
    }
    else if (dib == 40U && compression == 6U)
    {
      palette_offset += 16U;
    }
  }
  else
  {
    return IMAGE_UNSUPPORTED;
  }

  const bool top_down = height < 0;
  const uint32_t ht = (uint32_t)(top_down ? -height : height);
  if (w <= 0 || ht == 0U)
  {
    return IMAGE_CORRUPT;
  }
  const bool bitfields = compression == 3U || compression == 6U;
  if (!(compression == 0U || (bitfields && (bpp == 16U || bpp == 32U))) ||
      !(bpp == 1U || bpp == 4U || bpp == 8U || bpp == 16U || bpp == 24U || bpp == 32U))
  {
    return IMAGE_UNSUPPORTED;
  }
  const uint32_t width = (uint32_t)w;
  s_w->scale.src_w = width;
  s_w->scale.src_h = ht;
  const uint32_t stride = ((width * bpp + 31U) / 32U) * 4U;
  if (width > SCALE_MAX_SRC || ht > SCALE_MAX_SRC || stride > MAX_ROW_BYTES)
  {
    return IMAGE_TOO_LARGE;
  }

  /* Bit fields: in the header (V4 / V5) or right after it (V3), at the same place */
  field_t fr, fg, fb;
  if (bitfields && got >= 66U)
  {
    fr = make_field(le32(&h[54]));
    fg = make_field(le32(&h[58]));
    fb = make_field(le32(&h[62]));
  }
  else if (bpp == 16U)
  {
    fr = make_field(0x7C00U); fg = make_field(0x03E0U); fb = make_field(0x001FU);
  }
  else
  {
    fr = make_field(0xFF0000U); fg = make_field(0x00FF00U); fb = make_field(0x0000FFU);
  }

  /* Palette, as RGB */
  uint8_t (*palette)[3] = (uint8_t (*)[3])s_w->in;   /* 768 bytes */
  if (bpp <= 8U)
  {
    const uint32_t max = 1U << bpp;
    const uint32_t n = (colours == 0U || colours > max) ? max : colours;
    uint8_t *raw = s_w->rows[0];
    memset(palette, 0, 256U * 3U);
    if (f_lseek(file, palette_offset) != FR_OK || !read_exact(file, raw, n * entry_size))
    {
      return IMAGE_CORRUPT;
    }
    for (uint32_t i = 0; i < n; i++)
    {
      palette[i][0] = raw[i * entry_size + 2U];
      palette[i][1] = raw[i * entry_size + 1U];
      palette[i][2] = raw[i * entry_size];
    }
  }

  /* Rows in file order; bottom-up files are turned over by the scaler */
  if (!scale_begin(&s_w->scale, width, ht, top_down ? 0U : SCALE_FLIP_Y, SCALE_RGB, target->out, target->max_w,
                   target->max_h))
  {
    return IMAGE_TOO_LARGE;
  }
  *out_w = s_w->scale.out_w;
  *out_h = s_w->scale.out_h;
  memset(target->out, 0, (size_t)s_w->scale.out_w * s_w->scale.out_h * 2U);
  if (f_lseek(file, data_offset) != FR_OK)
  {
    return IMAGE_CORRUPT;
  }

  image_result_t res = IMAGE_OK;
  uint8_t *row = s_w->rows[0];
  for (uint32_t y = 0; y < ht; y++)
  {
    if (aborted(target))
    {
      return IMAGE_ABORTED;
    }
    if (!read_exact(file, row, stride))
    {
      res = IMAGE_CORRUPT;
      break;
    }
    uint8_t *o = s_w->line;
    for (uint32_t x = 0; x < width; x++, o += 3)
    {
      switch (bpp)
      {
        case 1: case 4: case 8:
        {
          const uint32_t bit = x * bpp;
          const uint32_t v = (row[bit / 8U] >> (8U - bpp - bit % 8U)) & ((1U << bpp) - 1U);
          o[0] = palette[v][0]; o[1] = palette[v][1]; o[2] = palette[v][2];
          break;
        }
        case 16:
        {
          const uint32_t v = le16(&row[x * 2U]);
          o[0] = field_value(&fr, v); o[1] = field_value(&fg, v); o[2] = field_value(&fb, v);
          break;
        }
        case 24:
          o[0] = row[x * 3U + 2U]; o[1] = row[x * 3U + 1U]; o[2] = row[x * 3U];
          break;
        default:
        {
          const uint32_t v = le32(&row[x * 4U]);
          o[0] = field_value(&fr, v); o[1] = field_value(&fg, v); o[2] = field_value(&fb, v);
          break;
        }
      }
    }
    scale_row(&s_w->scale, y, s_w->line);
  }
  scale_end(&s_w->scale);
  return res;
}
