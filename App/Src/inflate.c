/**
 * @file  inflate.c
 * @brief Streaming DEFLATE decompression (after Mark Adler's puff.c, with a
 *        lookup table for the common short codes).
 */
#include "inflate.h"

#include <string.h>

#define FAST_BITS   9U
#define MAX_BITS    15U
#define WINDOW_MASK (INFLATE_WINDOW - 1U)
#define MAX_PAD     64U          /* zero bits past the end before giving up */

static const uint16_t k_len_base[29] = { 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
                                         35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258 };
static const uint8_t  k_len_extra[29] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
                                          3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0 };
static const uint16_t k_dist_base[30] = { 1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
                                          193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097,
                                          6145, 8193, 12289, 16385, 24577 };
static const uint8_t  k_dist_extra[30] = { 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
                                           7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13 };

/* ---- bits ------------------------------------------------------------------ */

/* At least @p n (<= 24) bits in the buffer; zeros past the end of the input */
static void need(inflate_t *s, uint32_t n)
{
  while (s->nbits < n)
  {
    uint32_t byte = 0;
    if (s->in == s->in_end)
    {
      const uint8_t *data = NULL;
      const uint32_t len = s->failed ? 0U : s->read(s->ctx, &data);
      if (len != 0U)
      {
        s->in     = data;
        s->in_end = data + len;
      }
    }
    if (s->in != s->in_end)
    {
      byte = *s->in++;
    }
    else
    {
      s->pad_bits += 8U;
      if (s->pad_bits > MAX_PAD)
      {
        s->failed = true;
      }
    }
    s->bits  |= byte << s->nbits;
    s->nbits += 8U;
  }
}

static uint32_t get_bits(inflate_t *s, uint32_t n)
{
  if (n == 0U)
  {
    return 0;
  }
  need(s, n);
  const uint32_t v = s->bits & ((1U << n) - 1U);
  s->bits  >>= n;
  s->nbits  -= n;
  return v;
}

/* ---- output ---------------------------------------------------------------- */

static void flush(inflate_t *s)
{
  if (s->wpos != 0U && !s->stopped && !s->write(s->ctx, s->window, s->wpos))
  {
    s->stopped = true;
  }
  if (s->wpos == INFLATE_WINDOW)
  {
    s->total = INFLATE_WINDOW;   /* the whole window is history from now on */
  }
  s->wpos = 0;
}

static inline void put(inflate_t *s, uint8_t byte)
{
  s->window[s->wpos++] = byte;
  if (s->wpos == INFLATE_WINDOW)
  {
    flush(s);
  }
}

/* ---- Huffman codes --------------------------------------------------------- */

static uint32_t reverse_bits(uint32_t code, uint32_t len)
{
  uint32_t r = 0;
  for (uint32_t i = 0; i < len; i++)
  {
    r = (r << 1) | (code & 1U);
    code >>= 1;
  }
  return r;
}

/* Canonical code from the code lengths; < 0: over-subscribed */
static int build(inflate_huffman_t *h, const uint8_t *lengths, uint32_t n)
{
  uint16_t offs[MAX_BITS + 2];
  uint16_t next[MAX_BITS + 1];

  memset(h->count, 0, sizeof(h->count));
  for (uint32_t i = 0; i < n; i++)
  {
    h->count[lengths[i]]++;
  }
  h->count[0] = 0;

  int left = 1;
  for (uint32_t len = 1; len <= MAX_BITS; len++)
  {
    left <<= 1;
    left -= h->count[len];
    if (left < 0)
    {
      return -1;
    }
  }

  offs[1] = 0;
  for (uint32_t len = 1; len <= MAX_BITS; len++)
  {
    offs[len + 1] = offs[len] + h->count[len];
  }
  for (uint32_t i = 0; i < n; i++)
  {
    if (lengths[i] != 0U)
    {
      h->symbol[offs[lengths[i]]++] = (uint16_t)i;
    }
  }

  /* Lookup table for the codes of up to FAST_BITS bits (stored bit-reversed,
   * since the input is read from bit 0 up) */
  memset(h->fast, 0, sizeof(h->fast));
  uint32_t code = 0;
  for (uint32_t len = 1; len <= MAX_BITS; len++)
  {
    code = (code + h->count[len - 1U]) << 1;
    next[len] = (uint16_t)code;
  }
  for (uint32_t i = 0; i < n; i++)
  {
    const uint32_t len = lengths[i];
    if (len == 0U || len > FAST_BITS)
    {
      continue;
    }
    const uint32_t r = reverse_bits(next[len]++, len);
    for (uint32_t k = r; k < (1U << FAST_BITS); k += 1U << len)
    {
      h->fast[k] = (uint16_t)(i | (len << FAST_BITS));
    }
  }
  return left;
}

/* Next symbol; < 0 if the code is not in the table */
static int decode(inflate_t *s, const inflate_huffman_t *h)
{
  need(s, FAST_BITS);
  const uint32_t e = h->fast[s->bits & ((1U << FAST_BITS) - 1U)];
  if (e != 0U)
  {
    const uint32_t len = e >> FAST_BITS;
    s->bits  >>= len;
    s->nbits  -= len;
    return (int)(e & ((1U << FAST_BITS) - 1U));
  }

  /* Longer code: bit by bit */
  int code = 0, first = 0, index = 0;
  for (uint32_t len = 1; len <= MAX_BITS; len++)
  {
    code |= (int)get_bits(s, 1);
    const int count = h->count[len];
    if (code - first < count)
    {
      return h->symbol[index + (code - first)];
    }
    index += count;
    first += count;
    first <<= 1;
    code  <<= 1;
  }
  return -1;
}

/* ---- blocks ---------------------------------------------------------------- */

static inflate_result_t stored(inflate_t *s)
{
  get_bits(s, s->nbits & 7U);   /* to a byte boundary */
  const uint32_t len  = get_bits(s, 16);
  const uint32_t nlen = get_bits(s, 16);
  if (len != (~nlen & 0xFFFFU))
  {
    return INFLATE_ERROR;
  }
  for (uint32_t i = 0; i < len && !s->failed && !s->stopped; i++)
  {
    put(s, (uint8_t)get_bits(s, 8));
  }
  return INFLATE_OK;
}

static inflate_result_t codes(inflate_t *s)
{
  for (;;)
  {
    const int sym = decode(s, &s->lit);
    if (s->failed || s->stopped)
    {
      return INFLATE_OK;   /* the caller tells which */
    }
    if (sym < 0)
    {
      return INFLATE_ERROR;
    }
    if (sym < 256)
    {
      put(s, (uint8_t)sym);
      continue;
    }
    if (sym == 256)
    {
      return INFLATE_OK;
    }

    const uint32_t li = (uint32_t)sym - 257U;
    if (li >= 29U)
    {
      return INFLATE_ERROR;
    }
    uint32_t len = k_len_base[li] + get_bits(s, k_len_extra[li]);
    const int ds = decode(s, &s->dist);
    if (ds < 0 || ds >= 30)
    {
      return INFLATE_ERROR;
    }
    const uint32_t dist = k_dist_base[ds] + get_bits(s, k_dist_extra[ds]);
    const uint32_t history = (s->total == INFLATE_WINDOW) ? INFLATE_WINDOW : s->wpos;
    if (dist > history)
    {
      return INFLATE_ERROR;
    }
    uint32_t from = (s->wpos - dist) & WINDOW_MASK;
    while (len-- != 0U)
    {
      put(s, s->window[from]);
      from = (from + 1U) & WINDOW_MASK;
    }
  }
}

static inflate_result_t fixed(inflate_t *s)
{
  uint8_t lengths[288];
  uint32_t i = 0;
  for (; i < 144U; i++) lengths[i] = 8;
  for (; i < 256U; i++) lengths[i] = 9;
  for (; i < 280U; i++) lengths[i] = 7;
  for (; i < 288U; i++) lengths[i] = 8;
  build(&s->lit, lengths, 288);
  memset(lengths, 5, 30);
  build(&s->dist, lengths, 30);
  return codes(s);
}

static inflate_result_t dynamic(inflate_t *s)
{
  static const uint8_t k_order[19] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
  uint8_t lengths[286 + 30];

  const uint32_t nlen  = get_bits(s, 5) + 257U;
  const uint32_t ndist = get_bits(s, 5) + 1U;
  const uint32_t ncode = get_bits(s, 4) + 4U;
  if (nlen > 286U || ndist > 30U)
  {
    return INFLATE_ERROR;
  }

  memset(lengths, 0, 19);
  for (uint32_t i = 0; i < ncode; i++)
  {
    lengths[k_order[i]] = (uint8_t)get_bits(s, 3);
  }
  if (build(&s->lit, lengths, 19) != 0)
  {
    return INFLATE_ERROR;   /* the code length code must be complete */
  }

  uint32_t i = 0;
  while (i < nlen + ndist)
  {
    const int sym = decode(s, &s->lit);
    if (sym < 0 || s->failed)
    {
      return INFLATE_ERROR;
    }
    if (sym < 16)
    {
      lengths[i++] = (uint8_t)sym;
      continue;
    }
    uint8_t value = 0;
    uint32_t repeat;
    if (sym == 16)
    {
      if (i == 0U)
      {
        return INFLATE_ERROR;
      }
      value  = lengths[i - 1U];
      repeat = 3U + get_bits(s, 2);
    }
    else if (sym == 17)
    {
      repeat = 3U + get_bits(s, 3);
    }
    else
    {
      repeat = 11U + get_bits(s, 7);
    }
    if (i + repeat > nlen + ndist)
    {
      return INFLATE_ERROR;
    }
    while (repeat-- != 0U)
    {
      lengths[i++] = value;
    }
  }
  if (lengths[256] == 0U || build(&s->lit, lengths, nlen) < 0 || build(&s->dist, &lengths[nlen], ndist) < 0)
  {
    return INFLATE_ERROR;
  }
  return codes(s);
}

inflate_result_t inflate_run(inflate_t *s, inflate_read_t read, inflate_write_t write, void *ctx)
{
  s->read     = read;
  s->write    = write;
  s->ctx      = ctx;
  s->in       = NULL;
  s->in_end   = NULL;
  s->bits     = 0;
  s->nbits    = 0;
  s->pad_bits = 0;
  s->failed   = false;
  s->wpos     = 0;
  s->total    = 0;
  s->stopped  = false;

  inflate_result_t res = INFLATE_OK;
  bool last = false;
  while (!last && res == INFLATE_OK && !s->failed && !s->stopped)
  {
    last = get_bits(s, 1) != 0U;
    switch (get_bits(s, 2))
    {
      case 0:  res = stored(s);  break;
      case 1:  res = fixed(s);   break;
      case 2:  res = dynamic(s); break;
      default: res = INFLATE_ERROR; break;
    }
  }
  if (res == INFLATE_OK && !s->stopped)
  {
    flush(s);
  }
  if (s->stopped)
  {
    return INFLATE_STOPPED;
  }
  if (res != INFLATE_OK)
  {
    return res;
  }
  /* Bits past the end of the input used: it ended too soon */
  return (s->failed || s->pad_bits > s->nbits) ? INFLATE_TRUNCATED : INFLATE_OK;
}
