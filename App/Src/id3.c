/**
 * @file  id3.c
 * @brief Minimal ID3 reader: only the title and artist text frames.
 *
 * Text is converted to UTF-8 for display. ID3 "ISO-8859-1" text is often
 * really GBK (written by Chinese software), so such text is decoded as GBK
 * when it forms valid GBK sequences and as Latin-1 otherwise.
 */
#include "id3.h"

#include <string.h>

#define ID3V2_HEADER_SIZE   10U
#define FRAME_TEXT_MAX      512U   /* bytes read from a text frame */

#define ENC_LATIN1   0U
#define ENC_UTF16    1U   /* with BOM */
#define ENC_UTF16BE  2U
#define ENC_UTF8     3U

/* ---- UTF-8 output ---------------------------------------------------------- */

typedef struct
{
  char  *buf;
  size_t len;
  size_t cap;
} utf8_out_t;

static void put_codepoint(utf8_out_t *o, uint32_t cp)
{
  uint8_t tmp[4];
  size_t n;

  if (cp < 0x80U)        { tmp[0] = (uint8_t)cp; n = 1; }
  else if (cp < 0x800U)  { tmp[0] = (uint8_t)(0xC0U | (cp >> 6)); tmp[1] = (uint8_t)(0x80U | (cp & 0x3FU)); n = 2; }
  else if (cp < 0x10000U)
  {
    tmp[0] = (uint8_t)(0xE0U | (cp >> 12)); tmp[1] = (uint8_t)(0x80U | ((cp >> 6) & 0x3FU));
    tmp[2] = (uint8_t)(0x80U | (cp & 0x3FU)); n = 3;
  }
  else
  {
    tmp[0] = (uint8_t)(0xF0U | (cp >> 18)); tmp[1] = (uint8_t)(0x80U | ((cp >> 12) & 0x3FU));
    tmp[2] = (uint8_t)(0x80U | ((cp >> 6) & 0x3FU)); tmp[3] = (uint8_t)(0x80U | (cp & 0x3FU)); n = 4;
  }
  /* Never write a partial character */
  if (o->len + n < o->cap)
  {
    memcpy(&o->buf[o->len], tmp, n);
    o->len += n;
  }
}

static void finish(utf8_out_t *o)
{
  /* Trim trailing spaces (ID3v1 pads with spaces or NULs) */
  while (o->len > 0U && o->buf[o->len - 1U] == ' ')
  {
    o->len--;
  }
  o->buf[o->len] = '\0';
}

/* ---- text decoding --------------------------------------------------------- */

static bool is_gbk(const uint8_t *s, size_t len)
{
  bool any_high = false;
  for (size_t i = 0; i < len && s[i] != 0U; i++)
  {
    if (s[i] < 0x80U)
    {
      continue;
    }
    any_high = true;
    if (s[i] < 0x81U || s[i] > 0xFEU || i + 1U >= len)
    {
      return false;
    }
    const uint8_t trail = s[i + 1U];
    if (trail < 0x40U || trail > 0xFEU || trail == 0x7FU ||
        ff_oem2uni((WCHAR)((s[i] << 8) | trail), 936) == 0U)
    {
      return false;
    }
    i++;
  }
  return any_high;
}

static void decode_8bit(const uint8_t *s, size_t len, utf8_out_t *o)
{
  const bool gbk = is_gbk(s, len);
  for (size_t i = 0; i < len && s[i] != 0U; i++)
  {
    if (gbk && s[i] >= 0x81U)
    {
      put_codepoint(o, ff_oem2uni((WCHAR)((s[i] << 8) | s[i + 1U]), 936));
      i++;
    }
    else
    {
      put_codepoint(o, s[i]);   /* Latin-1 maps 1:1 to U+0000..U+00FF */
    }
  }
}

static void decode_utf16(const uint8_t *s, size_t len, bool big_endian, utf8_out_t *o)
{
  for (size_t i = 0; i + 1U < len; i += 2U)
  {
    uint32_t u = big_endian ? ((uint32_t)s[i] << 8) | s[i + 1U] : ((uint32_t)s[i + 1U] << 8) | s[i];
    if (u == 0U)
    {
      break;
    }
    if (u >= 0xD800U && u < 0xDC00U && i + 3U < len)
    {
      const uint32_t lo = big_endian ? ((uint32_t)s[i + 2U] << 8) | s[i + 3U]
                                     : ((uint32_t)s[i + 3U] << 8) | s[i + 2U];
      if (lo >= 0xDC00U && lo < 0xE000U)
      {
        u = 0x10000U + ((u - 0xD800U) << 10) + (lo - 0xDC00U);
        i += 2U;
      }
    }
    put_codepoint(o, u);
  }
}

static void decode_text(uint8_t encoding, const uint8_t *s, size_t len, char *out, size_t out_size)
{
  utf8_out_t o = { .buf = out, .len = 0, .cap = out_size };

  switch (encoding)
  {
    case ENC_UTF16:
      if (len >= 2U && s[0] == 0xFEU && s[1] == 0xFFU)
      {
        decode_utf16(s + 2, len - 2U, true, &o);
      }
      else if (len >= 2U && s[0] == 0xFFU && s[1] == 0xFEU)
      {
        decode_utf16(s + 2, len - 2U, false, &o);
      }
      else
      {
        decode_utf16(s, len, false, &o);
      }
      break;
    case ENC_UTF16BE:
      decode_utf16(s, len, true, &o);
      break;
    case ENC_UTF8:
      for (size_t i = 0; i < len && s[i] != 0U && o.len + 1U < o.cap; i++)
      {
        o.buf[o.len++] = (char)s[i];
      }
      break;
    default:
      decode_8bit(s, len, &o);
      break;
  }
  finish(&o);
}

/* ---- ID3v2 ----------------------------------------------------------------- */

static uint32_t syncsafe32(const uint8_t *p)
{
  return ((uint32_t)(p[0] & 0x7FU) << 21) | ((uint32_t)(p[1] & 0x7FU) << 14) |
         ((uint32_t)(p[2] & 0x7FU) << 7)  |  (uint32_t)(p[3] & 0x7FU);
}

static uint32_t be32(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static bool read_at(FIL *file, FSIZE_t offset, void *buf, UINT len)
{
  UINT got = 0;
  return f_lseek(file, offset) == FR_OK && f_read(file, buf, len, &got) == FR_OK && got == len;
}

uint32_t id3v2_parse(FIL *file, id3_tags_t *tags)
{
  uint8_t hdr[ID3V2_HEADER_SIZE];

  if (!read_at(file, 0, hdr, sizeof(hdr)) || memcmp(hdr, "ID3", 3) != 0 ||
      hdr[3] < 2U || hdr[3] > 4U)
  {
    return 0U;
  }
  const uint8_t  version = hdr[3];
  const uint8_t  flags   = hdr[5];
  const uint32_t size    = syncsafe32(&hdr[6]);
  const uint32_t total   = ID3V2_HEADER_SIZE + size + ((flags & 0x10U) ? ID3V2_HEADER_SIZE : 0U);
  const uint32_t end     = ID3V2_HEADER_SIZE + size;

  uint32_t pos = ID3V2_HEADER_SIZE;

  /* Skip the extended header */
  if ((flags & 0x40U) != 0U && version >= 3U)
  {
    uint8_t ext[4];
    if (!read_at(file, pos, ext, sizeof(ext)))
    {
      return total;
    }
    pos += (version == 4U) ? syncsafe32(ext) : be32(ext) + 4U;
  }

  const uint32_t frame_hdr_size = (version == 2U) ? 6U : 10U;
  const char *id_title  = (version == 2U) ? "TT2" : "TIT2";
  const char *id_artist = (version == 2U) ? "TP1" : "TPE1";
  const size_t id_len   = (version == 2U) ? 3U : 4U;

  while (pos + frame_hdr_size <= end && (tags->title[0] == '\0' || tags->artist[0] == '\0'))
  {
    uint8_t fh[10];
    if (!read_at(file, pos, fh, frame_hdr_size) || fh[0] == 0U)
    {
      break;   /* padding reached */
    }
    uint32_t frame_size;
    if (version == 2U)
    {
      frame_size = ((uint32_t)fh[3] << 16) | ((uint32_t)fh[4] << 8) | fh[5];
    }
    else
    {
      frame_size = (version == 4U) ? syncsafe32(&fh[4]) : be32(&fh[4]);
    }
    const uint32_t body = pos + frame_hdr_size;
    if (frame_size == 0U || body + frame_size > end)
    {
      break;
    }

    char *dest = NULL;
    if (memcmp(fh, id_title, id_len) == 0)
    {
      dest = tags->title;
    }
    else if (memcmp(fh, id_artist, id_len) == 0)
    {
      dest = tags->artist;
    }
    /* Skip compressed / encrypted frames (2.3: flags 0x80/0x40; 2.4: 0x08/0x04,
     * and 0x01 = a data length field precedes the text) */
    const bool packed = (version == 3U && (fh[9] & 0xC0U) != 0U) ||
                        (version == 4U && (fh[9] & 0x0DU) != 0U);

    if (dest != NULL && dest[0] == '\0' && !packed && frame_size > 1U)
    {
      uint8_t text[FRAME_TEXT_MAX];
      const UINT n = (frame_size < sizeof(text)) ? (UINT)frame_size : (UINT)sizeof(text);
      if (read_at(file, body, text, n))
      {
        decode_text(text[0], &text[1], n - 1U, dest, ID3_TEXT_MAX);
      }
    }
    pos = body + frame_size;
  }
  return total;
}

/* ---- ID3v1 ----------------------------------------------------------------- */

bool id3v1_parse(FIL *file, id3_tags_t *tags)
{
  const FSIZE_t size = f_size(file);
  uint8_t tag[128];

  if (size < sizeof(tag) || !read_at(file, size - sizeof(tag), tag, sizeof(tag)) ||
      memcmp(tag, "TAG", 3) != 0)
  {
    return false;
  }
  if (tags->title[0] == '\0')
  {
    decode_text(ENC_LATIN1, &tag[3], 30U, tags->title, ID3_TEXT_MAX);
  }
  if (tags->artist[0] == '\0')
  {
    decode_text(ENC_LATIN1, &tag[33], 30U, tags->artist, ID3_TEXT_MAX);
  }
  return true;
}
