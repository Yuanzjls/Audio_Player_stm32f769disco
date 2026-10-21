/**
 * @file  test_id3.c
 * @brief ID3 tag reader: text frames in every encoding and tag version, the
 *        embedded picture's location, ID3v1, and malformed tags.
 *        Tags are built byte by byte in memory.
 */
#include <stdint.h>
#include <string.h>

#include "check.h"
#include "fakes.h"
#include "id3.h"

#define PATH "/Music/t.mp3"

/* ---- tag builder ------------------------------------------------------------ */

typedef struct
{
  uint8_t buf[8192];
  size_t  len;
  uint8_t version;     /* 2, 3 or 4 */
} tag_t;

static void put(tag_t *t, const void *data, size_t n)
{
  memcpy(&t->buf[t->len], data, n);
  t->len += n;
}

static void put_syncsafe(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)((v >> 21) & 0x7F);
  p[1] = (uint8_t)((v >> 14) & 0x7F);
  p[2] = (uint8_t)((v >> 7) & 0x7F);
  p[3] = (uint8_t)(v & 0x7F);
}

static void tag_begin(tag_t *t, uint8_t version, uint8_t flags)
{
  memset(t, 0, sizeof(*t));
  t->version = version;
  const uint8_t hdr[10] = { 'I', 'D', '3', version, 0, flags, 0, 0, 0, 0 };
  put(t, hdr, sizeof(hdr));
}

/* One frame: @p id, then @p body of @p n bytes */
static void frame(tag_t *t, const char *id, const void *body, size_t n)
{
  if (t->version == 2)
  {
    const uint8_t h[6] = { (uint8_t)id[0], (uint8_t)id[1], (uint8_t)id[2],
                           (uint8_t)(n >> 16), (uint8_t)(n >> 8), (uint8_t)n };
    put(t, h, sizeof(h));
  }
  else
  {
    uint8_t h[10] = { (uint8_t)id[0], (uint8_t)id[1], (uint8_t)id[2], (uint8_t)id[3], 0, 0, 0, 0, 0, 0 };
    if (t->version == 4)
    {
      put_syncsafe(&h[4], (uint32_t)n);
    }
    else
    {
      h[4] = (uint8_t)(n >> 24); h[5] = (uint8_t)(n >> 16); h[6] = (uint8_t)(n >> 8); h[7] = (uint8_t)n;
    }
    put(t, h, sizeof(h));
  }
  put(t, body, n);
}

/* Text frame: encoding byte + text bytes */
static void text_frame(tag_t *t, const char *id, uint8_t encoding, const void *text, size_t n)
{
  uint8_t body[512];
  body[0] = encoding;
  memcpy(&body[1], text, n);
  frame(t, id, body, n + 1U);
}

/* Write the tag size, append some "audio" and store the file. Returns the tag size. */
static size_t tag_store(tag_t *t, size_t padding)
{
  memset(&t->buf[t->len], 0, padding);   /* padding: zero bytes */
  t->len += padding;
  const size_t tag_size = t->len;
  put_syncsafe(&t->buf[6], (uint32_t)(tag_size - 10U));
  const uint8_t audio[64] = { 0xFF, 0xFB, 0x90, 0x00 };
  put(t, audio, sizeof(audio));
  fakefs_add(PATH, t->buf, t->len);
  return tag_size;
}

static uint32_t parse(id3_tags_t *tags)
{
  FIL f;
  memset(tags, 0, sizeof(*tags));
  if (f_open(&f, PATH, FA_READ) != FR_OK)
  {
    return 0xFFFFFFFFU;
  }
  const uint32_t r = id3v2_parse(&f, tags);
  f_close(&f);
  return r;
}

/* ---- text ------------------------------------------------------------------- */

static void test_v23_latin1(void)
{
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  text_frame(&t, "TIT2", 0, "Hello", 5);
  text_frame(&t, "TPE1", 0, "Caf\xE9", 4);   /* Latin-1 e acute */
  const size_t size = tag_store(&t, 20);

  CHECK_EQ_INT(parse(&tags), size);
  CHECK_EQ_STR(tags.title, "Hello");
  CHECK_EQ_STR(tags.artist, "Caf\xC3\xA9");     /* UTF-8 */
  CHECK_EQ_INT(tags.art_format, ID3_ART_NONE);
}

static void test_v23_utf16_bom(void)
{
  /* "你好" as UTF-16LE with a byte order mark, and a terminator */
  static const uint8_t text[] = { 0xFF, 0xFE, 0x60, 0x4F, 0x7D, 0x59, 0x00, 0x00 };
  /* "AB" as UTF-16BE with its byte order mark */
  static const uint8_t artist[] = { 0xFE, 0xFF, 0x00, 'A', 0x00, 'B' };
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  text_frame(&t, "TIT2", 1, text, sizeof(text));
  text_frame(&t, "TPE1", 1, artist, sizeof(artist));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "\xE4\xBD\xA0\xE5\xA5\xBD");
  CHECK_EQ_STR(tags.artist, "AB");
}

static void test_v24_utf8_and_utf16be(void)
{
  static const uint8_t artist_be[] = { 0x4E, 0x2D, 0x65, 0x87 };   /* "中文", UTF-16BE without BOM */
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 4, 0);
  text_frame(&t, "TIT2", 3, "Gr\xC3\xBC\xC3\x9F" "e", 7);         /* "Grüße", UTF-8 */
  text_frame(&t, "TPE1", 2, artist_be, sizeof(artist_be));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "Gr\xC3\xBC\xC3\x9F" "e");
  CHECK_EQ_STR(tags.artist, "\xE4\xB8\xAD\xE6\x96\x87");
}

static void test_v22_frames(void)
{
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 2, 0);
  text_frame(&t, "TT2", 0, "Old", 3);
  text_frame(&t, "TP1", 0, "Format", 6);
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "Old");
  CHECK_EQ_STR(tags.artist, "Format");
}

/* "Latin-1" text written by Chinese software is often GBK */
static void test_latin1_that_is_gbk(void)
{
  static const uint8_t gbk[] = { 0xC4, 0xE3, 0xBA, 0xC3 };          /* "你好" in GBK */
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  text_frame(&t, "TIT2", 0, gbk, sizeof(gbk));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "\xE4\xBD\xA0\xE5\xA5\xBD");
}

static void test_first_text_frame_wins(void)
{
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  text_frame(&t, "TIT2", 0, "First", 5);
  text_frame(&t, "TIT2", 0, "Second", 6);
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "First");
}

/* ---- pictures ---------------------------------------------------------------- */

static const uint8_t k_jpeg[] = { 0xFF, 0xD8, 0xFF, 0xE0, 'J', 'F', 'I', 'F', 0, 1, 2, 3, 0xFF, 0xD9 };
static const uint8_t k_png[]  = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, 1, 2, 3, 4 };

/* APIC body: encoding, MIME, picture type, description, data */
static size_t apic_body(uint8_t *body, uint8_t encoding, const char *mime, uint8_t type,
                        const void *desc, size_t desc_len, const void *data, size_t data_len)
{
  size_t n = 0;
  body[n++] = encoding;
  memcpy(&body[n], mime, strlen(mime) + 1U);
  n += strlen(mime) + 1U;
  body[n++] = type;
  memcpy(&body[n], desc, desc_len);
  n += desc_len;
  memcpy(&body[n], data, data_len);
  return n + data_len;
}

/* Offset of @p needle in the stored file, for checking where art_offset points */
static long find_in_tag(const tag_t *t, const uint8_t *needle, size_t n)
{
  for (size_t i = 0; i + n <= t->len; i++)
  {
    if (memcmp(&t->buf[i], needle, n) == 0)
    {
      return (long)i;
    }
  }
  return -1;
}

static void test_front_cover_preferred(void)
{
  uint8_t body[256];
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  text_frame(&t, "TIT2", 0, "With art", 8);
  /* First an "other" PNG picture, then the front cover (JPEG) */
  frame(&t, "APIC", body, apic_body(body, 0, "image/png", 0, "", 1, k_png, sizeof(k_png)));
  frame(&t, "APIC", body, apic_body(body, 0, "image/jpeg", 3, "Front", 6, k_jpeg, sizeof(k_jpeg)));
  tag_store(&t, 10);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "With art");
  CHECK_EQ_INT(tags.art_format, ID3_ART_JPEG);
  CHECK_EQ_INT(tags.art_offset, find_in_tag(&t, k_jpeg, sizeof(k_jpeg)));
  CHECK_EQ_INT(tags.art_size, sizeof(k_jpeg));
}

static void test_first_picture_kept_without_front(void)
{
  uint8_t body[256];
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  frame(&t, "APIC", body, apic_body(body, 0, "image/png", 0, "", 1, k_png, sizeof(k_png)));
  frame(&t, "APIC", body, apic_body(body, 0, "image/jpeg", 4, "", 1, k_jpeg, sizeof(k_jpeg)));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_INT(tags.art_format, ID3_ART_PNG);
  CHECK_EQ_INT(tags.art_offset, find_in_tag(&t, k_png, sizeof(k_png)));
}

/* The declared MIME type is wrong: the image signature decides */
static void test_picture_format_from_signature(void)
{
  uint8_t body[256];
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 4, 0);
  frame(&t, "APIC", body, apic_body(body, 0, "image/png", 3, "", 1, k_jpeg, sizeof(k_jpeg)));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_INT(tags.art_format, ID3_ART_JPEG);
}

/* A UTF-16 description ends with two zero bytes */
static void test_picture_utf16_description(void)
{
  static const uint8_t desc[] = { 0xFF, 0xFE, 'C', 0, 'o', 0, 'v', 0, 0, 0 };
  uint8_t body[256];
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  frame(&t, "APIC", body, apic_body(body, 1, "image/jpeg", 3, desc, sizeof(desc), k_jpeg, sizeof(k_jpeg)));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_INT(tags.art_format, ID3_ART_JPEG);
  CHECK_EQ_INT(tags.art_offset, find_in_tag(&t, k_jpeg, sizeof(k_jpeg)));
}

/* ID3v2.2 PIC: a 3-letter format instead of the MIME type */
static void test_v22_picture(void)
{
  uint8_t body[256];
  size_t n = 0;
  body[n++] = 0;
  memcpy(&body[n], "JPG", 3);
  n += 3;
  body[n++] = 3;
  body[n++] = 0;     /* empty description */
  memcpy(&body[n], k_jpeg, sizeof(k_jpeg));
  n += sizeof(k_jpeg);

  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 2, 0);
  frame(&t, "PIC", body, n);
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_INT(tags.art_format, ID3_ART_JPEG);
  CHECK_EQ_INT(tags.art_offset, find_in_tag(&t, k_jpeg, sizeof(k_jpeg)));
}

/* Unsynchronised tags alter the image bytes: no cover then, but the text is read */
static void test_unsynchronised_tag_has_no_cover(void)
{
  uint8_t body[256];
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0x80);
  text_frame(&t, "TIT2", 0, "Unsync", 6);
  frame(&t, "APIC", body, apic_body(body, 0, "image/jpeg", 3, "", 1, k_jpeg, sizeof(k_jpeg)));
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "Unsync");
  CHECK_EQ_INT(tags.art_format, ID3_ART_NONE);
}

/* ---- tag structure ------------------------------------------------------------ */

static void test_no_tag(void)
{
  const uint8_t audio[128] = { 0xFF, 0xFB, 0x90, 0x00 };
  id3_tags_t tags;
  fakefs_add(PATH, audio, sizeof(audio));
  CHECK_EQ_INT(parse(&tags), 0);
  CHECK_EQ_STR(tags.title, "");
  CHECK_EQ_INT(tags.art_format, ID3_ART_NONE);
}

/* The extended header (v2.3) is skipped */
static void test_extended_header(void)
{
  static const uint8_t ext[10] = { 0, 0, 0, 6, 0, 0, 0, 0, 0, 0 };   /* size 6 + 4 */
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0x40);
  put(&t, ext, sizeof(ext));
  text_frame(&t, "TIT2", 0, "Ext", 3);
  tag_store(&t, 0);

  parse(&tags);
  CHECK_EQ_STR(tags.title, "Ext");
}

/* A frame claiming more bytes than the tag holds ends the parsing safely */
static void test_oversized_frame(void)
{
  tag_t t;
  id3_tags_t tags;
  tag_begin(&t, 3, 0);
  text_frame(&t, "TIT2", 0, "Good", 4);
  const uint8_t bad[10] = { 'T', 'P', 'E', '1', 0x7F, 0xFF, 0xFF, 0xFF, 0, 0 };
  put(&t, bad, sizeof(bad));
  const size_t size = tag_store(&t, 0);

  CHECK_EQ_INT(parse(&tags), size);
  CHECK_EQ_STR(tags.title, "Good");
  CHECK_EQ_STR(tags.artist, "");
}

static void test_id3v1(void)
{
  uint8_t file[512];
  memset(file, 0, sizeof(file));
  file[0] = 0xFF;
  uint8_t *tag = &file[sizeof(file) - 128U];
  memcpy(tag, "TAG", 3);
  memcpy(&tag[3], "Old Title                     ", 30);    /* space padded */
  memcpy(&tag[33], "Old Artist", 10);                       /* zero padded */
  fakefs_add(PATH, file, sizeof(file));

  FIL f;
  id3_tags_t tags;
  memset(&tags, 0, sizeof(tags));
  f_open(&f, PATH, FA_READ);
  CHECK(id3v1_parse(&f, &tags));
  f_close(&f);
  CHECK_EQ_STR(tags.title, "Old Title");
  CHECK_EQ_STR(tags.artist, "Old Artist");
}

/* ID3v1 only fills in what ID3v2 did not provide */
static void test_id3v1_does_not_override(void)
{
  uint8_t file[256];
  memset(file, 0, sizeof(file));
  memcpy(&file[128], "TAG", 3);
  memcpy(&file[131], "V1 Title", 8);
  memcpy(&file[161], "V1 Artist", 9);
  fakefs_add(PATH, file, sizeof(file));

  FIL f;
  id3_tags_t tags;
  memset(&tags, 0, sizeof(tags));
  strcpy(tags.title, "V2 Title");
  f_open(&f, PATH, FA_READ);
  CHECK(id3v1_parse(&f, &tags));
  f_close(&f);
  CHECK_EQ_STR(tags.title, "V2 Title");
  CHECK_EQ_STR(tags.artist, "V1 Artist");
}

int main(void)
{
  RUN(test_v23_latin1);
  RUN(test_v23_utf16_bom);
  RUN(test_v24_utf8_and_utf16be);
  RUN(test_v22_frames);
  RUN(test_latin1_that_is_gbk);
  RUN(test_first_text_frame_wins);
  RUN(test_front_cover_preferred);
  RUN(test_first_picture_kept_without_front);
  RUN(test_picture_format_from_signature);
  RUN(test_picture_utf16_description);
  RUN(test_v22_picture);
  RUN(test_unsynchronised_tag_has_no_cover);
  RUN(test_no_tag);
  RUN(test_extended_header);
  RUN(test_oversized_frame);
  RUN(test_id3v1);
  RUN(test_id3v1_does_not_override);
  fakefs_reset();
  return CHECK_RESULT();
}
