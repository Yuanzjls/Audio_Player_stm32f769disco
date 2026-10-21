/**
 * @file  test_wav.c
 * @brief WAV header parsing and decoding through the decoder interface:
 *        formats, extra chunks, mono to stereo, seeking, errors.
 */
#include <stdint.h>
#include <string.h>

#include "check.h"
#include "decoder.h"
#include "fakes.h"
#include "wav.h"

#define PATH "/Music/t.wav"

typedef struct
{
  uint8_t buf[1 << 20];
  size_t  len;
} file_t;

static file_t s_file;

static void put(const void *data, size_t n)
{
  memcpy(&s_file.buf[s_file.len], data, n);
  s_file.len += n;
}

static void put16(uint16_t v) { const uint8_t b[2] = { (uint8_t)v, (uint8_t)(v >> 8) }; put(b, 2); }
static void put32(uint32_t v) { const uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) }; put(b, 4); }

typedef struct
{
  uint16_t format;          /* 1 = PCM, 3 = float, 0xFFFE = extensible */
  uint16_t sub_format;      /* for extensible */
  uint16_t channels;
  uint32_t sample_rate;
  uint16_t bits;
  bool     list_chunk;      /* an odd-sized LIST chunk before "data" */
  bool     data_first;      /* "data" before "fmt " (invalid) */
  bool     no_data;
} wav_spec_t;

/* Samples: frame f, channel c = f * 2 + c (distinct values to check the order) */
static void build_wav(const wav_spec_t *w, uint32_t frames)
{
  const uint16_t block   = (uint16_t)(w->channels * w->bits / 8U);
  const uint32_t data_sz = frames * block;
  s_file.len = 0;
  put("RIFF", 4);
  put32(0);                 /* RIFF size: not checked by the parser */
  put("WAVE", 4);

  if (!w->data_first)
  {
    const bool ext = (w->format == 0xFFFEU);
    put("fmt ", 4);
    put32(ext ? 40U : 16U);
    put16(w->format);
    put16(w->channels);
    put32(w->sample_rate);
    put32(w->sample_rate * block);
    put16(block);
    put16(w->bits);
    if (ext)
    {
      put16(22);            /* extension size */
      put16(w->bits);       /* valid bits */
      put32(3);             /* channel mask */
      put16(w->sub_format); /* first two bytes of the sub-format GUID */
      static const uint8_t guid_rest[14] = { 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00,
                                             0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 };
      put(guid_rest, sizeof(guid_rest));
    }
  }
  if (w->list_chunk)
  {
    put("LIST", 4);
    put32(5);
    put("INFOx", 5);
    put("\0", 1);           /* pad byte: chunks are word aligned */
  }
  if (!w->no_data)
  {
    put("data", 4);
    put32(data_sz);
    for (uint32_t f = 0; f < frames; f++)
    {
      for (uint16_t c = 0; c < w->channels; c++)
      {
        if (w->bits == 16U)
        {
          put16((uint16_t)(int16_t)(f * 2U + c));
        }
        else
        {
          const uint8_t b = (uint8_t)f;
          put(&b, 1);
        }
      }
    }
  }
  fakefs_add(PATH, s_file.buf, s_file.len);
}

static wav_result_t parse_only(wav_info_t *info)
{
  FIL f;
  f_open(&f, PATH, FA_READ);
  const wav_result_t r = wav_parse(&f, info);
  f_close(&f);
  return r;
}

static const wav_spec_t k_stereo = { .format = 1, .channels = 2, .sample_rate = 44100, .bits = 16 };

static void test_supported_extensions(void)
{
  CHECK(decoder_is_supported("song.wav"));
  CHECK(decoder_is_supported("SONG.WAV"));
  CHECK(decoder_is_supported("a.b.mp3"));
  CHECK(!decoder_is_supported("song.flac"));
  CHECK(!decoder_is_supported("wav"));
  CHECK(!decoder_is_supported("song.wav.txt"));
}

static void test_stereo_info_and_samples(void)
{
  build_wav(&k_stereo, 44100U * 2U);   /* 2 s */
  decoder_info_t info;
  CHECK(decoder_open(PATH, &info));
  CHECK_EQ_STR(info.format, "WAV");
  CHECK_EQ_INT(info.sample_rate, 44100);
  CHECK_EQ_INT(info.channels, 2);
  CHECK_EQ_INT(info.bitrate_kbps, 1411);
  CHECK_EQ_INT(info.duration_s, 2);
  CHECK(info.duration_exact);
  CHECK(!info.vbr);

  int16_t out[2 * 1000];
  CHECK_EQ_INT(decoder_read(out, 1000), 1000);
  CHECK_EQ_INT(out[0], 0);
  CHECK_EQ_INT(out[1], 1);
  CHECK_EQ_INT(out[2 * 999 + 1], 2 * 999 + 1);

  /* Read to the end: exactly the number of frames in the file */
  uint32_t total = 1000;
  uint32_t got;
  while ((got = decoder_read(out, 1000)) > 0U)
  {
    total += got;
  }
  CHECK_EQ_INT(total, 44100U * 2U);
  decoder_close();
}

static void test_mono_becomes_stereo(void)
{
  const wav_spec_t mono = { .format = 1, .channels = 1, .sample_rate = 22050, .bits = 16 };
  build_wav(&mono, 500);
  decoder_info_t info;
  CHECK(decoder_open(PATH, &info));
  CHECK_EQ_INT(info.channels, 1);

  int16_t out[2 * 500];
  CHECK_EQ_INT(decoder_read(out, 500), 500);
  for (int i = 0; i < 500; i++)
  {
    /* Mono sample i was written as i * 2: both output channels carry it */
    if (out[2 * i] != i * 2 || out[2 * i + 1] != i * 2)
    {
      CHECK_EQ_INT(out[2 * i], i * 2);
      CHECK_EQ_INT(out[2 * i + 1], i * 2);
      break;
    }
  }
  decoder_close();
}

static void test_extra_chunks_and_extensible(void)
{
  wav_spec_t w = k_stereo;
  w.list_chunk = true;
  build_wav(&w, 100);
  wav_info_t info;
  CHECK_EQ_INT(parse_only(&info), WAV_OK);
  CHECK_EQ_INT(info.data_size, 400);

  w = k_stereo;
  w.format     = 0xFFFE;
  w.sub_format = 1;      /* PCM */
  build_wav(&w, 100);
  CHECK_EQ_INT(parse_only(&info), WAV_OK);
  CHECK_EQ_INT(info.channels, 2);

  w.sub_format = 3;      /* IEEE float */
  build_wav(&w, 100);
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_NOT_PCM);
}

static void test_rejected_formats(void)
{
  wav_info_t info;
  wav_spec_t w = k_stereo;

  w.bits = 8;
  build_wav(&w, 100);
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_BITS);

  w = k_stereo;
  w.format = 3;
  build_wav(&w, 100);
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_NOT_PCM);

  w = k_stereo;
  w.channels = 6;
  build_wav(&w, 100);
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_CHANNELS);

  w = k_stereo;
  w.no_data = true;
  build_wav(&w, 100);
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_NO_DATA);

  w = k_stereo;
  w.data_first = true;
  build_wav(&w, 100);
  CHECK(parse_only(&info) != WAV_OK);

  const uint8_t junk[64] = "RIFX....WAVEfmt ";
  fakefs_add(PATH, junk, sizeof(junk));
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_NOT_WAVE);

  fakefs_add(PATH, "RIFF", 4);
  CHECK_EQ_INT(parse_only(&info), WAV_ERR_IO);

  /* A rejected file is not opened, with a warning */
  fakelog_reset();
  decoder_info_t dinfo;
  CHECK(!decoder_open(PATH, &dinfo));
  CHECK(fakelog_warnings() > 0U);
}

static void test_seek(void)
{
  build_wav(&k_stereo, 44100U * 3U);
  decoder_info_t info;
  CHECK(decoder_open(PATH, &info));

  uint32_t actual = 99;
  CHECK(decoder_seek(2, &actual));
  CHECK_EQ_INT(actual, 2);
  int16_t out[2 * 10];
  CHECK_EQ_INT(decoder_read(out, 10), 10);
  CHECK_EQ_INT((uint16_t)out[0], (uint16_t)(44100U * 2U * 2U));   /* frame 88200, left */

  /* Past the end: the last frame */
  CHECK(decoder_seek(10, &actual));
  CHECK(decoder_read(out, 10) <= 1U);
  decoder_close();
}

static void test_read_error(void)
{
  build_wav(&k_stereo, 44100);
  decoder_info_t info;
  CHECK(decoder_open(PATH, &info));
  fakelog_reset();
  fakefs_fail_reads_after(1000);

  int16_t out[2 * 1024];
  uint32_t total = 0;
  for (int i = 0; i < 100; i++)
  {
    total += decoder_read(out, 1024);
  }
  CHECK(total < 44100U);            /* stopped at the error ... */
  CHECK_EQ_INT(fakelog_errors(), 1); /* ... and said so once */
  CHECK_EQ_INT(decoder_read(out, 1024), 0);
  decoder_close();
  fakefs_fail_reads_after(0);
}

static void test_missing_file(void)
{
  decoder_info_t info;
  fakelog_reset();
  CHECK(!decoder_open("/Music/none.wav", &info));
  CHECK(fakelog_warnings() > 0U);
  int16_t out[4];
  CHECK_EQ_INT(decoder_read(out, 2), 0);   /* nothing open */
}

int main(void)
{
  RUN(test_supported_extensions);
  RUN(test_stereo_info_and_samples);
  RUN(test_mono_becomes_stereo);
  RUN(test_extra_chunks_and_extensible);
  RUN(test_rejected_formats);
  RUN(test_seek);
  RUN(test_read_error);
  RUN(test_missing_file);
  fakefs_reset();
  return CHECK_RESULT();
}
