/**
 * @file  test_mp3.c
 * @brief MP3 decoding through the decoder interface, with real MP3 files
 *        (tests/fixtures, made by make_fixtures.sh): stream properties, length
 *        (Xing header, estimate, background scan), decoded audio, seeking,
 *        tags and the cover location, damaged files.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "decoder.h"
#include "fakes.h"
#include "ff.h"

#ifndef TEST_FIXTURES_DIR
#error "TEST_FIXTURES_DIR must point to tests/fixtures"
#endif

#define MAX_FRAMES (44100U * 4U)

static int16_t s_pcm[2U * MAX_FRAMES];

static bool load(const char *name, const char *sd_path)
{
  char host[512];
  snprintf(host, sizeof(host), "%s/%s", TEST_FIXTURES_DIR, name);
  return fakefs_add_host_file(sd_path, host);
}

/* Decode everything into s_pcm; returns the number of stereo frames */
static uint32_t decode_all(void)
{
  uint32_t total = 0;
  uint32_t got;
  while (total < MAX_FRAMES && (got = decoder_read(&s_pcm[2U * total], 1152U)) > 0U)
  {
    total += got;
  }
  return total;
}

/* Tone frequency from the zero crossings of channel @p ch in frames [from, to) */
static double tone_hz(uint32_t from, uint32_t to, uint32_t ch, uint32_t rate)
{
  uint32_t crossings = 0;
  for (uint32_t i = from + 1U; i < to; i++)
  {
    const int16_t a = s_pcm[2U * (i - 1U) + ch];
    const int16_t b = s_pcm[2U * i + ch];
    if ((a < 0 && b >= 0) || (a >= 0 && b < 0))
    {
      crossings++;
    }
  }
  return (double)crossings * rate / (2.0 * (double)(to - from));
}

static int32_t peak(uint32_t from, uint32_t to)
{
  int32_t p = 0;
  for (uint32_t i = 2U * from; i < 2U * to; i++)
  {
    const int32_t v = abs(s_pcm[i]);
    p = (v > p) ? v : p;
  }
  return p;
}

/* Constant bitrate, no Xing header: estimated length, then the exact one */
static void test_cbr_stereo(void)
{
  CHECK(load("cbr_64k_stereo_3s.mp3", "/Music/cbr.mp3"));
  decoder_info_t info;
  CHECK(decoder_open("/Music/cbr.mp3", &info));
  CHECK_EQ_STR(info.format, "MP3");
  CHECK_EQ_INT(info.sample_rate, 44100);
  CHECK_EQ_INT(info.channels, 2);
  CHECK_EQ_INT(info.bitrate_kbps, 64);
  CHECK(!info.vbr);
  CHECK(!info.duration_exact);
  CHECK_NEAR(info.duration_s, 3, 1);

  /* The background scan measures the exact length */
  uint32_t exact = 0;
  bool done = false;
  for (int i = 0; i < 1000 && !done; i++)
  {
    done = decoder_poll(&exact);
  }
  CHECK(done);
  CHECK_EQ_INT(exact, 3);
  CHECK(!decoder_poll(&exact));     /* reported once */

  const uint32_t frames = decode_all();
  CHECK_NEAR(frames, 3U * 44100U, 3U * 1152U);   /* plus encoder delay / padding */
  CHECK_NEAR(tone_hz(frames / 3U, 2U * frames / 3U, 0, 44100), 1000.0, 10.0);
  CHECK_NEAR(tone_hz(frames / 3U, 2U * frames / 3U, 1, 44100), 1000.0, 10.0);
  CHECK(peak(frames / 3U, 2U * frames / 3U) > 2000);
  CHECK_EQ_INT(decoder_read(s_pcm, 1152), 0);    /* end of file */
  decoder_close();
}

/* Variable bitrate with a Xing header: exact length at once; mono to stereo */
static void test_vbr_mono_xing(void)
{
  CHECK(load("vbr_mono_2s.mp3", "/Music/vbr.mp3"));
  decoder_info_t info;
  CHECK(decoder_open("/Music/vbr.mp3", &info));
  CHECK_EQ_INT(info.channels, 1);
  CHECK(info.vbr);
  CHECK(info.duration_exact);
  CHECK_EQ_INT(info.duration_s, 2);
  CHECK(info.bitrate_kbps > 16U && info.bitrate_kbps < 160U);

  const uint32_t frames = decode_all();
  CHECK_NEAR(frames, 2U * 44100U, 3U * 1152U);
  CHECK_NEAR(tone_hz(frames / 4U, 3U * frames / 4U, 0, 44100), 440.0, 6.0);
  /* Mono is played on both channels */
  bool same = true;
  for (uint32_t i = 0; i < frames; i++)
  {
    same = same && (s_pcm[2U * i] == s_pcm[2U * i + 1U]);
  }
  CHECK(same);
  decoder_close();
}

/* MPEG-2 Layer III (half sample rate, 576 samples per frame) */
static void test_mpeg2_22k(void)
{
  CHECK(load("mpeg2_22k_mono_2s.mp3", "/Music/m2.mp3"));
  decoder_info_t info;
  CHECK(decoder_open("/Music/m2.mp3", &info));
  CHECK_EQ_INT(info.sample_rate, 22050);
  CHECK_EQ_INT(info.channels, 1);
  CHECK_NEAR(info.duration_s, 2, 1);

  const uint32_t frames = decode_all();
  CHECK_NEAR(frames, 2U * 22050U, 3U * 576U);
  CHECK_NEAR(tone_hz(frames / 4U, 3U * frames / 4U, 0, 22050), 500.0, 8.0);
  decoder_close();
}

static void test_seek(void)
{
  CHECK(load("cbr_64k_stereo_3s.mp3", "/Music/cbr.mp3"));
  decoder_info_t info;
  CHECK(decoder_open("/Music/cbr.mp3", &info));

  /* Before the scan: CBR estimate */
  uint32_t actual = 99;
  CHECK(decoder_seek(1, &actual));
  CHECK_NEAR(actual, 1, 1);
  uint32_t frames = decode_all();
  CHECK_NEAR(frames, 2U * 44100U, 3U * 1152U);           /* about 2 s left */
  CHECK_NEAR(tone_hz(1000, frames - 1000, 0, 44100), 1000.0, 10.0);

  /* After the scan: frame-accurate table */
  uint32_t exact;
  for (int i = 0; i < 1000 && !decoder_poll(&exact); i++)
  {
  }
  CHECK(decoder_seek(2, &actual));
  CHECK_EQ_INT(actual, 2);
  frames = decode_all();
  CHECK_NEAR(frames, 44100U, 3U * 1152U);                 /* about 1 s left */
  CHECK(peak(0, frames / 2U) > 2000);                      /* audio, not silence */

  /* Back to the start */
  CHECK(decoder_seek(0, &actual));
  CHECK_EQ_INT(actual, 0);
  CHECK_NEAR(decode_all(), 3U * 44100U, 3U * 1152U);
  decoder_close();
}

/* ID3v2.3 title and artist, and where the embedded JPEG is */
static void test_tags_and_cover(void)
{
  CHECK(load("tagged_cover.mp3", "/Music/tagged.mp3"));
  decoder_info_t info;
  CHECK(decoder_open("/Music/tagged.mp3", &info));
  CHECK_EQ_STR(info.title, "Test Title");
  CHECK_EQ_STR(info.artist, "Test Artist");
  CHECK_EQ_INT(info.art_format, 1);   /* JPEG */
  CHECK(info.art_size > 100U);

  /* The recorded location holds a whole JPEG: start and end markers */
  FIL f;
  uint8_t soi[2] = { 0 }, eoi[2] = { 0 };
  UINT got;
  CHECK_EQ_INT(f_open(&f, "/Music/tagged.mp3", FA_READ), FR_OK);
  f_lseek(&f, info.art_offset);
  f_read(&f, soi, 2, &got);
  f_lseek(&f, info.art_offset + info.art_size - 2U);
  f_read(&f, eoi, 2, &got);
  f_close(&f);
  CHECK(soi[0] == 0xFF && soi[1] == 0xD8);
  CHECK(eoi[0] == 0xFF && eoi[1] == 0xD9);

  /* The tag is skipped: the audio decodes */
  const uint32_t frames = decode_all();
  CHECK_NEAR(tone_hz(frames / 4U, 3U * frames / 4U, 0, 44100), 1000.0, 15.0);
  decoder_close();
}

static void test_damaged_files(void)
{
  /* Not MP3 at all */
  uint8_t junk[4096];
  srand(1234);
  for (size_t i = 0; i < sizeof(junk); i++)
  {
    junk[i] = (uint8_t)(rand() & 0x7F);    /* no 0xFF: no frame sync */
  }
  fakefs_add("/Music/junk.mp3", junk, sizeof(junk));
  decoder_info_t info;
  fakelog_reset();
  CHECK(!decoder_open("/Music/junk.mp3", &info));
  CHECK(fakelog_warnings() > 0U);

  /* Only an ID3 tag, no audio */
  static const uint8_t tag_only[20] = { 'I', 'D', '3', 3, 0, 0, 0, 0, 0, 10 };
  fakefs_add("/Music/tag.mp3", tag_only, sizeof(tag_only));
  CHECK(!decoder_open("/Music/tag.mp3", &info));

  /* A file cut in the middle decodes up to the cut */
  CHECK(load("cbr_64k_stereo_3s.mp3", "/Music/cut.mp3"));
  FIL f;
  static uint8_t half[16384];
  UINT got;
  f_open(&f, "/Music/cut.mp3", FA_READ);
  f_read(&f, half, sizeof(half), &got);
  f_close(&f);
  fakefs_add("/Music/cut.mp3", half, got);
  CHECK(decoder_open("/Music/cut.mp3", &info));
  const uint32_t frames = decode_all();
  CHECK(frames > 44100U && frames < 3U * 44100U);
  decoder_close();

  /* A read error while playing stops the track (with an error message) */
  CHECK(load("cbr_64k_stereo_3s.mp3", "/Music/err.mp3"));
  CHECK(decoder_open("/Music/err.mp3", &info));
  fakelog_reset();
  fakefs_fail_reads_after(4000);
  const uint32_t before_error = decode_all();
  CHECK(before_error < 3U * 44100U);
  CHECK(fakelog_errors() > 0U);
  fakefs_fail_reads_after(0);
  decoder_close();
}

int main(void)
{
  RUN(test_cbr_stereo);
  RUN(test_vbr_mono_xing);
  RUN(test_mpeg2_22k);
  RUN(test_seek);
  RUN(test_tags_and_cover);
  RUN(test_damaged_files);
  fakefs_reset();
  return CHECK_RESULT();
}
