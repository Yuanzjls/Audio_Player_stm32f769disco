/**
 * @file  test_avi.c
 * @brief AVI reader: stream formats, the chunks in file order, seeking with
 *        the index, and files it must reject. Uses the AVI files in tests/fixtures
 *        (MJPEG 96x64 with PCM audio, with MP3 audio, without audio).
 */
#include <stdlib.h>
#include <string.h>

#include "avi.h"
#include "check.h"
#include "fakes.h"
#include "ff.h"

#ifndef TEST_FIXTURES_DIR
#error "TEST_FIXTURES_DIR must point to tests/fixtures"
#endif

#define PATH "/Avi/t.avi"

static FIL s_file;

static bool open_fixture(const char *name, avi_info_t *info, avi_result_t *result)
{
  char host[512];
  snprintf(host, sizeof(host), "%s/%s", TEST_FIXTURES_DIR, name);
  if (!fakefs_add_host_file(PATH, host) || f_open(&s_file, PATH, FA_READ) != FR_OK)
  {
    return false;
  }
  *result = avi_open(&s_file, info);
  return true;
}

typedef struct
{
  uint32_t video, audio;
  uint32_t audio_bytes;
  uint32_t empty_video;       /* empty frames: "show the previous one again" */
  bool     all_jpeg;          /* every other video chunk starts with a JPEG start-of-image */
} counts_t;

static counts_t count_chunks(const avi_info_t *info, uint32_t pos)
{
  counts_t c = { 0, 0, 0, 0, true };
  avi_chunk_t chunk;
  while (avi_next_chunk(&s_file, info, &pos, &chunk))
  {
    if (chunk.type == AVI_CHUNK_VIDEO && chunk.size == 0U)
    {
      c.empty_video++;
      c.video++;
    }
    else if (chunk.type == AVI_CHUNK_VIDEO)
    {
      uint8_t soi[2] = { 0, 0 };
      UINT got;
      f_lseek(&s_file, chunk.offset);
      f_read(&s_file, soi, 2, &got);
      c.all_jpeg = c.all_jpeg && soi[0] == 0xFF && soi[1] == 0xD8;
      c.video++;
    }
    else
    {
      c.audio++;
      c.audio_bytes += chunk.size;
    }
  }
  return c;
}

static void test_pcm(void)
{
  avi_info_t info;
  avi_result_t r;
  CHECK(open_fixture("mjpeg_pcm_96x64.avi", &info, &r));
  CHECK_EQ_INT(r, AVI_OK);
  CHECK_EQ_INT(info.width, 96);
  CHECK_EQ_INT(info.height, 64);
  CHECK_EQ_INT(info.frame_us, 100000);       /* 10 fps */
  CHECK_EQ_INT(info.frames, 20);
  CHECK(info.has_audio);
  CHECK_EQ_INT(info.audio_format, AVI_AUDIO_PCM);
  CHECK_EQ_INT(info.channels, 1);
  CHECK_EQ_INT(info.sample_rate, 8000);
  CHECK_EQ_INT(info.bits, 16);
  CHECK_EQ_INT(info.block_align, 2);
  CHECK_EQ_INT(info.audio_bytes_per_s, 16000);
  CHECK(info.idx1_entries > 0U);
  CHECK(info.movi_start > 0U && info.movi_end > info.movi_start);

  const counts_t c = count_chunks(&info, info.movi_start);
  CHECK_EQ_INT(c.video, 20);
  CHECK(c.all_jpeg);
  CHECK_EQ_INT(c.audio_bytes, 2U * 16000U);  /* 2 s of 16-bit mono at 8 kHz */
  f_close(&s_file);
}

static void test_mp3(void)
{
  avi_info_t info;
  avi_result_t r;
  CHECK(open_fixture("mjpeg_mp3_96x64.avi", &info, &r));
  CHECK_EQ_INT(r, AVI_OK);
  CHECK_EQ_INT(info.frame_us, 83333);        /* 12 fps */
  CHECK(info.has_audio);
  CHECK_EQ_INT(info.audio_format, AVI_AUDIO_MP3);
  CHECK_EQ_INT(info.sample_rate, 22050);
  CHECK_EQ_INT(info.channels, 1);

  const counts_t c = count_chunks(&info, info.movi_start);
  CHECK_EQ_INT(c.video, info.frames);
  CHECK(c.all_jpeg);
  CHECK_EQ_INT(c.empty_video, 1);            /* FFmpeg repeats one frame to keep the timing */
  CHECK(c.audio > 0U);
  /* 32 kbit/s for about 2 s */
  CHECK_NEAR(c.audio_bytes, 8000, 1500);
  f_close(&s_file);
}

static void test_no_audio(void)
{
  avi_info_t info;
  avi_result_t r;
  CHECK(open_fixture("mjpeg_silent_96x64.avi", &info, &r));
  CHECK_EQ_INT(r, AVI_OK);
  CHECK(!info.has_audio);
  const counts_t c = count_chunks(&info, info.movi_start);
  CHECK_EQ_INT(c.video, 10);
  CHECK_EQ_INT(c.audio, 0);
  f_close(&s_file);
}

static void test_seek(void)
{
  avi_info_t info;
  avi_result_t r;
  CHECK(open_fixture("mjpeg_pcm_96x64.avi", &info, &r));

  uint32_t pos, audio;
  CHECK(avi_seek(&s_file, &info, 0, &pos, &audio));
  CHECK_EQ_INT(audio, 0);
  avi_chunk_t chunk;
  uint32_t p = pos;
  CHECK(avi_next_chunk(&s_file, &info, &p, &chunk));
  CHECK_EQ_INT(chunk.type, AVI_CHUNK_VIDEO);

  /* Frame 10 of 20 is 1 s in: the audio before it is about 1 s */
  CHECK(avi_seek(&s_file, &info, 10, &pos, &audio));
  CHECK_NEAR(audio, 16000, 1600);
  p = pos;
  CHECK(avi_next_chunk(&s_file, &info, &p, &chunk));
  CHECK_EQ_INT(chunk.type, AVI_CHUNK_VIDEO);
  /* From there on: the remaining 10 frames */
  CHECK_EQ_INT(count_chunks(&info, pos).video, 10);

  /* Past the end: the last frame */
  CHECK(avi_seek(&s_file, &info, 1000, &pos, &audio));
  CHECK_EQ_INT(count_chunks(&info, pos).video, 1);
  f_close(&s_file);
}

static void test_rejected(void)
{
  avi_info_t info;
  FIL f;

  /* Not RIFF / AVI at all */
  fakefs_add(PATH, "RIFF\x10\0\0\0WAVEfmt ", 16);
  f_open(&f, PATH, FA_READ);
  CHECK_EQ_INT(avi_open(&f, &info), AVI_ERR_NOT_AVI);
  f_close(&f);

  fakefs_add(PATH, "RI", 2);
  f_open(&f, PATH, FA_READ);
  CHECK_EQ_INT(avi_open(&f, &info), AVI_ERR_IO);
  f_close(&f);

  /* Cut in the middle of the "movi" list: the chunks up to the cut are read */
  char host[512];
  snprintf(host, sizeof(host), "%s/%s", TEST_FIXTURES_DIR, "mjpeg_pcm_96x64.avi");
  CHECK(fakefs_add_host_file(PATH, host));
  const uint8_t *data;
  size_t size;
  CHECK(fakefs_get(PATH, &data, &size));
  uint8_t *half = malloc(size / 2U);
  memcpy(half, data, size / 2U);
  fakefs_add(PATH, half, size / 2U);
  free(half);
  f_open(&s_file, PATH, FA_READ);
  CHECK_EQ_INT(avi_open(&s_file, &info), AVI_OK);
  CHECK_EQ_INT(info.idx1_entries, 0);       /* the index was cut off */
  const counts_t c = count_chunks(&info, info.movi_start);
  CHECK(c.video > 3U && c.video < 20U);
  uint32_t pos, audio;
  CHECK(!avi_seek(&s_file, &info, 5, &pos, &audio));
  f_close(&s_file);
}

int main(void)
{
  RUN(test_pcm);
  RUN(test_mp3);
  RUN(test_no_audio);
  RUN(test_seek);
  RUN(test_rejected);
  fakefs_reset();
  return CHECK_RESULT();
}
