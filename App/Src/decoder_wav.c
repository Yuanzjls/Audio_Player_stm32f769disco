/**
 * @file  decoder_wav.c
 * @brief 16-bit PCM WAV decoder (header parsing in wav.c).
 */
#include <stdio.h>
#include <string.h>

#include "decoder_priv.h"
#include "log.h"
#include "wav.h"

static FIL       *s_file;
static wav_info_t s_wav;
static uint32_t   s_remaining;   /* bytes of sample data not yet read */

static bool wav_open(FIL *file, const char *path, decoder_info_t *info)
{
  const wav_result_t res = wav_parse(file, &s_wav);
  if (res != WAV_OK)
  {
    LOG_WARN("wav: %s: %s", path, wav_result_str(res));
    return false;
  }
  s_file      = file;
  s_remaining = s_wav.data_size;

  info->format         = "WAV";
  info->sample_rate    = s_wav.sample_rate;
  info->channels       = (uint8_t)s_wav.channels;
  info->bitrate_kbps   = s_wav.byte_rate * 8U / 1000U;
  info->vbr            = false;
  info->duration_s     = wav_duration_s(&s_wav);
  info->duration_exact = true;
  return true;
}

static uint32_t wav_read(int16_t *out, uint32_t frames)
{
  /* Mono is read into the first half of @p out and expanded in place */
  uint32_t want = frames * s_wav.block_align;
  if (want > s_remaining)
  {
    want = s_remaining;
  }
  if (want == 0U)
  {
    return 0U;
  }

  UINT got = 0;
  if (f_read(s_file, out, want, &got) != FR_OK)
  {
    LOG_ERROR("wav: read error, skipping rest of track");
    s_remaining = 0;
    return 0U;
  }
  got -= got % s_wav.block_align;
  s_remaining = (got == 0U) ? 0U : s_remaining - got;

  const uint32_t read_frames = got / s_wav.block_align;
  if (s_wav.channels == 1U)
  {
    /* Walk backwards so samples are not overwritten before being copied */
    for (int32_t i = (int32_t)read_frames - 1; i >= 0; i--)
    {
      const int16_t sample = out[i];
      out[2 * i]     = sample;
      out[2 * i + 1] = sample;
    }
  }
  return read_frames;
}

static bool wav_seek(uint32_t position_s, uint32_t *actual_s)
{
  uint32_t offset = position_s * s_wav.byte_rate;
  if (offset >= s_wav.data_size)
  {
    offset = s_wav.data_size - s_wav.block_align;
  }
  offset -= offset % s_wav.block_align;

  if (f_lseek(s_file, s_wav.data_offset + offset) != FR_OK)
  {
    return false;
  }
  s_remaining = s_wav.data_size - offset;
  *actual_s   = offset / s_wav.byte_rate;
  return true;
}

static void wav_close(void)
{
  s_file = NULL;
}

const decoder_ops_t decoder_wav_ops =
{
  .extension = ".wav",
  .open      = wav_open,
  .read      = wav_read,
  .seek      = wav_seek,
  .poll      = NULL,
  .close     = wav_close,
};
