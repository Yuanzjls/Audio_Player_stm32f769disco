/**
 * @file  wav.c
 * @brief RIFF/WAVE header parser.
 *
 * Walks the chunk list instead of assuming a fixed 44-byte header, so files
 * with LIST/INFO, "fact" or other extra chunks are handled correctly.
 */
#include "wav.h"

#include <string.h>

#define FOURCC(a, b, c, d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

#define WAVE_FORMAT_PCM        0x0001U
#define WAVE_FORMAT_EXTENSIBLE 0xFFFEU

/* Upper bound on chunks visited, protects against corrupt files */
#define MAX_CHUNKS 32

typedef struct
{
  uint32_t id;
  uint32_t size;
} chunk_header_t;

static uint16_t rd16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int read_exact(FIL *file, void *buf, UINT len)
{
  UINT got = 0;
  return f_read(file, buf, len, &got) == FR_OK && got == len;
}

static int read_chunk_header(FIL *file, chunk_header_t *hdr)
{
  uint8_t raw[8];
  if (!read_exact(file, raw, sizeof(raw)))
  {
    return 0;
  }
  hdr->id   = rd32(&raw[0]);
  hdr->size = rd32(&raw[4]);
  return 1;
}

static wav_result_t parse_fmt(FIL *file, uint32_t size, wav_info_t *info)
{
  uint8_t fmt[40];

  if (size < 16U)
  {
    return WAV_ERR_NO_FORMAT;
  }
  const UINT to_read = (size < sizeof(fmt)) ? (UINT)size : (UINT)sizeof(fmt);
  if (!read_exact(file, fmt, to_read))
  {
    return WAV_ERR_IO;
  }

  uint16_t format = rd16(&fmt[0]);
  info->channels    = rd16(&fmt[2]);
  info->sample_rate = rd32(&fmt[4]);
  info->byte_rate   = rd32(&fmt[8]);
  info->block_align = rd16(&fmt[12]);
  const uint16_t bits = rd16(&fmt[14]);

  /* WAVE_FORMAT_EXTENSIBLE: the real format is the first 2 bytes of the sub-format GUID */
  if (format == WAVE_FORMAT_EXTENSIBLE && to_read >= 26U)
  {
    format = rd16(&fmt[24]);
  }

  if (format != WAVE_FORMAT_PCM)
  {
    return WAV_ERR_NOT_PCM;
  }
  if (bits != 16U)
  {
    return WAV_ERR_BITS;
  }
  if (info->channels != 1U && info->channels != 2U)
  {
    return WAV_ERR_CHANNELS;
  }
  if (info->sample_rate == 0U || info->block_align != info->channels * 2U)
  {
    return WAV_ERR_NO_FORMAT;
  }
  /* Some encoders write a wrong byte rate; derive it */
  info->byte_rate = info->sample_rate * info->block_align;
  return WAV_OK;
}

wav_result_t wav_parse(FIL *file, wav_info_t *info)
{
  uint8_t riff[12];
  int have_fmt = 0;

  memset(info, 0, sizeof(*info));

  if (f_lseek(file, 0) != FR_OK || !read_exact(file, riff, sizeof(riff)))
  {
    return WAV_ERR_IO;
  }
  if (rd32(&riff[0]) != FOURCC('R', 'I', 'F', 'F') || rd32(&riff[8]) != FOURCC('W', 'A', 'V', 'E'))
  {
    return WAV_ERR_NOT_WAVE;
  }

  for (int i = 0; i < MAX_CHUNKS; i++)
  {
    chunk_header_t hdr;
    if (!read_chunk_header(file, &hdr))
    {
      break;
    }
    const FSIZE_t body = f_tell(file);

    if (hdr.id == FOURCC('f', 'm', 't', ' '))
    {
      const wav_result_t res = parse_fmt(file, hdr.size, info);
      if (res != WAV_OK)
      {
        return res;
      }
      have_fmt = 1;
    }
    else if (hdr.id == FOURCC('d', 'a', 't', 'a'))
    {
      if (!have_fmt)
      {
        return WAV_ERR_NO_FORMAT;
      }
      /* Clamp to the real file size (truncated downloads, streaming writers) */
      const FSIZE_t available = f_size(file) - body;
      uint32_t size = hdr.size;
      if (size > available)
      {
        size = (uint32_t)available;
      }
      size -= size % info->block_align;
      if (size == 0U)
      {
        return WAV_ERR_NO_DATA;
      }
      info->data_offset = (uint32_t)body;
      info->data_size   = size;
      return (f_lseek(file, body) == FR_OK) ? WAV_OK : WAV_ERR_IO;
    }

    /* Chunks are word aligned: skip the body plus a pad byte if the size is odd */
    if (f_lseek(file, body + hdr.size + (hdr.size & 1U)) != FR_OK || f_tell(file) >= f_size(file))
    {
      break;
    }
  }

  return have_fmt ? WAV_ERR_NO_DATA : WAV_ERR_NO_FORMAT;
}

const char *wav_result_str(wav_result_t result)
{
  switch (result)
  {
    case WAV_OK:            return "ok";
    case WAV_ERR_IO:        return "read error";
    case WAV_ERR_NOT_WAVE:  return "not a WAV file";
    case WAV_ERR_NO_FORMAT: return "bad format chunk";
    case WAV_ERR_NOT_PCM:   return "not PCM (compressed or float)";
    case WAV_ERR_BITS:      return "only 16-bit samples are supported";
    case WAV_ERR_CHANNELS:  return "only mono/stereo are supported";
    case WAV_ERR_NO_DATA:   return "no audio data";
    default:                return "unknown error";
  }
}

uint32_t wav_duration_s(const wav_info_t *info)
{
  return (info->byte_rate != 0U) ? info->data_size / info->byte_rate : 0U;
}
