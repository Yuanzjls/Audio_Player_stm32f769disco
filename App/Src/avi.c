/**
 * @file  avi.c
 * @brief AVI (RIFF) reader, see avi.h.
 *
 *   RIFF 'AVI '
 *     LIST 'hdrl'  avih (main header), LIST 'strl' per stream: strh + strf
 *     LIST 'movi'  chunks "NNdc" (video frame), "NNwb" (audio), maybe in LIST 'rec '
 *     idx1         16-byte entries: chunk id, flags, offset, size
 */
#include "avi.h"

#include <string.h>

#define FOURCC(a, b, c, d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define TWOCC(a, b)        ((uint16_t)((uint16_t)(a) | ((uint16_t)(b) << 8)))

#define ID_RIFF FOURCC('R', 'I', 'F', 'F')
#define ID_AVI  FOURCC('A', 'V', 'I', ' ')
#define ID_LIST FOURCC('L', 'I', 'S', 'T')
#define ID_HDRL FOURCC('h', 'd', 'r', 'l')
#define ID_STRL FOURCC('s', 't', 'r', 'l')
#define ID_MOVI FOURCC('m', 'o', 'v', 'i')
#define ID_REC  FOURCC('r', 'e', 'c', ' ')
#define ID_AVIH FOURCC('a', 'v', 'i', 'h')
#define ID_STRH FOURCC('s', 't', 'r', 'h')
#define ID_STRF FOURCC('s', 't', 'r', 'f')
#define ID_IDX1 FOURCC('i', 'd', 'x', '1')
#define ID_VIDS FOURCC('v', 'i', 'd', 's')
#define ID_AUDS FOURCC('a', 'u', 'd', 's')

#define MAX_CHUNKS     64U       /* top-level and header chunks visited */
#define IDX_BATCH      64U       /* index entries read at a time        */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool read_at(FIL *file, uint32_t offset, void *buf, UINT len)
{
  UINT got = 0;
  return f_lseek(file, offset) == FR_OK && f_read(file, buf, len, &got) == FR_OK && got == len;
}

static bool is_mjpeg(uint32_t fourcc)
{
  /* Compare without case: "MJPG", "mjpg" */
  return (fourcc | 0x20202020U) == (FOURCC('M', 'J', 'P', 'G') | 0x20202020U);
}

/* Stream number @p n as the two digits that start its chunk ids */
static uint16_t stream_id(uint32_t n)
{
  return TWOCC('0' + (n / 10U) % 10U, '0' + n % 10U);
}

/* One LIST 'strl': stream @p n's header and format */
static avi_result_t parse_strl(FIL *file, uint32_t pos, uint32_t end, uint32_t n, avi_info_t *info,
                               bool *have_video)
{
  uint32_t type = 0;
  uint32_t handler = 0;
  uint32_t scale = 0, rate = 0, length = 0;

  for (uint32_t i = 0; i < MAX_CHUNKS && pos + 8U <= end; i++)
  {
    uint8_t h[8];
    if (!read_at(file, pos, h, sizeof(h)))
    {
      return AVI_ERR_IO;
    }
    const uint32_t id   = rd32(&h[0]);
    const uint32_t size = rd32(&h[4]);
    const uint32_t body = pos + 8U;

    if (id == ID_STRH && size >= 36U)
    {
      uint8_t s[36];
      if (!read_at(file, body, s, sizeof(s)))
      {
        return AVI_ERR_IO;
      }
      type    = rd32(&s[0]);
      handler = rd32(&s[4]);
      scale   = rd32(&s[20]);
      rate    = rd32(&s[24]);
      length  = rd32(&s[32]);
    }
    else if (id == ID_STRF && type == ID_VIDS && size >= 20U && !*have_video)
    {
      uint8_t f[20];
      if (!read_at(file, body, f, sizeof(f)))
      {
        return AVI_ERR_IO;
      }
      if (is_mjpeg(rd32(&f[16])) || is_mjpeg(handler))
      {
        *have_video  = true;
        info->video_id = stream_id(n);
        info->width  = rd32(&f[4]);
        const int32_t height = (int32_t)rd32(&f[8]);
        info->height = (uint32_t)(height < 0 ? -height : height);
        if (scale != 0U && rate != 0U)
        {
          info->frame_us = (uint32_t)(((uint64_t)scale * 1000000U + rate / 2U) / rate);
        }
        if (length != 0U)
        {
          info->frames = length;
        }
      }
    }
    else if (id == ID_STRF && type == ID_AUDS && size >= 16U && !info->has_audio)
    {
      uint8_t f[16];
      if (!read_at(file, body, f, sizeof(f)))
      {
        return AVI_ERR_IO;
      }
      const uint16_t format = rd16(&f[0]);
      const uint16_t bits   = rd16(&f[14]);
      if (!(format == AVI_AUDIO_PCM && bits == 16U) && format != AVI_AUDIO_MP3)
      {
        return AVI_ERR_AUDIO_FORMAT;
      }
      info->has_audio         = true;
      info->audio_id          = stream_id(n);
      info->audio_format      = format;
      info->channels          = rd16(&f[2]);
      info->sample_rate       = rd32(&f[4]);
      info->audio_bytes_per_s = rd32(&f[8]);
      info->block_align       = rd16(&f[12]);
      info->bits              = bits;
      if (info->channels == 0U || info->channels > 2U || info->sample_rate == 0U)
      {
        return AVI_ERR_AUDIO_FORMAT;
      }
    }
    pos = body + size + (size & 1U);
  }
  return AVI_OK;
}

static avi_result_t parse_hdrl(FIL *file, uint32_t pos, uint32_t end, avi_info_t *info)
{
  uint32_t streams = 0;
  bool have_video = false;

  for (uint32_t i = 0; i < MAX_CHUNKS && pos + 8U <= end; i++)
  {
    uint8_t h[12];
    if (!read_at(file, pos, h, sizeof(h)))
    {
      return AVI_ERR_IO;
    }
    const uint32_t id   = rd32(&h[0]);
    const uint32_t size = rd32(&h[4]);
    const uint32_t body = pos + 8U;

    if (id == ID_AVIH && size >= 40U)
    {
      uint8_t a[40];
      if (!read_at(file, body, a, sizeof(a)))
      {
        return AVI_ERR_IO;
      }
      info->frame_us = rd32(&a[0]);
      info->frames   = rd32(&a[16]);
      info->width    = rd32(&a[32]);
      info->height   = rd32(&a[36]);
    }
    else if (id == ID_LIST && size >= 4U && rd32(&h[8]) == ID_STRL)
    {
      const avi_result_t r = parse_strl(file, body + 4U, body + size, streams++, info, &have_video);
      if (r != AVI_OK)
      {
        return r;
      }
    }
    pos = body + size + (size & 1U);
  }
  return have_video ? AVI_OK : AVI_ERR_NO_VIDEO;
}

avi_result_t avi_open(FIL *file, avi_info_t *info)
{
  memset(info, 0, sizeof(*info));

  uint8_t riff[12];
  if (!read_at(file, 0, riff, sizeof(riff)))
  {
    return AVI_ERR_IO;
  }
  if (rd32(&riff[0]) != ID_RIFF || rd32(&riff[8]) != ID_AVI)
  {
    return AVI_ERR_NOT_AVI;
  }
  const uint32_t file_end = (uint32_t)f_size(file);
  bool have_hdrl = false;
  uint32_t pos = 12;

  for (uint32_t i = 0; i < MAX_CHUNKS && pos + 8U <= file_end; i++)
  {
    uint8_t h[12];
    if (!read_at(file, pos, h, sizeof(h)))
    {
      break;
    }
    const uint32_t id   = rd32(&h[0]);
    uint32_t       size = rd32(&h[4]);
    const uint32_t body = pos + 8U;
    if (size > file_end - body)
    {
      size = file_end - body;          /* truncated file: up to its end */
    }

    if (id == ID_LIST && rd32(&h[8]) == ID_HDRL)
    {
      const avi_result_t r = parse_hdrl(file, body + 4U, body + size, info);
      if (r != AVI_OK)
      {
        return r;
      }
      have_hdrl = true;
    }
    else if (id == ID_LIST && rd32(&h[8]) == ID_MOVI)
    {
      info->movi_start = body + 4U;
      info->movi_end   = body + size;
    }
    else if (id == ID_IDX1)
    {
      info->idx1_offset  = body;
      info->idx1_entries = size / 16U;
    }
    pos = body + size + (size & 1U);
  }

  if (!have_hdrl)
  {
    return AVI_ERR_NOT_AVI;
  }
  if (info->movi_start == 0U)
  {
    return AVI_ERR_NOT_AVI;
  }
  if (info->frame_us == 0U)
  {
    info->frame_us = 40000U;          /* no frame rate: 25 fps */
  }
  return AVI_OK;
}

const char *avi_result_str(avi_result_t result)
{
  switch (result)
  {
    case AVI_OK:               return "ok";
    case AVI_ERR_IO:           return "read error";
    case AVI_ERR_NOT_AVI:      return "not an AVI file";
    case AVI_ERR_NO_VIDEO:     return "no MJPEG video";
    case AVI_ERR_AUDIO_FORMAT: return "audio is neither 16-bit PCM nor MP3";
    default:                   return "unknown error";
  }
}

static avi_chunk_type_t chunk_type(const avi_info_t *info, uint32_t id)
{
  const uint16_t stream = (uint16_t)(id & 0xFFFFU);
  const uint16_t kind   = (uint16_t)(id >> 16);
  if (stream == info->video_id && (kind == TWOCC('d', 'c') || kind == TWOCC('d', 'b')))
  {
    return AVI_CHUNK_VIDEO;
  }
  if (info->has_audio && stream == info->audio_id && kind == TWOCC('w', 'b'))
  {
    return AVI_CHUNK_AUDIO;
  }
  return AVI_CHUNK_OTHER;
}

bool avi_next_chunk(FIL *file, const avi_info_t *info, uint32_t *pos, avi_chunk_t *chunk)
{
  while (*pos + 8U <= info->movi_end)
  {
    uint8_t h[8];
    if (!read_at(file, *pos, h, sizeof(h)))
    {
      return false;
    }
    const uint32_t id   = rd32(&h[0]);
    const uint32_t size = rd32(&h[4]);
    const uint32_t body = *pos + 8U;

    if (id == ID_LIST)
    {
      /* "rec " groups are entered, other lists skipped */
      uint8_t list_type[4];
      if (!read_at(file, body, list_type, sizeof(list_type)))
      {
        return false;
      }
      *pos = (rd32(list_type) == ID_REC) ? body + 4U : body + size + (size & 1U);
      continue;
    }
    if (size > info->movi_end - body)
    {
      return false;                    /* corrupt or cut short */
    }
    *pos = body + size + (size & 1U);
    const avi_chunk_type_t type = chunk_type(info, id);
    if (type != AVI_CHUNK_OTHER)
    {
      chunk->type   = type;
      chunk->offset = body;
      chunk->size   = size;
      return true;
    }
  }
  return false;
}

bool avi_seek(FIL *file, const avi_info_t *info, uint32_t frame, uint32_t *pos, uint32_t *audio_bytes)
{
  if (info->idx1_entries == 0U)
  {
    return false;
  }

  uint8_t batch[IDX_BATCH * 16U];
  uint32_t base       = 0;
  bool     base_known = false;
  uint32_t video      = 0;
  uint32_t audio      = 0;
  uint32_t last_video = 0;
  bool     found_any  = false;

  for (uint32_t i = 0; i < info->idx1_entries; i += IDX_BATCH)
  {
    const uint32_t n = (info->idx1_entries - i < IDX_BATCH) ? info->idx1_entries - i : IDX_BATCH;
    if (!read_at(file, info->idx1_offset + i * 16U, batch, n * 16U))
    {
      return false;
    }
    for (uint32_t e = 0; e < n; e++)
    {
      const uint8_t *entry = &batch[e * 16U];
      const uint32_t id     = rd32(&entry[0]);
      const uint32_t offset = rd32(&entry[8]);
      const uint32_t size   = rd32(&entry[12]);
      if (!base_known)
      {
        /* Offsets count from the "movi" id (usual) or from the file start */
        base       = (offset < info->movi_start) ? info->movi_start - 4U : 0U;
        base_known = true;
      }
      const avi_chunk_type_t type = chunk_type(info, id);
      if (type == AVI_CHUNK_VIDEO)
      {
        last_video = base + offset;
        found_any  = true;
        if (video++ == frame)
        {
          *pos         = last_video;
          *audio_bytes = audio;
          return true;
        }
      }
      else if (type == AVI_CHUNK_AUDIO)
      {
        audio += size;
      }
    }
  }
  /* Past the end: the last frame */
  if (found_any)
  {
    *pos         = last_video;
    *audio_bytes = audio;
  }
  return found_any;
}
