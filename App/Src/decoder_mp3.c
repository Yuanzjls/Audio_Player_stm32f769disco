/**
 * @file  decoder_mp3.c
 * @brief MPEG-1/2/2.5 Layer III decoder on top of minimp3.
 *
 * - Compressed data is streamed through a 16 KB buffer; minimp3 decodes one
 *   frame at a time (CBR and VBR alike: every frame carries its own bitrate).
 * - Length: from the Xing/Info or VBRI header when present (exact). Otherwise
 *   it is estimated from the first frame's bitrate, and a background scan of
 *   the frame headers (decoder_poll) measures the exact length while playing.
 * - Seeking: through a frame-accurate seek table (from the background scan or
 *   a VBRI table), else the Xing table of contents, else a CBR estimate.
 *   Exact seeks start two frames early and drop them, so the decoder's bit
 *   reservoir is filled and the target frame decodes cleanly.
 */
#include <stdio.h>
#include <string.h>

#include "decoder_priv.h"
#include "id3.h"
#include "log.h"
#include "minimp3.h"

#define IN_BUF_SIZE        16384U
#define REFILL_BELOW       8192U    /* keep enough data for minimp3's frame sync */
#define SEEK_TABLE_MAX     1024U
#define SEEK_STEP_INITIAL  8U       /* frames per seek table entry (doubles when full) */
#define SEEK_WARMUP_FRAMES 2U
#define SEEK_MAX_DISCARD_FRAMES 32U
#define SCAN_FRAMES_PER_POLL 256U
#define SCAN_RESYNC_LIMIT  65536U   /* give up the scan after this much garbage */

typedef struct
{
  uint32_t hz;
  uint32_t bitrate_kbps;
  uint32_t frame_len;
  uint32_t spf;          /* samples per frame and channel */
  uint8_t  channels;
} frame_header_t;

static struct
{
  FIL     *file;
  mp3dec_t dec;

  /* compressed input */
  uint8_t  in[IN_BUF_SIZE];
  uint32_t in_len;
  uint32_t in_pos;
  bool     in_eof;
  FSIZE_t  audio_end;            /* first byte after the audio (trailing tags excluded) */
  FSIZE_t  first_frame;          /* first audio frame (after a Xing/VBRI frame)        */

  /* decoded output */
  mp3d_sample_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  uint32_t pcm_frames;
  uint32_t pcm_pos;
  uint8_t  pcm_channels;
  uint32_t discard;              /* samples per channel to drop (exact seek) */

  /* stream properties */
  uint32_t hz;
  uint32_t spf;
  uint32_t bitrate_kbps;         /* first frame, or average once known */
  uint32_t total_frames;         /* 0 = unknown */
  uint32_t vbr_bytes;            /* audio bytes from the Xing header */
  uint8_t  toc[100];
  bool     has_toc;

  /* seek table: entry i = file offset of audio frame i * seek_step */
  uint32_t seek_off[SEEK_TABLE_MAX];
  uint32_t seek_step;
  uint32_t seek_count;
  bool     seek_complete;

  /* background header scan */
  FIL      scan_file;
  bool     scanning;
  FSIZE_t  scan_pos;
  uint32_t scan_frames;
  uint32_t scan_garbage;
  uint64_t scan_bytes;
} m;

/* ---- helpers --------------------------------------------------------------- */

static uint32_t be32(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static uint16_t be16(const uint8_t *p)
{
  return (uint16_t)((p[0] << 8) | p[1]);
}

static bool read_at(FIL *file, FSIZE_t offset, void *buf, UINT len)
{
  UINT got = 0;
  return f_lseek(file, offset) == FR_OK && f_read(file, buf, len, &got) == FR_OK && got == len;
}

/* Layer III frame header (free-format frames are rejected) */
static bool parse_header(const uint8_t *h, frame_header_t *out)
{
  static const uint16_t k_kbps_v1[15] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320};
  static const uint16_t k_kbps_v2[15] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160};
  static const uint32_t k_hz[3]       = {44100, 48000, 32000};

  if (h[0] != 0xFFU || (h[1] & 0xE0U) != 0xE0U)
  {
    return false;
  }
  const uint8_t version = (h[1] >> 3) & 3U;   /* 3: MPEG1, 2: MPEG2, 0: MPEG2.5 */
  const uint8_t layer   = (h[1] >> 1) & 3U;   /* 1: Layer III */
  const uint8_t br_idx  = h[2] >> 4;
  const uint8_t sr_idx  = (h[2] >> 2) & 3U;
  if (version == 1U || layer != 1U || br_idx == 0U || br_idx == 15U || sr_idx == 3U)
  {
    return false;
  }
  const bool mpeg1 = (version == 3U);
  out->hz           = k_hz[sr_idx] >> (mpeg1 ? 0 : (version == 2U ? 1 : 2));
  out->bitrate_kbps = (mpeg1 ? k_kbps_v1 : k_kbps_v2)[br_idx];
  out->spf          = mpeg1 ? 1152U : 576U;
  out->frame_len    = (mpeg1 ? 144000U : 72000U) * out->bitrate_kbps / out->hz + ((h[2] >> 1) & 1U);
  out->channels     = ((h[3] >> 6) == 3U) ? 1U : 2U;
  return true;
}

static void seek_table_add(uint32_t frame_index, uint32_t offset)
{
  if (frame_index % m.seek_step != 0U)
  {
    return;
  }
  if (m.seek_count == SEEK_TABLE_MAX)
  {
    /* Full: keep every other entry and halve the resolution */
    for (uint32_t i = 0; i < SEEK_TABLE_MAX / 2U; i++)
    {
      m.seek_off[i] = m.seek_off[2U * i];
    }
    m.seek_count = SEEK_TABLE_MAX / 2U;
    m.seek_step *= 2U;
    if (frame_index % m.seek_step != 0U)
    {
      return;
    }
  }
  m.seek_off[m.seek_count++] = offset;
}

/* Xing/Info or VBRI header in the first frame. Returns true if it is one
 * (that frame then carries no audio). */
static bool parse_vbr_header(const uint8_t *f, uint32_t len, const frame_header_t *h, FSIZE_t frame_pos,
                             bool *vbr)
{
  const bool mpeg1  = (h->spf == 1152U);
  const uint32_t side = mpeg1 ? (h->channels == 1U ? 17U : 32U) : (h->channels == 1U ? 9U : 17U);
  const uint32_t x = 4U + side;

  if (x + 8U <= len && (memcmp(&f[x], "Xing", 4) == 0 || memcmp(&f[x], "Info", 4) == 0))
  {
    *vbr = (memcmp(&f[x], "Xing", 4) == 0);
    const uint32_t flags = be32(&f[x + 4U]);
    uint32_t p = x + 8U;
    if ((flags & 1U) != 0U && p + 4U <= len) { m.total_frames = be32(&f[p]); p += 4U; }
    if ((flags & 2U) != 0U && p + 4U <= len) { m.vbr_bytes    = be32(&f[p]); p += 4U; }
    if ((flags & 4U) != 0U && p + 100U <= len)
    {
      memcpy(m.toc, &f[p], sizeof(m.toc));
      m.has_toc = true;
    }
    return true;
  }

  const uint32_t v = 4U + 32U;
  if (v + 26U <= len && memcmp(&f[v], "VBRI", 4) == 0)
  {
    *vbr = true;
    m.vbr_bytes    = be32(&f[v + 10U]);
    m.total_frames = be32(&f[v + 14U]);
    const uint32_t entries   = be16(&f[v + 18U]);
    const uint32_t scale     = be16(&f[v + 20U]);
    const uint32_t ent_size  = be16(&f[v + 22U]);
    const uint32_t per_entry = be16(&f[v + 24U]);

    /* Table of segment sizes -> seek table of absolute offsets */
    if (ent_size >= 1U && ent_size <= 4U && per_entry > 0U && entries <= SEEK_TABLE_MAX - 1U &&
        v + 26U + entries * ent_size <= len)
    {
      uint32_t off = (uint32_t)frame_pos;
      m.seek_step  = per_entry;
      m.seek_count = 0;
      m.seek_off[m.seek_count++] = off;
      for (uint32_t i = 0; i < entries; i++)
      {
        uint32_t size = 0;
        for (uint32_t b = 0; b < ent_size; b++)
        {
          size = (size << 8) | f[v + 26U + i * ent_size + b];
        }
        off += size * scale;
        m.seek_off[m.seek_count++] = off;
      }
      m.seek_complete = true;
    }
    return true;
  }
  return false;
}

/* ---- input / decoding ------------------------------------------------------ */

static void refill(void)
{
  if (m.in_pos > 0U)
  {
    memmove(m.in, &m.in[m.in_pos], m.in_len - m.in_pos);
    m.in_len -= m.in_pos;
    m.in_pos  = 0;
  }
  const FSIZE_t pos = f_tell(m.file);
  if (pos >= m.audio_end)
  {
    m.in_eof = true;
    return;
  }
  UINT want = IN_BUF_SIZE - m.in_len;
  if (want > m.audio_end - pos)
  {
    want = (UINT)(m.audio_end - pos);
  }
  UINT got = 0;
  if (f_read(m.file, &m.in[m.in_len], want, &got) != FR_OK)
  {
    LOG_ERROR("mp3: read error, skipping rest of track");
    m.in_eof = true;
    return;
  }
  m.in_len += got;
  if (got < want || pos + got >= m.audio_end)
  {
    m.in_eof = true;
  }
}

/* Decode the next frame into m.pcm. Returns samples per channel, 0 at the end. */
static uint32_t decode_next(void)
{
  for (;;)
  {
    if (!m.in_eof && m.in_len - m.in_pos < REFILL_BELOW)
    {
      refill();
    }
    const uint32_t avail = m.in_len - m.in_pos;
    if (avail == 0U)
    {
      return 0U;
    }

    mp3dec_frame_info_t fi;
    const int samples = mp3dec_decode_frame(&m.dec, &m.in[m.in_pos], (int)avail, m.pcm, &fi);
    if (fi.frame_bytes == 0)
    {
      if (m.in_eof)
      {
        return 0U;   /* only a partial frame or garbage left */
      }
      /* No frame in a full buffer: drop it, keeping a few bytes for a header split across reads */
      m.in_pos = (avail > 3U) ? m.in_len - 3U : m.in_len;
      continue;
    }
    m.in_pos += (uint32_t)fi.frame_bytes;
    if (samples > 0 && (uint32_t)fi.hz == m.hz)
    {
      m.pcm_channels = (uint8_t)fi.channels;
      return (uint32_t)samples;
    }
    /* Skipped data, a frame needing more bit reservoir, or a stray rate change */
  }
}

static void restart_at(FSIZE_t offset, uint32_t discard_samples)
{
  f_lseek(m.file, offset);
  m.in_len = m.in_pos = 0;
  m.in_eof = false;
  m.pcm_frames = m.pcm_pos = 0;
  m.discard = discard_samples;
  mp3dec_init(&m.dec);
}

/* ---- decoder operations ---------------------------------------------------- */

static uint32_t duration_s(void)
{
  if (m.total_frames != 0U)
  {
    return (uint32_t)(((uint64_t)m.total_frames * m.spf) / m.hz);
  }
  /* CBR estimate */
  return (m.bitrate_kbps != 0U) ? (uint32_t)((m.audio_end - m.first_frame) / (m.bitrate_kbps * 125U)) : 0U;
}

static FSIZE_t trailing_tags_start(FIL *file, id3_tags_t *tags)
{
  FSIZE_t end = f_size(file);

  if (id3v1_parse(file, tags))
  {
    end -= 128U;
  }
  /* APEv2 footer just before (32 bytes, "APETAGEX") */
  uint8_t ape[32];
  if (end >= sizeof(ape) && read_at(file, end - sizeof(ape), ape, sizeof(ape)) &&
      memcmp(ape, "APETAGEX", 8) == 0)
  {
    const uint32_t size  = (uint32_t)ape[12] | ((uint32_t)ape[13] << 8) | ((uint32_t)ape[14] << 16) | ((uint32_t)ape[15] << 24);
    const uint32_t flags = (uint32_t)ape[20] | ((uint32_t)ape[21] << 8) | ((uint32_t)ape[22] << 16) | ((uint32_t)ape[23] << 24);
    const uint32_t total = size + (((flags >> 31) & 1U) ? 32U : 0U);
    if (total <= end)
    {
      end -= total;
    }
  }
  return end;
}

static bool mp3_open(FIL *file, const char *path, decoder_info_t *info)
{
  id3_tags_t tags;

  memset(&m, 0, sizeof(m));
  memset(&tags, 0, sizeof(tags));
  m.file      = file;
  m.seek_step = SEEK_STEP_INITIAL;

  const uint32_t audio_start = id3v2_parse(file, &tags);
  m.audio_end = trailing_tags_start(file, &tags);
  if (audio_start >= m.audio_end)
  {
    LOG_WARN("mp3: %s: no audio data", path);
    return false;
  }

  restart_at(audio_start, 0);
  refill();

  /* First frame: stream properties, and possibly a Xing/VBRI header */
  mp3dec_frame_info_t fi;
  const int samples = mp3dec_decode_frame(&m.dec, m.in, (int)m.in_len, m.pcm, &fi);
  frame_header_t hdr;
  if (fi.frame_bytes == 0 || !parse_header(&m.in[fi.frame_offset], &hdr))
  {
    LOG_WARN("mp3: %s: no MPEG Layer III frames", path);
    return false;
  }
  m.hz           = hdr.hz;
  m.spf          = hdr.spf;
  m.bitrate_kbps = hdr.bitrate_kbps;

  const FSIZE_t frame_pos = audio_start + (FSIZE_t)fi.frame_offset;
  const uint32_t frame_len = (uint32_t)(fi.frame_bytes - fi.frame_offset);
  bool vbr = false;
  const bool is_header = parse_vbr_header(&m.in[fi.frame_offset], frame_len, &hdr, frame_pos, &vbr);

  m.in_pos = (uint32_t)fi.frame_bytes;
  if (is_header)
  {
    /* The header frame decodes to silence: skip it */
    m.first_frame = frame_pos + frame_len;
    mp3dec_init(&m.dec);
  }
  else
  {
    m.first_frame  = frame_pos;
    m.pcm_frames   = (samples > 0) ? (uint32_t)samples : 0U;
    m.pcm_channels = (uint8_t)fi.channels;
  }

  if (m.total_frames != 0U && m.vbr_bytes != 0U)
  {
    m.bitrate_kbps = (uint32_t)(((uint64_t)m.vbr_bytes * 8U * m.hz) / ((uint64_t)m.total_frames * m.spf * 1000U));
  }

  /* No usable seek data: scan the frame headers in the background */
  if (!m.has_toc && !m.seek_complete && f_open(&m.scan_file, path, FA_READ) == FR_OK)
  {
    m.scanning = true;
    m.scan_pos = m.first_frame;
  }

  info->format         = "MP3";
  info->sample_rate    = m.hz;
  info->channels       = hdr.channels;
  info->bitrate_kbps   = m.bitrate_kbps;
  info->vbr            = vbr;
  info->duration_s     = duration_s();
  info->duration_exact = (m.total_frames != 0U);
  snprintf(info->title, sizeof(info->title), "%s", tags.title);
  snprintf(info->artist, sizeof(info->artist), "%s", tags.artist);
  info->art_format = (uint8_t)tags.art_format;
  info->art_offset = tags.art_offset;
  info->art_size   = tags.art_size;
  return true;
}

static uint32_t mp3_read(int16_t *out, uint32_t frames)
{
  uint32_t done = 0;

  while (done < frames)
  {
    if (m.pcm_pos >= m.pcm_frames)
    {
      const uint32_t samples = decode_next();
      if (samples == 0U)
      {
        break;
      }
      m.pcm_frames = samples;
      m.pcm_pos    = 0;
      if (m.discard > 0U)
      {
        const uint32_t drop = (m.discard < samples) ? m.discard : samples;
        m.pcm_pos  = drop;
        m.discard -= drop;
        continue;
      }
    }

    uint32_t n = m.pcm_frames - m.pcm_pos;
    if (n > frames - done)
    {
      n = frames - done;
    }
    if (m.pcm_channels == 2U)
    {
      memcpy(&out[2U * done], &m.pcm[2U * m.pcm_pos], n * 2U * sizeof(int16_t));
    }
    else
    {
      for (uint32_t i = 0; i < n; i++)
      {
        out[2U * (done + i)] = out[2U * (done + i) + 1U] = m.pcm[m.pcm_pos + i];
      }
    }
    done      += n;
    m.pcm_pos += n;
  }
  return done;
}

static bool mp3_seek(uint32_t position_s, uint32_t *actual_s)
{
  const uint32_t target_frame = (uint32_t)(((uint64_t)position_s * m.hz) / m.spf);

  /* 1) Frame-accurate table (background scan or VBRI), if it covers the target */
  if (m.seek_count > 0U && (m.seek_complete || target_frame / m.seek_step < m.seek_count))
  {
    const uint32_t start = (target_frame > SEEK_WARMUP_FRAMES) ? target_frame - SEEK_WARMUP_FRAMES : 0U;
    uint32_t idx = start / m.seek_step;
    if (idx >= m.seek_count)
    {
      idx = m.seek_count - 1U;
    }
    const uint32_t entry_frame = idx * m.seek_step;
    /* Decoding frames to drop costs time; for coarse tables (very long files)
     * land a little early instead of stalling the audio */
    uint32_t skip = target_frame - entry_frame;
    if (skip > SEEK_MAX_DISCARD_FRAMES)
    {
      skip = SEEK_MAX_DISCARD_FRAMES;
    }
    restart_at(m.seek_off[idx], skip * m.spf);
    *actual_s = (uint32_t)(((uint64_t)(entry_frame + skip) * m.spf) / m.hz);
    return true;
  }

  const uint32_t duration = duration_s();
  FSIZE_t offset;
  if (m.has_toc && duration > 0U)
  {
    /* 2) Xing table of contents: percent of time -> percent of bytes */
    const uint32_t bytes = (m.vbr_bytes != 0U) ? m.vbr_bytes : (uint32_t)(m.audio_end - m.first_frame);
    const uint32_t permille = (position_s >= duration) ? 999U : (position_s * 1000U) / duration;
    const uint32_t i  = permille / 10U;
    const uint32_t fa = m.toc[i];
    const uint32_t fb = (i < 99U) ? m.toc[i + 1U] : 256U;
    const uint32_t fx = fa * 10U + (fb - fa) * (permille % 10U);   /* in 1/2560 */
    offset = m.first_frame + (FSIZE_t)(((uint64_t)bytes * fx) / 2560U);
  }
  else
  {
    /* 3) Constant bitrate estimate */
    offset = m.first_frame + (FSIZE_t)position_s * m.bitrate_kbps * 125U;
  }
  if (offset >= m.audio_end)
  {
    offset = m.first_frame;
    position_s = 0;
  }
  restart_at(offset, 0);
  *actual_s = position_s;
  return true;
}

/* Find the next frame header after m.scan_pos. */
static bool scan_resync(void)
{
  uint8_t buf[512];
  FSIZE_t pos = m.scan_pos + 1U;

  while (pos + 4U <= m.audio_end && m.scan_garbage < SCAN_RESYNC_LIMIT)
  {
    UINT n = sizeof(buf);
    if (n > m.audio_end - pos)
    {
      n = (UINT)(m.audio_end - pos);
    }
    if (!read_at(&m.scan_file, pos, buf, n))
    {
      return false;
    }
    for (UINT i = 0; i + 4U <= n; i++)
    {
      frame_header_t h;
      if (buf[i] == 0xFFU && parse_header(&buf[i], &h) && h.hz == m.hz)
      {
        m.scan_pos = pos + i;
        return true;
      }
    }
    pos += n - 3U;
    m.scan_garbage += n;
  }
  return false;
}

static bool mp3_poll(uint32_t *duration_out)
{
  if (!m.scanning)
  {
    return false;
  }

  bool finished = false;
  for (uint32_t n = 0; n < SCAN_FRAMES_PER_POLL; n++)
  {
    uint8_t h[4];
    frame_header_t hdr;
    if (m.scan_pos + 4U > m.audio_end || !read_at(&m.scan_file, m.scan_pos, h, sizeof(h)))
    {
      finished = true;
      break;
    }
    if (!parse_header(h, &hdr) || hdr.hz != m.hz)
    {
      if (!scan_resync())
      {
        finished = true;
        break;
      }
      continue;
    }
    seek_table_add(m.scan_frames, (uint32_t)m.scan_pos);
    m.scan_frames++;
    m.scan_bytes += hdr.frame_len;
    m.scan_pos   += hdr.frame_len;
  }

  if (!finished)
  {
    return false;
  }

  f_close(&m.scan_file);
  m.scanning      = false;
  m.seek_complete = true;
  if (m.scan_frames == 0U)
  {
    return false;
  }
  m.total_frames = m.scan_frames;
  m.bitrate_kbps = (uint32_t)((m.scan_bytes * 8U * m.hz) / ((uint64_t)m.scan_frames * m.spf * 1000U));
  *duration_out  = duration_s();
  return true;
}

static void mp3_close(void)
{
  if (m.scanning)
  {
    f_close(&m.scan_file);
    m.scanning = false;
  }
  m.file = NULL;
}

const decoder_ops_t decoder_mp3_ops =
{
  .extension = ".mp3",
  .open      = mp3_open,
  .read      = mp3_read,
  .seek      = mp3_seek,
  .poll      = mp3_poll,
  .close     = mp3_close,
};
