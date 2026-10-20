/**
 * @file  decoder.c
 * @brief Picks the format decoder from the file extension and forwards calls.
 */
#include "decoder.h"

#include <string.h>
#include <strings.h>

#include "decoder_priv.h"
#include "log.h"
#include "lv_fs_sd.h"

static const decoder_ops_t *const k_decoders[] =
{
  &decoder_wav_ops,
  &decoder_mp3_ops,
};

static FIL                  s_file;
static DWORD                s_linkmap[256];   /* fast seek cluster map */
static const decoder_ops_t *s_active;

static const decoder_ops_t *find_decoder(const char *filename)
{
  const char *dot = strrchr(filename, '.');
  if (dot == NULL)
  {
    return NULL;
  }
  for (size_t i = 0; i < sizeof(k_decoders) / sizeof(k_decoders[0]); i++)
  {
    if (strcasecmp(dot, k_decoders[i]->extension) == 0)
    {
      return k_decoders[i];
    }
  }
  return NULL;
}

bool decoder_is_supported(const char *filename)
{
  return find_decoder(filename) != NULL;
}

bool decoder_open(const char *path, decoder_info_t *info)
{
  decoder_close();
  memset(info, 0, sizeof(*info));

  const decoder_ops_t *ops = find_decoder(path);
  if (ops == NULL)
  {
    return false;
  }
  if (f_open(&s_file, path, FA_READ) != FR_OK)
  {
    LOG_WARN("decoder: cannot open %s", path);
    return false;
  }
  /* Fast seek makes seeking within long files cheap (skipped if too fragmented) */
  fatfs_enable_fast_seek(&s_file, s_linkmap, sizeof(s_linkmap) / sizeof(s_linkmap[0]));

  if (!ops->open(&s_file, path, info))
  {
    f_close(&s_file);
    return false;
  }
  s_active = ops;
  return true;
}

uint32_t decoder_read(int16_t *out, uint32_t frames)
{
  return (s_active != NULL) ? s_active->read(out, frames) : 0U;
}

bool decoder_seek(uint32_t position_s, uint32_t *actual_s)
{
  return (s_active != NULL) ? s_active->seek(position_s, actual_s) : false;
}

bool decoder_poll(uint32_t *duration_s)
{
  return (s_active != NULL && s_active->poll != NULL) ? s_active->poll(duration_s) : false;
}

void decoder_close(void)
{
  if (s_active != NULL)
  {
    s_active->close();
    f_close(&s_file);
    s_active = NULL;
  }
}
