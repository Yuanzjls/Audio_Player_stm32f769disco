/**
 * @file  decoder_priv.h
 * @brief Interface between decoder.c and the individual format decoders.
 */
#ifndef DECODER_PRIV_H
#define DECODER_PRIV_H

#include "decoder.h"
#include "ff.h"

typedef struct
{
  const char *extension;   /* ".wav", lower case */
  /* @p file is open at offset 0; @p path is given for decoders that need a second handle */
  bool     (*open)(FIL *file, const char *path, decoder_info_t *info);
  uint32_t (*read)(int16_t *out, uint32_t frames);
  bool     (*seek)(uint32_t position_s, uint32_t *actual_s);
  bool     (*poll)(uint32_t *duration_s);       /* optional */
  void     (*close)(void);
} decoder_ops_t;

extern const decoder_ops_t decoder_wav_ops;
extern const decoder_ops_t decoder_mp3_ops;

#endif /* DECODER_PRIV_H */
