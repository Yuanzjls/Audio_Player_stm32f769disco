/**
 * @file  decoder.h
 * @brief Format-independent audio file decoding (WAV, MP3).
 *
 * One file is decoded at a time. Whatever the format, decoder_read() returns
 * interleaved stereo 16-bit samples (mono files are duplicated to both
 * channels), so the player never deals with file formats.
 */
#ifndef DECODER_H
#define DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DECODER_TEXT_MAX 128

typedef struct
{
  const char *format;            /**< "WAV", "MP3"                                   */
  uint32_t    sample_rate;       /**< Hz                                             */
  uint8_t     channels;          /**< channels in the file (output is always stereo) */
  uint32_t    bitrate_kbps;      /**< average for VBR                                */
  bool        vbr;               /**< variable bitrate                               */
  uint32_t    duration_s;
  bool        duration_exact;    /**< false: estimate, refined by decoder_poll()     */
  char        title[DECODER_TEXT_MAX];   /**< from tags, UTF-8, may be empty         */
  char        artist[DECODER_TEXT_MAX];
  /* Embedded cover picture, if any: format and where its data is in the file */
  uint8_t     art_format;        /**< id3_art_format_t: 0 = none, 1 = JPEG, 2 = PNG  */
  uint32_t    art_offset;
  uint32_t    art_size;
} decoder_info_t;

/** True if the file name has an extension a decoder handles. */
bool decoder_is_supported(const char *filename);

/** Open @p path and fill @p info. Any previously open file is closed. */
bool decoder_open(const char *path, decoder_info_t *info);

/**
 * Decode up to @p frames stereo frames into @p out (2 * frames int16).
 * Returns the number of frames written; less than requested only at the end
 * of the file or on a read error.
 */
uint32_t decoder_read(int16_t *out, uint32_t frames);

/** Jump to @p position_s. @p actual_s receives the position actually reached. */
bool decoder_seek(uint32_t position_s, uint32_t *actual_s);

/**
 * Do a slice of background work (e.g. measuring the exact length of a VBR
 * MP3). Call regularly while playing. Returns true when the duration has
 * changed; the new value is stored in @p duration_s.
 */
bool decoder_poll(uint32_t *duration_s);

void decoder_close(void);

#ifdef __cplusplus
}
#endif

#endif /* DECODER_H */
