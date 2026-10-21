/**
 * @file  avi.h
 * @brief AVI (RIFF) reader: stream formats, the chunks of the "movi" list in
 *        file order, and seeking with the "idx1" index.
 *
 * Supported: one MJPEG video stream and at most one audio stream (16-bit PCM
 * or MP3), as written by FFmpeg and most cameras. OpenDML (files over 1 GB,
 * "AVIX" parts) is not.
 */
#ifndef AVI_H
#define AVI_H

#include <stdbool.h>
#include <stdint.h>

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AVI_AUDIO_PCM   0x0001U
#define AVI_AUDIO_MP3   0x0055U

typedef struct
{
  /* video */
  uint32_t width, height;
  uint32_t frame_us;          /**< duration of one frame, microseconds */
  uint32_t frames;            /**< number of video frames              */
  /* audio (has_audio false: none) */
  bool     has_audio;
  uint16_t audio_format;      /**< AVI_AUDIO_PCM or AVI_AUDIO_MP3      */
  uint16_t channels;
  uint32_t sample_rate;
  uint16_t bits;              /**< PCM: bits per sample                */
  uint16_t block_align;       /**< PCM: bytes per sample frame         */
  uint32_t audio_bytes_per_s; /**< average                             */
  /* layout */
  uint16_t video_id;          /**< chunk id prefix: stream number as two digits, "00" */
  uint16_t audio_id;
  uint32_t movi_start;        /**< file offset of the first chunk in "movi" */
  uint32_t movi_end;
  uint32_t idx1_offset;       /**< 0: no index (no seeking)            */
  uint32_t idx1_entries;
} avi_info_t;

typedef enum
{
  AVI_OK,
  AVI_ERR_IO,
  AVI_ERR_NOT_AVI,
  AVI_ERR_NO_VIDEO,           /**< no MJPEG video stream               */
  AVI_ERR_AUDIO_FORMAT,       /**< audio neither 16-bit PCM nor MP3    */
} avi_result_t;

typedef enum
{
  AVI_CHUNK_VIDEO,
  AVI_CHUNK_AUDIO,
  AVI_CHUNK_OTHER,
} avi_chunk_type_t;

typedef struct
{
  avi_chunk_type_t type;
  uint32_t         offset;    /**< file offset of the chunk data */
  uint32_t         size;      /**< bytes of data                 */
} avi_chunk_t;

/** Read the headers of the AVI file open in @p file. */
avi_result_t avi_open(FIL *file, avi_info_t *info);

const char *avi_result_str(avi_result_t result);

/**
 * The next chunk at or after file offset @p pos in the "movi" list; @p pos is
 * advanced past it. False at the end of the list or on a read error.
 */
bool avi_next_chunk(FIL *file, const avi_info_t *info, uint32_t *pos, avi_chunk_t *chunk);

/**
 * Where to read from to start at video frame @p frame (with the index): @p pos
 * receives the file offset of that frame's chunk, @p audio_bytes the audio
 * data that comes before it (its time). False without an index.
 */
bool avi_seek(FIL *file, const avi_info_t *info, uint32_t frame, uint32_t *pos, uint32_t *audio_bytes);

#ifdef __cplusplus
}
#endif

#endif /* AVI_H */
