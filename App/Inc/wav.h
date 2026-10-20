/**
 * @file  wav.h
 * @brief Minimal RIFF/WAVE parser (16-bit PCM, mono or stereo).
 */
#ifndef WAV_H
#define WAV_H

#include <stdint.h>

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  WAV_OK = 0,
  WAV_ERR_IO,           /**< read/seek failed                         */
  WAV_ERR_NOT_WAVE,     /**< not a RIFF/WAVE file                      */
  WAV_ERR_NO_FORMAT,    /**< "fmt " chunk missing or malformed         */
  WAV_ERR_NOT_PCM,      /**< compressed / floating point audio         */
  WAV_ERR_BITS,         /**< not 16 bits per sample                    */
  WAV_ERR_CHANNELS,     /**< not mono or stereo                        */
  WAV_ERR_NO_DATA,      /**< "data" chunk missing or empty             */
} wav_result_t;

typedef struct
{
  uint32_t sample_rate;     /**< Hz                                     */
  uint16_t channels;        /**< 1 or 2                                 */
  uint16_t block_align;     /**< bytes per frame (all channels)         */
  uint32_t byte_rate;       /**< bytes per second                       */
  uint32_t data_offset;     /**< file offset of the first sample        */
  uint32_t data_size;       /**< bytes of sample data                   */
} wav_info_t;

/**
 * Parse the header of an open file. On success the file position is left at
 * the first sample (info->data_offset).
 */
wav_result_t wav_parse(FIL *file, wav_info_t *info);

/** Human readable description of a result code */
const char *wav_result_str(wav_result_t result);

/** Duration in whole seconds */
uint32_t wav_duration_s(const wav_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* WAV_H */
