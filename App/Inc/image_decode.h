/**
 * @file  image_decode.h
 * @brief PNG and BMP pictures from a file, shrunk to fit a given size into an
 *        RGB565 buffer (image_scale.h) while they are decoded, so that their
 *        size is limited only by SCALE_MAX_SRC, not by memory.
 *
 * PNG: all colour types and bit depths, interlaced or not, with transparency
 * (blended over a background colour). BMP: uncompressed 1, 4, 8, 16, 24 and
 * 32 bits per pixel, with or without bit fields. One picture at a time.
 */
#ifndef IMAGE_DECODE_H
#define IMAGE_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#include "ff.h"
#include "image_scale.h"
#include "inflate.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  IMAGE_OK,
  IMAGE_UNSUPPORTED,   /**< not a picture of a supported kind      */
  IMAGE_TOO_LARGE,     /**< wider or taller than SCALE_MAX_SRC     */
  IMAGE_CORRUPT,       /**< damaged or cut short (what was decoded is in the output) */
  IMAGE_ABORTED,       /**< *abort became true                     */
} image_result_t;

#define IMAGE_MAX_ROW_BYTES 65536U    /* PNG / BMP row, without the PNG filter byte */
#define IMAGE_IN_CHUNK      4096U

/** Working memory of a decode (about 530 KB; the caller provides it, so it can
 *  be shared with other short-lived buffers) */
typedef struct
{
  scale_t   scale;
  inflate_t inflate;
  uint8_t   rows[2][IMAGE_MAX_ROW_BYTES + 4U];   /**< filter byte, row, spare for word access */
  uint8_t   line[SCALE_MAX_SRC * 3U];            /**< one row as RGB */
  uint8_t   in[IMAGE_IN_CHUNK];
} image_work_t;

typedef struct
{
  uint16_t      *out;           /**< RGB565, max_w x max_h pixels at least */
  uint32_t       max_w, max_h;  /**< at most SCALE_MAX_OUT */
  uint32_t       background;    /**< 0xRRGGBB behind transparent pixels */
  volatile const bool *abort;   /**< optional: stop when it becomes true */
  image_work_t  *work;          /**< working memory; work->scale has the picture's size afterwards */
} image_target_t;

typedef enum
{
  IMAGE_KIND_UNKNOWN,
  IMAGE_KIND_JPEG,
  IMAGE_KIND_PNG,
  IMAGE_KIND_GIF,
  IMAGE_KIND_BMP,
} image_kind_t;

/** Kind of picture from the first bytes of the file (at least 8) */
image_kind_t image_kind(const uint8_t *head, uint32_t len);

/** Decode the PNG in @p file (from its start); the picture's size is out_w x out_h. */
image_result_t image_decode_png(FIL *file, const image_target_t *target, uint32_t *out_w, uint32_t *out_h);

/** Decode the BMP in @p file (from its start). */
image_result_t image_decode_bmp(FIL *file, const image_target_t *target, uint32_t *out_w, uint32_t *out_h);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_DECODE_H */
