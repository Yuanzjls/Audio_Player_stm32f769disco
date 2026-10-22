/**
 * @file  picture.h
 * @brief Pictures (JPEG, PNG, BMP) from the SD card, decoded in the background
 *        and shrunk to fit the screen, for the Files app's viewer.
 *
 * JPEG uses the hardware codec (baseline JPEG of up to 8192 x 8192 pixels; a
 * photo is turned as its EXIF orientation says). PNG and BMP are decoded in
 * software (image_decode.h). Large pictures are shrunk while they are decoded,
 * so their size is not limited by memory. A new request stops the one before.
 */
#ifndef PICTURE_H
#define PICTURE_H

#include <stddef.h>
#include <stdint.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PICTURE_MAX_W 800
#define PICTURE_MAX_H 416

typedef enum
{
  PICTURE_IDLE,        /**< nothing asked for       */
  PICTURE_PENDING,     /**< still decoding          */
  PICTURE_READY,
  PICTURE_FAILED,      /**< see message             */
} picture_state_t;

typedef struct
{
  picture_state_t       state;
  const lv_image_dsc_t *image;        /**< READY (also FAILED for a damaged picture: what was decoded) */
  uint32_t              width, height; /**< of the picture in the file */
  const char           *message;      /**< FAILED: why, for the user */
} picture_result_t;

/** Start the decoding task (once). */
void picture_init(void);

/** Bytes of memory the decoding needs (about 1.2 MB: the picture and the decoders' working memory). */
size_t picture_mem_size(void);

/** Lend the decoding @p mem (picture_mem_size() bytes, 32-byte aligned) until picture_detach(). */
void picture_attach(void *mem);

/** Stop decoding and take the memory back; returns once the task no longer uses it. */
void picture_detach(void);

/** Decode the picture at @p path. The image of an earlier request must no
 *  longer be shown: its pixels are overwritten. */
void picture_request(const char *path);

/** Stop decoding (the viewer was closed). */
void picture_cancel(void);

/** State of the last request. */
picture_result_t picture_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* PICTURE_H */
