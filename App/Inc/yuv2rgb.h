/**
 * @file  yuv2rgb.h
 * @brief Decoded JPEG MCUs (YCbCr) to RGB565, written straight into a frame
 *        buffer at a given place (the picture may be partly or wholly off it:
 *        what lies outside is cut off).
 */
#ifndef YUV2RGB_H
#define YUV2RGB_H

#include <stdint.h>

#include "jpeg_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t        *dst;          /**< frame buffer, RGB565                        */
  uint32_t         dst_w, dst_h; /**< its size, pixels (also the row stride)       */
  int32_t          x0, y0;       /**< where the picture's top-left corner goes      */
  jpeg_hw_layout_t layout;       /**< of the picture (jpeg_hw begin callback)        */
  uint32_t         mcu_index;    /**< next MCU in the stream: start at 0            */
} yuv2rgb_t;

/** Build the conversion tables (once). */
void yuv2rgb_init(void);

/** Convert @p len bytes of whole MCUs, the next ones in the picture. */
void yuv2rgb_mcus(yuv2rgb_t *t, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* YUV2RGB_H */
