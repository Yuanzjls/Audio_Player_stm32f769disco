/**
 * @file  jpeg_hw.h
 * @brief The STM32F769's hardware JPEG decoder, shared by the album covers and
 *        the video player (one picture at a time).
 *
 * The caller supplies the compressed data through read() and receives the
 * decoded picture as YCbCr MCUs (minimum coded units: 8x8 blocks of Y, Cb and
 * Cr samples) through mcus(), in picture order, while the codec works on the
 * next ones. Event driven: DMA feeds and drains the codec, and the calling task
 * sleeps until its interrupts have something for it.
 */
#ifndef JPEG_HW_H
#define JPEG_HW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Picture size and how its MCUs are laid out */
typedef struct
{
  uint32_t width, height;     /**< pixels                                          */
  bool     gray;              /**< luma only (no Cb / Cr blocks)                   */
  uint32_t mcu_w, mcu_h;      /**< pixels per MCU: 16x16 (4:2:0), 16x8 (4:2:2), 8x8 */
  uint32_t y_blocks;          /**< 8x8 luma blocks per MCU (Cb and Cr follow them)  */
  uint32_t mcu_bytes;
  uint32_t mcus_per_row;
} jpeg_hw_layout_t;

typedef struct
{
  /** Copy the next compressed bytes into @p buf (at most @p max); 0 = no more */
  uint32_t (*read)(void *ctx, uint8_t *buf, uint32_t max);
  /** Header parsed: false stops the decoding (picture not wanted) */
  bool (*begin)(void *ctx, const jpeg_hw_layout_t *layout);
  /** Decoded MCUs, whole ones, in order (@p len is a multiple of mcu_bytes) */
  void (*mcus)(void *ctx, const uint8_t *data, uint32_t len);
  void *ctx;
} jpeg_hw_job_t;

typedef enum
{
  JPEG_HW_OK,
  JPEG_HW_REJECTED,       /**< begin() returned false, or CMYK / too large */
  JPEG_HW_ERROR,          /**< corrupt, progressive, or timed out           */
} jpeg_hw_result_t;

/** Set the codec up (once). */
void jpeg_hw_init(void);

/** Decode one picture; returns when done. Any task; one decode at a time. */
jpeg_hw_result_t jpeg_hw_decode(const jpeg_hw_job_t *job);

#ifdef __cplusplus
}
#endif

#endif /* JPEG_HW_H */
