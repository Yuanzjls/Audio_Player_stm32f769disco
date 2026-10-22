/**
 * @file  image_scale.h
 * @brief Shrinks a picture to fit a given size while it is being decoded, and
 *        turns or mirrors it (photo orientation), into an RGB565 buffer.
 *
 * Every output pixel is the average of the source pixels it covers (box
 * filter), so even a large photo needs no more memory than a band of output
 * rows: the source rows come in from top to bottom, and an output row is
 * written once its last source row is in. Pictures whose rows come in another
 * order (interlaced) use scale_point(): the source pixel nearest to the
 * middle of each output pixel. Pictures are never enlarged.
 *
 * Samples are either RGB or JPEG YCbCr (averaged as they are, converted when
 * the output row is written). One picture at a time.
 */
#ifndef IMAGE_SCALE_H
#define IMAGE_SCALE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCALE_MAX_SRC   16384U   /* widest / tallest source picture */
#define SCALE_MAX_OUT   800U     /* widest / tallest output */
#define SCALE_BAND      24U      /* output rows being summed at a time (>= 16 source rows + 1) */

/* How the picture is turned: transpose first, then mirror the result */
#define SCALE_TRANSPOSE 1U
#define SCALE_FLIP_X    2U
#define SCALE_FLIP_Y    4U

typedef enum
{
  SCALE_RGB,
  SCALE_YCBCR,
} scale_space_t;

typedef struct
{
  uint32_t src_w, src_h;
  uint32_t tw, th;             /**< output size before the transform */
  uint32_t out_w, out_h;       /**< output size after it (the RGB565 buffer's) */
  uint32_t transform;
  scale_space_t space;
  uint16_t *out;
  uint32_t next_row;           /**< next output row (before the transform) to write */
  bool     pointwise;          /**< scale_point() was used: nothing to sum */
  uint16_t map_x[SCALE_MAX_SRC];  /**< source column -> output column */
  uint16_t map_y[SCALE_MAX_SRC];  /**< source row    -> output row    */
  uint16_t last_y[SCALE_MAX_OUT]; /**< output row -> its last source row */
  uint16_t n_x[SCALE_MAX_OUT];    /**< source columns per output column */
  uint16_t n_y[SCALE_MAX_OUT];    /**< source rows per output row */
  uint16_t pick_x[SCALE_MAX_OUT]; /**< scale_point(): source column of each output column */
  uint16_t pick_y[SCALE_MAX_OUT];
  uint32_t acc[SCALE_BAND][SCALE_MAX_OUT * 3U];   /**< sums of the rows being collected */
} scale_t;

/**
 * Start a picture of @p src_w x @p src_h. The output is the picture after
 * @p transform, shrunk (keeping its shape) to fit @p max_w x @p max_h, in @p out
 * (RGB565, out_w x out_h, rows of out_w pixels). False if the source is too large.
 */
bool scale_begin(scale_t *s, uint32_t src_w, uint32_t src_h, uint32_t transform, scale_space_t space,
                 uint16_t *out, uint32_t max_w, uint32_t max_h);

/** Sums of output row @p ty (3 per output column), to add samples to directly */
static inline uint32_t *scale_acc(scale_t *s, uint32_t ty) { return s->acc[ty % SCALE_BAND]; }

/** Add source row @p y (3 bytes per pixel, src_w pixels) and write what is complete */
void scale_row(scale_t *s, uint32_t y, const uint8_t *px);

/** Source rows up to @p y were added (directly): write the output rows that are complete */
void scale_rows_done(scale_t *s, uint32_t y);

/** Pixels (x0 + i * step, y), i < n, in any order: nearest pixel (interlaced pictures) */
void scale_point(scale_t *s, uint32_t y, uint32_t x0, uint32_t step, const uint8_t *px, uint32_t n);

/** Write the output rows still pending (for pictures that ended early: what was summed) */
void scale_end(scale_t *s);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_SCALE_H */
