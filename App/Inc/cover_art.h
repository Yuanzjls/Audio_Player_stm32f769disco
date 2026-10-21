/**
 * @file  cover_art.h
 * @brief Album cover pictures embedded in the music files, decoded in the
 *        background into LVGL images.
 *
 * The UI asks for the cover of a track (where its JPEG is in the file, from the
 * player status) and polls for the result. Decoding runs in a task of its own,
 * so a large picture never blocks the UI; a newer request replaces an older one.
 */
#ifndef COVER_ART_H
#define COVER_ART_H

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Width and height of the decoded cover (square, rounded corners) */
#define COVER_ART_SIZE 160

/** Size of the backdrop: a blurred, washed-out copy of the cover (RGB565) */
#define COVER_ART_BACKDROP_W 800
#define COVER_ART_BACKDROP_H 480

typedef enum
{
  COVER_ART_PENDING,   /**< still decoding                              */
  COVER_ART_READY,     /**< the image is ready                          */
  COVER_ART_NONE,      /**< no usable picture: show a placeholder        */
} cover_art_state_t;

/** Start the decoding task. @p backdrop_tint (0xRRGGBB): colour the backdrops are washed with. */
void cover_art_init(uint32_t backdrop_tint);

/**
 * Ask for the cover of a track. @p id identifies the request (e.g. the track
 * serial). @p format: 0 = none, 1 = JPEG, 2 = PNG (not supported: no cover).
 */
void cover_art_request(uint32_t id, const char *path, uint8_t format, uint32_t offset, uint32_t size);

/**
 * State of request @p id. When ready, @p image receives the cover and @p backdrop
 * its blurred copy; both stay valid until the next-but-one request is decoded.
 */
cover_art_state_t cover_art_poll(uint32_t id, const lv_image_dsc_t **image, const lv_image_dsc_t **backdrop);

#ifdef __cplusplus
}
#endif

#endif /* COVER_ART_H */
