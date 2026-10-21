/**
 * @file  video_player.h
 * @brief AVI playback: MJPEG video decoded by the hardware JPEG codec straight
 *        into the display's frame buffers, with 16-bit PCM or MP3 audio.
 *
 * The audio is the clock: every frame is shown when the audio reaches its time,
 * frames that come too late are dropped. Without audio, the system tick is the
 * clock. The caller puts the display into video mode while playing
 * (display_video_begin / end) and takes the audio output from the music player
 * first (player_release_audio).
 */
#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  VIDEO_IDLE,       /**< nothing open                      */
  VIDEO_PAUSED,     /**< open, not playing (also after open) */
  VIDEO_PLAYING,
  VIDEO_ENDED,      /**< played to the end                  */
} video_state_t;

typedef struct
{
  uint32_t width, height;
  uint32_t fps_x100;          /**< frames per second x 100 */
  uint32_t duration_ms;
  char     audio[32];         /**< e.g. "PCM 44.1 kHz stereo", "no sound" */
} video_info_t;

/** Start the video tasks (once). */
void video_init(void);

/** Open @p path, paused at the start. False (and a log message) if it cannot be played. */
bool video_open(const char *path, video_info_t *info);

/** Play from the current position (the display must be in video mode). */
void video_play(void);

/** Pause; returns once no frame is being drawn any more. */
void video_pause(void);

/** Jump to @p position_ms (paused or playing). */
void video_seek(uint32_t position_ms);

/** Stop and close the file. */
void video_close(void);

video_state_t video_state(void);
uint32_t video_position_ms(void);

/** Frames shown and dropped since the file was opened, and the time spent decoding them. */
void video_stats(uint32_t *shown, uint32_t *dropped, uint32_t *decode_ms);

/** A picture blended over every frame (stats, controls). */
typedef struct
{
  const uint16_t *rgb;        /**< RGB565, premultiplied by alpha, w x h          */
  const uint8_t  *alpha;      /**< 0..255 per pixel, w x h; NULL: opaque         */
  int32_t         x, y;       /**< on the screen: the layer must lie wholly on it */
  uint32_t        w, h;       /**< w x h at most 800 x 128                         */
} video_layer_t;

#define VIDEO_LAYERS 3

/** Blend @p layer (NULL: none) in @p slot over every frame from the next one on,
 *  slots in ascending order. The layer and its pixels must stay unchanged
 *  until it is replaced: draw the next one into other buffers, then switch. */
void video_set_layer(uint32_t slot, const video_layer_t *layer);

/** The last frame shown, without the layers (DISPLAY_WIDTH x DISPLAY_HEIGHT;
 *  call when paused). */
void video_copy_frame(uint16_t *dst);

#ifdef __cplusplus
}
#endif

#endif /* VIDEO_PLAYER_H */
