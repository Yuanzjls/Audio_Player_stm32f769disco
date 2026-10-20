/**
 * @file  player.h
 * @brief WAV playlist player running in its own task.
 *
 * Plays the *.wav files of /Music on the SD card in directory order.
 * Control functions are thread safe: they queue a command for the player
 * task and return immediately. The UI polls player_get_status().
 */
#ifndef PLAYER_H
#define PLAYER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PLAYER_MUSIC_DIR   "/Music"
#define PLAYER_TITLE_MAX   256   /* bytes, UTF-8, including terminator */
#define PLAYER_MESSAGE_MAX 64

typedef enum
{
  PLAYER_STATE_NO_CARD,    /**< SD card missing or unreadable */
  PLAYER_STATE_NO_TRACKS,  /**< no playable .wav in /Music    */
  PLAYER_STATE_STOPPED,
  PLAYER_STATE_PLAYING,
  PLAYER_STATE_PAUSED,
} player_state_t;

typedef enum
{
  PLAYER_REPEAT_OFF,       /**< play the list once            */
  PLAYER_REPEAT_ONE,       /**< loop the current track        */
} player_repeat_t;

typedef struct
{
  player_state_t  state;
  player_repeat_t repeat;
  uint8_t         volume;        /**< 0..100                  */
  uint16_t        track_index;   /**< 0-based                 */
  uint16_t        track_count;
  uint32_t        position_s;
  uint32_t        duration_s;
  uint32_t        sample_rate;
  uint8_t         channels;
  uint32_t        track_serial;  /**< changes whenever a new track is loaded */
  char            title[PLAYER_TITLE_MAX];
  char            message[PLAYER_MESSAGE_MAX]; /**< error / hint for the user, may be empty */
} player_status_t;

/** Create the player task. It mounts the SD card and starts playing. */
void player_start(uint8_t initial_volume);

/** Copy the current status (thread safe). */
void player_get_status(player_status_t *status);

void player_toggle_play(void);
void player_next(void);
void player_seek(uint32_t position_s);
void player_set_volume(uint8_t volume);
void player_set_repeat(player_repeat_t repeat);

#ifdef __cplusplus
}
#endif

#endif /* PLAYER_H */
