/**
 * @file  player.h
 * @brief WAV playlist player running in its own task.
 *
 * Plays the .wav and .mp3 files of /Music on the SD card, sorted by name.
 * Control functions are thread safe: they queue a command for the player
 * task and return immediately. The UI polls player_get_status().
 */
#ifndef PLAYER_H
#define PLAYER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PLAYER_MUSIC_DIR   "/Music"
#define PLAYER_TITLE_MAX   256   /* bytes, UTF-8, including terminator */
#define PLAYER_MAX_TRACKS  4096  /* library size limit (names cached in SDRAM: 1 MB) */
#define PLAYER_MESSAGE_MAX 64

typedef enum
{
  PLAYER_STATE_NO_CARD,    /**< SD card missing or unreadable */
  PLAYER_STATE_NO_TRACKS,  /**< no playable file in /Music    */
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
  bool            shuffle;       /**< random play order       */
  uint8_t         volume;        /**< 0..100                  */
  uint16_t        track_index;   /**< 0-based                 */
  uint16_t        track_count;
  uint32_t        position_s;
  uint32_t        duration_s;
  uint32_t        sample_rate;
  uint8_t         channels;
  uint32_t        bitrate_kbps;
  bool            vbr;
  char            format[8];     /**< "WAV", "MP3" */
  uint32_t        track_serial;  /**< changes whenever a new track is loaded */
  uint32_t        library_serial;/**< changes whenever the track list is (re)read */
  char            title[PLAYER_TITLE_MAX];   /**< tag title, else the file name */
  char            artist[PLAYER_TITLE_MAX];  /**< tag artist, may be empty       */
  char            message[PLAYER_MESSAGE_MAX]; /**< error / hint for the user, may be empty */
  uint8_t         art_format;    /**< embedded cover: 0 = none, 1 = JPEG, 2 = PNG */
  uint32_t        art_offset;    /**< position of the cover image in the file      */
  uint32_t        art_size;
  uint32_t        card_mb;       /**< SD card file system size, MB (0: no card) */
  char            card_fs[8];    /**< "FAT32", "exFAT", ... */
} player_status_t;

/** Create the player task. It mounts the SD card and starts playing once
 *  player_ui_ready() has been called. */
void player_start(uint8_t initial_volume);

/** Tell the player the screen is up: playback starts only after this. */
void player_ui_ready(void);

/** Copy the current status (thread safe). */
void player_get_status(player_status_t *status);

void player_toggle_play(void);
void player_next(void);
/** Previous track, or back to the start of the current one if past 3 s. */
void player_prev(void);
/** Play track @p index of the library (0-based). */
void player_play_track(uint16_t index);

/** Copy the display name (file name without its extension) of library entry @p index. */
bool player_get_track_name(uint16_t index, char *name, size_t len);

/** Copy the full path of the file of library entry @p index. */
bool player_get_track_path(uint16_t index, char *path, size_t len);
void player_seek(uint32_t position_s);
void player_set_volume(uint8_t volume);
void player_set_repeat(player_repeat_t repeat);

/**
 * Random play order on or off (on at start-up, which also picks a random first
 * song). Every track plays once in a shuffled order
 * (a new order is drawn when it runs out); previous goes back along it.
 */
void player_set_shuffle(bool shuffle);

#ifdef __cplusplus
}
#endif

#endif /* PLAYER_H */
