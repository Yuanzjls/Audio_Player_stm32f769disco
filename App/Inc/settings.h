/**
 * @file  settings.h
 * @brief User settings, kept in /settings.txt on the SD card (key=value lines).
 *
 * Loaded by the player task once the card is mounted; changed and saved by the
 * GUI task. Saving waits until the settings have not changed for a moment, so
 * dragging a slider writes the file once.
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_FILE "/settings.txt"

typedef struct
{
  uint8_t  volume;            /**< 0..100 */
  uint8_t  brightness;        /**< 10..100 % */
  bool     shuffle_on_start;  /**< start in shuffle mode, at a random song */
  uint16_t screen_off_s;      /**< switch the screen off after this long untouched, 0 = never */
} settings_t;

/** Read the settings file (defaults where it is missing or incomplete). Player task. */
void settings_load(void);

/** True once settings_load() has run. */
bool settings_loaded(void);

/** Current settings. */
settings_t settings_get(void);

/** Change the settings; saved by settings_save_if_due(). GUI task. */
void settings_set(const settings_t *settings);

/** Write the file if the settings changed and then stayed unchanged for a while. GUI task. */
void settings_save_if_due(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_H */
