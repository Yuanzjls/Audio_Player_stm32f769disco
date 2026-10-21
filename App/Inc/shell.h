/**
 * @file  shell.h
 * @brief The home screen and the apps it opens.
 *
 * Every app has a screen of its own, built once at start-up. The shell shows
 * one screen at a time: the home screen (app tiles and a now-playing bar) or an
 * app, which has a home button to come back. Leaving an app does not stop it:
 * the music keeps playing; apps pause only their own costly work in hide().
 */
#ifndef SHELL_H
#define SHELL_H

#include <stdint.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  const char *name;
  const char *icon;                   /**< text on the tile: an LVGL symbol or a letter */
  uint32_t    color;                  /**< tile icon colour, 0xRRGGBB */
  void (*create)(lv_obj_t *screen);   /**< build the app's screen (once, at start-up) */
  void (*show)(void);                 /**< optional: the app is about to be shown */
  void (*hide)(void);                 /**< optional: the app was left */
  void (*fonts_changed)(void);        /**< optional: the SD-card fonts are now loaded */
} app_t;

extern const app_t app_music;
extern const app_t app_video;
extern const app_t app_sysinfo;
extern const app_t app_settings;

/** Build the home screen and the apps, and show the home screen. */
void shell_init(void);

void shell_open(const app_t *app);
void shell_go_home(void);

/** The standard home button (44 x 40); the caller positions it. */
lv_obj_t *shell_home_button(lv_obj_t *parent);

/** Header of a plain app: home button and title. Returns the content area below it. */
lv_obj_t *shell_app_header(lv_obj_t *screen, const char *title);

/** CPU load in percent over the last second. */
uint32_t shell_cpu_load(void);

/** Fonts for song titles and text in any language (TrueType from the SD card,
 *  built-in Latin fonts until it is loaded). */
const lv_font_t *shell_font_title(void);   /* 32 px */
const lv_font_t *shell_font_text(void);    /* 22 px */

/** Cover of the current song (NULL: none); app_music.c. */
const lv_image_dsc_t *app_music_cover(void);

#ifdef __cplusplus
}
#endif

#endif /* SHELL_H */
