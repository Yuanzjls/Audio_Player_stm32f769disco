/**
 * @file  theme.h
 * @brief Colours shared by the home screen and the apps (light theme).
 */
#ifndef THEME_H
#define THEME_H

#include "lvgl.h"

#define THEME_BG_RGB        0xF4F6F9U        /* also the tint of the cover backdrops */

#define COLOR_BG            lv_color_hex(THEME_BG_RGB)
#define COLOR_PANEL         lv_color_hex(0xFFFFFF)
#define COLOR_DIVIDER       lv_color_hex(0xE2E6EC)
#define COLOR_SURFACE       lv_color_hex(0xE2E6EC)
#define COLOR_ACCENT        lv_color_hex(0x1A73E8)
#define COLOR_ACCENT_DARK   lv_color_hex(0x0B4FB3)
#define COLOR_ACCENT_SOFT   lv_color_hex(0xE8F0FE)
#define COLOR_TEXT          lv_color_hex(0x1F2328)
#define COLOR_TEXT_DIM      lv_color_hex(0x5F6875)
#define COLOR_WARNING       lv_color_hex(0xB45309)
#define COLOR_ON_ACCENT     lv_color_hex(0xFFFFFF)

#endif /* THEME_H */
