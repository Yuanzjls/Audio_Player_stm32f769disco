/**
 * @file  display.h
 * @brief 800x480 MIPI-DSI LCD (OTM8009A) driven as an LVGL display.
 */
#ifndef DISPLAY_H
#define DISPLAY_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_WIDTH   800
#define DISPLAY_HEIGHT  480

/**
 * Initialise DSI, LTDC and the panel, and register the LVGL display.
 * Requires lv_init() and the SDRAM (frame buffers live there).
 */
lv_display_t *display_init(void);

/** To be called from DSI_IRQHandler */
void display_dsi_irq_handler(void);

/** Panel brightness, 0..100 %. GUI task only. */
void display_set_brightness(uint8_t percent);

/** Switch the panel off (dark, nothing rendered) or on again. GUI task only. */
void display_set_on(bool on);
bool display_is_on(void);

/**
 * Video mode, for the video player: LVGL draws nothing, frames are written
 * straight into the display's two frame buffers (RGB565, DISPLAY_WIDTH wide).
 * begin() / end(): GUI task. frame() / present(): the video task.
 */
void display_video_begin(void);
/** A buffer to draw the next frame into (waits until one is free). */
uint16_t *display_video_frame(void);
/** Show @p frame (from display_video_frame()) from the next panel refresh on. */
void display_video_present(uint16_t *frame);
/** The frame on screen now. */
const uint16_t *display_video_shown(void);
/** Back to LVGL, which redraws everything. */
void display_video_end(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */
