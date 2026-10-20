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

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H */
