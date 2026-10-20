/**
 * @file  ui.h
 * @brief LVGL user interface of the audio player.
 */
#ifndef UI_H
#define UI_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Build the screen. Must be called from the GUI task after the display and
 * input device are registered. The UI then polls the player by itself.
 */
void ui_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_H */
