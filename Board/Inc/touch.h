/**
 * @file  touch.h
 * @brief FT6x06 capacitive touch panel as an LVGL pointer input device.
 */
#ifndef TOUCH_H
#define TOUCH_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise the controller and register it with LVGL. Returns NULL on failure. */
lv_indev_t *touch_init(void);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_H */
