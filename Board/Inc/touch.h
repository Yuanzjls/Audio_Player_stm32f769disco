/**
 * @file  touch.h
 * @brief FT6x06 capacitive touch panel as an event-driven LVGL pointer.
 *
 * The controller's INT line (PI13) wakes the GUI task on new touch data, so
 * the I2C bus is only read while the panel is being touched.
 */
#ifndef TOUCH_H
#define TOUCH_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/** How often the touch is re-read while a finger is down (release detection). */
#define TOUCH_PRESSED_POLL_MS 20U

/**
 * Initialise the controller and register it with LVGL. @p gui_task is
 * notified (xTaskNotifyGive) when new touch data is available.
 * Returns NULL on failure.
 */
lv_indev_t *touch_init(TaskHandle_t gui_task);

/** True while the panel needs reading: new data pending or a finger is down. */
bool touch_read_pending(void);

/** Feed the latest touch state to LVGL. Call from the GUI task only. */
void touch_process(void);

/** To be called from EXTI15_10_IRQHandler */
void touch_irq_handler(void);

/** The touches of the last read: up to two fingers, in screen coordinates */
typedef struct
{
  uint8_t  count;
  uint16_t x[2], y[2];
} touch_points_t;

/** Copy the touches of the last read (any task; for code that needs both
 *  fingers, which LVGL's single pointer does not give). */
void touch_get_points(touch_points_t *points);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_H */
