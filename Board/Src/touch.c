/**
 * @file  touch.c
 * @brief FT6x06 touch controller, polled by LVGL from the GUI task.
 */
#include "touch.h"

#include "display.h"
#include "i2c_bus.h"
#include "log.h"
#include "stm32f769i_discovery_ts.h"

static void read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  static int32_t last_x;
  static int32_t last_y;
  TS_StateTypeDef ts;

  (void)indev;

  i2c_bus_lock();
  const uint8_t status = BSP_TS_GetState(&ts);
  i2c_bus_unlock();

  if (status == TS_OK && ts.touchDetected > 0U)
  {
    last_x = ts.touchX[0];
    last_y = ts.touchY[0];
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  /* LVGL expects the last known position also while released */
  data->point.x = last_x;
  data->point.y = last_y;
}

lv_indev_t *touch_init(void)
{
  i2c_bus_lock();
  const uint8_t status = BSP_TS_Init(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  i2c_bus_unlock();

  if (status != TS_OK)
  {
    LOG_ERROR("touch: controller not found");
    return NULL;
  }

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, read_cb);
  return indev;
}
