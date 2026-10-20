/**
 * @file  touch.c
 * @brief Event-driven FT6x06 touch input for LVGL.
 *
 * The FT6x06 runs in "trigger" interrupt mode: it pulses INT (PI13, falling
 * edge) whenever it has new coordinates. The EXTI interrupt only sets a flag
 * and wakes the GUI task, which then reads the controller over I2C and hands
 * the result to LVGL (LVGL input device in event mode). While a finger is down
 * the GUI task also re-reads every TOUCH_PRESSED_POLL_MS, so a lost "lift"
 * pulse can never leave the pointer stuck in the pressed state. With nobody
 * touching the screen, nothing is read and the GUI task can sleep.
 */
#include "touch.h"

#include "display.h"
#include "i2c_bus.h"
#include "log.h"
#include "stm32f769i_discovery_ts.h"

static lv_indev_t      *s_indev;
static TaskHandle_t     s_gui_task;
static volatile bool    s_data_pending;
static bool             s_pressed;

static void read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
  static int32_t last_x;
  static int32_t last_y;
  TS_StateTypeDef ts;

  (void)indev;
  s_data_pending = false;

  i2c_bus_lock();
  const uint8_t status = BSP_TS_GetState(&ts);
  i2c_bus_unlock();

  const bool pressed = (status == TS_OK && ts.touchDetected > 0U);
  if (pressed)
  {
    last_x = ts.touchX[0];
    last_y = ts.touchY[0];
  }
  s_pressed = pressed;

  data->state   = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  /* LVGL expects the last known position also while released */
  data->point.x = last_x;
  data->point.y = last_y;
}

lv_indev_t *touch_init(TaskHandle_t gui_task)
{
  s_gui_task = gui_task;

  i2c_bus_lock();
  uint8_t status = BSP_TS_Init(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  if (status == TS_OK)
  {
    /* INT pin on EXTI, controller in trigger mode */
    status = BSP_TS_ITConfig();
  }
  i2c_bus_unlock();

  if (status != TS_OK)
  {
    LOG_ERROR("touch: controller not found");
    return NULL;
  }

  s_indev = lv_indev_create();
  lv_indev_set_type(s_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(s_indev, read_cb);
  lv_indev_set_mode(s_indev, LV_INDEV_MODE_EVENT);
  return s_indev;
}

bool touch_read_pending(void)
{
  return s_indev != NULL && (s_data_pending || s_pressed);
}

void touch_process(void)
{
  if (touch_read_pending())
  {
    lv_indev_read(s_indev);
  }
}

void touch_irq_handler(void)
{
  HAL_GPIO_EXTI_IRQHandler(TS_INT_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != TS_INT_PIN)
  {
    return;
  }
  s_data_pending = true;
  if (s_gui_task != NULL)
  {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_gui_task, &woken);
    portYIELD_FROM_ISR(woken);
  }
}
