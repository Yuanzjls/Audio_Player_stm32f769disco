/**
 * @file  i2c_bus.c
 * @brief Recursive mutex protecting the shared audio/touch I2C bus.
 */
#include "i2c_bus.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t s_mutex;

void i2c_bus_init(void)
{
  s_mutex = xSemaphoreCreateRecursiveMutex();
  configASSERT(s_mutex != NULL);
}

/* Before the scheduler runs there is only one thread of execution, so the
 * lock is a no-op then (taking a mutex before vTaskStartScheduler is not allowed). */
static int scheduler_running(void)
{
  return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

void i2c_bus_lock(void)
{
  if (scheduler_running())
  {
    xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
  }
}

void i2c_bus_unlock(void)
{
  if (scheduler_running())
  {
    xSemaphoreGiveRecursive(s_mutex);
  }
}
