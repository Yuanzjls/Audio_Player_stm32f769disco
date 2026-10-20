/**
 * @file  log.c
 * @brief Logging over USART1 (connected to the ST-LINK virtual COM port).
 *
 * Messages are formatted into a local buffer and sent with a blocking
 * transmit, guarded by a mutex so lines from different tasks never interleave.
 */
#include "log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define LOG_BAUDRATE      115200U
#define LOG_LINE_MAX      160U
#define LOG_TX_TIMEOUT_MS 100U

static UART_HandleTypeDef s_uart;
static SemaphoreHandle_t  s_mutex;

void log_init(void)
{
  s_uart.Instance            = USARTx;
  s_uart.Init.BaudRate       = LOG_BAUDRATE;
  s_uart.Init.WordLength     = UART_WORDLENGTH_8B;
  s_uart.Init.StopBits       = UART_STOPBITS_1;
  s_uart.Init.Parity         = UART_PARITY_NONE;
  s_uart.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  s_uart.Init.Mode           = UART_MODE_TX_RX;
  s_uart.Init.OverSampling   = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&s_uart) != HAL_OK)
  {
    Error_Handler();
  }

  s_mutex = xSemaphoreCreateMutex();
  configASSERT(s_mutex != NULL);
}

void log_printf(const char *level, const char *fmt, ...)
{
  char line[LOG_LINE_MAX];
  int len = snprintf(line, sizeof(line), "[%8lu] %s: ",
                     (unsigned long)xTaskGetTickCount(), level);

  va_list args;
  va_start(args, fmt);
  len += vsnprintf(&line[len], sizeof(line) - (size_t)len, fmt, args);
  va_end(args);

  /* Truncate overlong lines but always keep the line ending */
  if (len > (int)sizeof(line) - 3)
  {
    len = (int)sizeof(line) - 3;
  }
  line[len++] = '\r';
  line[len++] = '\n';

  log_write(line, (size_t)len);
}

void log_write(const char *data, size_t len)
{
  /* Taking a mutex before the scheduler runs is not allowed (and not needed) */
  const bool rtos_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
  if (rtos_running)
  {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
  }
  while (len > 0U)
  {
    const uint16_t chunk = (len > 0xFFFFU) ? 0xFFFFU : (uint16_t)len;
    HAL_UART_Transmit(&s_uart, (uint8_t *)data, chunk, LOG_TX_TIMEOUT_MS);
    data += chunk;
    len  -= chunk;
  }
  if (rtos_running)
  {
    xSemaphoreGive(s_mutex);
  }
}
