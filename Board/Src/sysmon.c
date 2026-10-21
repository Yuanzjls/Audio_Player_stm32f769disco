/**
 * @file  sysmon.c
 * @brief CPU load from the time the idle task sleeps in WFI.
 *
 * Time is measured with TIM2 running freely at 1 MHz. (The core's DWT cycle
 * counter cannot be used: it stops while the core sleeps.) The idle hook
 * disables interrupts, reads the timer, sleeps with WFI (a pending interrupt
 * still wakes the core), reads the timer again and only then re-enables
 * interrupts, so the measured sleep excludes interrupt handlers.
 * load = 1 - sleep / elapsed.
 */
#include "sysmon.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define SYSMON_TIMER      TIM2       /* 32-bit, unused elsewhere */
#define SYSMON_TICK_HZ    1000000U

static volatile uint64_t s_sleep_us;   /* written by the idle task only */
static uint32_t s_last_us;
static uint64_t s_last_sleep;

static inline uint32_t now_us(void)
{
  return SYSMON_TIMER->CNT;
}

void sysmon_init(void)
{
  /* APB1 timers run at twice PCLK1 when the APB1 prescaler is not 1 */
  RCC_ClkInitTypeDef clk;
  uint32_t latency;
  HAL_RCC_GetClockConfig(&clk, &latency);
  uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
  if (clk.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    timer_hz *= 2U;
  }

  __HAL_RCC_TIM2_CLK_ENABLE();
  SYSMON_TIMER->CR1 = 0;
  SYSMON_TIMER->PSC = timer_hz / SYSMON_TICK_HZ - 1U;
  SYSMON_TIMER->ARR = 0xFFFFFFFFU;
  SYSMON_TIMER->EGR = TIM_EGR_UG;           /* load the prescaler */
  SYSMON_TIMER->CR1 = TIM_CR1_CEN;

  s_last_us = now_us();
}

void sysmon_idle(void)
{
  __disable_irq();
  const uint32_t start = now_us();
  __DSB();
  __WFI();
  s_sleep_us += now_us() - start;
  __enable_irq();
}

uint32_t sysmon_cpu_load(void)
{
  taskENTER_CRITICAL();
  const uint32_t now   = now_us();
  const uint64_t sleep = s_sleep_us;
  taskEXIT_CRITICAL();

  const uint32_t elapsed = now - s_last_us;              /* wraps every ~71 min */
  const uint32_t slept   = (uint32_t)(sleep - s_last_sleep);
  s_last_us    = now;
  s_last_sleep = sleep;

  if (elapsed == 0U || slept >= elapsed)
  {
    return 0U;
  }
  return (uint32_t)(((uint64_t)(elapsed - slept) * 100U + elapsed / 2U) / elapsed);
}

void sysmon_rtos_heap(uint32_t *used, uint32_t *total)
{
  *total = configTOTAL_HEAP_SIZE;
  *used  = configTOTAL_HEAP_SIZE - (uint32_t)xPortGetFreeHeapSize();
}
