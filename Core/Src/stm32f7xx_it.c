/**
 * @file  stm32f7xx_it.c
 * @brief Exception and interrupt handlers. Each peripheral handler forwards
 *        to the driver that owns the peripheral.
 */
#include "main.h"
#include "stm32f7xx_it.h"

#include "FreeRTOS.h"
#include "task.h"

#include "audio_out.h"
#include "display.h"

/* Provided by the FreeRTOS port (port.c) */
extern void xPortSysTickHandler(void);

/* Owned by the BSP SD driver */
extern SD_HandleTypeDef uSdHandle;

/* ---- Cortex-M7 exceptions -------------------------------------------------- */
/* SVC_Handler and PendSV_Handler are provided by the FreeRTOS port.            */

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  Error_Handler();
}

void MemManage_Handler(void)
{
  Error_Handler();
}

void BusFault_Handler(void)
{
  Error_Handler();
}

void UsageFault_Handler(void)
{
  Error_Handler();
}

void DebugMon_Handler(void)
{
}

/* The HAL time base uses TIM6 (stm32f7xx_hal_timebase_tim.c); SysTick is the RTOS tick */
void SysTick_Handler(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
}

/* ---- Peripherals ----------------------------------------------------------- */

/* SAI1 block A TX DMA (audio output) */
void DMA2_Stream6_IRQHandler(void)
{
  audio_out_dma_irq_handler();
}

/* SDMMC2 RX DMA */
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uSdHandle.hdmarx);
}

/* SDMMC2 TX DMA */
void DMA2_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uSdHandle.hdmatx);
}

void SDMMC2_IRQHandler(void)
{
  HAL_SD_IRQHandler(&uSdHandle);
}

/* Tearing effect / end of refresh of the LCD */
void DSI_IRQHandler(void)
{
  display_dsi_irq_handler();
}
