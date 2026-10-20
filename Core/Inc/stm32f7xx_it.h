/**
 * @file  stm32f7xx_it.h
 * @brief Exception and interrupt handler prototypes.
 */
#ifndef STM32F7XX_IT_H
#define STM32F7XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);

void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void SDMMC2_IRQHandler(void);
void DSI_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F7XX_IT_H */
