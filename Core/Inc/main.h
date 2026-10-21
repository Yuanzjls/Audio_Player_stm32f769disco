/**
 * @file  main.h
 * @brief Board-wide definitions shared by the HAL MSP code and the drivers.
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "stm32f769i_discovery.h"

/* Places a DMA buffer in DTCM, which is never cached (see the linker script).
 * Contents are not zeroed at start-up. */
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

/* USART1 on PA9/PA10 is wired to the ST-LINK virtual COM port (used for logging) */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/** Unrecoverable error: halts with the red LED on (breaks into the debugger if attached). */
void Error_Handler(void);

/** Firmware version ("git describe" at build time, e.g. "v2.0.0"). */
const char *firmware_version(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
