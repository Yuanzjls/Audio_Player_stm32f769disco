/**
 * @file  sysmon.h
 * @brief CPU load (time not spent sleeping in the idle task) and RTOS heap use.
 */
#ifndef SYSMON_H
#define SYSMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Start the measurement timer (TIM2). Call once, after the clock configuration. */
void sysmon_init(void);

/** Sleep until the next interrupt and account the time as idle (FreeRTOS idle hook). */
void sysmon_idle(void);

/** CPU load in percent since the previous call. */
uint32_t sysmon_cpu_load(void);

/** FreeRTOS heap: bytes in use and total size. */
void sysmon_rtos_heap(uint32_t *used, uint32_t *total);

#ifdef __cplusplus
}
#endif

#endif /* SYSMON_H */
