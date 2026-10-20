/**
 * @file  log.h
 * @brief Thread-safe text logging over the ST-LINK virtual COM port
 *        (USART1, 115200 baud, 8N1).
 */
#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/** Configure USART1. Safe to call before the scheduler starts. */
void log_init(void);

#include <stddef.h>

/** Raw output (used by printf via _write). Usable from tasks only (not ISRs). */
void log_write(const char *data, size_t len);

/** printf-style logging; a newline is appended. Usable from tasks only (not ISRs). */
void log_printf(const char *level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#define LOG_INFO(...)  log_printf("I", __VA_ARGS__)
#define LOG_WARN(...)  log_printf("W", __VA_ARGS__)
#define LOG_ERROR(...) log_printf("E", __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* LOG_H */
