/**
 * @file  i2c_bus.h
 * @brief Lock for the I2C4 bus shared by the WM8994 audio codec and the
 *        FT6x06 touch controller.
 *
 * The BSP drives both devices through the same HAL I2C handle without any
 * locking, so every BSP/codec/touch call must be wrapped in
 * i2c_bus_lock() / i2c_bus_unlock().
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Create the lock. Must be called before the scheduler starts. */
void i2c_bus_init(void);
void i2c_bus_lock(void);
void i2c_bus_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_H */
