/**
 * @file hal_i2c.h
 * @brief API del bus I2C del Nodo 2.
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/** @brief Inicializa el bus I2C maestro del Nodo 2. */
esp_err_t hal_i2c_init(void);
/** @brief Escribe un registro de un esclavo I2C. */
esp_err_t hal_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value);
/** @brief Lee uno o varios bytes desde un esclavo I2C. */
esp_err_t hal_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len);

#endif /* HAL_I2C_H */
