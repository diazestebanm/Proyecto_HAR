/**
 * @file hal_i2c.h
 * @brief API de acceso I2C del Nodo 2.
 *
 * Este modulo encapsula la configuracion del bus I2C maestro usado por el
 * MPU6050. Se separa en una HAL propia para que la logica del sensor quede
 * desacoplada de los detalles del driver de ESP-IDF.
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Inicializa el periférico I2C con los pines y frecuencia definidos
 * en @ref app_config.h.
 *
 * @return
 *      - ESP_OK si el bus queda listo.
 *      - Codigo de error de ESP-IDF si la configuracion o la instalacion del
 *        driver falla.
 */
esp_err_t hal_i2c_init(void);

/**
 * @brief Libera el driver I2C del Nodo 2.
 *
 * @return ESP_OK incluso si el bus ya estaba desmontado.
 */
esp_err_t hal_i2c_deinit(void);

/**
 * @brief Verifica si un dispositivo responde en una direccion I2C.
 *
 * @param[in] dev_addr Direccion de 7 bits del dispositivo.
 * @return ESP_OK si el esclavo responde con ACK.
 */
esp_err_t hal_i2c_ping(uint8_t dev_addr);

/**
 * @brief Escribe un solo registro de 8 bits en un dispositivo I2C.
 *
 * @param[in] dev_addr Direccion de 7 bits del esclavo.
 * @param[in] reg Registro a modificar.
 * @param[in] value Valor a escribir.
 * @return Resultado de la transaccion sobre el bus.
 */
esp_err_t hal_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value);

/**
 * @brief Lee uno o mas bytes a partir de un registro de un esclavo I2C.
 *
 * @param[in] dev_addr Direccion de 7 bits del esclavo.
 * @param[in] reg Registro inicial desde el que se leerá.
 * @param[out] data Buffer de salida.
 * @param[in] len Numero de bytes a recibir.
 * @return Resultado de la operacion de lectura.
 */
esp_err_t hal_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len);

#endif /* HAL_I2C_H */
