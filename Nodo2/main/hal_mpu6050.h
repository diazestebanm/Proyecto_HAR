/**
 * @file hal_mpu6050.h
 * @brief HAL del MPU6050 orientada a bajo consumo y captura por interrupcion.
 */

#ifndef HAL_MPU6050_H
#define HAL_MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_types.h"

/**
 * @brief Parametros de calibracion del giroscopio del MPU6050.
 *
 * El sesgo se estima al inicio con el sensor quieto y luego se resta durante
 * la captura de cada ventana para reducir el offset DC en los ejes del gyro.
 */
typedef struct {
    int32_t gyro_bias_raw[3]; /**< Sesgo medio crudo por eje en LSB del gyro. */
    bool bias_valid;          /**< Indica si la calibracion puede aplicarse. */
} mpu6050_calibration_t;

/**
 * @brief Inicializa el MPU6050, el pin de interrupcion y los recursos RTOS.
 *
 * @param[in] imu_task_to_notify Tarea asociada a la captura local.
 * @return ESP_OK si el sensor queda operativo.
 */
esp_err_t hal_mpu6050_init(TaskHandle_t imu_task_to_notify);

/**
 * @brief Verifica que el MPU6050 siga respondiendo por I2C.
 *
 * @return ESP_OK si el WHO_AM_I es valido.
 */
esp_err_t hal_mpu6050_verify(void);

/**
 * @brief Estima el sesgo del giroscopio con el nodo en reposo.
 *
 * @param[out] cal Estructura donde se almacena la calibracion.
 * @return ESP_OK si se reunieron suficientes muestras.
 */
esp_err_t hal_mpu6050_calibrate_gyro(mpu6050_calibration_t *cal);

/**
 * @brief Captura una ventana local de muestras del MPU6050.
 *
 * @param[in] cal Calibracion del giroscopio; puede ser nula.
 * @param[out] samples Buffer destino para las muestras convertidas.
 * @param[in] target_samples Numero de muestras objetivo de la ventana.
 * @param[out] captured_samples Numero real de muestras obtenidas.
 * @param[out] flags Banderas de calidad de la ventana.
 * @param[in] t0_ms Referencia temporal de la sesion en milisegundos.
 * @return ESP_OK si la captura finaliza sin errores fatales.
 */
esp_err_t hal_mpu6050_capture_window(const mpu6050_calibration_t *cal,
                                     imu_sample_t *samples,
                                     uint16_t target_samples,
                                     uint16_t *captured_samples,
                                     uint16_t *flags,
                                     uint32_t t0_ms);

/**
 * @brief Reintenta recuperar el sensor y el bus I2C despues de un fallo.
 *
 * @return ESP_OK si la reconfiguracion del MPU vuelve a ser valida.
 */
esp_err_t hal_mpu6050_recover(void);

#endif /* HAL_MPU6050_H */
