#ifndef HAL_MPU6050_H
#define HAL_MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_types.h"

/**
 * @file hal_mpu6050.h
 * @brief HAL del MPU6050 orientada a bajo consumo y captura por interrupcion.
 */

/**
 * @brief Parametros de calibracion del giroscopio del MPU6050.
 */
typedef struct {
    int32_t gyro_bias_raw[3];
    bool bias_valid;
} mpu6050_calibration_t;

/** @brief Inicializa el MPU6050 y su pin de interrupcion. */
esp_err_t hal_mpu6050_init(TaskHandle_t imu_task_to_notify);
/** @brief Verifica la presencia del MPU6050. */
esp_err_t hal_mpu6050_verify(void);
/** @brief Obtiene el sesgo del giroscopio en reposo. */
esp_err_t hal_mpu6050_calibrate_gyro(mpu6050_calibration_t *cal);
/** @brief Captura una ventana local de muestras del MPU6050. */
esp_err_t hal_mpu6050_capture_window(const mpu6050_calibration_t *cal,
                                     imu_sample_t *samples,
                                     uint16_t target_samples,
                                     uint16_t *captured_samples,
                                     uint16_t *flags,
                                     uint32_t t0_ms);

#endif /* HAL_MPU6050_H */
