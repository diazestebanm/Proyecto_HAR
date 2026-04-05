/**
 * @file hal_sensors.h
 * @brief API publica de acceso a sensores del Nodo 1.
 */

#pragma once

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Inicializa IMU, PPG y bateria.
 * @param[in] ppg_task_handle Tarea a notificar cuando ocurra una interrupcion PPG.
 */
esp_err_t hal_sensors_init(TaskHandle_t ppg_task_handle);

/** @brief Activa la IMU integrada. */
esp_err_t hal_imu_start(void);
/** @brief Detiene temporalmente la IMU integrada. */
esp_err_t hal_imu_stop(void);
/** @brief Lee una muestra corregida de la IMU. */
esp_err_t hal_imu_read_sample(imu_sample_t *out);
/** @brief Consulta si el sesgo del giroscopio ya fue calibrado. */
bool      hal_imu_bias_ok(void);

/** @brief Arranca el sensor PPG. */
esp_err_t hal_ppg_start(void);
/** @brief Detiene el sensor PPG. */
esp_err_t hal_ppg_stop(void);
/** @brief Atiende el sensor PPG y actualiza sus metricas. */
esp_err_t hal_ppg_service(ppg_metrics_t *out);

/** @brief Lee y resume el estado de la bateria. */
esp_err_t hal_battery_read(battery_status_t *out);