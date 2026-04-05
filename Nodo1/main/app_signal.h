/**
 * @file app_signal.h
 * @brief Tipos y prototipos para el procesamiento ligero de senales del Nodo 1.
 */

#pragma once

#include "config.h"

/**
 * @brief Contexto acumulador usado para resumir una ventana completa de IMU.
 */
typedef struct {
    uint16_t count;
    float sum_acc[3];
    float sum2_acc[3];
    float sum_gyro[3];
    float sum2_gyro[3];

    float sum_acc_mag;
    float sum2_acc_mag;
    float sum_gyro_mag;
    float sum2_gyro_mag;
    float sum_sma;

    float acc_mag_hist[APP_IMU_EXPECTED_SAMPLES + 4];
} app_signal_ctx_t;

/**
 * @brief Limpia el contexto de acumulacion para iniciar una nueva ventana.
 * @param[in,out] ctx Contexto a reiniciar.
 */
void app_signal_reset(app_signal_ctx_t *ctx);
/**
 * @brief Agrega una muestra al contexto estadistico.
 * @param[in,out] ctx Contexto de la ventana actual.
 * @param[in] s Muestra corregida de acelerometro y giroscopio.
 */
void app_signal_push_imu(app_signal_ctx_t *ctx, const imu_sample_t *s);
/**
 * @brief Genera los rasgos finales de IMU para una ventana ya capturada.
 * @param[in] ctx Contexto de acumulacion.
 * @param[out] out Rasgos calculados.
 */
void app_signal_build_features(const app_signal_ctx_t *ctx, imu_features_t *out);
/**
 * @brief Empaqueta rasgos y metadatos en el formato de radio del proyecto.
 */
void app_signal_build_payload(adl_payload_t *p,
                              app_msg_type_t type,
                              uint16_t boot_id,
                              uint16_t seq,
                              uint32_t t_rel_ms,
                              uint16_t flags,
                              const imu_features_t *imu,
                              const ppg_metrics_t *ppg,
                              const battery_status_t *bat);