/**
 * @file imu_features.c
 * @brief Extraccion ligera de rasgos temporales para el Nodo 2.
 */

#include "imu_features.h"

#include <math.h>
#include <string.h>
#include "app_config.h"

/** @brief Satura un valor flotante al rango de int16_t. */
static int16_t sat16f(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lroundf(v);
}

/**
 * @brief Resume una ventana local de IMU en rasgos compactos.
 * @param[in] samples Ventana de muestras locales.
 * @param[in] count Numero de muestras validas en la ventana.
 * @param[out] out Resumen final de rasgos.
 */
void imu_features_compute(const imu_sample_t *samples, size_t count, imu_feature_summary_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!samples || count == 0) {
        return;
    }

    float acc_sum[3] = {0};
    float acc_sq_sum[3] = {0};
    float gyro_sum[3] = {0};
    float gyro_sq_sum[3] = {0};
    float acc_mag_sum = 0.0f;
    float acc_mag_sq_sum = 0.0f;
    float gyro_mag_sum = 0.0f;
    float gyro_mag_sq_sum = 0.0f;
    float acc_sma_sum = 0.0f;

    float acc_mag[APP_IMU_WINDOW_SAMPLES];

    for (size_t i = 0; i < count; ++i) {
        const float ax = samples[i].acc_mg[0];
        const float ay = samples[i].acc_mg[1];
        const float az = samples[i].acc_mg[2];
        const float gx = samples[i].gyro_dps10[0];
        const float gy = samples[i].gyro_dps10[1];
        const float gz = samples[i].gyro_dps10[2];

        acc_sum[0] += ax; acc_sum[1] += ay; acc_sum[2] += az;
        gyro_sum[0] += gx; gyro_sum[1] += gy; gyro_sum[2] += gz;

        acc_sq_sum[0] += ax * ax; acc_sq_sum[1] += ay * ay; acc_sq_sum[2] += az * az;
        gyro_sq_sum[0] += gx * gx; gyro_sq_sum[1] += gy * gy; gyro_sq_sum[2] += gz * gz;

        const float amag = sqrtf(ax * ax + ay * ay + az * az);
        const float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
        acc_mag[i] = amag;

        acc_mag_sum += amag;
        acc_mag_sq_sum += amag * amag;
        gyro_mag_sum += gmag;
        gyro_mag_sq_sum += gmag * gmag;
        acc_sma_sum += fabsf(ax) + fabsf(ay) + fabsf(az);
    }

    const float inv_n = 1.0f / (float)count;
    float acc_mean[3];
    float gyro_mean[3];
    float acc_std[3];
    float gyro_std[3];
    float acc_rms[3];
    float gyro_rms[3];

    for (int k = 0; k < 3; ++k) {
        acc_mean[k] = acc_sum[k] * inv_n;
        gyro_mean[k] = gyro_sum[k] * inv_n;

        acc_rms[k] = sqrtf(acc_sq_sum[k] * inv_n);
        gyro_rms[k] = sqrtf(gyro_sq_sum[k] * inv_n);

        const float avar = fmaxf(0.0f, (acc_sq_sum[k] * inv_n) - (acc_mean[k] * acc_mean[k]));
        const float gvar = fmaxf(0.0f, (gyro_sq_sum[k] * inv_n) - (gyro_mean[k] * gyro_mean[k]));
        acc_std[k] = sqrtf(avar);
        gyro_std[k] = sqrtf(gvar);

        out->acc_mean_xyz[k] = sat16f(acc_mean[k]);
        out->acc_std_xyz[k]  = sat16f(acc_std[k]);
        out->acc_rms_xyz[k]  = sat16f(acc_rms[k]);
        out->gyro_mean_xyz[k] = sat16f(gyro_mean[k]);
        out->gyro_std_xyz[k]  = sat16f(gyro_std[k]);
        out->gyro_rms_xyz[k]  = sat16f(gyro_rms[k]);
    }

    const float acc_mag_mean = acc_mag_sum * inv_n;
    const float gyro_mag_mean = gyro_mag_sum * inv_n;
    const float acc_mag_var = fmaxf(0.0f, (acc_mag_sq_sum * inv_n) - (acc_mag_mean * acc_mag_mean));
    const float gyro_mag_var = fmaxf(0.0f, (gyro_mag_sq_sum * inv_n) - (gyro_mag_mean * gyro_mag_mean));
    const float acc_mag_std = sqrtf(acc_mag_var);
    const float gyro_mag_std = sqrtf(gyro_mag_var);

    out->acc_mag_mean = sat16f(acc_mag_mean);
    out->acc_mag_std  = sat16f(acc_mag_std);
    out->acc_sma      = sat16f(acc_sma_sum * inv_n);
    out->gyro_mag_mean = sat16f(gyro_mag_mean);
    out->gyro_mag_std  = sat16f(gyro_mag_std);

    uint8_t peaks = 0;
    const float peak_threshold = acc_mag_mean + (APP_MPU_PEAK_SIGMA_FACTOR * acc_mag_std);
    for (size_t i = 1; i + 1 < count; ++i) {
        if (acc_mag[i] > acc_mag[i - 1] && acc_mag[i] >= acc_mag[i + 1] && acc_mag[i] > peak_threshold) {
            ++peaks;
        }
    }
    out->peak_count = peaks;
}
