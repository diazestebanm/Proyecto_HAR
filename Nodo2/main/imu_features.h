#ifndef IMU_FEATURES_H
#define IMU_FEATURES_H

#include <stddef.h>
#include <stdint.h>
#include "app_types.h"

/**
 * @file imu_features.h
 * @brief Extraccion ligera de rasgos temporales para IMU.
 */

/** @brief Calcula el resumen de rasgos temporales para una ventana IMU. */
void imu_features_compute(const imu_sample_t *samples, size_t count, imu_feature_summary_t *out);

#endif /* IMU_FEATURES_H */
