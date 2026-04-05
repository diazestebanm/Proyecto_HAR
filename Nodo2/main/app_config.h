#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/**
 * @file app_config.h
 * @brief Configuracion central del proyecto Nodo 2.
 *
 * Este archivo concentra pines, frecuencias, periodos y limites del sistema.
 * La idea es que cualquier ajuste de hardware o politica de energia se haga aqui,
 * sin tener que tocar la logica principal del proyecto.
 */

/* -------------------------------------------------------------------------- */
/* Hardware: ESP32 DevKit V1 + MPU6050 + divisor de bateria                    */
/* -------------------------------------------------------------------------- */
#define APP_I2C_PORT                 0 /* I2C_NUM_0 */
#define APP_I2C_SDA_GPIO             GPIO_NUM_21
#define APP_I2C_SCL_GPIO             GPIO_NUM_22
#define APP_I2C_FREQ_HZ              100000

#define APP_MPU_ADDR                 0x68
#define APP_MPU_INT_GPIO             GPIO_NUM_25

#define APP_BAT_ADC_UNIT             ADC_UNIT_1
#define APP_BAT_ADC_CHANNEL          ADC_CHANNEL_6   /* GPIO34 */
#define APP_BAT_GPIO                 GPIO_NUM_34
#define APP_BAT_R1_OHMS              100000.0f
#define APP_BAT_R2_OHMS              100000.0f
#define APP_BAT_DIVIDER_RATIO        ((APP_BAT_R1_OHMS + APP_BAT_R2_OHMS) / APP_BAT_R2_OHMS)

/* -------------------------------------------------------------------------- */
/* Politica temporal y de potencia                                             */
/* -------------------------------------------------------------------------- */
#define APP_CYCLE_PERIOD_MS          10000U
#define APP_CAPTURE_WINDOW_MS        3000U
#define APP_MATCH_WINDOW_MS          1500U
#define APP_NODE1_WAIT_TIMEOUT_MS    20000U
#define APP_BATTERY_SAMPLE_PERIOD_MS 60000U
#define APP_SD_FLUSH_PERIOD_MS       60000U
#define APP_SD_BATCH_LINES           6U

/* -------------------------------------------------------------------------- */
/* IMU: decisiones finales de Nodo 2                                           */
/* -------------------------------------------------------------------------- */
#define APP_IMU_SAMPLE_RATE_HZ       25U
#define APP_IMU_WINDOW_SAMPLES       ((APP_CAPTURE_WINDOW_MS * APP_IMU_SAMPLE_RATE_HZ) / 1000U)
#define APP_GYRO_BIAS_SAMPLES        64U
#define APP_MPU_ACCEL_LSB_PER_G      16384.0f
#define APP_MPU_GYRO_LSB_PER_DPS     131.0f
#define APP_MPU_PEAK_SIGMA_FACTOR    0.50f

/* -------------------------------------------------------------------------- */
/* ESP-NOW                                                                     */
/* -------------------------------------------------------------------------- */
#define APP_WIFI_CHANNEL             1
#define APP_USE_ESPNOW_RX_PS         0
#define APP_ESPNOW_WAKE_INTERVAL_MS  APP_CYCLE_PERIOD_MS
#define APP_ESPNOW_WAKE_WINDOW_MS    1500U

/*
 * Si se quiere filtrar solo paquetes del nodo 1, se puede activar este macro
 * y editar la MAC fija. Por defecto queda desactivado para facilitar pruebas.
 */
#define APP_FILTER_NODE1_MAC         0
#define APP_NODE1_MAC0               0x00
#define APP_NODE1_MAC1               0x00
#define APP_NODE1_MAC2               0x00
#define APP_NODE1_MAC3               0x00
#define APP_NODE1_MAC4               0x00
#define APP_NODE1_MAC5               0x00

/* -------------------------------------------------------------------------- */
/* Prioridades y tamanos de tareas                                             */
/* -------------------------------------------------------------------------- */
#define APP_TASK_STACK_DEFAULT       4096
#define APP_TASK_STACK_SD            6144
#define APP_TASK_PRIO_CONTROL        7
#define APP_TASK_PRIO_IMU            6
#define APP_TASK_PRIO_RADIO          5
#define APP_TASK_PRIO_SD             4

/* -------------------------------------------------------------------------- */
/* Bateria                                                                     */
/* -------------------------------------------------------------------------- */
#define APP_BAT_NUM_SAMPLES          16
#define APP_BAT_FLOAT_RANGE_THRESH   350
#define APP_BAT_EMA_ALPHA            0.15f
#define APP_BAT_PCT_HYSTERESIS       2.0f
#define APP_BAT_MIN_VALID_V          2.80f
#define APP_BAT_MAX_VALID_V          4.40f
#define APP_BAT_LOW_V                3.50f
#define APP_BAT_CRITICAL_V           3.40f

/* -------------------------------------------------------------------------- */
/* Logging                                                                     */
/* -------------------------------------------------------------------------- */
#define APP_LOG_LINE_MAX             1024
#define APP_SESSION_FILENAME_MAX     64

#define APP_SERIAL_SUMMARY_ENABLE          1
#define APP_SERIAL_SUMMARY_EVERY_N_WINDOWS 1U

#endif /* APP_CONFIG_H */
