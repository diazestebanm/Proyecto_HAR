/**
 * @file config.h
 * @brief Configuracion central y tipos compartidos del Nodo 1.
 *
 * Reune pines, tiempos, umbrales, estructuras de intercambio y banderas
 * utilizadas por los modulos de adquisicion, procesamiento y radio.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

/* =========================
 * Proyecto / Logs
 * ========================= */
#define APP_TAG                 "NODO1_ADL"

/* =========================
 * Hardware
 * ========================= */
#define APP_IMU_I2C_PORT        I2C_NUM_0
#define APP_IMU_SDA_GPIO        GPIO_NUM_6
#define APP_IMU_SCL_GPIO        GPIO_NUM_7
#define APP_IMU_I2C_HZ          400000

#define APP_QMI8658_ADDR1       0x6A
#define APP_QMI8658_ADDR2       0x6B
#define APP_QMI_REG_WHOAMI      0x00
#define APP_QMI_REG_CTRL1       0x02
#define APP_QMI_REG_CTRL2       0x03
#define APP_QMI_REG_CTRL3       0x04
#define APP_QMI_REG_CTRL7       0x08
#define APP_QMI_REG_AX_L        0x35

#define APP_PPG_I2C_PORT        I2C_NUM_1
#define APP_PPG_SDA_GPIO        GPIO_NUM_16
#define APP_PPG_SCL_GPIO        GPIO_NUM_18
#define APP_PPG_INT_GPIO        GPIO_NUM_15
#define APP_PPG_I2C_HZ          100000

#define APP_MAX30100_ADDR       0x57
#define APP_MAX_REG_INT_STATUS  0x00
#define APP_MAX_REG_INT_ENABLE  0x01
#define APP_MAX_REG_FIFO_WR     0x02
#define APP_MAX_REG_FIFO_OVF    0x03
#define APP_MAX_REG_FIFO_RD     0x04
#define APP_MAX_REG_FIFO_DATA   0x05
#define APP_MAX_REG_MODE_CFG    0x06
#define APP_MAX_REG_SPO2_CFG    0x07
#define APP_MAX_REG_LED_CFG     0x09
#define APP_MAX_REG_REV_ID      0xFE
#define APP_MAX_REG_PART_ID     0xFF

#define APP_MAX_MODE_SPO2       0x03
#define APP_MAX_MODE_RESET      0x40
#define APP_MAX_SPO2_CFG        0x47   /* valor ya usado en tu código funcional */
#define APP_MAX_LED_CFG         0x77

#define APP_BAT_ADC_UNIT        ADC_UNIT_2
#define APP_BAT_ADC_CHANNEL     ADC_CHANNEL_6  /* GPIO17 */
#define APP_BAT_GPIO            GPIO_NUM_17
#define APP_BAT_DIV_RATIO       2.0f

/* =========================
 * Muestreo / tiempos
 * ========================= */
#define APP_IMU_WINDOW_MS               3000U
#define APP_IMU_RATE_HZ                 11U
#define APP_IMU_EXPECTED_SAMPLES        ((APP_IMU_WINDOW_MS * APP_IMU_RATE_HZ) / 1000U)
#define APP_IMU_SAMPLE_PERIOD_MS        (1000U / APP_IMU_RATE_HZ)

#define APP_PPG_WINDOW_SEC              4U
#define APP_PPG_SAMPLE_RATE_HZ          100U
#define APP_PPG_WINDOW_SAMPLES          (APP_PPG_WINDOW_SEC * APP_PPG_SAMPLE_RATE_HZ)

#define APP_TX_PERIOD_MS                1000U
#define APP_BAT_ACTIVE_PERIOD_MS        30000U
#define APP_BAT_IDLE_PERIOD_MS          120000U
#define APP_RADIO_SEND_TIMEOUT_MS       400U

/* =========================
 * Calidad / umbrales
 * ========================= */
#define APP_IMU_MIN_VALID_PCT           80U
#define APP_LINK_FAIL_THRESHOLD         3U
#define APP_BAT_CRITICAL_MV             3300U
#define APP_BAT_LOW_MV                  3400U

/* Ajustables: confírmalos luego con tu configuración final de escala */
#define APP_ACC_MG_PER_LSB              1.0f
#define APP_GYRO_DPS10_PER_LSB          1.0f

/* =========================
 * Peer ESP-NOW
 * ========================= */
#define APP_ESPNOW_CHANNEL              1
static const uint8_t APP_PEER_MAC[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

/* =========================
 * Flags payload
 * ========================= */
/**
 * @brief Banderas codificadas en el payload del Nodo 1.
 */
typedef enum {
    APP_FLAG_BOOT_EVENT          = (1u << 0),
    APP_FLAG_IMU_VALID_WINDOW    = (1u << 1),
    APP_FLAG_IMU_WINDOW_COMPLETE = (1u << 2),
    APP_FLAG_IMU_SAMPLE_LOSS     = (1u << 3),
    APP_FLAG_GYRO_BIAS_OK        = (1u << 4),
    APP_FLAG_PPG_FINGER_PRESENT  = (1u << 5),
    APP_FLAG_BPM_VALID           = (1u << 6),
    APP_FLAG_SPO2_VALID          = (1u << 7),
    APP_FLAG_BAT_VALID           = (1u << 8),
    APP_FLAG_BAT_LOW             = (1u << 9),
    APP_FLAG_BAT_CRITICAL        = (1u << 10),
    APP_FLAG_LINK_DEGRADED       = (1u << 11),
} app_flags_t;

/* =========================
 * FSM
 * ========================= */
/**
 * @brief Estados principales de la FSM de control del Nodo 1.
 */
typedef enum {
    APP_STATE_BOOT = 0,
    APP_STATE_IDLE_LOW_POWER,
    APP_STATE_ACQUIRE_IMU,
    APP_STATE_ACQUIRE_PPG,
    APP_STATE_BUILD_PAYLOAD,
    APP_STATE_TX_NORMAL,
    APP_STATE_TX_ALARM,
    APP_STATE_RECOVERY
} app_state_t;

/* =========================
 * Tipos internos
 * ========================= */
/**
 * @brief Muestra instantanea de la IMU integrada del nodo de muneca.
 */
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t t_ms;
} imu_sample_t;

/**
 * @brief Metricas resumidas del sensor PPG.
 */
typedef struct {
    bool finger_present;
    bool bpm_valid;
    bool spo2_valid;
    uint8_t bpm;
    uint8_t spo2;
    uint16_t dc_ir;
} ppg_metrics_t;

/**
 * @brief Estado resumido de la bateria local.
 */
typedef struct {
    bool valid;
    bool low;
    bool critical;
    uint16_t mv;
    uint8_t pct;
} battery_status_t;

/**
 * @brief Rasgos temporales calculados a partir de una ventana IMU.
 */
typedef struct {
    bool valid;
    uint16_t sample_count;
    uint8_t peak_count;

    int16_t acc_mean[3];
    int16_t acc_std[3];
    int16_t acc_rms[3];
    int16_t acc_mag_mean;
    int16_t acc_mag_std;
    int16_t acc_sma;

    int16_t gyro_mean[3];
    int16_t gyro_std[3];
    int16_t gyro_rms[3];
    int16_t gyro_mag_mean;
    int16_t gyro_mag_std;
} imu_features_t;

/**
 * @brief Tipos de mensaje transmitidos por el Nodo 1.
 */
typedef enum {
    APP_MSG_NORMAL = 1,
    APP_MSG_ALARM  = 2
} app_msg_type_t;

/**
 * @brief Payload binario enviado por ESP-NOW desde el Nodo 1.
 */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;
    uint16_t boot_id;
    uint16_t seq;
    uint32_t t_rel_ms;
    uint16_t window_ms;
    uint16_t sample_count;
    uint16_t flags;

    int16_t acc_mean[3];
    int16_t acc_std[3];
    int16_t acc_rms[3];
    int16_t acc_mag_mean;
    int16_t acc_mag_std;
    int16_t acc_sma;

    int16_t gyro_mean[3];
    int16_t gyro_std[3];
    int16_t gyro_rms[3];
    int16_t gyro_mag_mean;
    int16_t gyro_mag_std;

    uint8_t peak_count;
    uint8_t bpm;
    uint8_t spo2;
    uint8_t battery_pct;
    uint16_t battery_mv;
} adl_payload_t;

/**
 * @brief Elemento de cola usado por la tarea de radio.
 */
typedef struct {
    adl_payload_t payload;
    bool critical;
} radio_tx_item_t;