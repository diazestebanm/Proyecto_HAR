#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file app_types.h
 * @brief Estructuras comunes del Nodo 2.
 */

/** @brief Tipos de mensaje esperados desde Nodo 1. */
typedef enum {
    APP_MSG_TYPE_NORMAL = 1,
    APP_MSG_TYPE_ALARM  = 2,
} app_msg_type_t;

/** @brief Flags generales de calidad para IMU. */
enum {
    APP_FLAG_IMU_VALID_WINDOW = (1u << 0),
    APP_FLAG_IMU_WINDOW_COMPLETE = (1u << 1),
    APP_FLAG_IMU_SAMPLE_LOSS = (1u << 2),
    APP_FLAG_GYRO_BIAS_OK = (1u << 3),
    APP_FLAG_BAT_VALID = (1u << 4),
    APP_FLAG_BAT_LOW = (1u << 5),
    APP_FLAG_BAT_CRITICAL = (1u << 6),
    APP_FLAG_MATCH_OK = (1u << 7),
};

/** @brief Resumen IMU comun para N1 y N2. Todo en enteros compactos. */
typedef struct {
    int16_t acc_mean_xyz[3];  /**< Media del acelerometro por eje en mg. */
    int16_t acc_std_xyz[3];   /**< Desviacion estandar del acelerometro por eje. */
    int16_t acc_rms_xyz[3];
    int16_t acc_mag_mean;
    int16_t acc_mag_std;
    int16_t acc_sma;

    int16_t gyro_mean_xyz[3]; /**< Media del giroscopio por eje en 0.1 dps. */
    int16_t gyro_std_xyz[3];
    int16_t gyro_rms_xyz[3];
    int16_t gyro_mag_mean;
    int16_t gyro_mag_std;

    uint8_t peak_count;       /**< Conteo simple de picos sobre la magnitud acelerometrica. */
} imu_feature_summary_t;

/** @brief Payload esperado del Nodo 1, alineado con las decisiones cerradas. */
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
} node1_packet_t;

/** @brief Muestra cruda del MPU6050 ya escalada para la ventana local. */
typedef struct {
    int16_t acc_mg[3];
    int16_t gyro_dps10[3];
    uint32_t t_rel_ms;
} imu_sample_t;

/** @brief Resultado de una ventana local del Nodo 2. */
typedef struct {
    uint16_t boot_id;
    uint16_t window_index;
    uint32_t win_start_ms;
    uint32_t win_end_ms;
    uint16_t sample_count;
    uint16_t flags;
    imu_feature_summary_t imu;
} node2_window_result_t;

/** @brief Estado resumido de bateria. */
typedef struct {
    bool valid;
    bool low;
    bool critical;
    float voltage_v;
    uint8_t percent;
    uint16_t flags;
} battery_status_t;

/** @brief Evento recibido desde ESP-NOW. */
typedef struct {
    uint8_t src_mac[6];
    uint32_t rx_local_ms;
    node1_packet_t packet;
} radio_rx_event_t;

/** @brief Registro final que se envia a la tarea SD. */
typedef struct {
    uint32_t session_id;
    uint32_t match_delta_ms;
    uint32_t rx_local_ms;
    int32_t  offset_est_ms;

    battery_status_t bat;
    node2_window_result_t n2;
    radio_rx_event_t n1;
} log_record_t;

/** @brief Comandos enviados desde ControlFSM hacia TaskIMU. */
typedef enum {
    APP_IMU_CMD_CAPTURE = 1,
    APP_IMU_CMD_CALIBRATE = 2,
} app_imu_command_t;

#endif /* APP_TYPES_H */
