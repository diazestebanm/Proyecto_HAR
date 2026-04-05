/**
 * @file hal_sdcard.c
 * @brief Backend de almacenamiento por serial hacia el host del Nodo 2.
 *
 * Mantiene la misma interfaz usada por la tarea de persistencia, pero en vez
 * de montar una tarjeta SD emite por stdout un protocolo de lineas con
 * prefijos bien definidos. Un script en el PC escucha el puerto serial y
 * guarda la cabecera y las filas CSV dentro del proyecto.
 *
 * @note El nombre historico del modulo se conserva para no cambiar el flujo de
 * tareas, pero en esta revision la microSD queda oficialmente fuera de uso.
 */

#include "hal_sdcard.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "HAL_STORAGE";
static bool s_ready = false;
static char s_filename[APP_SESSION_FILENAME_MAX] = {0};

#define HOST_META_PREFIX   "CSV_META|"
#define HOST_HEADER_PREFIX "CSV_HEADER|"
#define HOST_ROW_PREFIX    "CSV_ROW|"

/**
 * @brief Agrega texto formateado a un buffer acumulativo.
 *
 * Se usa para construir la cabecera CSV y cada fila sin depender de memoria
 * dinamica. La funcion valida el espacio libre antes de confirmar la escritura.
 */
static esp_err_t buf_append(char *dst, size_t dst_size, size_t *used, const char *fmt, ...)
{
    if (!dst || !used || !fmt || *used >= dst_size) {
        return ESP_ERR_INVALID_ARG;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(dst + *used, dst_size - *used, fmt, args);
    va_end(args);

    if (written < 0) {
        return ESP_FAIL;
    }

    size_t w = (size_t)written;
    if (w >= (dst_size - *used)) {
        return ESP_ERR_INVALID_SIZE;
    }

    *used += w;
    return ESP_OK;
}

/**
 * @brief Emite una linea del protocolo serial consumido por logger.py.
 */
static void host_emit_line(const char *prefix, const char *payload)
{
    if (!prefix || !payload) {
        return;
    }

    printf("%s%s\n", prefix, payload);
    fflush(stdout);
}

/**
 * @brief Construye la cabecera CSV completa de la sesion.
 */
static esp_err_t csv_build_header(char *line, size_t line_size)
{
    size_t used = 0;
    return buf_append(line, line_size, &used,
        "session_id,match_delta_ms,rx_local_ms,offset_est_ms,"
        "n2_boot_id,n2_win_idx,n2_win_start_ms,n2_win_end_ms,n2_sample_count,n2_flags,n2_bat_pct,n2_bat_v,"
        "n2_acc_mean_x,n2_acc_mean_y,n2_acc_mean_z,n2_acc_std_x,n2_acc_std_y,n2_acc_std_z,n2_acc_rms_x,n2_acc_rms_y,n2_acc_rms_z,"
        "n2_acc_mag_mean,n2_acc_mag_std,n2_acc_sma,"
        "n2_gyro_mean_x,n2_gyro_mean_y,n2_gyro_mean_z,n2_gyro_std_x,n2_gyro_std_y,n2_gyro_std_z,n2_gyro_rms_x,n2_gyro_rms_y,n2_gyro_rms_z,"
        "n2_gyro_mag_mean,n2_gyro_mag_std,n2_peak_count,"
        "n1_boot_id,n1_seq,n1_t_rel_ms,n1_window_ms,n1_sample_count,n1_flags,n1_bpm,n1_spo2,n1_bat_pct,"
        "n1_acc_mean_x,n1_acc_mean_y,n1_acc_mean_z,n1_acc_std_x,n1_acc_std_y,n1_acc_std_z,n1_acc_rms_x,n1_acc_rms_y,n1_acc_rms_z,"
        "n1_acc_mag_mean,n1_acc_mag_std,n1_acc_sma,"
        "n1_gyro_mean_x,n1_gyro_mean_y,n1_gyro_mean_z,n1_gyro_std_x,n1_gyro_std_y,n1_gyro_std_z,n1_gyro_rms_x,n1_gyro_rms_y,n1_gyro_rms_z,"
        "n1_gyro_mag_mean,n1_gyro_mag_std,n1_peak_count");
}

/**
 * @brief Serializa un resumen IMU compacto dentro de una fila CSV.
 */
static esp_err_t csv_append_imu(char *line, size_t line_size, size_t *used, const imu_feature_summary_t *imu)
{
    ESP_RETURN_ON_FALSE(imu != NULL, ESP_ERR_INVALID_ARG, TAG, "imu nulo");

    ESP_RETURN_ON_ERROR(buf_append(line, line_size, used,
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%d,%d,%d,%d,%d,%d,"
        "%d,%d,%d,%d,%d,%u",
        imu->acc_mean_xyz[0], imu->acc_mean_xyz[1], imu->acc_mean_xyz[2],
        imu->acc_std_xyz[0], imu->acc_std_xyz[1], imu->acc_std_xyz[2],
        imu->acc_rms_xyz[0], imu->acc_rms_xyz[1], imu->acc_rms_xyz[2],
        imu->acc_mag_mean, imu->acc_mag_std, imu->acc_sma,
        imu->gyro_mean_xyz[0], imu->gyro_mean_xyz[1], imu->gyro_mean_xyz[2],
        imu->gyro_std_xyz[0], imu->gyro_std_xyz[1], imu->gyro_std_xyz[2],
        imu->gyro_rms_xyz[0], imu->gyro_rms_xyz[1], imu->gyro_rms_xyz[2],
        imu->gyro_mag_mean, imu->gyro_mag_std, imu->peak_count),
        TAG, "imu no cabe en buffer");

    return ESP_OK;
}

/**
 * @brief Convierte el paquete del Nodo 1 al formato interno de resumen IMU.
 */
static void n1_packet_to_imu_summary(const node1_packet_t *p, imu_feature_summary_t *imu)
{
    memset(imu, 0, sizeof(*imu));

    for (int i = 0; i < 3; ++i) {
        imu->acc_mean_xyz[i]  = p->acc_mean[i];
        imu->acc_std_xyz[i]   = p->acc_std[i];
        imu->acc_rms_xyz[i]   = p->acc_rms[i];
        imu->gyro_mean_xyz[i] = p->gyro_mean[i];
        imu->gyro_std_xyz[i]  = p->gyro_std[i];
        imu->gyro_rms_xyz[i]  = p->gyro_rms[i];
    }

    imu->acc_mag_mean  = p->acc_mag_mean;
    imu->acc_mag_std   = p->acc_mag_std;
    imu->acc_sma       = p->acc_sma;
    imu->gyro_mag_mean = p->gyro_mag_mean;
    imu->gyro_mag_std  = p->gyro_mag_std;
    imu->peak_count    = p->peak_count;
}

/**
 * @brief Construye una fila CSV fusionando la ventana local y el paquete remoto.
 */
static esp_err_t csv_build_record_line(const log_record_t *r, char *line, size_t line_size)
{
    ESP_RETURN_ON_FALSE(r != NULL, ESP_ERR_INVALID_ARG, TAG, "record nulo");
    ESP_RETURN_ON_FALSE(line != NULL, ESP_ERR_INVALID_ARG, TAG, "linea nula");

    line[0] = '\0';
    size_t used = 0;

    ESP_RETURN_ON_ERROR(buf_append(line, line_size, &used,
        "%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRId32 ","
        "%u,%u,%" PRIu32 ",%" PRIu32 ",%u,%u,%u,%.3f,",
        r->session_id,
        r->match_delta_ms,
        r->rx_local_ms,
        r->offset_est_ms,
        r->n2.boot_id,
        r->n2.window_index,
        r->n2.win_start_ms,
        r->n2.win_end_ms,
        r->n2.sample_count,
        r->n2.flags,
        r->bat.percent,
        (double)r->bat.voltage_v),
        TAG, "encabezado de record no cabe");

    ESP_RETURN_ON_ERROR(csv_append_imu(line, line_size, &used, &r->n2.imu), TAG, "imu n2 invalida");

    ESP_RETURN_ON_ERROR(buf_append(line, line_size, &used,
        ",%u,%u,%" PRIu32 ",%u,%u,%u,%u,%u,%u,",
        r->n1.packet.boot_id,
        r->n1.packet.seq,
        r->n1.packet.t_rel_ms,
        r->n1.packet.window_ms,
        r->n1.packet.sample_count,
        r->n1.packet.flags,
        r->n1.packet.bpm,
        r->n1.packet.spo2,
        r->n1.packet.battery_pct),
        TAG, "encabezado n1 no cabe");

    imu_feature_summary_t n1_imu;
    n1_packet_to_imu_summary(&r->n1.packet, &n1_imu);
    ESP_RETURN_ON_ERROR(csv_append_imu(line, line_size, &used, &n1_imu), TAG, "imu n1 invalida");

    return ESP_OK;
}

/**
 * @brief Inicializa el backend serial y publica el nombre del CSV de la sesion.
 */
esp_err_t hal_sdcard_init(uint16_t boot_id)
{
    char header[APP_LOG_LINE_MAX] = {0};

    snprintf(s_filename, sizeof(s_filename), "adl_session_%04X.csv", boot_id);
    ESP_RETURN_ON_ERROR(csv_build_header(header, sizeof(header)), TAG, "header csv demasiado grande");

    s_ready = true;

    host_emit_line(HOST_META_PREFIX, s_filename);
    host_emit_line(HOST_HEADER_PREFIX, header);
    ESP_LOGI(TAG, "Backend serial listo. Archivo destino en host: %s", s_filename);
    return ESP_OK;
}

/**
 * @brief Indica si el backend serial ya puede aceptar registros.
 */
bool hal_sdcard_is_ready(void)
{
    return s_ready;
}

/**
 * @brief Emite por serial un lote de registros CSV ya fusionados.
 */
esp_err_t hal_sdcard_append_records(const log_record_t *records, size_t count)
{
    if (!s_ready || !records || count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < count; ++i) {
        char line[APP_LOG_LINE_MAX] = {0};
        ESP_RETURN_ON_ERROR(csv_build_record_line(&records[i], line, sizeof(line)), TAG, "record %u no cabe en buffer", (unsigned)i);
        host_emit_line(HOST_ROW_PREFIX, line);
    }

    return ESP_OK;
}
