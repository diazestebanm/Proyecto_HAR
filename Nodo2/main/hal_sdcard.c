/**
 * @file hal_sdcard.c
 * @brief Manejo del almacenamiento en tarjeta SD del Nodo 2.
 *
 * Monta el sistema de archivos FAT, prepara el archivo de sesion y escribe
 * registros sincronizados de Nodo 1 y Nodo 2 en formato CSV.
 */

#include "hal_sdcard.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"

static const char *TAG = "HAL_SD";
static bool s_ready = false;
static char s_path[APP_SESSION_FILENAME_MAX] = {0};
static sdmmc_card_t *s_card = NULL;
static sdmmc_host_t s_host = {0};

/** @brief Escribe la cabecera CSV del archivo de sesion. */
static void csv_write_header(FILE *f)
{
    fprintf(f,
        "session_id,match_delta_ms,rx_local_ms,offset_est_ms," \
        "n2_boot_id,n2_win_idx,n2_win_start_ms,n2_win_end_ms,n2_sample_count,n2_flags,n2_bat_pct,n2_bat_v," \
        "n2_acc_mean_x,n2_acc_mean_y,n2_acc_mean_z,n2_acc_std_x,n2_acc_std_y,n2_acc_std_z,n2_acc_rms_x,n2_acc_rms_y,n2_acc_rms_z," \
        "n2_acc_mag_mean,n2_acc_mag_std,n2_acc_sma," \
        "n2_gyro_mean_x,n2_gyro_mean_y,n2_gyro_mean_z,n2_gyro_std_x,n2_gyro_std_y,n2_gyro_std_z,n2_gyro_rms_x,n2_gyro_rms_y,n2_gyro_rms_z," \
        "n2_gyro_mag_mean,n2_gyro_mag_std,n2_peak_count," \
        "n1_boot_id,n1_seq,n1_t_rel_ms,n1_window_ms,n1_sample_count,n1_flags,n1_bpm,n1_spo2,n1_bat_pct," \
        "n1_acc_mean_x,n1_acc_mean_y,n1_acc_mean_z,n1_acc_std_x,n1_acc_std_y,n1_acc_std_z,n1_acc_rms_x,n1_acc_rms_y,n1_acc_rms_z," \
        "n1_acc_mag_mean,n1_acc_mag_std,n1_acc_sma," \
        "n1_gyro_mean_x,n1_gyro_mean_y,n1_gyro_mean_z,n1_gyro_std_x,n1_gyro_std_y,n1_gyro_std_z,n1_gyro_rms_x,n1_gyro_rms_y,n1_gyro_rms_z," \
        "n1_gyro_mag_mean,n1_gyro_mag_std,n1_peak_count\n");
}

/** @brief Serializa un resumen IMU dentro de la fila CSV actual. */
static void csv_write_imu(FILE *f, const imu_feature_summary_t *imu)
{
    fprintf(f,
            "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u",
            imu->acc_mean_xyz[0], imu->acc_mean_xyz[1], imu->acc_mean_xyz[2],
            imu->acc_std_xyz[0], imu->acc_std_xyz[1], imu->acc_std_xyz[2],
            imu->acc_rms_xyz[0], imu->acc_rms_xyz[1], imu->acc_rms_xyz[2],
            imu->acc_mag_mean, imu->acc_mag_std, imu->acc_sma,
            imu->gyro_mean_xyz[0], imu->gyro_mean_xyz[1], imu->gyro_mean_xyz[2],
            imu->gyro_std_xyz[0], imu->gyro_std_xyz[1], imu->gyro_std_xyz[2]);
    fprintf(f,",%d,%d,%d,%d,%d,%u",
            imu->gyro_rms_xyz[0], imu->gyro_rms_xyz[1], imu->gyro_rms_xyz[2],
            imu->gyro_mag_mean, imu->gyro_mag_std, imu->peak_count);
}

/** @brief Convierte el payload del Nodo 1 a la estructura comun de resumen IMU. */
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
 * @brief Monta la tarjeta SD y crea el archivo CSV de sesion.
 * @param[in] boot_id Identificador de arranque usado en el nombre del archivo.
 */
esp_err_t hal_sdcard_init(uint16_t boot_id)
{
    esp_log_level_set("sdspi_transaction", ESP_LOG_WARN);
    vTaskDelay(pdMS_TO_TICKS(200));

    s_host = (sdmmc_host_t)SDSPI_HOST_DEFAULT();
    s_host.max_freq_khz = SDMMC_FREQ_PROBING;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = APP_SD_MOSI_GPIO,
        .miso_io_num = APP_SD_MISO_GPIO,
        .sclk_io_num = APP_SD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(s_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA), TAG, "spi bus init fallo");

    sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev_cfg.gpio_cs = APP_SD_CS_GPIO;
    dev_cfg.host_id = s_host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    esp_err_t err = esp_vfs_fat_sdspi_mount(APP_SD_MOUNT_POINT, &s_host, &dev_cfg, &mount_cfg, &s_card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo montar SD: %s", esp_err_to_name(err));
        s_ready = false;
        return err;
    }

    snprintf(s_path, sizeof(s_path), APP_SD_MOUNT_POINT "/adl_session_%04X.csv", boot_id);
    FILE *f = fopen(s_path, "r");
    if (f) {
        fclose(f);
    } else {
        f = fopen(s_path, "w");
        if (!f) {
            s_ready = false;
            return ESP_FAIL;
        }
        csv_write_header(f);
        fclose(f);
    }

    s_ready = true;
    ESP_LOGI(TAG, "SD lista. Archivo: %s", s_path);
    return ESP_OK;
}

/** @brief Indica si la SD esta lista para escritura. */
bool hal_sdcard_is_ready(void)
{
    return s_ready;
}

/**
 * @brief Escribe un lote de registros fusionados en la tarjeta SD.
 * @param[in] records Registros listos para persistencia.
 * @param[in] count Numero de elementos validos dentro del lote.
 */
esp_err_t hal_sdcard_append_records(const log_record_t *records, size_t count)
{
    if (!s_ready || !records || count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    FILE *f = fopen(s_path, "a");
    if (!f) {
        s_ready = false;
        return ESP_FAIL;
    }

    for (size_t i = 0; i < count; ++i) {
        const log_record_t *r = &records[i];

        fprintf(f,
                "%lu,%lu,%lu,%ld,%u,%u,%lu,%lu,%u,%u,%u,%.3f,",
                (unsigned long)r->session_id,
                (unsigned long)r->match_delta_ms,
                (unsigned long)r->rx_local_ms,
                (long)r->offset_est_ms,
                r->n2.boot_id,
                r->n2.window_index,
                (unsigned long)r->n2.win_start_ms,
                (unsigned long)r->n2.win_end_ms,
                r->n2.sample_count,
                r->n2.flags,
                r->bat.percent,
                r->bat.voltage_v);

        csv_write_imu(f, &r->n2.imu);

        fprintf(f,
                ",%u,%u,%lu,%u,%u,%u,%u,%u,%u,",
                r->n1.packet.boot_id,
                r->n1.packet.seq,
                (unsigned long)r->n1.packet.t_rel_ms,
                r->n1.packet.window_ms,
                r->n1.packet.sample_count,
                r->n1.packet.flags,
                r->n1.packet.bpm,
                r->n1.packet.spo2,
                r->n1.packet.battery_pct);

        /*
         * node1_packet_t esta marcado como packed para mantener compatibilidad
         * binaria con el payload recibido por ESP-NOW. Para evitar el warning
         * por direccion de miembro packed, se copia el resumen IMU a una
         * variable local alineada antes de pasarlo al escritor CSV.
         */
        imu_feature_summary_t n1_imu;
        n1_packet_to_imu_summary(&r->n1.packet, &n1_imu);
        csv_write_imu(f, &n1_imu);
        fputc('\n', f);
    }

    fclose(f);
    return ESP_OK;
}
