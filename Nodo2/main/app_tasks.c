/**
 * @file app_tasks.c
 * @brief Orquestacion principal del Nodo 2.
 *
 * Este modulo crea las tareas FreeRTOS, coordina la calibracion del MPU6050,
 * sincroniza la recepcion del Nodo 1 y genera los registros fusionados para el host.
 */

#include "app_tasks.h"

#include <string.h>
#include "app_config.h"
#include "app_types.h"
#include "hal_battery.h"
#include "hal_mpu6050.h"
#include "hal_radio.h"
#include "hal_sdcard.h"
#include "imu_features.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "APP_TASKS";

static TaskHandle_t s_task_ctrl = NULL;
static TaskHandle_t s_task_imu = NULL;
static TaskHandle_t s_task_radio = NULL;
static TaskHandle_t s_task_sd = NULL;

static QueueHandle_t s_radio_rx_queue = NULL;
static QueueHandle_t s_imu_result_queue = NULL;
static QueueHandle_t s_sd_queue = NULL;
static SemaphoreHandle_t s_n1_mutex = NULL;

static uint16_t s_boot_id = 0;
static uint32_t s_session_id = 0;
static uint32_t s_t0_ms = 0;
static mpu6050_calibration_t s_cal = {0};

static radio_rx_event_t s_last_n1 = {0};
static bool s_last_n1_valid = false;
static uint16_t s_last_consumed_seq = 0;
static uint16_t s_last_seen_boot = 0;
static uint16_t s_last_seen_seq = 0;

static battery_status_t s_bat = {0};
static uint32_t s_last_bat_ms = 0;

/**
 * @brief Obtiene el tiempo absoluto del sistema en milisegundos.
 * @return Tiempo actual derivado del temporizador de alta resolucion.
 */
static uint32_t app_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief Aplica la politica de gestion de potencia del Nodo 2.
 *
 * Si el proyecto fue compilado con soporte PM, habilita DFS y light-sleep
 * automatico segun las opciones configuradas en menuconfig.
 */


/**
 * @brief Verifica si el ultimo paquete recibido de Nodo 1 puede emparejarse.
 * @param[in] expected_rx_ms Instante local esperado para el siguiente paquete.
 * @param[out] out Copia del evento coincidente, si existe.
 * @param[out] delta_ms Diferencia absoluta entre tiempo esperado y tiempo real.
 * @return true si el paquete cae dentro de la ventana de emparejamiento.
 */
static bool is_n1_matchable(uint32_t expected_rx_ms, radio_rx_event_t *out, uint32_t *delta_ms)
{
    bool ok = false;
    xSemaphoreTake(s_n1_mutex, portMAX_DELAY);
    if (s_last_n1_valid && s_last_n1.packet.seq != s_last_consumed_seq) {
        const uint32_t rx_ms = s_last_n1.rx_local_ms;
        const uint32_t diff = (rx_ms > expected_rx_ms) ? (rx_ms - expected_rx_ms) : (expected_rx_ms - rx_ms);
        if (diff <= APP_MATCH_WINDOW_MS) {
            if (out) *out = s_last_n1;
            if (delta_ms) *delta_ms = diff;
            s_last_consumed_seq = s_last_n1.packet.seq;
            ok = true;
        }
    }
    xSemaphoreGive(s_n1_mutex);
    return ok;
}

/**
 * @brief Tarea receptora de eventos de radio ya desacoplados del callback Wi-Fi.
 */
static void task_radio(void *arg)
{
    (void)arg;
    radio_rx_event_t ev;

    while (1) {
        if (xQueueReceive(s_radio_rx_queue, &ev, portMAX_DELAY) == pdTRUE) {
            if (ev.packet.boot_id == s_last_seen_boot &&
                ev.packet.seq == s_last_seen_seq) {
                continue;
            }

            s_last_seen_boot = ev.packet.boot_id;
            s_last_seen_seq = ev.packet.seq;

            xSemaphoreTake(s_n1_mutex, portMAX_DELAY);
            s_last_n1 = ev;
            s_last_n1_valid = true;
            xSemaphoreGive(s_n1_mutex);

            ESP_LOGI("RADIO", "N1 seq=%u type=%u rx_local_ms=%lu",
                     ev.packet.seq, ev.packet.msg_type, (unsigned long)ev.rx_local_ms);
        }
    }
}

/**
 * @brief Tarea del MPU6050 encargada de calibrar y capturar ventanas locales.
 */
static void task_imu(void *arg)
{
    (void)arg;
    imu_sample_t samples[APP_IMU_WINDOW_SAMPLES];

    while (1) {
        uint32_t cmd = 0;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &cmd, portMAX_DELAY);

        if (cmd == APP_IMU_CMD_CALIBRATE) {
            esp_err_t err = hal_mpu6050_calibrate_gyro(&s_cal);
            if (err != ESP_OK) {
                memset(&s_cal, 0, sizeof(s_cal));
            }
            ESP_LOGI("IMU", "Calibracion gyro: %s", esp_err_to_name(err));
            continue;
        }

        if (cmd == APP_IMU_CMD_CAPTURE) {
            node2_window_result_t result = {
                .boot_id = s_boot_id,
            };
            result.win_start_ms = app_now_ms() - s_t0_ms;
            result.window_index = 0;

            uint16_t captured = 0;
            uint16_t flags = 0;
            esp_err_t err = hal_mpu6050_capture_window(&s_cal, samples, APP_IMU_WINDOW_SAMPLES, &captured, &flags, s_t0_ms);
            if (err != ESP_OK) {
                ESP_LOGE("IMU", "Captura IMU fallo: %s", esp_err_to_name(err));

                if (hal_mpu6050_recover() == ESP_OK) {
                    memset(&s_cal, 0, sizeof(s_cal));
                    if (hal_mpu6050_calibrate_gyro(&s_cal) == ESP_OK) {
                        ESP_LOGW("IMU", "MPU recuperado y recalibrado");
                    }
                }

                continue;
            }

            result.win_end_ms = app_now_ms() - s_t0_ms;
            result.sample_count = captured;
            result.flags = flags;
            imu_features_compute(samples, captured, &result.imu);

            xQueueOverwrite(s_imu_result_queue, &result);
        }
    }
}

/**
 * @brief Tarea de persistencia hacia el host con emision por lotes.
 */
static void task_sd(void *arg)
{
    (void)arg;
    log_record_t batch[APP_SD_BATCH_LINES];
    size_t count = 0;
    uint32_t last_flush_ms = app_now_ms();
    log_record_t item;

    while (1) {
        if (xQueueReceive(s_sd_queue, &item, pdMS_TO_TICKS(APP_SD_FLUSH_PERIOD_MS)) == pdTRUE) {
            if (count < APP_SD_BATCH_LINES) {
                batch[count++] = item;
            }
        }

        if (count == 0) {
            continue;
        }

        if (count >= APP_SD_BATCH_LINES ||
            (app_now_ms() - last_flush_ms) >= APP_SD_FLUSH_PERIOD_MS) {
            if (hal_sdcard_is_ready()) {
                esp_err_t err = hal_sdcard_append_records(batch, count);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Persistencia host fallo: %s", esp_err_to_name(err));
                }
            }
            count = 0;
            last_flush_ms = app_now_ms();
        }
    }
}

/**
 * @brief Espera el primer paquete util del Nodo 1 para alinear el reloj local.
 * @param[out] first Primer evento valido recibido desde radio.
 */
static esp_err_t initial_sync_wait(radio_rx_event_t *first)
{
    const uint32_t start_ms = app_now_ms();
    while ((app_now_ms() - start_ms) < APP_NODE1_WAIT_TIMEOUT_MS) {
        radio_rx_event_t ev;
        uint32_t delta = 0;
        if (is_n1_matchable(app_now_ms(), &ev, &delta)) {
            (void)delta;
            *first = ev;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_ERR_TIMEOUT;
}

static void log_imu_summary(const char *tag_prefix, const imu_feature_summary_t *imu)
{
    if (!tag_prefix || !imu) {
        return;
    }

    ESP_LOGI("SUMMARY",
             "%s acc_mean=[%d,%d,%d] acc_std=[%d,%d,%d] acc_rms=[%d,%d,%d] acc_mag_mean=%d acc_mag_std=%d acc_sma=%d",
             tag_prefix,
             imu->acc_mean_xyz[0], imu->acc_mean_xyz[1], imu->acc_mean_xyz[2],
             imu->acc_std_xyz[0],  imu->acc_std_xyz[1],  imu->acc_std_xyz[2],
             imu->acc_rms_xyz[0],  imu->acc_rms_xyz[1],  imu->acc_rms_xyz[2],
             imu->acc_mag_mean, imu->acc_mag_std, imu->acc_sma);

    ESP_LOGI("SUMMARY",
             "%s gyro_mean=[%d,%d,%d] gyro_std=[%d,%d,%d] gyro_rms=[%d,%d,%d] gyro_mag_mean=%d gyro_mag_std=%d peak_count=%u",
             tag_prefix,
             imu->gyro_mean_xyz[0], imu->gyro_mean_xyz[1], imu->gyro_mean_xyz[2],
             imu->gyro_std_xyz[0],  imu->gyro_std_xyz[1],  imu->gyro_std_xyz[2],
             imu->gyro_rms_xyz[0],  imu->gyro_rms_xyz[1],  imu->gyro_rms_xyz[2],
             imu->gyro_mag_mean, imu->gyro_mag_std, imu->peak_count);
}

static void log_fused_window_summary(const log_record_t *r)
{
    if (!r) {
        return;
    }

    imu_feature_summary_t n1_imu = {0};

    for (int i = 0; i < 3; ++i) {
        n1_imu.acc_mean_xyz[i]  = r->n1.packet.acc_mean[i];
        n1_imu.acc_std_xyz[i]   = r->n1.packet.acc_std[i];
        n1_imu.acc_rms_xyz[i]   = r->n1.packet.acc_rms[i];
        n1_imu.gyro_mean_xyz[i] = r->n1.packet.gyro_mean[i];
        n1_imu.gyro_std_xyz[i]  = r->n1.packet.gyro_std[i];
        n1_imu.gyro_rms_xyz[i]  = r->n1.packet.gyro_rms[i];
    }

    n1_imu.acc_mag_mean  = r->n1.packet.acc_mag_mean;
    n1_imu.acc_mag_std   = r->n1.packet.acc_mag_std;
    n1_imu.acc_sma       = r->n1.packet.acc_sma;
    n1_imu.gyro_mag_mean = r->n1.packet.gyro_mag_mean;
    n1_imu.gyro_mag_std  = r->n1.packet.gyro_mag_std;
    n1_imu.peak_count    = r->n1.packet.peak_count;

    ESP_LOGI("SUMMARY",
             "win=%u seq_n1=%u delta_ms=%" PRIu32 " rx_local_ms=%" PRIu32 " offset_est_ms=%" PRId32 " bat=%u%% v=%.3f",
             r->n2.window_index,
             r->n1.packet.seq,
             r->match_delta_ms,
             r->rx_local_ms,
             r->offset_est_ms,
             r->bat.percent,
             (double)r->bat.voltage_v);

    ESP_LOGI("SUMMARY",
             "N1 boot=%u t_rel_ms=%" PRIu32 " window_ms=%u samples=%u flags=%u bpm=%u spo2=%u bat_pct=%u",
             r->n1.packet.boot_id,
             r->n1.packet.t_rel_ms,
             r->n1.packet.window_ms,
             r->n1.packet.sample_count,
             r->n1.packet.flags,
             r->n1.packet.bpm,
             r->n1.packet.spo2,
             r->n1.packet.battery_pct);

    ESP_LOGI("SUMMARY",
             "N2 boot=%u win_start=%" PRIu32 " win_end=%" PRIu32 " samples=%u flags=%u",
             r->n2.boot_id,
             r->n2.win_start_ms,
             r->n2.win_end_ms,
             r->n2.sample_count,
             r->n2.flags);

    log_imu_summary("N1", &n1_imu);
    log_imu_summary("N2", &r->n2.imu);
}

/**
 * @brief FSM principal del Nodo 2.
 *
 * Controla el arranque del sistema, la calibracion inicial, la sincronizacion
 * con Nodo 1, la captura local y el envio de registros a la tarea SD.
 */
static void task_control_fsm(void *arg)
{
    (void)arg;
    

    battery_status_t bat;
    if (hal_battery_sample(&bat) == ESP_OK && bat.valid) {
        s_bat = bat;
    }
    s_last_bat_ms = app_now_ms();

    xTaskNotify(s_task_imu, APP_IMU_CMD_CALIBRATE, eSetValueWithOverwrite);
    vTaskDelay(pdMS_TO_TICKS(3500));

    radio_rx_event_t first = {0};
    if (initial_sync_wait(&first) != ESP_OK) {
        ESP_LOGW(TAG, "Nodo 1 aun no sincroniza. Esperando paquetes...");
        while (initial_sync_wait(&first) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    uint32_t expected_next_rx_ms = first.rx_local_ms + APP_CYCLE_PERIOD_MS;
    uint16_t window_index = 0;

    while (1) {
        const uint32_t now_ms = app_now_ms();
        const uint32_t capture_start_ms = (expected_next_rx_ms > APP_CAPTURE_WINDOW_MS)
                                            ? (expected_next_rx_ms - APP_CAPTURE_WINDOW_MS)
                                            : now_ms;

        if (capture_start_ms > now_ms) {
            vTaskDelay(pdMS_TO_TICKS(capture_start_ms - now_ms));
        }

        if ((app_now_ms() - s_last_bat_ms) >= APP_BATTERY_SAMPLE_PERIOD_MS) {
            battery_status_t bat_now;
            if (hal_battery_sample(&bat_now) == ESP_OK && bat_now.valid) {
                s_bat = bat_now;
            }
            s_last_bat_ms = app_now_ms();
        }

        xTaskNotify(s_task_imu, APP_IMU_CMD_CAPTURE, eSetValueWithOverwrite);

        node2_window_result_t n2 = {0};
        if (xQueueReceive(s_imu_result_queue, &n2, pdMS_TO_TICKS(APP_CAPTURE_WINDOW_MS + 2000)) != pdTRUE) {
            ESP_LOGW(TAG, "No se recibio resultado local de IMU");
            expected_next_rx_ms += APP_CYCLE_PERIOD_MS;
            continue;
        }
        n2.window_index = ++window_index;
        n2.flags |= s_bat.flags;

        radio_rx_event_t n1 = {0};
        uint32_t delta_ms = 0;
        const uint32_t wait_deadline = expected_next_rx_ms + APP_MATCH_WINDOW_MS;
        bool matched = false;
        while (app_now_ms() <= wait_deadline) {
            if (is_n1_matchable(expected_next_rx_ms, &n1, &delta_ms)) {
                matched = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (!matched) {
            ESP_LOGW(TAG, "Sin match N1 para win=%u. Se omite escritura.", n2.window_index);
            expected_next_rx_ms += APP_CYCLE_PERIOD_MS;
            continue;
        }

        n2.flags |= APP_FLAG_MATCH_OK;

        log_record_t record = {
            .session_id = s_session_id,
            .match_delta_ms = delta_ms,
            .rx_local_ms = n1.rx_local_ms,
            .offset_est_ms = (int32_t)n1.rx_local_ms - (int32_t)n1.packet.t_rel_ms,
            .bat = s_bat,
            .n2 = n2,
            .n1 = n1,
        };

        #if APP_SERIAL_SUMMARY_ENABLE
                if ((record.n2.window_index % APP_SERIAL_SUMMARY_EVERY_N_WINDOWS) == 0U) {
                    log_fused_window_summary(&record);
                }
        #endif

        xQueueSend(s_sd_queue, &record, portMAX_DELAY);
        
        /*
         * Se reajusta el siguiente instante esperado usando el tiempo real del
         * paquete recibido. Esto corrige deriva entre nodos sin requerir ACK.
         */
        expected_next_rx_ms = n1.rx_local_ms + APP_CYCLE_PERIOD_MS;
    }
}

/**
 * @brief Inicializa colas, tareas y hardware de Nodo 2.
 * @return ESP_OK si todo el subsistema queda operativo.
 */
esp_err_t app_tasks_start(void)
{
    s_boot_id = (uint16_t)(esp_random() & 0xFFFF);
    s_session_id = (uint32_t)esp_random();
    s_t0_ms = app_now_ms();

    s_radio_rx_queue = xQueueCreate(8, sizeof(radio_rx_event_t));
    s_imu_result_queue = xQueueCreate(1, sizeof(node2_window_result_t));
    s_sd_queue = xQueueCreate(8, sizeof(log_record_t));
    s_n1_mutex = xSemaphoreCreateMutex();

    if (!s_radio_rx_queue || !s_imu_result_queue || !s_sd_queue || !s_n1_mutex) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreatePinnedToCore(task_imu, "TaskIMU", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_IMU, &s_task_imu, 1);

    esp_err_t mpu_err = ESP_FAIL;
    for (int i = 0; i < 5; ++i) {
        mpu_err = hal_mpu6050_init(s_task_imu);
        if (mpu_err == ESP_OK) {
            mpu_err = hal_mpu6050_verify();
            if (mpu_err == ESP_OK) {
                break;
            }
        }

        ESP_LOGW(TAG, "Reintentando init MPU (%d/5): %s", i + 1, esp_err_to_name(mpu_err));
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_RETURN_ON_ERROR(mpu_err, TAG, "mpu init fallo");
    ESP_RETURN_ON_ERROR(hal_battery_init(), TAG, "battery init fallo");
    ESP_RETURN_ON_ERROR(hal_sdcard_init(s_boot_id), TAG, "host logger init fallo");
    ESP_RETURN_ON_ERROR(hal_radio_init(s_radio_rx_queue), TAG, "radio init fallo");

    xTaskCreatePinnedToCore(task_radio, "TaskRadioRx", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_RADIO, &s_task_radio, 0);
    xTaskCreatePinnedToCore(task_sd, "TaskSD", APP_TASK_STACK_SD, NULL, APP_TASK_PRIO_SD, &s_task_sd, 1);
    xTaskCreatePinnedToCore(task_control_fsm, "TaskControlFSM", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_CONTROL, &s_task_ctrl, 1);

    ESP_LOGI(TAG, "Nodo 2 listo. boot_id=%u session_id=%lu", s_boot_id, (unsigned long)s_session_id);
    return ESP_OK;
}
