/**
 * @file app_tasks.c
 * @brief Orquestacion principal del Nodo 2.
 *
 * Este modulo crea las tareas FreeRTOS, coordina la calibracion del MPU6050,
 * sincroniza la recepcion del Nodo 1 y genera los registros fusionados para SD.
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
#include "esp_pm.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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
static void power_management_init(void)
{
#ifdef CONFIG_PM_ENABLE
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true,
#else
        .light_sleep_enable = false,
#endif
    };
    esp_pm_configure(&pm_cfg);
#endif
}

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
            hal_mpu6050_capture_window(&s_cal, samples, APP_IMU_WINDOW_SAMPLES, &captured, &flags, s_t0_ms);

            result.win_end_ms = app_now_ms() - s_t0_ms;
            result.sample_count = captured;
            result.flags = flags;
            imu_features_compute(samples, captured, &result.imu);

            xQueueOverwrite(s_imu_result_queue, &result);
        }
    }
}

/**
 * @brief Tarea de almacenamiento en SD con escritura por lotes.
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
                hal_sdcard_append_records(batch, count);
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

/**
 * @brief FSM principal del Nodo 2.
 *
 * Controla el arranque del sistema, la calibracion inicial, la sincronizacion
 * con Nodo 1, la captura local y el envio de registros a la tarea SD.
 */
static void task_control_fsm(void *arg)
{
    (void)arg;
    power_management_init();

    battery_status_t bat;
    hal_battery_sample(&bat);
    s_bat = bat;
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
            hal_battery_sample(&s_bat);
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

        xQueueSend(s_sd_queue, &record, portMAX_DELAY);
        ESP_LOGI(TAG, "MATCH OK win=%u seq_n1=%u delta=%lu ms bat=%u%%",
                 n2.window_index, n1.packet.seq, (unsigned long)delta_ms, s_bat.percent);

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

    ESP_RETURN_ON_ERROR(hal_mpu6050_init(s_task_imu), TAG, "mpu init fallo");
    ESP_RETURN_ON_ERROR(hal_mpu6050_verify(), TAG, "mpu verify fallo");
    ESP_RETURN_ON_ERROR(hal_battery_init(), TAG, "battery init fallo");
    (void)hal_sdcard_init(s_boot_id);
    ESP_RETURN_ON_ERROR(hal_radio_init(s_radio_rx_queue), TAG, "radio init fallo");

    xTaskCreatePinnedToCore(task_radio, "TaskRadioRx", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_RADIO, &s_task_radio, 0);
    xTaskCreatePinnedToCore(task_sd, "TaskSD", APP_TASK_STACK_SD, NULL, APP_TASK_PRIO_SD, &s_task_sd, 1);
    xTaskCreatePinnedToCore(task_control_fsm, "TaskControlFSM", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_CONTROL, &s_task_ctrl, 1);

    ESP_LOGI(TAG, "Nodo 2 listo. boot_id=%u session_id=%lu", s_boot_id, (unsigned long)s_session_id);
    return ESP_OK;
}
