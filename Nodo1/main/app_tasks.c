/**
 * @file app_tasks.c
 * @brief Orquestacion FreeRTOS del Nodo 1.
 *
 * Implementa la FSM principal, la captura de ventanas IMU, el servicio del PPG
 * y la cola de transmision hacia ESP-NOW.
 */

#include <string.h>
#include "app_tasks.h"

#include "config.h"
#include "hal_sensors.h"
#include "app_signal.h"
#include "app_radio.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "app_tasks";
static volatile bool s_hal_ready = false;

/* Notifications */
#define NTF_IMU_START   (1UL << 0)
#define NTF_IMU_DONE    (1UL << 1)

/* Handles / sync */
static TaskHandle_t s_task_ctrl = NULL;
static TaskHandle_t s_task_imu = NULL;
static TaskHandle_t s_task_ppg = NULL;
static TaskHandle_t s_task_radio = NULL;

static QueueHandle_t s_radio_queue = NULL;
static SemaphoreHandle_t s_data_mutex = NULL;

/* Shared data */
static ppg_metrics_t s_last_ppg = {0};
static battery_status_t s_last_bat = {0};
static imu_features_t s_last_imu = {0};

static uint16_t s_boot_id = 0;
static uint16_t s_seq = 0;
static uint64_t s_t0_us = 0;

/**
 * @brief Obtiene el tiempo relativo del arranque en milisegundos.
 * @return Tiempo transcurrido desde que la aplicacion se inicializo.
 */
static uint32_t rel_ms(void)
{
    return (uint32_t)((esp_timer_get_time() - s_t0_us) / 1000ULL);
}

/**
 * @brief Compone las banderas del payload a partir del estado actual del nodo.
 */
static uint16_t build_flags(const imu_features_t *imu, const ppg_metrics_t *ppg, const battery_status_t *bat, bool boot_event)
{
    uint16_t f = 0;

    if (boot_event) f |= APP_FLAG_BOOT_EVENT;
    if (imu->valid) f |= APP_FLAG_IMU_VALID_WINDOW;
    if (imu->sample_count >= APP_IMU_EXPECTED_SAMPLES) f |= APP_FLAG_IMU_WINDOW_COMPLETE;
    if (imu->sample_count < APP_IMU_EXPECTED_SAMPLES) f |= APP_FLAG_IMU_SAMPLE_LOSS;
    if (hal_imu_bias_ok()) f |= APP_FLAG_GYRO_BIAS_OK;

    if (ppg->finger_present) f |= APP_FLAG_PPG_FINGER_PRESENT;
    if (ppg->bpm_valid) f |= APP_FLAG_BPM_VALID;
    if (ppg->spo2_valid) f |= APP_FLAG_SPO2_VALID;

    if (bat->valid) f |= APP_FLAG_BAT_VALID;
    if (bat->low) f |= APP_FLAG_BAT_LOW;
    if (bat->critical) f |= APP_FLAG_BAT_CRITICAL;

    if (app_radio_link_degraded()) f |= APP_FLAG_LINK_DEGRADED;
    return f;
}

/**
 * @brief Tarea dedicada al sensor PPG.
 *
 * Espera notificaciones disparadas por la interrupcion del MAX30100 y actualiza
 * la estructura compartida con las metricas mas recientes.
 */
static void task_ppg(void *arg)
{
    while (!s_hal_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(hal_ppg_start());

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        ppg_metrics_t p;
        if (hal_ppg_service(&p) == ESP_OK) {
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            s_last_ppg = p;
            xSemaphoreGive(s_data_mutex);
        }
    }
}

/**
 * @brief Tarea que captura una ventana IMU y calcula sus rasgos.
 */
static void task_imu(void *arg)
{
    app_signal_ctx_t sig;
    imu_sample_t s;

    while (1) {
        uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!n) continue;

        app_signal_reset(&sig);
        hal_imu_start();

        TickType_t last = xTaskGetTickCount();
        for (uint32_t i = 0; i < APP_IMU_EXPECTED_SAMPLES; ++i) {
            if (hal_imu_read_sample(&s) == ESP_OK) {
                s.t_ms = rel_ms();
                app_signal_push_imu(&sig, &s);
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(APP_IMU_SAMPLE_PERIOD_MS));
        }

        hal_imu_stop();

        imu_features_t feat;
        app_signal_build_features(&sig, &feat);

        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_last_imu = feat;
        xSemaphoreGive(s_data_mutex);

        xTaskNotify(s_task_ctrl, NTF_IMU_DONE, eSetBits);
    }
}

/**
 * @brief Tarea que serializa los envios hacia ESP-NOW desde una cola local.
 */
static void task_radio(void *arg)
{
    radio_tx_item_t item;
    while (1) {
        if (xQueueReceive(s_radio_queue, &item, portMAX_DELAY)) {
            esp_err_t err = app_radio_send_blocking(&item.payload, item.critical);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "TX ok type=%u seq=%u", item.payload.msg_type, item.payload.seq);
            } else {
                ESP_LOGW(TAG, "TX fail type=%u seq=%u", item.payload.msg_type, item.payload.seq);
            }
        }
    }
}

/**
 * @brief Toma una instantanea del estado del nodo y la encola para transmision.
 */
static void enqueue_payload(app_msg_type_t type, bool critical, bool boot_event)
{
    ppg_metrics_t ppg;
    battery_status_t bat;
    imu_features_t imu;

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    ppg = s_last_ppg;
    bat = s_last_bat;
    imu = s_last_imu;
    xSemaphoreGive(s_data_mutex);

    uint16_t flags = build_flags(&imu, &ppg, &bat, boot_event);

    adl_payload_t payload;
    app_signal_build_payload(&payload,
                             type,
                             s_boot_id,
                             ++s_seq,
                             rel_ms(),
                             flags,
                             &imu,
                             &ppg,
                             &bat);

    radio_tx_item_t item = {
        .payload = payload,
        .critical = critical
    };
    xQueueSend(s_radio_queue, &item, portMAX_DELAY);
}

/**
 * @brief FSM central del Nodo 1.
 *
 * Decide cuando medir bateria, capturar IMU y si el siguiente mensaje se envia
 * como telemetria normal o como alarma.
 */
static void task_control_fsm(void *arg)
{
    (void)arg;

    app_state_t state = APP_STATE_BOOT;
    TickType_t last_bat = 0;
    TickType_t cycle_ref = xTaskGetTickCount();
    bool boot_event_pending = true;

    battery_status_t bat;
    if (hal_battery_read(&bat) == ESP_OK) {
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_last_bat = bat;
        xSemaphoreGive(s_data_mutex);
    }
    last_bat = xTaskGetTickCount();

    while (1) {
        switch (state) {
        case APP_STATE_BOOT:
            enqueue_payload(APP_MSG_ALARM, true, true);
            boot_event_pending = false;
            vTaskDelayUntil(&cycle_ref, pdMS_TO_TICKS(APP_CYCLE_PERIOD_MS));
            state = APP_STATE_IDLE_LOW_POWER;
            break;

        case APP_STATE_IDLE_LOW_POWER:
            if ((xTaskGetTickCount() - last_bat) >= pdMS_TO_TICKS(APP_BAT_ACTIVE_PERIOD_MS)) {
                battery_status_t bat_now;
                if (hal_battery_read(&bat_now) == ESP_OK) {
                    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
                    s_last_bat = bat_now;
                    xSemaphoreGive(s_data_mutex);
                }
                last_bat = xTaskGetTickCount();
            }
            state = APP_STATE_ACQUIRE_IMU;
            break;

        case APP_STATE_ACQUIRE_IMU: {
            xTaskNotifyGive(s_task_imu);

            uint32_t notified = 0;
            if (xTaskNotifyWait(0, UINT32_MAX, &notified,
                                pdMS_TO_TICKS(APP_IMU_WINDOW_MS + 1500)) != pdTRUE ||
                !(notified & NTF_IMU_DONE)) {
                ESP_LOGW(TAG, "Timeout esperando ventana IMU");
            }

            state = APP_STATE_BUILD_PAYLOAD;
            break;
        }

        case APP_STATE_ACQUIRE_PPG:
            state = APP_STATE_BUILD_PAYLOAD;
            break;

        case APP_STATE_BUILD_PAYLOAD: {
            battery_status_t bat_snapshot;
            xSemaphoreTake(s_data_mutex, portMAX_DELAY);
            bat_snapshot = s_last_bat;
            xSemaphoreGive(s_data_mutex);

            if (bat_snapshot.critical || app_radio_link_degraded() || boot_event_pending) {
                state = APP_STATE_TX_ALARM;
            } else {
                state = APP_STATE_TX_NORMAL;
            }
            break;
        }

        case APP_STATE_TX_NORMAL:
            enqueue_payload(APP_MSG_NORMAL, false, false);
            boot_event_pending = false;
            vTaskDelayUntil(&cycle_ref, pdMS_TO_TICKS(APP_CYCLE_PERIOD_MS));
            state = APP_STATE_IDLE_LOW_POWER;
            break;

        case APP_STATE_TX_ALARM:
            enqueue_payload(APP_MSG_ALARM, true, boot_event_pending);
            boot_event_pending = false;
            vTaskDelayUntil(&cycle_ref, pdMS_TO_TICKS(APP_CYCLE_PERIOD_MS));
            state = APP_STATE_IDLE_LOW_POWER;
            break;

        case APP_STATE_RECOVERY:
            vTaskDelayUntil(&cycle_ref, pdMS_TO_TICKS(APP_CYCLE_PERIOD_MS));
            state = APP_STATE_IDLE_LOW_POWER;
            break;

        default:
            state = APP_STATE_IDLE_LOW_POWER;
            break;
        }
    }
}

/**
 * @brief Crea colas, semaforos y tareas principales del Nodo 1.
 * @return ESP_OK si la inicializacion del nodo finaliza correctamente.
 */
esp_err_t app_tasks_start(void)
{
    s_boot_id = (uint16_t)(esp_random() & 0xFFFF);
    s_t0_us = esp_timer_get_time();

    s_radio_queue = xQueueCreate(8, sizeof(radio_tx_item_t));
    s_data_mutex = xSemaphoreCreateMutex();

    xTaskCreate(task_ppg, "TaskPPG", 4096, NULL, 6, &s_task_ppg);

    ESP_ERROR_CHECK(hal_sensors_init(s_task_ppg));
    ESP_ERROR_CHECK(app_radio_init());
    s_hal_ready = true;

    xTaskCreate(task_imu, "TaskIMU", 4096, NULL, 5, &s_task_imu);
    xTaskCreate(task_radio, "TaskRadio", 4096, NULL, 4, &s_task_radio);
    xTaskCreate(task_control_fsm, "TaskControlFSM", 4096, NULL, 7, &s_task_ctrl);

    return ESP_OK;
}