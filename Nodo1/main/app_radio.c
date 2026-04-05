/**
 * @file app_radio.c
 * @brief Implementa la capa de radio del Nodo 1 basada en Wi-Fi STA y ESP-NOW.
 *
 * Este modulo prepara la red, registra el callback de confirmacion de envio y
 * ofrece una API bloqueante para despachar el payload de actividad construido
 * por la FSM principal.
 */

#include <string.h>
#include "app_radio.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_radio";

static TaskHandle_t s_sender_task = NULL;
static volatile esp_now_send_status_t s_last_status = ESP_NOW_SEND_FAIL;
static volatile uint8_t s_consec_fail = 0;

/**
 * @brief Callback de confirmacion de envio de ESP-NOW.
 * @param[in] tx_info Informacion del frame transmitido entregada por la pila Wi-Fi.
 * @param[in] status Resultado final del intento de transmision.
 *
 * El callback actualiza el estado del enlace y despierta a la tarea que
 * espera la confirmacion del envio actual.
 */
static void radio_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    s_last_status = status;

    if (status == ESP_NOW_SEND_SUCCESS) {
        s_consec_fail = 0;
    } else if (s_consec_fail < 255) {
        s_consec_fail++;
    }

    if (s_sender_task) {
        xTaskNotifyGive(s_sender_task);
    }
}

/**
 * @brief Inicializa NVS, la interfaz Wi-Fi en modo estacion y ESP-NOW.
 * @return ESP_OK si la radio queda lista para transmitir al peer configurado.
 */
esp_err_t app_radio_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(APP_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(radio_send_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, APP_PEER_MAC, 6);
    peer.channel = APP_ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    return ESP_OK;
}

/**
 * @brief Intenta enviar un payload por ESP-NOW y espera confirmacion.
 * @param[in] payload Mensaje ya serializado por la capa de procesamiento.
 * @param[in] critical Indica si el mensaje puede reintentarse mas de una vez.
 * @return ESP_OK si se obtuvo confirmacion de envio; ESP_FAIL en caso contrario.
 */
esp_err_t app_radio_send_blocking(const adl_payload_t *payload, bool critical)
{
    const int tries = critical ? 2 : 1;
    s_sender_task = xTaskGetCurrentTaskHandle();

    for (int i = 0; i < tries; ++i) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send(APP_PEER_MAC, (const uint8_t *)payload, sizeof(*payload)));
        uint32_t ok = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_RADIO_SEND_TIMEOUT_MS));
        if (ok && s_last_status == ESP_NOW_SEND_SUCCESS) {
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

/**
 * @brief Indica si el enlace se considera degradado segun fallos consecutivos.
 * @return true si el numero de fallos supera el umbral definido en configuracion.
 */
bool app_radio_link_degraded(void)
{
    return s_consec_fail >= APP_LINK_FAIL_THRESHOLD;
}

/**
 * @brief Devuelve el contador actual de fallos consecutivos de transmision.
 * @return Numero de envios fallidos contabilizados desde el ultimo exito.
 */
uint8_t app_radio_consecutive_failures(void)
{
    return s_consec_fail;
}