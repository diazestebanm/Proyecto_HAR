/**
 * @file hal_radio.c
 * @brief Inicializacion del receptor ESP-NOW del Nodo 2.
 *
 * El callback de recepcion copia los paquetes a una cola de FreeRTOS para que
 * el procesamiento se realice fuera del contexto de la tarea Wi-Fi.
 */

#include "hal_radio.h"

#include <string.h>
#include "app_config.h"
#include "app_types.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "HAL_RADIO";
static QueueHandle_t s_rx_queue = NULL;

/** @brief Aplica el filtro opcional de MAC del Nodo 1. */
static bool node1_mac_allowed(const uint8_t *mac)
{
#if APP_FILTER_NODE1_MAC
    static const uint8_t allowed[6] = {
        APP_NODE1_MAC0, APP_NODE1_MAC1, APP_NODE1_MAC2,
        APP_NODE1_MAC3, APP_NODE1_MAC4, APP_NODE1_MAC5,
    };
    return (memcmp(mac, allowed, 6) == 0);
#else
    (void)mac;
    return true;
#endif
}

/**
 * @brief Callback de recepcion de ESP-NOW.
 *
 * Valida longitud y origen antes de encolar el evento para la tarea de radio.
 */
static void radio_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len != (int)sizeof(node1_packet_t)) {
        return;
    }
    if (!node1_mac_allowed(info->src_addr)) {
        return;
    }

    radio_rx_event_t ev = {0};
    memcpy(ev.src_mac, info->src_addr, 6);
    memcpy(&ev.packet, data, sizeof(node1_packet_t));
    ev.rx_local_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (s_rx_queue) {
        xQueueSend(s_rx_queue, &ev, 0);
    }
}

/**
 * @brief Inicializa Wi-Fi en modo estacion y prepara ESP-NOW en recepcion.
 * @param[in] rx_queue Cola donde se depositan los paquetes recibidos.
 */
esp_err_t hal_radio_init(QueueHandle_t rx_queue)
{
    s_rx_queue = rx_queue;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs init fallo");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init fallo");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop fallo");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_cfg), TAG, "wifi init fallo");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode fallo");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage fallo");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start fallo");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(APP_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "wifi channel fallo");

    /*
     * Nodo 2 actua como receptor, asi que no registra peers de salida.
     * Solo necesita que el stack Wi-Fi este levantado y ESP-NOW inicializado.
     */
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp_now_init fallo");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(radio_rx_cb), TAG, "register recv cb fallo");

#if APP_USE_ESPNOW_RX_PS
    esp_now_set_wake_window(APP_ESPNOW_WAKE_WINDOW_MS);
    esp_wifi_connectionless_module_set_wake_interval(APP_ESPNOW_WAKE_INTERVAL_MS);
#endif

    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "ESP-NOW RX listo. CH=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X",
             APP_WIFI_CHANNEL,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return ESP_OK;
}
