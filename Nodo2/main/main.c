#include "app_tasks.h"
#include "esp_log.h"

/**
 * @file main.c
 * @brief Punto de entrada del Nodo 2.
 *
 * Este nodo actua como:
 *  - capturador local de IMU en pierna/tobillo,
 *  - receptor ESP-NOW de los resumentes del Nodo 1,
 *  - logger sincronizado de ambos nodos hacia un archivo CSV en el host.
 */

static const char *TAG = "MAIN";

/**
 * @brief Punto de entrada del firmware del Nodo 2.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Arrancando Nodo 2...");
    ESP_ERROR_CHECK(app_tasks_start());
    ESP_LOGI(TAG, "app_main finalizo; tareas activas en ejecucion.");
}
