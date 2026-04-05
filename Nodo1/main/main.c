/**
 * @file main.c
 * @brief Punto de entrada del firmware del Nodo 1.
 */

#include "esp_log.h"
#include "app_tasks.h"

/**
 * @brief Punto de arranque de ESP-IDF.
 *
 * La aplicacion delega toda la logica de ejecucion en el subsistema de tareas.
 */
void app_main(void)
{
    ESP_ERROR_CHECK(app_tasks_start());
}