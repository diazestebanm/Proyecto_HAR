#ifndef HAL_RADIO_H
#define HAL_RADIO_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @file hal_radio.h
 * @brief Inicializacion de Wi-Fi + ESP-NOW en modo receptor.
 */

/** @brief Inicializa el receptor ESP-NOW y asocia la cola de eventos. */
esp_err_t hal_radio_init(QueueHandle_t rx_queue);

#endif /* HAL_RADIO_H */
