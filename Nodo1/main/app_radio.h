/**
 * @file app_radio.h
 * @brief API publica del modulo de radio del Nodo 1.
 */

#pragma once

#include "config.h"

/** @brief Inicializa Wi-Fi + ESP-NOW y registra el peer remoto. */
esp_err_t app_radio_init(void);
/**
 * @brief Envia un payload y espera la confirmacion del callback de ESP-NOW.
 * @param[in] payload Mensaje listo para transmision.
 * @param[in] critical Habilita reintentos para mensajes criticos.
 */
esp_err_t app_radio_send_blocking(const adl_payload_t *payload, bool critical);
/** @brief Consulta si el enlace esta degradado. */
bool      app_radio_link_degraded(void);
/** @brief Obtiene el contador de fallos consecutivos de radio. */
uint8_t   app_radio_consecutive_failures(void);