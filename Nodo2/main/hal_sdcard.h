#ifndef HAL_SDCARD_H
#define HAL_SDCARD_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "app_types.h"

/**
 * @file hal_sdcard.h
 * @brief HAL de almacenamiento en SD para el Nodo 2.
 */

/** @brief Monta la tarjeta SD y crea el archivo de sesion. */
esp_err_t hal_sdcard_init(uint16_t boot_id);
/** @brief Consulta si la SD esta lista para recibir escrituras. */
bool hal_sdcard_is_ready(void);
/** @brief Escribe un lote de registros fusionados en la tarjeta SD. */
esp_err_t hal_sdcard_append_records(const log_record_t *records, size_t count);

#endif /* HAL_SDCARD_H */
