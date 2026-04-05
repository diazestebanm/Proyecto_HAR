#ifndef HAL_SDCARD_H
#define HAL_SDCARD_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "app_types.h"

/**
 * @file hal_sdcard.h
 * @brief HAL de persistencia del Nodo 2 con backend serial hacia el host.
 *
 * @note El nombre del modulo se conserva para no romper la arquitectura ya
 * existente, pero actualmente no monta una microSD. En esta version la
 * persistencia real se hace por puerto serie y un script Python arma el CSV
 * definitivo en el computador.
 */

/** @brief Inicializa el backend de salida y anuncia el archivo destino. */
esp_err_t hal_sdcard_init(uint16_t boot_id);
/** @brief Consulta si el backend de persistencia esta listo. */
bool hal_sdcard_is_ready(void);
/** @brief Emite un lote de registros fusionados en formato CSV. */
esp_err_t hal_sdcard_append_records(const log_record_t *records, size_t count);

#endif /* HAL_SDCARD_H */
