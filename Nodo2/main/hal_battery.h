#ifndef HAL_BATTERY_H
#define HAL_BATTERY_H

#include "esp_err.h"
#include "app_types.h"

/**
 * @file hal_battery.h
 * @brief Medicion robusta de bateria para Nodo 2.
 */

/** @brief Inicializa el subsistema ADC usado para medir bateria. */
esp_err_t hal_battery_init(void);
/** @brief Toma y resume una muestra del estado de bateria. */
esp_err_t hal_battery_sample(battery_status_t *out);

#endif /* HAL_BATTERY_H */
