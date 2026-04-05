#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "esp_err.h"

/**
 * @file app_tasks.h
 * @brief Creacion y arranque de tareas principales del Nodo 2.
 */

/** @brief Inicializa el hardware y lanza las tareas del Nodo 2. */
esp_err_t app_tasks_start(void);

#endif /* APP_TASKS_H */
