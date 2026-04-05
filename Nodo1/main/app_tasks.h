/**
 * @file app_tasks.h
 * @brief API publica de arranque de tareas del Nodo 1.
 */

#pragma once

#include "esp_err.h"

/** @brief Inicializa el hardware y lanza las tareas FreeRTOS del Nodo 1. */
esp_err_t app_tasks_start(void);