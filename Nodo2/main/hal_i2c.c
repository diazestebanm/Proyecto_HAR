/**
 * @file hal_i2c.c
 * @brief Abstraccion minima del bus I2C del Nodo 2.
 *
 * Centraliza la inicializacion del bus y las operaciones basicas empleadas por
 * el driver del MPU6050.
 */

#include "hal_i2c.h"

#include "app_config.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "HAL_I2C";
static bool s_initialized = false;

/**
 * @brief Inicializa el bus I2C maestro segun la configuracion del proyecto.
 */
esp_err_t hal_i2c_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = APP_I2C_SDA_GPIO,
        .scl_io_num = APP_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = APP_I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config((i2c_port_t)APP_I2C_PORT, &cfg), TAG, "i2c_param_config fallo");
    ESP_RETURN_ON_ERROR(i2c_driver_install((i2c_port_t)APP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0), TAG, "i2c_driver_install fallo");

    s_initialized = true;
    ESP_LOGI(TAG, "I2C listo en SDA=%d SCL=%d @ %u Hz",
             APP_I2C_SDA_GPIO, APP_I2C_SCL_GPIO, APP_I2C_FREQ_HZ);
    return ESP_OK;
}

/** @brief Escribe un byte en un registro de un esclavo I2C. */
esp_err_t hal_i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin((i2c_port_t)APP_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

/** @brief Lee uno o varios bytes consecutivos desde un esclavo I2C. */
esp_err_t hal_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (!data || !len) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin((i2c_port_t)APP_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}
