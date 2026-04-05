/**
 * @file hal_i2c.c
 * @brief Implementacion de la HAL I2C del Nodo 2.
 *
 * Centraliza la inicializacion, cierre y transferencias basicas del bus I2C
 * usado por el MPU6050. La meta es ofrecer una capa pequena y estable para
 * que la parte de sensores no repita configuraciones del driver.
 */

#include "hal_i2c.h"

#include "app_config.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HAL_I2C";
static bool s_initialized = false;

/**
 * @brief Desinstala el driver I2C si estaba activo.
 *
 * Se usa como parte del flujo de recuperacion cuando el MPU deja el bus en un
 * estado inconsistente o cuando se requiere reconfigurar el periférico.
 */
esp_err_t hal_i2c_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    i2c_driver_delete((i2c_port_t)APP_I2C_PORT);
    s_initialized = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

/**
 * @brief Configura el bus I2C maestro con la topologia del proyecto.
 *
 * Si el driver ya habia quedado instalado en un estado invalido, el modulo
 * intenta desmontarlo y volverlo a crear antes de propagar el error.
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

    ESP_RETURN_ON_ERROR(
        i2c_param_config((i2c_port_t)APP_I2C_PORT, &cfg),
        TAG, "i2c_param_config fallo"
    );

    esp_err_t err = i2c_driver_install((i2c_port_t)APP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(hal_i2c_deinit(), TAG, "i2c deinit previo fallo");
        ESP_RETURN_ON_ERROR(
            i2c_driver_install((i2c_port_t)APP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0),
            TAG, "i2c_driver_install fallo"
        );
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "i2c_driver_install fallo");
    }

    s_initialized = true;
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "I2C listo en SDA=%d SCL=%d @ %u Hz",
             APP_I2C_SDA_GPIO, APP_I2C_SCL_GPIO, APP_I2C_FREQ_HZ);
    return ESP_OK;
}

/**
 * @brief Ejecuta una transaccion minima para comprobar presencia del esclavo.
 *
 * @param[in] dev_addr Direccion de 7 bits del dispositivo que se desea sondear.
 * @return ESP_OK si el esclavo responde con ACK.
 */
esp_err_t hal_i2c_ping(uint8_t dev_addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin((i2c_port_t)APP_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

/**
 * @brief Escribe un byte en un registro del dispositivo seleccionado.
 *
 * @param[in] dev_addr Direccion del esclavo.
 * @param[in] reg Registro a modificar.
 * @param[in] value Dato de 8 bits a escribir.
 */
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

/**
 * @brief Lee uno o mas bytes consecutivos desde un registro del esclavo.
 *
 * La transaccion se hace con una fase de escritura del registro seguida de un
 * repeated-start para la lectura, que es el patron esperado por el MPU6050.
 *
 * @param[in] dev_addr Direccion del esclavo.
 * @param[in] reg Registro inicial.
 * @param[out] data Buffer destino.
 * @param[in] len Cantidad de bytes a leer.
 */
esp_err_t hal_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (!data || len == 0) {
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

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin((i2c_port_t)APP_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}
