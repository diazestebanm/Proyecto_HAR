/**
 * @file hal_mpu6050.c
 * @brief Driver de alto nivel del MPU6050 para el Nodo 2.
 *
 * Implementa la configuracion del sensor, la calibracion del sesgo del
 * giroscopio y la captura de ventanas mediante interrupciones.
 */

#include "hal_mpu6050.h"

#include <math.h>
#include <string.h>
#include "app_config.h"
#include "hal_i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HAL_MPU";

/* Registros clave del MPU6050 */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_FIFO_EN        0x23
#define REG_INT_PIN_CFG    0x37
#define REG_INT_ENABLE     0x38
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_PWR_MGMT_2     0x6C
#define REG_WHO_AM_I       0x75

#define MPU_SLEEP_BIT      0x40
#define MPU_CLKSEL_PLL_XGYRO 0x01
#define MPU_DRDY_INT       0x01

static TaskHandle_t s_imu_task = NULL;

/** @brief Reconstruye un entero de 16 bits big-endian a partir de dos bytes. */
static inline int16_t be16(const uint8_t hi, const uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}

/**
 * @brief ISR del pin de datos listos del MPU6050.
 *
 * Solo notifica a la tarea IMU para que la lectura completa ocurra fuera del
 * contexto de interrupcion.
 */
static void IRAM_ATTR mpu_isr_handler(void *arg)
{
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_imu_task, &hp_task_woken);
    if (hp_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/** @brief Escribe un registro del MPU6050. */
static esp_err_t mpu_write(uint8_t reg, uint8_t value)
{
    return hal_i2c_write_reg(APP_MPU_ADDR, reg, value);
}

/** @brief Lee uno o varios registros contiguos del MPU6050. */
static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return hal_i2c_read_reg(APP_MPU_ADDR, reg, data, len);
}

/** @brief Activa o suspende el MPU6050 segun la politica de captura. */
static esp_err_t mpu_set_awake(bool awake)
{
    if (awake) {
        ESP_RETURN_ON_ERROR(mpu_write(REG_PWR_MGMT_1, MPU_CLKSEL_PLL_XGYRO), TAG, "despertar fallo");
        ESP_RETURN_ON_ERROR(mpu_write(REG_PWR_MGMT_2, 0x00), TAG, "power mgmt2 fallo");
    } else {
        ESP_RETURN_ON_ERROR(mpu_write(REG_INT_ENABLE, 0x00), TAG, "deshabilitar int fallo");
        ESP_RETURN_ON_ERROR(mpu_write(REG_PWR_MGMT_1, MPU_SLEEP_BIT), TAG, "sleep fallo");
    }
    return ESP_OK;
}

/**
 * @brief Inicializa el bus, GPIO de interrupcion y registros base del MPU6050.
 * @param[in] imu_task_to_notify Tarea que recibira notificaciones desde la ISR.
 */
esp_err_t hal_mpu6050_init(TaskHandle_t imu_task_to_notify)
{
    s_imu_task = imu_task_to_notify;
    ESP_RETURN_ON_ERROR(hal_i2c_init(), TAG, "I2C no disponible");

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << APP_MPU_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio int config fallo");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "isr service fallo");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(APP_MPU_INT_GPIO, mpu_isr_handler, NULL), TAG, "agregar isr fallo");

    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "despertar inicial fallo");
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * DLPF moderado y 25 Hz de salida. Con DLPF activado la base es 1 kHz,
     * por eso: rate = 1000 / (1 + SMPLRT_DIV) => 25 Hz -> div = 39.
     */
    ESP_RETURN_ON_ERROR(mpu_write(REG_CONFIG, 0x03), TAG, "config dlpf fallo");
    ESP_RETURN_ON_ERROR(mpu_write(REG_SMPLRT_DIV, 39), TAG, "smplrt_div fallo");
    ESP_RETURN_ON_ERROR(mpu_write(REG_GYRO_CONFIG, 0x00), TAG, "gyro config fallo");
    ESP_RETURN_ON_ERROR(mpu_write(REG_ACCEL_CONFIG, 0x00), TAG, "accel config fallo");
    ESP_RETURN_ON_ERROR(mpu_write(REG_FIFO_EN, 0x00), TAG, "fifo en fallo");
    ESP_RETURN_ON_ERROR(mpu_write(REG_INT_PIN_CFG, 0x00), TAG, "int pin cfg fallo");
    ESP_RETURN_ON_ERROR(mpu_write(REG_INT_ENABLE, 0x00), TAG, "int disable fallo");

    ESP_RETURN_ON_ERROR(mpu_set_awake(false), TAG, "sleep inicial fallo");
    ESP_LOGI(TAG, "MPU6050 listo en INT=%d, Fs=%u Hz", APP_MPU_INT_GPIO, APP_IMU_SAMPLE_RATE_HZ);
    return ESP_OK;
}

/** @brief Verifica la presencia del sensor mediante WHO_AM_I. */
esp_err_t hal_mpu6050_verify(void)
{
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "wake verify fallo");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t err = mpu_read(REG_WHO_AM_I, &who, 1);
    mpu_set_awake(false);
    ESP_RETURN_ON_ERROR(err, TAG, "who_am_i read fallo");
    if (who == 0x00 || who == 0xFF) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who);
    return ESP_OK;
}

/**
 * @brief Calcula el sesgo del giroscopio con el sensor en reposo.
 * @param[out] cal Resultado de la calibracion.
 */
esp_err_t hal_mpu6050_calibrate_gyro(mpu6050_calibration_t *cal)
{
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(cal, 0, sizeof(*cal));
    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "wake calibrate fallo");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(mpu_write(REG_INT_ENABLE, MPU_DRDY_INT), TAG, "enable int fallo");

    int64_t sum[3] = {0};
    uint8_t buf[14];
    uint16_t good = 0;

    for (uint16_t i = 0; i < APP_GYRO_BIAS_SAMPLES; ++i) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250)) == 0) {
            continue;
        }
        if (mpu_read(REG_ACCEL_XOUT_H, buf, sizeof(buf)) != ESP_OK) {
            continue;
        }
        sum[0] += be16(buf[8],  buf[9]);
        sum[1] += be16(buf[10], buf[11]);
        sum[2] += be16(buf[12], buf[13]);
        ++good;
    }

    mpu_write(REG_INT_ENABLE, 0x00);
    mpu_set_awake(false);

    if (good < (APP_GYRO_BIAS_SAMPLES / 2)) {
        ESP_LOGW(TAG, "calibracion gyro insuficiente: %u/%u", good, APP_GYRO_BIAS_SAMPLES);
        return ESP_ERR_TIMEOUT;
    }

    cal->gyro_bias_raw[0] = (int32_t)(sum[0] / good);
    cal->gyro_bias_raw[1] = (int32_t)(sum[1] / good);
    cal->gyro_bias_raw[2] = (int32_t)(sum[2] / good);
    cal->bias_valid = true;

    ESP_LOGI(TAG, "Bias gyro raw: [%ld, %ld, %ld]",
             (long)cal->gyro_bias_raw[0], (long)cal->gyro_bias_raw[1], (long)cal->gyro_bias_raw[2]);
    return ESP_OK;
}

/**
 * @brief Captura una ventana local completa del MPU6050.
 * @param[in] cal Sesgo del giroscopio previamente calculado.
 * @param[out] samples Buffer donde se guardan las muestras de la ventana.
 * @param[in] target_samples Numero objetivo de muestras para la ventana.
 * @param[out] captured_samples Numero real de muestras validas obtenidas.
 * @param[out] flags Banderas de calidad generadas durante la captura.
 * @param[in] t0_ms Tiempo de referencia del arranque para timestamps relativos.
 */
esp_err_t hal_mpu6050_capture_window(const mpu6050_calibration_t *cal,
                                     imu_sample_t *samples,
                                     uint16_t target_samples,
                                     uint16_t *captured_samples,
                                     uint16_t *flags,
                                     uint32_t t0_ms)
{
    if (!samples || !captured_samples || !flags) {
        return ESP_ERR_INVALID_ARG;
    }

    *captured_samples = 0;
    *flags = 0;

    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "wake capture fallo");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(mpu_write(REG_INT_ENABLE, MPU_DRDY_INT), TAG, "int enable capture fallo");

    uint8_t buf[14];
    for (uint16_t i = 0; i < target_samples; ++i) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250)) == 0) {
            *flags |= APP_FLAG_IMU_SAMPLE_LOSS;
            continue;
        }

        if (mpu_read(REG_ACCEL_XOUT_H, buf, sizeof(buf)) != ESP_OK) {
            *flags |= APP_FLAG_IMU_SAMPLE_LOSS;
            continue;
        }

        const int16_t ax_raw = be16(buf[0],  buf[1]);
        const int16_t ay_raw = be16(buf[2],  buf[3]);
        const int16_t az_raw = be16(buf[4],  buf[5]);
        int32_t gx_raw = be16(buf[8],  buf[9]);
        int32_t gy_raw = be16(buf[10], buf[11]);
        int32_t gz_raw = be16(buf[12], buf[13]);

        if (cal && cal->bias_valid) {
            gx_raw -= cal->gyro_bias_raw[0];
            gy_raw -= cal->gyro_bias_raw[1];
            gz_raw -= cal->gyro_bias_raw[2];
            *flags |= APP_FLAG_GYRO_BIAS_OK;
        }

        imu_sample_t *s = &samples[*captured_samples];
        s->acc_mg[0] = (int16_t)lroundf(((float)ax_raw * 1000.0f) / APP_MPU_ACCEL_LSB_PER_G);
        s->acc_mg[1] = (int16_t)lroundf(((float)ay_raw * 1000.0f) / APP_MPU_ACCEL_LSB_PER_G);
        s->acc_mg[2] = (int16_t)lroundf(((float)az_raw * 1000.0f) / APP_MPU_ACCEL_LSB_PER_G);
        s->gyro_dps10[0] = (int16_t)lroundf(((float)gx_raw * 10.0f) / APP_MPU_GYRO_LSB_PER_DPS);
        s->gyro_dps10[1] = (int16_t)lroundf(((float)gy_raw * 10.0f) / APP_MPU_GYRO_LSB_PER_DPS);
        s->gyro_dps10[2] = (int16_t)lroundf(((float)gz_raw * 10.0f) / APP_MPU_GYRO_LSB_PER_DPS);
        s->t_rel_ms = (uint32_t)(esp_timer_get_time() / 1000ULL) - t0_ms;

        ++(*captured_samples);
    }

    mpu_write(REG_INT_ENABLE, 0x00);
    mpu_set_awake(false);

    if (*captured_samples >= (target_samples * 80u / 100u)) {
        *flags |= APP_FLAG_IMU_VALID_WINDOW;
    }
    if (*captured_samples == target_samples) {
        *flags |= APP_FLAG_IMU_WINDOW_COMPLETE;
    }

    return ESP_OK;
}
