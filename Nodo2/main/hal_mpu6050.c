/**
 * @file hal_mpu6050.c
 * @brief Implementacion de la HAL del MPU6050 para el Nodo 2.
 *
 * Este modulo se encarga de detectar el sensor, configurarlo en bajo consumo,
 * habilitar la interrupcion de data-ready, calibrar el sesgo del giroscopio y
 * capturar ventanas locales listas para la etapa de extraccion de rasgos.
 */

#include "hal_mpu6050.h"

#include <math.h>
#include <string.h>
#include "app_config.h"
#include "hal_i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HAL_MPU";

/* Registros clave del MPU6050 */
#define REG_SMPLRT_DIV     0x19 /**< Divisor del sample rate interno. */
#define REG_CONFIG         0x1A /**< Configuracion del filtro digital DLPF. */
#define REG_GYRO_CONFIG    0x1B /**< Escala completa del giroscopio. */
#define REG_ACCEL_CONFIG   0x1C /**< Escala completa del acelerometro. */
#define REG_FIFO_EN        0x23 /**< Control de habilitacion de FIFO. */
#define REG_INT_PIN_CFG    0x37 /**< Politica electrica del pin de interrupcion. */
#define REG_INT_ENABLE     0x38 /**< Mascara de interrupciones del sensor. */
#define REG_ACCEL_XOUT_H   0x3B /**< Inicio del bloque de lectura acelerometro/gyro. */
#define REG_PWR_MGMT_1     0x6B /**< Registro principal de energia y reloj. */
#define REG_PWR_MGMT_2     0x6C /**< Habilitacion individual de ejes. */
#define REG_WHO_AM_I       0x75 /**< Registro de identificacion del dispositivo. */

#define MPU_SLEEP_BIT         0x40 /**< Bit de bajo consumo del registro PWR_MGMT_1. */
#define MPU_SOFT_RESET        0x80 /**< Bit de reset por software. */
#define MPU_CLKSEL_PLL_XGYRO  0x01 /**< Seleccion del PLL del eje X como reloj. */
#define MPU_DRDY_INT          0x01 /**< Interrupcion de data-ready. */

static TaskHandle_t s_imu_task = NULL;
static SemaphoreHandle_t s_drdy_sem = NULL;
static uint8_t s_mpu_addr = APP_MPU_ADDR;
static bool s_isr_service_installed = false;

/**
 * @brief Convierte dos bytes big-endian del MPU en un entero con signo.
 */
static inline int16_t be16(const uint8_t hi, const uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}

/**
 * @brief ISR del pin INT del MPU6050.
 *
 * La rutina no procesa muestras; solo libera el semaforo que despierta la
 * captura en contexto de tarea cuando el sensor anuncia data-ready.
 */
static void IRAM_ATTR mpu_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t hp_task_woken = pdFALSE;
    if (s_drdy_sem) {
        xSemaphoreGiveFromISR(s_drdy_sem, &hp_task_woken);
    }
    if (hp_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Envia una escritura simple al registro indicado del MPU.
 */
static esp_err_t mpu_write(uint8_t reg, uint8_t value)
{
    return hal_i2c_write_reg(s_mpu_addr, reg, value);
}

/**
 * @brief Lee uno o mas bytes consecutivos desde el MPU actualmente activo.
 */
static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return hal_i2c_read_reg(s_mpu_addr, reg, data, len);
}

/**
 * @brief Cambia el estado de energia del MPU6050.
 *
 * @param[in] awake true para activar el sensor, false para ponerlo a dormir.
 * @return ESP_OK si el cambio de estado se aplica correctamente.
 */
static esp_err_t mpu_set_awake(bool awake)
{
    esp_err_t err = ESP_FAIL;

    for (int attempt = 0; attempt < 3; ++attempt) {
        if (awake) {
            err = mpu_write(REG_PWR_MGMT_1, MPU_CLKSEL_PLL_XGYRO);
            if (err == ESP_OK) {
                err = mpu_write(REG_PWR_MGMT_2, 0x00);
            }
        } else {
            err = mpu_write(REG_INT_ENABLE, 0x00);
            if (err == ESP_OK) {
                err = mpu_write(REG_PWR_MGMT_1, MPU_SLEEP_BIT);
            }
        }

        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }

    return err;
}

/**
 * @brief Sonda una direccion I2C concreta y valida el WHO_AM_I.
 *
 * @param[in] addr Direccion candidata del MPU.
 * @param[out] who_out Valor leido de WHO_AM_I, si el puntero es valido.
 */
static esp_err_t mpu_probe_one(uint8_t addr, uint8_t *who_out)
{
    uint8_t who = 0;
    s_mpu_addr = addr;

    if (hal_i2c_ping(addr) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    if (mpu_read(REG_WHO_AM_I, &who, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    if (who == 0x00 || who == 0xFF) {
        return ESP_ERR_NOT_FOUND;
    }

    if (who_out) {
        *who_out = who;
    }
    return ESP_OK;
}

/**
 * @brief Detecta el MPU6050 probando la direccion primaria y la alternativa.
 *
 * Algunos modulos cambian la direccion por hardware con AD0; este helper evita
 * que el firmware dependa de una sola direccion fija.
 */
static esp_err_t mpu_probe_and_select(uint8_t *who_out)
{
    esp_err_t err;
    uint8_t who = 0;

    err = mpu_probe_one(APP_MPU_ADDR, &who);
    if (err == ESP_OK) {
        if (who_out) *who_out = who;
        return ESP_OK;
    }

    /* Probar la direccion alternativa por si AD0 cambia o el modulo la usa */
    const uint8_t alt = (APP_MPU_ADDR == 0x68) ? 0x69 : 0x68;
    err = mpu_probe_one(alt, &who);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "MPU detectado en direccion alternativa 0x%02X", alt);
        if (who_out) *who_out = who;
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Aplica reset por software y deja el MPU en el modo final del proyecto.
 *
 * La configuracion final usa DLPF moderado y una salida efectiva cercana a
 * 25 Hz, coherente con la ventana usada en el Nodo 2.
 */
static esp_err_t mpu_soft_reset_and_configure(void)
{
    ESP_RETURN_ON_ERROR(mpu_write(REG_PWR_MGMT_1, MPU_SOFT_RESET), TAG, "soft reset fallo");
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "despertar fallo");
    vTaskDelay(pdMS_TO_TICKS(50));

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

    return ESP_OK;
}

/**
 * @brief Inicializa la HAL del MPU6050 y deja el sensor en reposo controlado.
 *
 * El flujo instala el semaforo de data-ready, configura el GPIO de interrupcion,
 * prueba varias veces la comunicacion I2C y finalmente deja al sensor dormido
 * hasta que una ventana de captura lo requiera.
 */
esp_err_t hal_mpu6050_init(TaskHandle_t imu_task_to_notify)
{
    s_imu_task = imu_task_to_notify;
    ESP_RETURN_ON_FALSE(s_imu_task != NULL, ESP_ERR_INVALID_ARG, TAG, "imu task nula");

    if (!s_drdy_sem) {
        s_drdy_sem = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_drdy_sem != NULL, ESP_ERR_NO_MEM, TAG, "sem drdy no creada");
    }

    while (xSemaphoreTake(s_drdy_sem, 0) == pdTRUE) {
    }

    ESP_RETURN_ON_ERROR(hal_i2c_init(), TAG, "I2C no disponible");

    /* Dar tiempo real al modulo para estabilizarse despues del power-up */
    vTaskDelay(pdMS_TO_TICKS(200));

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << APP_MPU_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio int config fallo");

    if (!s_isr_service_installed) {
        esp_err_t isr_err = gpio_install_isr_service(0);
        if (isr_err == ESP_OK || isr_err == ESP_ERR_INVALID_STATE) {
            s_isr_service_installed = true;
        } else {
            return isr_err;
        }
    }

    gpio_isr_handler_remove(APP_MPU_INT_GPIO);
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(APP_MPU_INT_GPIO, mpu_isr_handler, NULL), TAG, "agregar isr fallo");

    esp_err_t err = ESP_FAIL;
    uint8_t who = 0;

    for (int attempt = 0; attempt < 5; ++attempt) {
        ESP_LOGW(TAG, "Intento init MPU %d/5", attempt + 1);

        hal_i2c_deinit();
        ESP_RETURN_ON_ERROR(hal_i2c_init(), TAG, "reinit i2c fallo");
        vTaskDelay(pdMS_TO_TICKS(100));

        err = mpu_probe_and_select(&who);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        err = mpu_soft_reset_and_configure();
        if (err == ESP_OK) {
            while (xSemaphoreTake(s_drdy_sem, 0) == pdTRUE) {
            }
            ESP_LOGI(TAG, "MPU6050 listo en addr=0x%02X INT=%d Fs=%u Hz WHO_AM_I=0x%02X",
                     s_mpu_addr, APP_MPU_INT_GPIO, APP_IMU_SAMPLE_RATE_HZ, who);
            ESP_RETURN_ON_ERROR(mpu_set_awake(false), TAG, "sleep inicial fallo");
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }

    return err == ESP_OK ? ESP_FAIL : err;
}

/**
 * @brief Lee el registro WHO_AM_I para validar que el MPU siga accesible.
 */
esp_err_t hal_mpu6050_verify(void)
{
    uint8_t who = 0;

    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "wake verify fallo");
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t err = mpu_read(REG_WHO_AM_I, &who, 1);

    (void)mpu_set_awake(false);
    ESP_RETURN_ON_ERROR(err, TAG, "who_am_i read fallo");

    if (who == 0x00 || who == 0xFF) {
        return ESP_ERR_NOT_FOUND;
    }

    if (who != 0x68 && who != 0x98) {
        ESP_LOGW(TAG, "WHO_AM_I no esperado: 0x%02X", who);
    } else {
        ESP_LOGI(TAG, "WHO_AM_I=0x%02X", who);
    }

    return ESP_OK;
}

/**
 * @brief Calcula el sesgo promedio del giroscopio con el sensor quieto.
 *
 * @param[out] cal Estructura donde se escribe el sesgo estimado.
 * @return ESP_OK si se logra acumular al menos la mitad de las muestras meta.
 */
esp_err_t hal_mpu6050_calibrate_gyro(mpu6050_calibration_t *cal)
{
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(cal, 0, sizeof(*cal));
    while (xSemaphoreTake(s_drdy_sem, 0) == pdTRUE) {
    }
    ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "wake calibrate fallo");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(mpu_write(REG_INT_ENABLE, MPU_DRDY_INT), TAG, "enable int fallo");

    int64_t sum[3] = {0};
    uint8_t buf[14];
    uint16_t good = 0;

    for (uint16_t i = 0; i < APP_GYRO_BIAS_SAMPLES; ++i) {
        if (xSemaphoreTake(s_drdy_sem, pdMS_TO_TICKS(250)) != pdTRUE) {
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

    (void)mpu_write(REG_INT_ENABLE, 0x00);
    (void)mpu_set_awake(false);

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
 * @brief Captura una ventana local de muestras del MPU6050.
 *
 * El metodo despierta el sensor, espera interrupciones data-ready y convierte
 * cada muestra a las unidades compactas usadas por el resto del proyecto.
 * Tambien actualiza las banderas de calidad de la ventana.
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

    while (xSemaphoreTake(s_drdy_sem, 0) == pdTRUE) {
    }
    esp_err_t err = mpu_set_awake(true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "wake capture fallo, intentando recovery...");
        ESP_RETURN_ON_ERROR(hal_mpu6050_recover(), TAG, "recovery mpu fallo");
        ESP_RETURN_ON_ERROR(mpu_set_awake(true), TAG, "wake capture tras recovery fallo");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(mpu_write(REG_INT_ENABLE, MPU_DRDY_INT), TAG, "int enable capture fallo");

    uint8_t buf[14];
    for (uint16_t i = 0; i < target_samples; ++i) {
        if (xSemaphoreTake(s_drdy_sem, pdMS_TO_TICKS(250)) != pdTRUE) {
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

    (void)mpu_write(REG_INT_ENABLE, 0x00);
    (void)mpu_set_awake(false);

    if (*captured_samples >= (target_samples * 80u / 100u)) {
        *flags |= APP_FLAG_IMU_VALID_WINDOW;
    }
    if (*captured_samples == target_samples) {
        *flags |= APP_FLAG_IMU_WINDOW_COMPLETE;
    }

    return ESP_OK;
}

/**
 * @brief Recompone el bus I2C y reconfigura el MPU tras un fallo de captura.
 *
 * @return ESP_OK si el sensor vuelve a responder y queda configurado.
 */
esp_err_t hal_mpu6050_recover(void)
{
    uint8_t who = 0;

    (void)mpu_write(REG_INT_ENABLE, 0x00);
    (void)mpu_set_awake(false);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_RETURN_ON_ERROR(hal_i2c_deinit(), TAG, "i2c deinit recovery fallo");
    ESP_RETURN_ON_ERROR(hal_i2c_init(), TAG, "i2c init recovery fallo");
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(mpu_probe_and_select(&who), TAG, "probe recovery fallo");
    ESP_RETURN_ON_ERROR(mpu_soft_reset_and_configure(), TAG, "reconfig recovery fallo");

    while (xSemaphoreTake(s_drdy_sem, 0) == pdTRUE) {
    }

    ESP_LOGW(TAG, "MPU recuperado. WHO_AM_I=0x%02X addr=0x%02X", who, s_mpu_addr);
    return ESP_OK;
}
