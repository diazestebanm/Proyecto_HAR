/**
 * @file hal_sensors.c
 * @brief HAL de sensores del Nodo 1.
 *
 * Integra el QMI8658, el MAX30100 y la lectura de bateria. Tambien administra
 * la interrupcion del PPG y la calibracion basica del sesgo del giroscopio.
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "hal_sensors.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

static const char *TAG = "hal_sensors";

/* ---------- I2C helpers ---------- */
/** @brief Lee uno o varios bytes desde un registro I2C. */
static esp_err_t i2c_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(port, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/** @brief Escribe un byte sobre un registro de un esclavo I2C. */
static esp_err_t i2c_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d[2] = {reg, val};
    return i2c_master_write_to_device(port, addr, d, sizeof(d), pdMS_TO_TICKS(100));
}

/* ---------- IMU state ---------- */
static uint8_t s_qmi_addr = 0;
static bool s_imu_bias_ok = false;
static int32_t s_gx_bias = 0, s_gy_bias = 0, s_gz_bias = 0;

/* ---------- PPG state ---------- */
static TaskHandle_t s_ppg_task_handle = NULL;

static uint16_t s_red_buf[APP_PPG_WINDOW_SAMPLES];
static uint16_t s_ir_buf[APP_PPG_WINDOW_SAMPLES];
static int s_ppg_head = 0;
static int s_ppg_count = 0;

/* ---------- Battery state ---------- */
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali = NULL;
static bool s_cali_ok = false;
static float s_ema_vbat = -1.0f;
static float s_last_pct = -1.0f;

/* ---------- MAX30100 ISR ---------- */
/**
 * @brief ISR del pin de interrupcion del MAX30100.
 *
 * Solo despierta a la tarea del PPG para que el procesamiento se haga fuera
 * del contexto de interrupcion.
 */
static void IRAM_ATTR max30100_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    if (s_ppg_task_handle) {
        vTaskNotifyGiveFromISR(s_ppg_task_handle, &hp);
    }
    if (hp) {
        portYIELD_FROM_ISR();
    }
}

/* ---------- Shared helpers ---------- */
/**
 * @brief Configura un bus I2C maestro con los pines y frecuencia indicados.
 */
static esp_err_t i2c_init_bus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t hz)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = hz,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(port, &conf), TAG, "i2c_param_config");
    return i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
}

/* ---------- IMU ---------- */
/** @brief Busca la direccion valida del QMI8658 en el bus I2C. */
static esp_err_t qmi_detect(void)
{
    uint8_t who = 0;
    if (i2c_read_reg(APP_IMU_I2C_PORT, APP_QMI8658_ADDR1, APP_QMI_REG_WHOAMI, &who, 1) == ESP_OK) {
        s_qmi_addr = APP_QMI8658_ADDR1;
        return ESP_OK;
    }
    if (i2c_read_reg(APP_IMU_I2C_PORT, APP_QMI8658_ADDR2, APP_QMI_REG_WHOAMI, &who, 1) == ESP_OK) {
        s_qmi_addr = APP_QMI8658_ADDR2;
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

/** @brief Aplica la configuracion inicial del QMI8658. */
static esp_err_t qmi_init_regs(void)
{
    ESP_RETURN_ON_ERROR(qmi_detect(), TAG, "QMI not found");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL7, 0x00), TAG, "CTRL7 off");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL1, 0x40), TAG, "CTRL1");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL2, 0x23), TAG, "CTRL2");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL3, 0x43), TAG, "CTRL3");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL7, 0x03), TAG, "CTRL7 on");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

/**
 * @brief Lee una muestra cruda completa de acelerometro y giroscopio.
 */
static esp_err_t qmi_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t b[12];
    ESP_RETURN_ON_ERROR(i2c_read_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_AX_L, b, sizeof(b)), TAG, "QMI raw");
    *ax = (int16_t)((b[1] << 8) | b[0]);
    *ay = (int16_t)((b[3] << 8) | b[2]);
    *az = (int16_t)((b[5] << 8) | b[4]);
    *gx = (int16_t)((b[7] << 8) | b[6]);
    *gy = (int16_t)((b[9] << 8) | b[8]);
    *gz = (int16_t)((b[11] << 8) | b[10]);
    return ESP_OK;
}

/** @brief Estima el sesgo del giroscopio promediando varias muestras. */
static void qmi_calibrate_bias(void)
{
    const int N = 50;
    int64_t sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < N; ++i) {
        int16_t ax, ay, az, gx, gy, gz;
        if (qmi_read_raw(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            sx += gx;
            sy += gy;
            sz += gz;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_gx_bias = (int32_t)(sx / N);
    s_gy_bias = (int32_t)(sy / N);
    s_gz_bias = (int32_t)(sz / N);
    s_imu_bias_ok = true;
}

/* ---------- MAX30100 ---------- */
/** @brief Reinicia los punteros de la FIFO interna del MAX30100. */
static esp_err_t max_clear_fifo(void)
{
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_FIFO_WR, 0x00), TAG, "FIFO WR");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_FIFO_OVF, 0x00), TAG, "FIFO OVF");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_FIFO_RD, 0x00), TAG, "FIFO RD");
    return ESP_OK;
}

/** @brief Configura el MAX30100 para operar en modo SpO2. */
static esp_err_t max_init_regs(void)
{
    uint8_t part = 0;
    ESP_RETURN_ON_ERROR(i2c_read_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_PART_ID, &part, 1), TAG, "PART_ID");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_MODE_CFG, APP_MAX_MODE_RESET), TAG, "RESET");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_INT_ENABLE, 0x10), TAG, "INT A_FULL");
    ESP_RETURN_ON_ERROR(max_clear_fifo(), TAG, "FIFO clear");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_SPO2_CFG, APP_MAX_SPO2_CFG), TAG, "SPO2 CFG");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_LED_CFG, APP_MAX_LED_CFG), TAG, "LED CFG");
    ESP_RETURN_ON_ERROR(i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_MODE_CFG, APP_MAX_MODE_SPO2), TAG, "MODE SPO2");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

/** @brief Calcula cuantas muestras pendientes hay en la FIFO del MAX30100. */
static esp_err_t max_get_fifo_count(uint8_t *count)
{
    uint8_t wr = 0, rd = 0;
    ESP_RETURN_ON_ERROR(i2c_read_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_FIFO_WR, &wr, 1), TAG, "FIFO_WR");
    ESP_RETURN_ON_ERROR(i2c_read_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_FIFO_RD, &rd, 1), TAG, "FIFO_RD");
    *count = (wr - rd) & 0x0F;
    return ESP_OK;
}

/** @brief Extrae una muestra roja e infrarroja desde la FIFO del MAX30100. */
static esp_err_t max_read_sample(uint16_t *red, uint16_t *ir)
{
    uint8_t d[4];
    ESP_RETURN_ON_ERROR(i2c_read_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_FIFO_DATA, d, sizeof(d)), TAG, "FIFO_DATA");
    *ir  = ((uint16_t)d[0] << 8) | d[1];
    *red = ((uint16_t)d[2] << 8) | d[3];
    return ESP_OK;
}

/** @brief Inserta una muestra PPG en el buffer circular de analisis. */
static void ppg_push(uint16_t red, uint16_t ir)
{
    s_red_buf[s_ppg_head] = red;
    s_ir_buf[s_ppg_head] = ir;
    s_ppg_head = (s_ppg_head + 1) % APP_PPG_WINDOW_SAMPLES;
    if (s_ppg_count < APP_PPG_WINDOW_SAMPLES) s_ppg_count++;
}

/** @brief Copia el contenido actual del buffer circular a arreglos lineales. */
static int ppg_copy(uint16_t *r, uint16_t *ir)
{
    int n = s_ppg_count;
    int start = s_ppg_head - n;
    if (start < 0) start += APP_PPG_WINDOW_SAMPLES;
    for (int i = 0; i < n; ++i) {
        int idx = (start + i) % APP_PPG_WINDOW_SAMPLES;
        r[i] = s_red_buf[idx];
        ir[i] = s_ir_buf[idx];
    }
    return n;
}

/** @brief Limita un valor flotante al rango permitido por un uint8_t. */
static uint8_t clamp_u8(float x, uint8_t lo, uint8_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return (uint8_t)x;
}

/**
 * @brief Calcula BPM, SpO2 y presencia de dedo a partir del buffer PPG.
 */
static void ppg_analyze(ppg_metrics_t *out)
{
    static uint16_t r[APP_PPG_WINDOW_SAMPLES];
    static uint16_t ir[APP_PPG_WINDOW_SAMPLES];
    static float ir_ac[APP_PPG_WINDOW_SAMPLES];

    memset(out, 0, sizeof(*out));
    int n = ppg_copy(r, ir);
    if (n < 200) return;

    double sum_r = 0.0, sum_ir = 0.0;
    uint16_t min_ir = 0xFFFF, max_ir = 0;
    for (int i = 0; i < n; ++i) {
        sum_r += r[i];
        sum_ir += ir[i];
        if (ir[i] < min_ir) min_ir = ir[i];
        if (ir[i] > max_ir) max_ir = ir[i];
    }

    float dc_r = (float)(sum_r / n);
    float dc_ir = (float)(sum_ir / n);
    out->dc_ir = (uint16_t)dc_ir;
    out->finger_present = (dc_ir > 8000.0f) && ((max_ir - min_ir) > 50);

    if (!out->finger_present) return;

    double sq_r = 0.0, sq_ir = 0.0;
    for (int i = 0; i < n; ++i) {
        float rac = (float)r[i] - dc_r;
        float iac = (float)ir[i] - dc_ir;
        sq_r += rac * rac;
        sq_ir += iac * iac;
        if (i == 0 || i == n - 1) ir_ac[i] = iac;
        else ir_ac[i] = (((float)ir[i - 1] + ir[i] + ir[i + 1]) / 3.0f) - dc_ir;
    }

    float ac_r = sqrtf((float)(sq_r / n));
    float ac_ir = sqrtf((float)(sq_ir / n));

    if (dc_r > 1 && dc_ir > 1 && ac_r > 1 && ac_ir > 1) {
        float ratio = (ac_r / dc_r) / (ac_ir / dc_ir);
        float spo2 = 110.0f - 25.0f * ratio;
        if (spo2 >= 70.0f && spo2 <= 100.0f) {
            out->spo2 = clamp_u8(spo2, 0, 100);
            out->spo2_valid = true;
        }
    }

    float thr = fmaxf(25.0f, ac_ir * 0.5f);
    int last_peak = -1000;
    int intervals = 0;
    int interval_sum = 0;

    for (int i = 1; i < n - 1; ++i) {
        bool peak = (ir_ac[i] > thr) && (ir_ac[i] > ir_ac[i - 1]) && (ir_ac[i] >= ir_ac[i + 1]);
        if (peak && (i - last_peak) >= 35) {
            if (last_peak >= 0) {
                interval_sum += (i - last_peak);
                intervals++;
            }
            last_peak = i;
        }
    }

    if (intervals > 0) {
        float avg = (float)interval_sum / intervals;
        float bpm = 60.0f * APP_PPG_SAMPLE_RATE_HZ / avg;
        if (bpm >= 40.0f && bpm <= 200.0f) {
            out->bpm = clamp_u8(bpm, 0, 255);
            out->bpm_valid = true;
        }
    }
}

/* ---------- Battery ---------- */
/** @brief Comparador entero utilizado para ordenar muestras ADC. */
static int cmp_int(const void *a, const void *b)
{
    return (*(int *)a - *(int *)b);
}

/** @brief Convierte una lectura cruda de ADC a milivoltios. */
static int raw_to_mv(int raw)
{
    if (s_cali_ok) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) return mv;
    }
    return (int)((raw * 3100.0f) / 4095.0f);
}

/** @brief Mapea un voltaje de bateria LiPo aproximado a porcentaje. */
static float vbat_to_percent(float v)
{
    static const float map_v[]   = {4.20f,4.10f,4.00f,3.90f,3.80f,3.70f,3.60f,3.50f,3.40f,3.30f,3.20f,3.10f,3.00f};
    static const float map_pct[] = {100.f,95.f,88.f,78.f,65.f,50.f,36.f,24.f,14.f,8.f,4.f,1.f,0.f};
    const int N = 13;

    if (v >= map_v[0]) return 100.0f;
    if (v <= map_v[N - 1]) return 0.0f;

    for (int i = 0; i < N - 1; ++i) {
        if (v >= map_v[i + 1]) {
            float t = (v - map_v[i + 1]) / (map_v[i] - map_v[i + 1]);
            return map_pct[i + 1] + t * (map_pct[i] - map_pct[i + 1]);
        }
    }
    return 0.0f;
}

/** @brief Inicializa el ADC one-shot y su calibracion para la bateria. */
static esp_err_t battery_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = APP_BAT_ADC_UNIT };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_cfg, &s_adc_handle), TAG, "adc new");

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, APP_BAT_ADC_CHANNEL, &ch_cfg), TAG, "adc channel");

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = APP_BAT_ADC_UNIT,
        .chan = APP_BAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    s_cali_ok = (adc_cali_create_scheme_curve_fitting(&cfg, &s_cali) == ESP_OK);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id = APP_BAT_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    s_cali_ok = (adc_cali_create_scheme_line_fitting(&cfg, &s_cali) == ESP_OK);
#else
    s_cali_ok = false;
#endif
    return ESP_OK;
}

/* ---------- Public API ---------- */
/**
 * @brief Inicializa los tres subsistemas de sensores del Nodo 1.
 * @param[in] ppg_task_handle Tarea que recibira la notificacion de la ISR PPG.
 * @return ESP_OK si todos los sensores quedan listos.
 */
esp_err_t hal_sensors_init(TaskHandle_t ppg_task_handle)
{
    s_ppg_task_handle = ppg_task_handle;

    ESP_RETURN_ON_ERROR(
        i2c_init_bus(APP_IMU_I2C_PORT, APP_IMU_SDA_GPIO, APP_IMU_SCL_GPIO, APP_IMU_I2C_HZ),
        TAG, "IMU I2C"
    );

    /* Si IMU y PPG comparten el mismo bus I2C, no reinstalar el driver */
    if (!(APP_PPG_I2C_PORT == APP_IMU_I2C_PORT &&
          APP_PPG_SDA_GPIO == APP_IMU_SDA_GPIO &&
          APP_PPG_SCL_GPIO == APP_IMU_SCL_GPIO)) {
        ESP_RETURN_ON_ERROR(
            i2c_init_bus(APP_PPG_I2C_PORT, APP_PPG_SDA_GPIO, APP_PPG_SCL_GPIO, APP_PPG_I2C_HZ),
            TAG, "PPG I2C"
        );
    }

    ESP_RETURN_ON_ERROR(qmi_init_regs(), TAG, "QMI init");
    qmi_calibrate_bias();
    ESP_RETURN_ON_ERROR(max_init_regs(), TAG, "MAX init");
    ESP_RETURN_ON_ERROR(battery_init(), TAG, "BAT init");

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << APP_PPG_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "GPIO INT");

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        return isr_err;
    }

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(APP_PPG_INT_GPIO, max30100_isr, NULL),
        TAG, "ISR add"
    );

    return ESP_OK;
}

/** @brief Habilita la captura del QMI8658. */
esp_err_t hal_imu_start(void)
{
    return i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL7, 0x03);
}

/** @brief Deshabilita temporalmente la adquisicion del QMI8658. */
esp_err_t hal_imu_stop(void)
{
    return i2c_write_reg(APP_IMU_I2C_PORT, s_qmi_addr, APP_QMI_REG_CTRL7, 0x00);
}

/**
 * @brief Lee una muestra IMU ya corregida por sesgo de giroscopio.
 * @param[out] out Muestra de salida.
 */
esp_err_t hal_imu_read_sample(imu_sample_t *out)
{
    int16_t ax, ay, az, gx, gy, gz;
    ESP_RETURN_ON_ERROR(qmi_read_raw(&ax, &ay, &az, &gx, &gy, &gz), TAG, "QMI read");
    out->ax = ax;
    out->ay = ay;
    out->az = az;
    out->gx = gx - s_gx_bias;
    out->gy = gy - s_gy_bias;
    out->gz = gz - s_gz_bias;
    out->t_ms = 0;
    return ESP_OK;
}

/** @brief Informa si la calibracion de sesgo del giroscopio ya es valida. */
bool hal_imu_bias_ok(void)
{
    return s_imu_bias_ok;
}

/** @brief Activa el modo de muestreo SpO2 del MAX30100. */
esp_err_t hal_ppg_start(void)
{
    return i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_MODE_CFG, APP_MAX_MODE_SPO2);
}

/** @brief Detiene la adquisicion del MAX30100. */
esp_err_t hal_ppg_stop(void)
{
    return i2c_write_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_MODE_CFG, 0x00);
}

/**
 * @brief Atiende la FIFO del MAX30100 y recalcula las metricas PPG.
 * @param[out] out Resultado resumido del analisis actual.
 */
esp_err_t hal_ppg_service(ppg_metrics_t *out)
{
    uint8_t count = 0;
    uint8_t status = 0;

    i2c_read_reg(APP_PPG_I2C_PORT, APP_MAX30100_ADDR, APP_MAX_REG_INT_STATUS, &status, 1);
    ESP_RETURN_ON_ERROR(max_get_fifo_count(&count), TAG, "MAX count");

    while (count-- > 0) {
        uint16_t red, ir;
        if (max_read_sample(&red, &ir) == ESP_OK) {
            ppg_push(red, ir);
        }
    }

    ppg_analyze(out);
    return ESP_OK;
}

/**
 * @brief Realiza una lectura robusta de bateria con mediana y EMA.
 * @param[out] out Estado resumido de bateria.
 */
esp_err_t hal_battery_read(battery_status_t *out)
{
    int samples[32];
    for (int i = 0; i < 32; ++i) {
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle, APP_BAT_ADC_CHANNEL, &samples[i]), TAG, "ADC read");
        vTaskDelay(pdMS_TO_TICKS(4));
    }

    qsort(samples, 32, sizeof(int), cmp_int);
    int raw = (samples[15] + samples[16]) / 2;
    int mv_gpio = raw_to_mv(raw);
    float vbat = (mv_gpio / 1000.0f) * APP_BAT_DIV_RATIO;

    if (s_ema_vbat < 0.0f) s_ema_vbat = vbat;
    else s_ema_vbat = 0.15f * vbat + 0.85f * s_ema_vbat;

    float pct = vbat_to_percent(s_ema_vbat);
    if (s_last_pct < 0.0f || fabsf(pct - s_last_pct) >= 2.0f) s_last_pct = pct;

    out->mv = (uint16_t)(s_ema_vbat * 1000.0f);
    out->pct = clamp_u8(s_last_pct, 0, 100);
    out->valid = (out->mv >= 2800 && out->mv <= 4400);
    out->critical = out->valid && (out->mv <= APP_BAT_CRITICAL_MV);
    out->low = out->valid && (out->mv <= APP_BAT_LOW_MV);

    return ESP_OK;
}