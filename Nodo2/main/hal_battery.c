/**
 * @file hal_battery.c
 * @brief Lectura robusta de bateria para el Nodo 2.
 *
 * Implementa muestreo multiple, deteccion de pin flotante, filtrado EMA y una
 * conversion aproximada de voltaje a porcentaje para una celda LiPo de 1S.
 */

#include "hal_battery.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include "app_config.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "HAL_BAT";

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static bool s_cali_ok = false;
static float s_ema_vbat = -1.0f;
static float s_last_pct = -1.0f;

/** @brief Comparador usado para ordenar muestras ADC. */
static int cmp_int(const void *a, const void *b)
{
    return (*(const int *)a - *(const int *)b);
}

/** @brief Inicializa la calibracion del ADC si el esquema esta disponible. */
static bool battery_cal_init(adc_cali_handle_t *out)
{
    esp_err_t err = ESP_FAIL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id = APP_BAT_ADC_UNIT,
        .chan = APP_BAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_cali_create_scheme_curve_fitting(&cfg, out);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id = APP_BAT_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_cali_create_scheme_line_fitting(&cfg, out);
#endif
    return (err == ESP_OK);
}

/** @brief Convierte una lectura ADC cruda a milivoltios. */
static int raw_to_mv(int raw)
{
    if (s_cali_ok) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            return mv;
        }
    }
    return (int)((raw * 3100.0f) / 4095.0f);
}

/** @brief Convierte el voltaje de la bateria a un porcentaje aproximado. */
static float vbat_to_percent(float v)
{
    static const float map_v[]   = {4.20f,4.10f,4.00f,3.90f,3.80f,3.70f,3.60f,3.50f,3.40f,3.30f,3.20f,3.10f,3.00f};
    static const float map_pct[] = {100.f,95.f,88.f,78.f,65.f,50.f,36.f,24.f,14.f,8.f,4.f,1.f,0.f};
    const int n = 13;

    if (v >= map_v[0]) return 100.0f;
    if (v <= map_v[n - 1]) return 0.0f;

    for (int i = 0; i < n - 1; ++i) {
        if (v >= map_v[i + 1]) {
            const float t = (v - map_v[i + 1]) / (map_v[i] - map_v[i + 1]);
            return map_pct[i + 1] + t * (map_pct[i] - map_pct[i + 1]);
        }
    }
    return 0.0f;
}

/**
 * @brief Inicializa el ADC de bateria y, si aplica, su calibracion.
 */
esp_err_t hal_battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = APP_BAT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit fallo");

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, APP_BAT_ADC_CHANNEL, &ch_cfg), TAG, "adc channel fallo");

    s_cali_ok = battery_cal_init(&s_cali);
    ESP_LOGI(TAG, "ADC bateria listo en GPIO=%d, calibracion=%s", APP_BAT_GPIO, s_cali_ok ? "OK" : "NO");
    return ESP_OK;
}

/**
 * @brief Toma una lectura robusta de bateria y genera su resumen.
 * @param[out] out Estructura de salida con voltaje, porcentaje y flags.
 */
esp_err_t hal_battery_sample(battery_status_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    int samples[APP_BAT_NUM_SAMPLES];
    for (int i = 0; i < APP_BAT_NUM_SAMPLES; ++i) {
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, APP_BAT_ADC_CHANNEL, &samples[i]), TAG, "adc read fallo");
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    qsort(samples, APP_BAT_NUM_SAMPLES, sizeof(int), cmp_int);
    const int range = samples[APP_BAT_NUM_SAMPLES - 1] - samples[0];
    const int median_raw = (samples[APP_BAT_NUM_SAMPLES / 2] + samples[(APP_BAT_NUM_SAMPLES / 2) - 1]) / 2;

    if (range > APP_BAT_FLOAT_RANGE_THRESH) {
        s_ema_vbat = -1.0f;
        s_last_pct = -1.0f;
        return ESP_OK;
    }

    const float v_gpio = raw_to_mv(median_raw) / 1000.0f;
    const float v_bat = v_gpio * APP_BAT_DIVIDER_RATIO;

    if (s_ema_vbat < 0.0f) {
        s_ema_vbat = v_bat;
    } else {
        s_ema_vbat = APP_BAT_EMA_ALPHA * v_bat + (1.0f - APP_BAT_EMA_ALPHA) * s_ema_vbat;
    }

    if (s_ema_vbat < APP_BAT_MIN_VALID_V || s_ema_vbat > APP_BAT_MAX_VALID_V) {
        return ESP_OK;
    }

    const float new_pct = vbat_to_percent(s_ema_vbat);
    if (s_last_pct < 0.0f || fabsf(new_pct - s_last_pct) >= APP_BAT_PCT_HYSTERESIS) {
        s_last_pct = new_pct;
    }

    out->valid = true;
    out->voltage_v = s_ema_vbat;
    out->percent = (uint8_t)lroundf(fmaxf(0.0f, fminf(100.0f, s_last_pct)));
    out->low = (s_ema_vbat < APP_BAT_LOW_V);
    out->critical = (s_ema_vbat < APP_BAT_CRITICAL_V);
    out->flags |= APP_FLAG_BAT_VALID;
    if (out->low) out->flags |= APP_FLAG_BAT_LOW;
    if (out->critical) out->flags |= APP_FLAG_BAT_CRITICAL;

    return ESP_OK;
}
