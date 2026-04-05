/**
 * @file app_signal.c
 * @brief Procesamiento ligero de IMU y empaquetado del payload del Nodo 1.
 *
 * Convierte muestras crudas a rasgos compactos por ventana y construye la
 * trama binaria que se transmite hacia el Nodo 2.
 */

#include <string.h>
#include <math.h>
#include "app_signal.h"

/**
 * @brief Satura un valor flotante al rango de int16_t.
 * @param[in] x Valor a convertir.
 * @return Valor saturado y redondeado en formato entero de 16 bits.
 */
static int16_t q16(float x)
{
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}

/**
 * @brief Reinicia el acumulador estadistico de una ventana IMU.
 * @param[in,out] ctx Contexto de procesamiento a limpiar.
 */
void app_signal_reset(app_signal_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

/**
 * @brief Incorpora una muestra IMU al contexto estadistico actual.
 * @param[in,out] ctx Acumuladores de la ventana.
 * @param[in] s Muestra cruda ya corregida en el dominio del Nodo 1.
 */
void app_signal_push_imu(app_signal_ctx_t *ctx, const imu_sample_t *s)
{
    float ax = s->ax * APP_ACC_MG_PER_LSB;
    float ay = s->ay * APP_ACC_MG_PER_LSB;
    float az = s->az * APP_ACC_MG_PER_LSB;

    float gx = s->gx * APP_GYRO_DPS10_PER_LSB;
    float gy = s->gy * APP_GYRO_DPS10_PER_LSB;
    float gz = s->gz * APP_GYRO_DPS10_PER_LSB;

    float amag = sqrtf(ax * ax + ay * ay + az * az);
    float gmag = sqrtf(gx * gx + gy * gy + gz * gz);

    ctx->sum_acc[0] += ax; ctx->sum_acc[1] += ay; ctx->sum_acc[2] += az;
    ctx->sum2_acc[0] += ax * ax; ctx->sum2_acc[1] += ay * ay; ctx->sum2_acc[2] += az * az;

    ctx->sum_gyro[0] += gx; ctx->sum_gyro[1] += gy; ctx->sum_gyro[2] += gz;
    ctx->sum2_gyro[0] += gx * gx; ctx->sum2_gyro[1] += gy * gy; ctx->sum2_gyro[2] += gz * gz;

    ctx->sum_acc_mag += amag;
    ctx->sum2_acc_mag += amag * amag;
    ctx->sum_gyro_mag += gmag;
    ctx->sum2_gyro_mag += gmag * gmag;

    ctx->sum_sma += fabsf(ax) + fabsf(ay) + fabsf(az);

    if (ctx->count < (APP_IMU_EXPECTED_SAMPLES + 4)) {
        ctx->acc_mag_hist[ctx->count] = amag;
    }
    ctx->count++;
}

/** @brief Calcula una media simple protegida contra division por cero. */
static float meanf(float sum, float n)
{
    return (n > 0) ? (sum / n) : 0.0f;
}

/**
 * @brief Estima la desviacion estandar a partir de suma y suma cuadratica.
 */
static float stdf(float sum, float sum2, float n)
{
    if (n <= 1) return 0.0f;
    float mu = sum / n;
    float v = (sum2 / n) - mu * mu;
    return (v > 0) ? sqrtf(v) : 0.0f;
}

/**
 * @brief Resume una ventana de IMU en un conjunto compacto de rasgos temporales.
 * @param[in] ctx Contexto con estadisticos acumulados de la ventana.
 * @param[out] out Estructura de salida con rasgos y banderas de validez.
 */
void app_signal_build_features(const app_signal_ctx_t *ctx, imu_features_t *out)
{
    memset(out, 0, sizeof(*out));
    out->sample_count = ctx->count;

    uint16_t min_valid = (APP_IMU_EXPECTED_SAMPLES * APP_IMU_MIN_VALID_PCT) / 100U;
    out->valid = (ctx->count >= min_valid);

    float n = (float)ctx->count;
    if (n <= 0) return;

    for (int i = 0; i < 3; ++i) {
        float am = meanf(ctx->sum_acc[i], n);
        float as = stdf(ctx->sum_acc[i], ctx->sum2_acc[i], n);
        float ar = sqrtf(ctx->sum2_acc[i] / n);

        float gm = meanf(ctx->sum_gyro[i], n);
        float gs = stdf(ctx->sum_gyro[i], ctx->sum2_gyro[i], n);
        float gr = sqrtf(ctx->sum2_gyro[i] / n);

        out->acc_mean[i] = q16(am);
        out->acc_std[i]  = q16(as);
        out->acc_rms[i]  = q16(ar);

        out->gyro_mean[i] = q16(gm);
        out->gyro_std[i]  = q16(gs);
        out->gyro_rms[i]  = q16(gr);
    }

    out->acc_mag_mean  = q16(meanf(ctx->sum_acc_mag, n));
    out->acc_mag_std   = q16(stdf(ctx->sum_acc_mag, ctx->sum2_acc_mag, n));
    out->acc_sma       = q16(meanf(ctx->sum_sma, n));

    out->gyro_mag_mean = q16(meanf(ctx->sum_gyro_mag, n));
    out->gyro_mag_std  = q16(stdf(ctx->sum_gyro_mag, ctx->sum2_gyro_mag, n));

    uint8_t peaks = 0;
    float thr = meanf(ctx->sum_acc_mag, n) + stdf(ctx->sum_acc_mag, ctx->sum2_acc_mag, n);
    for (uint16_t i = 1; i + 1 < ctx->count && i + 1 < (APP_IMU_EXPECTED_SAMPLES + 4); ++i) {
        float a = ctx->acc_mag_hist[i - 1];
        float b = ctx->acc_mag_hist[i];
        float c = ctx->acc_mag_hist[i + 1];
        if (b > thr && b > a && b >= c) peaks++;
    }
    out->peak_count = peaks;
}

/**
 * @brief Construye el payload binario que se enviara por radio.
 * @param[out] p Payload final serializado.
 * @param[in] type Tipo de mensaje a emitir.
 * @param[in] boot_id Identificador del arranque actual.
 * @param[in] seq Numero de secuencia local.
 * @param[in] t_rel_ms Tiempo relativo del nodo al momento de construir el paquete.
 * @param[in] flags Banderas de calidad y estado.
 * @param[in] imu Rasgos IMU locales.
 * @param[in] ppg Metricas derivadas del PPG.
 * @param[in] bat Estado resumido de bateria.
 */
void app_signal_build_payload(adl_payload_t *p,
                              app_msg_type_t type,
                              uint16_t boot_id,
                              uint16_t seq,
                              uint32_t t_rel_ms,
                              uint16_t flags,
                              const imu_features_t *imu,
                              const ppg_metrics_t *ppg,
                              const battery_status_t *bat)
{
    memset(p, 0, sizeof(*p));
    p->msg_type = (uint8_t)type;
    p->boot_id = boot_id;
    p->seq = seq;
    p->t_rel_ms = t_rel_ms;
    p->window_ms = APP_IMU_WINDOW_MS;
    p->sample_count = imu->sample_count;
    p->flags = flags;

    memcpy(p->acc_mean, imu->acc_mean, sizeof(p->acc_mean));
    memcpy(p->acc_std, imu->acc_std, sizeof(p->acc_std));
    memcpy(p->acc_rms, imu->acc_rms, sizeof(p->acc_rms));
    p->acc_mag_mean = imu->acc_mag_mean;
    p->acc_mag_std  = imu->acc_mag_std;
    p->acc_sma      = imu->acc_sma;

    memcpy(p->gyro_mean, imu->gyro_mean, sizeof(p->gyro_mean));
    memcpy(p->gyro_std, imu->gyro_std, sizeof(p->gyro_std));
    memcpy(p->gyro_rms, imu->gyro_rms, sizeof(p->gyro_rms));
    p->gyro_mag_mean = imu->gyro_mag_mean;
    p->gyro_mag_std  = imu->gyro_mag_std;

    p->peak_count = imu->peak_count;
    p->bpm = ppg->bpm_valid ? ppg->bpm : 0;
    p->spo2 = ppg->spo2_valid ? ppg->spo2 : 0;
    p->battery_pct = bat->valid ? bat->pct : 0;
    p->battery_mv  = bat->valid ? bat->mv : 0;
}