/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "audio_ai.h"
#include "audio_ai_model.h"
#include "app_config.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_dsp.h"
#include "esp_log.h"

static const char *AUDIO_AI_TAG = "AUDIO_AI";

static float compute_rms(const int16_t *samples, uint32_t count)
{
    if (count == 0) return 0.0f;
    double sum_sq = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        float v = (float)samples[i];
        sum_sq += (double)(v * v);
    }
    return (float)sqrt(sum_sq / (double)count);
}

static uint32_t find_peak_index(const int16_t *samples, uint32_t count)
{
    uint32_t peak_idx = 0;
    int32_t peak_abs = 0;
    for (uint32_t i = 0; i < count; i++) {
        int32_t a = abs((int32_t)samples[i]);
        if (a > peak_abs) {
            peak_abs = a;
            peak_idx = i;
        }
    }
    return peak_idx;
}

static float compute_window_rms(const int16_t *samples, uint32_t count,
                                uint32_t center_idx, float window_ms,
                                uint32_t sample_rate)
{
    if (count == 0 || sample_rate == 0) return 0.0f;
    uint32_t half_samples = (uint32_t)((window_ms / 1000.0f) * (float)sample_rate / 2.0f);
    uint32_t start = (center_idx > half_samples) ? (center_idx - half_samples) : 0;
    uint32_t end = center_idx + half_samples;
    if (end > count) end = count;
    if (start >= end) return 0.0f;
    return compute_rms(samples + start, end - start);
}

static float compute_energy_decay(const int16_t *samples, uint32_t count)
{
    if (count < 2) return 1.0f;
    uint32_t mid = count / 2;
    float rms_first = compute_rms(samples, mid);
    float rms_second = compute_rms(samples + mid, count - mid);
    if (rms_first <= 1e-9f) return 1.0f;
    return rms_second / rms_first;
}

static float compute_zcr(const int16_t *samples, uint32_t count)
{
    if (count < 2) return 0.0f;
    uint32_t crossings = 0;
    for (uint32_t i = 1; i < count; i++) {
        if (((int32_t)samples[i - 1] < 0 && (int32_t)samples[i] >= 0) ||
            ((int32_t)samples[i - 1] >= 0 && (int32_t)samples[i] < 0)) {
            crossings++;
        }
    }
    return (float)crossings / (float)(count - 1);
}

static float compute_abs_mean(const int16_t *samples, uint32_t count)
{
    if (count == 0) return 0.0f;
    double sum = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        sum += (double)abs((int32_t)samples[i]);
    }
    return (float)(sum / (double)count);
}

static float compute_peak_dbfs(const int16_t *samples, uint32_t count)
{
    if (count == 0) return -120.0f;
    int32_t peak = 0;
    for (uint32_t i = 0; i < count; i++) {
        int32_t a = abs((int32_t)samples[i]);
        if (a > peak) peak = a;
    }
    if (peak <= 0) return -120.0f;
    return 20.0f * log10f((float)peak / 32768.0f);
}

/* ========================================================================== */
/* 频域特征提取（1024点 FFT，与 Python 端 audio_batch_process.py 保持一致）    */
/* ========================================================================== */
#define AUDIO_AI_FFT_SIZE 1024

static bool s_fft_initialized = false;
__attribute__((aligned(16))) static float s_fft_buf[AUDIO_AI_FFT_SIZE * 2];
__attribute__((aligned(16))) static float s_fft_window[AUDIO_AI_FFT_SIZE];

static int audio_ai_fft_init(void)
{
    if (s_fft_initialized) {
        return 0;
    }
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(AUDIO_AI_TAG, "FFT init failed");
        return -1;
    }
    dsps_wind_hann_f32(s_fft_window, AUDIO_AI_FFT_SIZE);
    s_fft_initialized = true;
    return 0;
}

static void extract_fft_features(const int16_t *samples, uint32_t count,
                                 float *out_centroid, float *out_rolloff,
                                 float *out_band_low, float *out_band_mid, float *out_band_high)
{
    *out_centroid = 0.0f;
    *out_rolloff = 0.0f;
    *out_band_low = 0.0f;
    *out_band_mid = 0.0f;
    *out_band_high = 0.0f;

    if (count == 0 || audio_ai_fft_init() != 0) {
        return;
    }

    // 找到峰值位置，取峰值附近 1024 个样本
    uint32_t peak_idx = find_peak_index(samples, count);
    uint32_t half = AUDIO_AI_FFT_SIZE / 2;
    uint32_t start = (peak_idx > half) ? (peak_idx - half) : 0;
    uint32_t end = start + AUDIO_AI_FFT_SIZE;
    if (end > count) {
        end = count;
        start = (end > AUDIO_AI_FFT_SIZE) ? (end - AUDIO_AI_FFT_SIZE) : 0;
    }
    uint32_t window_count = end - start;
    if (window_count == 0) return;

    // 去直流 + 加窗
    float mean = 0.0f;
    for (uint32_t i = start; i < end; i++) {
        mean += (float)samples[i];
    }
    mean /= (float)window_count;

    for (uint32_t i = 0; i < AUDIO_AI_FFT_SIZE; i++) {
        if (i < window_count) {
            s_fft_buf[i * 2 + 0] = ((float)samples[start + i] - mean) * s_fft_window[i];
        } else {
            s_fft_buf[i * 2 + 0] = 0.0f;
        }
        s_fft_buf[i * 2 + 1] = 0.0f;
    }

    dsps_fft2r_fc32(s_fft_buf, AUDIO_AI_FFT_SIZE);
    dsps_bit_rev_fc32(s_fft_buf, AUDIO_AI_FFT_SIZE);

    // 计算频谱特征
    float weighted_sum = 0.0f;
    float total = 0.0f;
    float low = 0.0f, mid = 0.0f, high = 0.0f;
    int bins = AUDIO_AI_FFT_SIZE / 2;
    float bin_hz = (float)AUDIO_SAMPLE_RATE / (float)AUDIO_AI_FFT_SIZE;

    // 忽略 DC 分量
    for (int k = 1; k < bins; k++) {
        float re = s_fft_buf[k * 2 + 0];
        float im = s_fft_buf[k * 2 + 1];
        float mag = sqrtf(re * re + im * im);
        float freq = k * bin_hz;

        weighted_sum += freq * mag;
        total += mag;

        if (freq < 200.0f) {
            low += mag;
        } else if (freq < 1000.0f) {
            mid += mag;
        } else {
            high += mag;
        }
    }

    if (total > 1e-9f) {
        *out_centroid = weighted_sum / total;

        // rolloff: 85% 能量累积点
        float cumsum = 0.0f;
        for (int k = 1; k < bins; k++) {
            float re = s_fft_buf[k * 2 + 0];
            float im = s_fft_buf[k * 2 + 1];
            float mag = sqrtf(re * re + im * im);
            cumsum += mag;
            if (cumsum >= 0.85f * total) {
                *out_rolloff = k * bin_hz;
                break;
            }
        }

        *out_band_low = low / total;
        *out_band_mid = mid / total;
        *out_band_high = high / total;
    }
}

/* ========================================================================== */
/* Runtime adaptation: learn only confirmed normal knock samples.               */
/* ========================================================================== */
static float s_adapt_mean[AUDIO_AI_FEATURE_DIM];
static bool s_adapt_enabled = false;
static float s_adapt_alpha = 0.05f;

#define AUDIO_AI_MIN_LEARN_RMS       100.0f
#define AUDIO_AI_MIN_LEARN_PEAK_DBFS -40.0f

void audio_ai_set_adaptation(bool enable)
{
    s_adapt_enabled = enable;
    if (enable) {
        memcpy(s_adapt_mean, audio_ai_mean, sizeof(s_adapt_mean));
        ESP_LOGI(AUDIO_AI_TAG, "audio AI normal-sample adaptation enabled");
    } else {
        ESP_LOGI(AUDIO_AI_TAG, "audio AI adaptation disabled");
    }
}

bool audio_ai_get_adaptation(void)
{
    return s_adapt_enabled;
}

void audio_ai_learn_sample(const float *features)
{
    if (!features) return;

    if (features[0] < AUDIO_AI_MIN_LEARN_RMS || features[6] < AUDIO_AI_MIN_LEARN_PEAK_DBFS) {
        ESP_LOGW(AUDIO_AI_TAG, "skip adaptation sample: too quiet (rms=%.1f peak_dbfs=%.1f)",
                 features[0], features[6]);
        return;
    }

    if (!s_adapt_enabled) {
        memcpy(s_adapt_mean, audio_ai_mean, sizeof(s_adapt_mean));
        s_adapt_enabled = true;
    }
    for (int i = 0; i < AUDIO_AI_FEATURE_DIM; i++) {
        s_adapt_mean[i] = s_adapt_mean[i] * (1.0f - s_adapt_alpha) + features[i] * s_adapt_alpha;
    }
}

/* ========================================================================== */
/* 特征提取与分类                                                               */
/* ========================================================================== */
void audio_ai_extract_features(const int16_t *samples, uint32_t sample_count, float *features)
{
    if (!features) {
        return;
    }
    if (!samples || sample_count == 0) {
        for (int i = 0; i < AUDIO_AI_FEATURE_DIM; i++) {
            features[i] = 0.0f;
        }
        return;
    }

    uint32_t peak_idx = find_peak_index(samples, sample_count);

    // 时域特征
    features[0] = compute_rms(samples, sample_count);
    features[1] = compute_window_rms(samples, sample_count, peak_idx, 20.0f, AUDIO_SAMPLE_RATE);
    features[2] = compute_window_rms(samples, sample_count, peak_idx, 50.0f, AUDIO_SAMPLE_RATE);
    features[3] = compute_zcr(samples, sample_count);
    features[4] = compute_abs_mean(samples, sample_count);
    features[5] = compute_energy_decay(samples, sample_count);
    features[6] = compute_peak_dbfs(samples, sample_count);

    // 频域特征
    extract_fft_features(samples, sample_count,
                         &features[7],   // spectral_centroid_hz
                         &features[8],   // spectral_rolloff_hz
                         &features[9],   // band_ratio_low
                         &features[10],  // band_ratio_mid
                         &features[11]); // band_ratio_high
}

audio_ai_result_t audio_ai_predict_with_features(const float *features)
{
    return audio_ai_predict_with_features_and_score(features, NULL);
}

audio_ai_result_t audio_ai_predict_with_features_and_score(const float *features, float *out_max_zscore)
{
    if (!features) {
        if (out_max_zscore) *out_max_zscore = 0.0f;
        return AUDIO_AI_ANOMALY;
    }

    const float *mean = s_adapt_enabled ? s_adapt_mean : audio_ai_mean;

    float max_zscore = 0.0f;
    for (int i = 0; i < AUDIO_AI_FEATURE_DIM; i++) {
        float diff = features[i] - mean[i];
        float zscore = fabsf(diff) * audio_ai_inv_std[i];
        if (zscore > max_zscore) {
            max_zscore = zscore;
        }
    }
    if (out_max_zscore) *out_max_zscore = max_zscore;
    return (max_zscore <= AUDIO_AI_ZSCORE_THRESHOLD) ? AUDIO_AI_GOOD : AUDIO_AI_ANOMALY;
}
audio_ai_result_t audio_ai_classify(const int16_t *samples, uint32_t sample_count)
{
    float features[AUDIO_AI_FEATURE_DIM];
    audio_ai_extract_features(samples, sample_count, features);
    return audio_ai_predict_with_features(features);
}

audio_ai_result_t audio_ai_classify_with_score(const int16_t *samples, uint32_t sample_count, float *out_max_zscore)
{
    float features[AUDIO_AI_FEATURE_DIM];
    audio_ai_extract_features(samples, sample_count, features);

    if (!samples || sample_count == 0) {
        if (out_max_zscore) *out_max_zscore = 0.0f;
        return AUDIO_AI_ANOMALY;
    }

    return audio_ai_predict_with_features_and_score(features, out_max_zscore);
}