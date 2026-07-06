/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "audio_analysis.h"
#include "app_config.h"

#include <math.h>
#include "esp_dsp.h"
#include "esp_log.h"

/* ========================================================================== */
/* FFT 缓冲与窗函数（模块私有）                                                */
/* ========================================================================== */
__attribute__((aligned(16))) static float fft_y_cf[N_SAMPLES * 2];
__attribute__((aligned(16))) static float fft_window[N_SAMPLES];

/* ========================================================================== */
/* 分析状态变量                                                               */
/* ========================================================================== */
static float env_mid = 0.0f;
static float env_high = 0.0f;
static float bg_noise = 1.0f;
static float health_score = 0.0f;

static float locked_score = 0.0f;
static int   stability_counter = 0;
static int   hit_cooldown = 0;
static bool  has_locked = false;

/* ========================================================================== */
/* 初始化                                                                     */
/* ========================================================================== */
int audio_analysis_init(void) {
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(FFT_TAG, "FFT init failed!");
        return -1;
    }
    dsps_wind_hann_f32(fft_window, N_SAMPLES);
    ESP_LOGI(FFT_TAG, "FFT engine ready");
    return 0;
}

/* ========================================================================== */
/* FFT 频谱分析与撞击检测                                                     */
/* ========================================================================== */
void audio_analysis_process(const float *audio_buffer, audio_analysis_result_t *result) {
    if (!audio_buffer || !result) return;

    // 1. 加窗
    for (int k = 0; k < N_SAMPLES; k++) {
        fft_y_cf[k * 2 + 0] = audio_buffer[k] * fft_window[k];
        fft_y_cf[k * 2 + 1] = 0;
    }

    // 2. FFT + 位反转
    dsps_fft2r_fc32(fft_y_cf, N_SAMPLES);
    dsps_bit_rev_fc32(fft_y_cf, N_SAMPLES);

    // 3. 频谱质心计算 + 分频段统计
    float weighted_sum = 0.0f;
    float total_energy = 0.0f;
    float current_low = 0.0f, current_mid = 0.0f, current_high = 0.0f;

    for (int k = 0; k < N_SAMPLES / 2; k++) {
        float re = fft_y_cf[k * 2 + 0];
        float im = fft_y_cf[k * 2 + 1];
        float magnitude = sqrtf(re * re + im * im);
        float real_amp = magnitude / 512.0f;

        if (k < 32)       current_low += real_amp;
        else if (k < 192) current_mid += real_amp;
        else              current_high += real_amp;

        weighted_sum += k * real_amp;
        total_energy += real_amp;
    }

    current_low /= 32.0f;
    current_mid /= 160.0f;
    current_high /= 320.0f;
    float band_total = current_low + current_mid + current_high;

    // 4. 包络线与环境底噪跟踪
    env_mid = (current_mid > env_mid) ? current_mid : env_mid * 0.90f;
    env_high = (current_high > env_high) ? current_high : env_high * 0.90f;

    if (band_total < bg_noise) {
        bg_noise = bg_noise * 0.80f + band_total * 0.20f;
    } else {
        bg_noise = bg_noise * 0.98f + band_total * 0.02f;
    }

    // 5. 撞击检测
    int hit_pulse = 0;
    if (band_total > bg_noise * 3.0f && band_total > 0.5f) {
        hit_pulse = 50;
    }

    // 6. 撞击瞬间抓取：频谱质心材质识别
    float raw_score = 0.0f;
    bool score_valid = false;

    if (hit_pulse > 0 && hit_cooldown == 0) {
        if (total_energy > 1e-6f) {
            raw_score = weighted_sum / total_energy;
            score_valid = true;
        }
        hit_cooldown = HIT_COOLDOWN_FRAMES;
    }

    if (hit_cooldown > 0) {
        hit_cooldown--;
    }

    // 7. 阶跃式防抖锁定
    if (score_valid) {
        if (!has_locked) {
            locked_score = raw_score;
            has_locked = true;
            stability_counter = 0;
        } else {
            float diff = fabsf(raw_score - locked_score);
            if (diff > JUMP_THRESHOLD) {
                locked_score = raw_score;
                stability_counter = 0;
            } else if (diff < STABILITY_THRESHOLD) {
                stability_counter++;
                locked_score = locked_score * 0.7f + raw_score * 0.3f;
            } else {
                stability_counter = 0;
            }
        }
        health_score = locked_score;
    }

    // 8. 填充结果
    result->band_total   = band_total;
    result->bg_noise     = bg_noise;
    result->health_score = health_score;
    result->hit_pulse    = hit_pulse;
    result->raw_score    = raw_score;
    result->score_valid  = score_valid;
}
