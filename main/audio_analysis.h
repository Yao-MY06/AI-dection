/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AUDIO_ANALYSIS_H
#define AUDIO_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 音频分析结果结构体                                                         */
/* ========================================================================== */
typedef struct {
    float band_total;       // 三频段总能量
    float bg_noise;         // 环境底噪
    float health_score;     // 频谱质心 Health Score
    int   hit_pulse;        // 撞击脉冲标志 (>0 = 检测到撞击)
    float raw_score;        // 原始频谱质心值（有效时）
    bool  score_valid;      // raw_score 是否有效
} audio_analysis_result_t;

/**
 * @brief 初始化 FFT 引擎与 Hann 窗函数
 * @return 0 成功，非 0 失败
 */
int audio_analysis_init(void);

/**
 * @brief 对一帧音频数据执行 FFT 频谱分析与撞击检测
 * @param audio_buffer N_SAMPLES 长度的浮点采样数据
 * @param result      输出分析结果
 */
void audio_analysis_process(const float *audio_buffer, audio_analysis_result_t *result);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_ANALYSIS_H
