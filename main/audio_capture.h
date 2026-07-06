/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 I2S 麦克风
 */
void i2s_mic_init(void);

/**
 * @brief 麦克风采集与 FFT 分析任务
 *        读取 I2S → 环形缓冲 → FFT 分析 → 撞击检测 → 音频截取
 */
void audio_capture_task(void *pvParameters);

/**
 * @brief 音频片段处理任务（保存 PCM 到 SD 卡，预留声学 AI 接口）
 */
void audio_process_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_CAPTURE_H
