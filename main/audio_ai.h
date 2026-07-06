/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AUDIO_AI_H
#define AUDIO_AI_H

#include <stdint.h>
#include <stdbool.h>
#include "audio_ai_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 从 1 秒 PCM 音频中提取 AI 特征向量
 * @param samples      int16 PCM 样本
 * @param sample_count 样本数
 * @param features     输出特征向量，长度 AUDIO_AI_FEATURE_DIM
 */
void audio_ai_extract_features(const int16_t *samples, uint32_t sample_count, float *features);

/**
 * @brief 对特征向量执行单分类异常检测
 * @param features 特征向量，长度 AUDIO_AI_FEATURE_DIM
 * @return AUDIO_AI_GOOD 或 AUDIO_AI_ANOMALY
 */
audio_ai_result_t audio_ai_predict_with_features(const float *features);
/**
 * @brief 对已提取的特征向量执行单分类异常检测，同时返回最大 z-score
 * @param features        特征向量，长度 AUDIO_AI_FEATURE_DIM
 * @param out_max_zscore 输出：12 个特征中偏离均值最大的 z-score
 * @return AUDIO_AI_GOOD 或 AUDIO_AI_ANOMALY
 */
audio_ai_result_t audio_ai_predict_with_features_and_score(const float *features, float *out_max_zscore);

/**
 * @brief 对 1 秒 PCM 音频执行单分类异常检测
 * @param samples      int16 PCM 样本
 * @param sample_count 样本数
 * @return AUDIO_AI_GOOD 或 AUDIO_AI_ANOMALY
 */
audio_ai_result_t audio_ai_classify(const int16_t *samples, uint32_t sample_count);

/**
 * @brief 对 1 秒 PCM 音频执行单分类异常检测，同时返回最大 z-score
 * @param samples        int16 PCM 样本
 * @param sample_count   样本数
 * @param out_max_zscore 输出：12 个特征中偏离均值最大的 z-score
 * @return AUDIO_AI_GOOD 或 AUDIO_AI_ANOMALY
 */
audio_ai_result_t audio_ai_classify_with_score(const int16_t *samples, uint32_t sample_count, float *out_max_zscore);

/**
 * @brief 启用/禁用运行时均值自适应（背景学习）
 */
void audio_ai_set_adaptation(bool enable);

/**
 * @brief 获取自适应状态
 */
bool audio_ai_get_adaptation(void);

/**
 * @brief 学习一个样本，更新运行时均值
 */
void audio_ai_learn_sample(const float *features);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_AI_H
