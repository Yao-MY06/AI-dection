/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AI_VISION_H
#define AI_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并加载 AI 视觉模型
 * @return 0 成功，非 0 失败
 */
int ai_vision_init(void);

/**
 * @brief AI 视觉推理任务
 */
void ai_vision_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // AI_VISION_H
