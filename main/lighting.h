/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef LIGHTING_H
#define LIGHTING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化照明模组 PWM (LEDC)
 * @note  使用 LEDC_TIMER_3 + LEDC_CHANNEL_3，避免与 BSP 背光/摄像头 XCLK 冲突
 */
void lighting_init(void);

/**
 * @brief 设置照明亮度
 * @param pct 亮度百分比，范围 0 ~ 100
 */
void lighting_set_brightness(uint8_t pct);

#ifdef __cplusplus
}
#endif

#endif // LIGHTING_H
