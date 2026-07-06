/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "servo_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化舵机 PWM (LEDC)
 * @note  使用 LEDC_TIMER_2 + LEDC_CHANNEL_2
 *        当前按 180° 位置舵机（SG90）配置，启动时居中到 90°
 */
void servo_init(void);

/**
 * @brief 直接设置舵机脉宽
 * @param pulse_us 脉宽 (500 ~ 2500 us)，500us = 0°，1500us = 90°，2500us = 180°
 */
void servo_set_pulse_us(uint32_t pulse_us);

/**
 * @brief 设置舵机到指定物理角度
 * @param angle 物理角度，范围 0 ~ 180；0° = 最小脉宽，90° = 中位，180° = 最大脉宽
 */
void servo_set_angle(int16_t angle);

/**
 * @brief 相对当前角度移动（全速）
 * @param delta_deg 相对角度，正 = 正转，负 = 反转
 * @note  直接跳到目标角度，无中间插值
 */
void servo_move_relative(int16_t delta_deg);

/**
 * @brief 以指定速度相对当前角度移动
 * @param delta_deg  相对角度，正 = 正转，负 = 反转
 * @param speed_pct  速度百分比 1~100，100 = 全速（约 600°/s），50 = 半速
 * @note  通过小步长 + vTaskDelay 实现平滑调速，会阻塞当前任务约 50~200ms
 */
void servo_move_relative_speed(int16_t delta_deg, uint8_t speed_pct);

/**
 * @brief 设置 360° 连续舵机转速
 * @param speed_pct 速度百分比，+100 = 正转最大速(500us)，0 = 停止(1500us)，-100 = 反转最大速(2500us)
 */
void servo_set_continuous_speed(int16_t speed_pct);

/**
 * @brief 360° 连续舵机按指定方向运行一小段时间后停止
 * @param speed_pct 速度百分比，正 = 正转，负 = 反转
 * @param duration_ms 运行时间，结束后自动输出停止脉宽
 */
void servo_jog_continuous(int16_t speed_pct, uint16_t duration_ms);

/**
 * @brief 停止舵机（保持当前角度）
 */
void servo_stop(void);

/**
 * @brief 已废弃：原 360° 连续舵机接口，保留仅用于兼容旧调用
 *        位置舵机下此函数不执行旋转动作
 */
void servo_rotate_for(int8_t speed_pct, uint16_t duration_ms) __attribute__((deprecated));

#ifdef __cplusplus
}
#endif

#endif // SERVO_H
