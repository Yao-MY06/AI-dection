/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef APP_GLOBALS_H
#define APP_GLOBALS_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"
#include "sample_roi.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 日志 TAG                                                                   */
/* ========================================================================== */
extern const char *TAG;

/* ========================================================================== */
/* FreeRTOS 队列句柄                                                          */
/* ========================================================================== */
extern QueueHandle_t ui_event_queue;
extern QueueHandle_t hw_cmd_queue;
extern QueueHandle_t audio_cap_queue;
extern QueueHandle_t audio_trigger_queue;
extern QueueHandle_t striker_trigger_queue;
extern QueueHandle_t sd_write_queue;

/* ========================================================================== */
/* 系统运行状态                                                               */
/* ========================================================================== */
extern vision_state_t g_vision_state;
extern audio_state_t  g_audio_state;
extern bool           g_emergency_active;
extern bool           g_servo_reversed;           // 舵机手动按钮是否处于反转位置

/* ========================================================================== */
/* 检测统计与 AI 结果                                                         */
/* ========================================================================== */
extern uint32_t count_pass;
extern uint32_t count_fail;
extern float    g_ai_ng_pct;            // 最新 AI NG 概率
extern float    g_ai_ok_pct;            // 最新 AI OK 概率

/* ========================================================================== */
/* 音频分析结果（供 UI 显示）                                                 */
/* ========================================================================== */
extern volatile float g_total_energy;
extern volatile float g_bg_noise;
extern volatile int   g_health_score;
extern volatile int   g_hit_pulse;

/* ========================================================================== */
/* 摄像头缓冲                                                                 */
/* ========================================================================== */
extern uint8_t  *cam_buff[NUM_BUFS];
extern uint32_t  cam_buff_size;

/* ========================================================================== */
/* PPA 双缓冲索引与同步                                                       */
/* ========================================================================== */
extern volatile uint8_t  g_ppa_write_idx;
extern volatile uint8_t  g_ppa_read_idx;
extern volatile uint8_t  g_ai_reading_cam_buf_idx;
extern volatile bool     g_has_camera_frame;
extern portMUX_TYPE      cam_buf_spinlock;
extern portMUX_TYPE      ppa_spinlock;
extern volatile bool     g_ppa_busy;
extern volatile uint32_t g_ppa_submit_cnt;
extern volatile uint32_t g_ppa_done_cnt;

/* ========================================================================== */
/* 动态样品框检测状态                                                         */
/* ========================================================================== */
extern sample_roi_t g_sample_roi;
extern volatile bool g_sample_present;
extern volatile bool g_sample_background_ready;
extern volatile bool g_sample_background_capture_requested;
extern volatile int g_sample_background_delay_frames;
extern uint8_t *g_sample_background_gray;

#ifdef __cplusplus
}
#endif

#endif // APP_GLOBALS_H
