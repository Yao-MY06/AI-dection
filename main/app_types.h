/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 视频画面顶部 AI 状态条状态                                                 */
/* ========================================================================== */
typedef enum {
    AI_OVERLAY_IDLE = 0,      // 未启动
    AI_OVERLAY_VISION,        // 视觉检测中
    AI_OVERLAY_AUDIO,         // 听觉检测中
    AI_OVERLAY_OK,            // 最终通过（多模态）
    AI_OVERLAY_NG             // 最终不通过（多模态）
} ai_overlay_state_t;

/* ========================================================================== */
/* 系统状态枚举                                                               */
/* ========================================================================== */
typedef enum {
    VISION_STATE_STOPPED = 0,                   // 视觉停止/待机状态
    VISION_STATE_RUNNING,                       // 视觉运行/检测状态
    VISION_STATE_EMERGENCY                      // 视觉急停报警状态
} vision_state_t;

typedef enum {
    AUDIO_STATE_STOPPED = 0,                    // 音频停止/待机状态
    AUDIO_STATE_RUNNING,                        // 音频运行/检测状态
    AUDIO_STATE_EMERGENCY                       // 音频急停报警状态
} audio_state_t;

/* ========================================================================== */
/* 事件类型与消息结构体                                                       */
/* ========================================================================== */
typedef enum {
    CMD_VISION_START = 0,
    CMD_VISION_STOP,
    CMD_AUDIO_START,
    CMD_SERVO_TOGGLE,                           // 舵机手动反转/正转切换按钮
    CMD_BTN_EMERGENCY,
    CMD_BTN_CAPTURE,                            // UI指令: 发起抓拍
    CMD_BTN_CAPTURE_DONE,                       // 后台事件: 抓拍并保存完成
    CMD_BTN_CAPTURE_FAIL,                       // 后台事件: 抓拍保存失败
    EVENT_AI_DEFECT_FOUND,
    EVENT_AI_PASS,                              // AI事件: 检测通过
    EVENT_AI_NO_DETECTION,                      // AI事件: 未检测到样品
    EVENT_HIT_TRIGGER,                          // striker任务: 继电器已触发敲击
    EVENT_AUDIO_START_RECORD,                   // UI手动触发: 开始1秒录音（不联动敲击）
    EVENT_AUDIO_CAP_DONE,                       // mic_fft_task: 音频截取完成
    EVENT_AUDIO_AI_GOOD,                        // 音频AI: 判断为正常
    EVENT_AUDIO_AI_ANOMALY,                     // 音频AI: 判断为异常
    EVENT_AI_DEBUG                              // AI 任务调试信息（输出到屏幕）
} event_type_t;

typedef struct {
    event_type_t type;                          // 事件类型
    char log_msg[64];                           // 附带的日志信息
} system_event_t;

typedef struct {
    uint8_t *raw_img_buf;                       // 原始图像数据指针
} capture_msg_t;

/* ========================================================================== */
/* 音频截取状态机与消息                                                       */
/* ========================================================================== */
typedef enum {
    AUDIO_CAP_IDLE = 0,
    AUDIO_CAP_ACTIVE,
} audio_cap_state_t;

typedef struct {
    int16_t *samples;                           // PSRAM 指针
    uint32_t sample_count;                      // 实际样本数
} audio_capture_t;

#ifdef __cplusplus
}
#endif

#endif // APP_TYPES_H
