/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ========================================================================== */
/* 屏幕与 UI 布局常量                                                         */
/* ========================================================================== */
#define SCREEN_W            1024                    // 屏幕总宽度
#define SCREEN_H            600                     // 屏幕总高度
#define TOP_H               60                      // 顶部全局状态栏高度
#define PANEL_W             300                     // 右侧操作面板宽度
#define BOTTOM_H            140                     // 底部面板总高度

// 动态计算各区域尺寸
#define CAM_W               (SCREEN_W - PANEL_W)            // 主视频区宽度 (724)
#define CAM_H               (SCREEN_H - TOP_H - BOTTOM_H)   // 主视频区高度 (400)
#define ACOUSTIC_W          CAM_W                           // 声纹分析区宽度
#define ACOUSTIC_H          BOTTOM_H                        // 声纹分析区高度
#define ALARM_W             PANEL_W                         // 报警结果区宽度
#define ALARM_H             BOTTOM_H                        // 报警结果区高度

/* ========================================================================== */
/* 摄像头与内存常量                                                           */
/* ========================================================================== */
#define NUM_BUFS            2                       // 摄像头缓冲帧数
#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

/* ========================================================================== */
/* AI 模型输入尺寸                                                            */
/* ========================================================================== */
#define AI_INPUT_W          320
#define AI_INPUT_H          320

#define AI_DET_CONF_THRESHOLD  0.25f   // 纯分类模式阈值：低于此值视为无样品，正常样品通常 0.05~0.2

#define AI_DET_SCORE_GAIN      1.0f    // 分数增益：正常量化模型用 1.0；如果量化把 score 压低了可临时调高
#define AI_NORMAL_BIAS         1.0f    // normal 偏向系数：1.0=原始输出，>1 偏向好品
#define AI_MONITOR_LOG_EVERY_N 1        // 每 N 次视觉推理输出一次 [AI_MON] 结构化诊断日志

/* ========================================================================== */
/* 视觉检测模式                                                               */
/* ========================================================================== */
#define APP_ENABLE_SAMPLE_ROI 0    // 0: 使用模型检测框；1: 启用旧背景差分 ROI

/* ========================================================================== */
/* 麦克风 I2S 与 FFT 配置                                                     */
/* ========================================================================== */
#define I2S_SCK_IO          ((gpio_num_t)21)
#define I2S_WS_IO           ((gpio_num_t)22)
#define I2S_SD_IO           ((gpio_num_t)20)
#define RELAY_PIN           ((gpio_num_t)23)
#define REJECTER_PIN        ((gpio_num_t)6)     // 剔除机构继电器
#define LIGHT_PIN           ((gpio_num_t)4)     // 照明模组 PWM
#define SERVO_PIN           ((gpio_num_t)2)     // 舵机 PWM

#define SERVO_MANUAL_SPEED_PCT     25          // 360°连续舵机手动正/反转速度
#define SERVO_MANUAL_JOG_MS        300         // 手动按钮每次运行时间，随后自动停止

#define N_SAMPLES           1024
#define FFT_TAG             "MIC_FFT"
#define AUDIO_FFT_MONITOR_LOGS 0       // 0: 关闭高频 MIC_FFT 串口日志；1: 输出能量/耗时诊断

/* ========================================================================== */
/* 音频截取配置                                                               */
/* ========================================================================== */
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_CAPTURE_GAIN  0.7f         // PCM 采集增益：回滚到旧版 0.7f，对比 0.003f 诊断版
#define AUDIO_RING_MS       100                         // 预触发环形缓冲: 100ms
#define AUDIO_CAPTURE_MS    900                         // 撞击后截取: 900ms（加上 100ms 预触发共 1s）
#define RING_BUF_SIZE       ((AUDIO_SAMPLE_RATE * AUDIO_RING_MS) / 1000)      // 1600 样本
#define CAPTURE_SAMPLES     ((AUDIO_SAMPLE_RATE * AUDIO_CAPTURE_MS) / 1000)   // 12800 样本
#define TOTAL_CAP_SAMPLES   (RING_BUF_SIZE + CAPTURE_SAMPLES)                 // 14400 样本 (~28.8 KB)

#define AUDIO_MANUAL_RECORD_MS  1000                                            // 手动 AUDIO START 录音时长
#define AUDIO_MANUAL_CAP_SAMPLES ((AUDIO_SAMPLE_RATE * AUDIO_MANUAL_RECORD_MS) / 1000)  // 16000 样本

/* ========================================================================== */
/* 声学检测算法参数                                                           */
/* ========================================================================== */
#define HIT_COOLDOWN_FRAMES     8       // 约 64ms 冷却窗口
#define STABILITY_THRESHOLD     25.0f   // 判定为同一材质的差值阈值
#define STABILITY_COUNT         3       // 连续 N 次接近才完全锁定
#define JUMP_THRESHOLD          80.0f   // 突变阈值，立刻切换材质

/* ========================================================================== */
/* FreeRTOS 队列容量                                                           */
/* ========================================================================== */
#define UI_EVENT_QUEUE_SIZE         15
#define SD_WRITE_QUEUE_SIZE         50      // 增大缓冲，避免连拍时丢图
#define HW_CMD_QUEUE_SIZE           10      // NG 事件关键队列，增大防溢出
#define AUDIO_CAP_QUEUE_SIZE        5
#define AUDIO_TRIGGER_QUEUE_SIZE    5
#define STRIKER_TRIGGER_QUEUE_SIZE  5

/* ========================================================================== */
/* 其他常量                                                                   */
/* ========================================================================== */
#define CAM_BUF_IDX_NONE    0xFF
#define MOUNT_POINT         "/sdcard"

#endif // APP_CONFIG_H
