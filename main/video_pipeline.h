/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef VIDEO_PIPELINE_H
#define VIDEO_PIPELINE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 PPA 硬件图像缩放模块
 */
void video_pipeline_init(void);

/**
 * @brief 显示刷新任务：将最新的 camera buffer 刷新到 LVGL canvas
 */
void display_refresh_task(void *pvParameters);

/**
 * @brief 摄像头帧处理回调：接收原始帧，通过 PPA 缩放到显示缓冲区
 */
void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index,
                                  uint32_t camera_buf_hes, uint32_t camera_buf_ves,
                                  size_t camera_buf_len);

#ifdef __cplusplus
}
#endif

#endif // VIDEO_PIPELINE_H
