/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "app_types.h"
#include "sample_roi.hpp"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建完整的工业检测 UI 界面
 */
void ui_create(void);

/**
 * @brief UI 事件处理任务，从 ui_event_queue 接收事件并更新界面
 */
void ui_event_task(void *pvParameters);

/**
 * @brief LVGL 按钮事件回调
 */
void btn_event_cb(lv_event_t *e);

/**
 * @brief 更新视觉检测状态显示
 */
void update_vision_status(vision_state_t new_state);

/**
 * @brief 更新音频检测状态显示
 */
void update_audio_status(audio_state_t new_state);

/**
 * @brief 更新系统整体状态显示
 */
void update_system_status(bool emergency);

/**
 * @brief 更新紧急停止状态显示
 */
void update_emergency_status(bool active);

/**
 * @brief 更新视频画面顶部 AI 状态条
 */
void update_ai_status_bar(ai_overlay_state_t state);

/**
 * @brief 根据是否正在检测更新 AI 状态条
 */
void update_ai_status_bar_bool(bool is_detecting);

/**
 * @brief 获取主视频 Canvas 句柄，供 video_pipeline 刷新显示
 */
lv_obj_t *ui_get_camera_canvas(void);

/**
 * @brief 更新右下角 AI 检测结果卡片（NG/OK 概率条）
 */
void update_ai_result_card(float ng_pct, float ok_pct);

/**
 * @brief 更新良率与顶部统计卡片
 */
void update_yield_stats(void);

/**
 * @brief 更新声纹分析区波形与状态文字
 */
void ui_update_acoustic_result(int health_score, int hit_pulse, float band_total, float bg_noise);

/**
 * @brief 声学 AI 敲击结果概率显示 — 每次敲击录音完成后由音频 AI 事件触发
 * @param is_good      true=判定GOOD，false=判定BAD
 * @param zscore_str   最大 z-score 的字符串形式（从事件 log_msg 解析）
 */
void ui_update_audio_ai_score(bool is_good, const char *zscore_str);

/**
 * @brief 更新样品检测框位置与颜色
 * @param roi        样品 ROI，NULL 表示隐藏
 * @param show_result 是否已出最终判定结果
 * @param final_good true=GOOD（绿色），false=BAD（红色），仅在 show_result=true 时有效
 */
void ui_update_sample_box(const sample_roi_t *roi, bool show_result, bool final_good);

/**
 * @brief 重置样品框为隐藏/检查中状态
 */
void ui_reset_sample_box(void);

#ifdef __cplusplus
}
#endif

#endif // UI_MANAGER_H
