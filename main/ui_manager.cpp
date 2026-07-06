/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "ui_manager.h"
#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"
#include "lighting.h"
#include "servo.h"
#include "ui_commands.h"
#include <stdlib.h>

#include <stdio.h>
#include <string.h>
#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

/* ========================================================================== */
/* LVGL 对象句柄（模块私有）                                                  */
/* ========================================================================== */
static lv_obj_t *camera_canvas = NULL;          // 主视频 Canvas
static lv_obj_t *part_detect_box = NULL;        // 样品检测框
static lv_obj_t *part_detect_label = NULL;      // 样品检测框内文字
static lv_obj_t *log_box = NULL;                // 日志输出框（保留在右下面板内）

// 顶部全局状态栏句柄
static lv_obj_t *sys_status_lab = NULL;         // 系统状态
static lv_obj_t *vision_status_lab = NULL;      // 视觉状态
static lv_obj_t *audio_status_lab = NULL;       // 音频状态
static lv_obj_t *emergency_status_lab = NULL;   // 紧急状态

// 视频画面顶部中央半透明状态条句柄
static lv_obj_t *ai_overlay_bar = NULL;
static lv_obj_t *ai_overlay_lab = NULL;

// 右侧操作面板按钮句柄（用于互斥禁用）
static lv_obj_t *b_vision_start = NULL;
static lv_obj_t *b_vision_stop = NULL;
static lv_obj_t *b_audio_start = NULL;
static lv_obj_t *b_audio_toggle = NULL;
static lv_obj_t *b_emergency = NULL;
static lv_obj_t *b_capture = NULL;

// 底部声纹分析区句柄
static lv_obj_t *acoustic_chart = NULL;
static lv_chart_series_t *acoustic_chart_ser = NULL;
static lv_obj_t *acoustic_result_lab = NULL;    // 大状态文字
static lv_obj_t *acoustic_result_sub = NULL;    // 小状态说明

// 右下角报警与 AI 结果卡片句柄
static lv_obj_t *alarm_card = NULL;
static lv_obj_t *alarm_label = NULL;
static lv_obj_t *ai_result_card = NULL;
static lv_obj_t *ai_ng_bar = NULL;              // NG 进度条背景
static lv_obj_t *ai_ng_fill = NULL;             // NG 进度条填充
static lv_obj_t *ai_ok_bar = NULL;              // OK 进度条背景
static lv_obj_t *ai_ok_fill = NULL;             // OK 进度条填充
static lv_obj_t *ai_ng_label = NULL;
static lv_obj_t *ai_ok_label = NULL;
static lv_obj_t *screenshot_log_label = NULL;  // 右下角截图状态日志
static uint32_t s_capture_saved_cnt = 0;        // 截图成功保存计数
static uint32_t s_capture_dropped_cnt = 0;      // 截图丢失计数

// 顶部统计卡片句柄
static lv_obj_t *p_lab = NULL;
static lv_obj_t *f_lab = NULL;
static lv_obj_t *y_lab = NULL;

/* ========================================================================== */
/* 动画与基础回调                                                             */
/* ========================================================================== */
static void anim_opa_cb(void *var, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t *)var, v, 0);
}

/**
 * @brief 启动闪烁动画（先清除已有动画，再创建新的呼吸闪烁效果）
 */
static void start_blink_anim(lv_obj_t *obj, uint32_t duration, lv_opa_t min_opa) {
    if (!obj) return;
    lv_anim_delete(obj, (lv_anim_exec_xcb_t)anim_opa_cb);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_values(&a, min_opa, LV_OPA_COVER);
    lv_anim_set_duration(&a, duration);
    lv_anim_set_playback_duration(&a, duration);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_opa_cb);
    lv_anim_start(&a);
}

/* ========================================================================== */
/* 视觉检测状态更新（顶部状态栏：图标+文字）                                     */
/* ========================================================================== */
void update_vision_status(vision_state_t new_state) {
    g_vision_state = new_state;
    if (!vision_status_lab) return;

    switch (new_state) {
        case VISION_STATE_RUNNING:
            lv_label_set_text(vision_status_lab, LV_SYMBOL_VIDEO " VISION: RUNNING");
            lv_obj_set_style_text_color(vision_status_lab, lv_palette_main(LV_PALETTE_GREEN), 0);
            start_blink_anim(vision_status_lab, 800, LV_OPA_50);
            break;

        case VISION_STATE_STOPPED:
            lv_label_set_text(vision_status_lab, LV_SYMBOL_VIDEO " VISION: IDLE");
            lv_obj_set_style_text_color(vision_status_lab, lv_palette_main(LV_PALETTE_GREY), 0);
            lv_obj_set_style_opa(vision_status_lab, LV_OPA_COVER, 0);
            break;

        case VISION_STATE_EMERGENCY:
            lv_label_set_text(vision_status_lab, LV_SYMBOL_VIDEO " VISION: HALTED!");
            lv_obj_set_style_text_color(vision_status_lab, lv_palette_main(LV_PALETTE_RED), 0);
            lv_obj_set_style_opa(vision_status_lab, LV_OPA_COVER, 0);
            break;
    }
}

/* ========================================================================== */
/* 音频检测状态更新（顶部状态栏：图标+文字）                                     */
/* ========================================================================== */
void update_audio_status(audio_state_t new_state) {
    g_audio_state = new_state;
    if (!audio_status_lab) return;

    switch (new_state) {
        case AUDIO_STATE_RUNNING:
            lv_label_set_text(audio_status_lab, LV_SYMBOL_AUDIO " AUDIO: LISTENING");
            lv_obj_set_style_text_color(audio_status_lab, lv_color_hex(0x00D2FF), 0);
            start_blink_anim(audio_status_lab, 800, LV_OPA_50);
            break;

        case AUDIO_STATE_STOPPED:
            lv_label_set_text(audio_status_lab, LV_SYMBOL_AUDIO " AUDIO: STOPPED");
            lv_obj_set_style_text_color(audio_status_lab, lv_palette_main(LV_PALETTE_GREY), 0);
            lv_obj_set_style_opa(audio_status_lab, LV_OPA_COVER, 0);
            break;

        case AUDIO_STATE_EMERGENCY:
            lv_label_set_text(audio_status_lab, LV_SYMBOL_AUDIO " AUDIO: HALTED!");
            lv_obj_set_style_text_color(audio_status_lab, lv_palette_main(LV_PALETTE_RED), 0);
            lv_obj_set_style_opa(audio_status_lab, LV_OPA_COVER, 0);
            break;
    }
}

/* ========================================================================== */
/* 视频画面顶部中央 AI 状态条更新                                               */
/* ========================================================================== */
void update_ai_status_bar(ai_overlay_state_t state) {
    if (!ai_overlay_bar || !ai_overlay_lab) return;

    switch (state) {
        case AI_OVERLAY_DETECTING:
            lv_label_set_text(ai_overlay_lab, LV_SYMBOL_REFRESH " AI: DETECTING");
            lv_obj_set_style_text_color(ai_overlay_lab, lv_color_hex(0x00D2FF), 0);
            lv_obj_set_style_bg_color(ai_overlay_bar, lv_color_hex(0x000000), 0);
            break;
        case AI_OVERLAY_NG:
            lv_label_set_text(ai_overlay_lab, LV_SYMBOL_CLOSE " AI: NG DETECTED");
            lv_obj_set_style_text_color(ai_overlay_lab, lv_color_hex(0xFF3333), 0);
            lv_obj_set_style_bg_color(ai_overlay_bar, lv_color_hex(0x330000), 0);
            break;
        case AI_OVERLAY_OK:
            lv_label_set_text(ai_overlay_lab, LV_SYMBOL_OK " AI: OK");
            lv_obj_set_style_text_color(ai_overlay_lab, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_obj_set_style_bg_color(ai_overlay_bar, lv_color_hex(0x003300), 0);
            break;
        case AI_OVERLAY_IDLE:
        default:
            lv_label_set_text(ai_overlay_lab, LV_SYMBOL_MINUS " AI: IDLE");
            lv_obj_set_style_text_color(ai_overlay_lab, lv_color_hex(0x888888), 0);
            lv_obj_set_style_bg_color(ai_overlay_bar, lv_color_hex(0x000000), 0);
            break;
    }
}

void update_ai_status_bar_bool(bool is_detecting) {
    update_ai_status_bar(is_detecting ? AI_OVERLAY_DETECTING : AI_OVERLAY_IDLE);
}

/* ========================================================================== */
/* 主视频 Canvas 句柄访问                                                     */
/* ========================================================================== */
lv_obj_t *ui_get_camera_canvas(void) {
    return camera_canvas;
}

/* ========================================================================== */
/* 系统整体状态更新（顶部状态栏系统状态卡片）                                   */
/* ========================================================================== */
void update_system_status(bool emergency) {
    g_emergency_active = emergency;
    if (!sys_status_lab) return;

    if (emergency) {
        lv_label_set_text(sys_status_lab, LV_SYMBOL_WARNING " SYSTEM ERROR");
        lv_obj_set_style_text_color(sys_status_lab, lv_color_white(), 0);
        lv_obj_set_style_bg_color(sys_status_lab, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_bg_opa(sys_status_lab, LV_OPA_COVER, 0);
        start_blink_anim(sys_status_lab, 400, LV_OPA_60);
        if (alarm_label) lv_label_set_text(alarm_label, "[ALARM] E-STOP ACTIVE");
        if (alarm_card) {
            lv_obj_set_style_bg_color(alarm_card, lv_palette_main(LV_PALETTE_RED), 0);
            lv_obj_set_style_bg_opa(alarm_card, LV_OPA_COVER, 0);
        }
        if (log_box) lv_textarea_add_text(log_box, "\n[ALARM] E-STOP ACTIVE");
    } else {
        lv_label_set_text(sys_status_lab, LV_SYMBOL_OK " SYSTEM NORMAL");
        lv_obj_set_style_text_color(sys_status_lab, lv_color_black(), 0);
        lv_obj_set_style_bg_color(sys_status_lab, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(sys_status_lab, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(sys_status_lab, LV_OPA_COVER, 0);
        if (alarm_label) lv_label_set_text(alarm_label, "NO ALARM");
        if (alarm_card) {
            lv_obj_set_style_bg_color(alarm_card, lv_color_hex(0x2A2A30), 0);
            lv_obj_set_style_bg_opa(alarm_card, LV_OPA_COVER, 0);
        }
    }
}

/* ========================================================================== */
/* 顶部状态栏紧急状态更新                                                       */
/* ========================================================================== */
void update_emergency_status(bool active) {
    if (!emergency_status_lab) return;

    if (active) {
        lv_label_set_text(emergency_status_lab, LV_SYMBOL_STOP " E-STOP: ACTIVE");
        lv_obj_set_style_text_color(emergency_status_lab, lv_color_white(), 0);
        lv_obj_set_style_bg_color(emergency_status_lab, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_bg_opa(emergency_status_lab, LV_OPA_COVER, 0);
        start_blink_anim(emergency_status_lab, 400, LV_OPA_50);
    } else {
        lv_label_set_text(emergency_status_lab, LV_SYMBOL_STOP " E-STOP: INACTIVE");
        lv_obj_set_style_text_color(emergency_status_lab, lv_color_black(), 0);
        lv_obj_set_style_bg_color(emergency_status_lab, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_bg_opa(emergency_status_lab, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(emergency_status_lab, LV_OPA_COVER, 0);
    }
}

/* ========================================================================== */
/* 右下角 AI 检测结果卡片更新（带进度条）                                        */
/* ========================================================================== */
void update_ai_result_card(float ng_pct, float ok_pct) {
    if (ai_ng_fill) {
        int32_t ng_w = (int32_t)((ALARM_W - 46) * ng_pct / 100.0f);
        if (ng_w < 0) ng_w = 0;
        if (ng_w > ALARM_W - 46) ng_w = ALARM_W - 46;
        lv_obj_set_width(ai_ng_fill, ng_w);
    }
    if (ai_ok_fill) {
        int32_t ok_w = (int32_t)((ALARM_W - 46) * ok_pct / 100.0f);
        if (ok_w < 0) ok_w = 0;
        if (ok_w > ALARM_W - 46) ok_w = ALARM_W - 46;
        lv_obj_set_width(ai_ok_fill, ok_w);
    }
    if (ai_ng_label) {
        lv_label_set_text_fmt(ai_ng_label, "AI NG: %.1f%%", ng_pct);
    }
    if (ai_ok_label) {
        lv_label_set_text_fmt(ai_ok_label, "AI OK: %.1f%%", ok_pct);
    }
    // 同步更新视频画面状态条
    if (ng_pct > 60.0f) {
        update_ai_status_bar(AI_OVERLAY_NG);
    } else if (ok_pct > 60.0f) {
        update_ai_status_bar(AI_OVERLAY_OK);
    }
}

/* ========================================================================== */
/* 更新良率与顶部统计卡片                                                       */
/* ========================================================================== */
void update_yield_stats(void) {
    if (p_lab) lv_label_set_text_fmt(p_lab, "PASS\n%lu", (unsigned long)count_pass);
    if (f_lab) lv_label_set_text_fmt(f_lab, "FAIL\n%lu", (unsigned long)count_fail);
    if (count_pass + count_fail > 0) {
        float yield = (float)count_pass / (count_pass + count_fail) * 100.0f;
        if (y_lab) lv_label_set_text_fmt(y_lab, "YIELD\n%.2f%%", yield);
    } else {
        if (y_lab) lv_label_set_text_fmt(y_lab, "YIELD\n%.2f%%", 0.0f);
    }
}

/* ========================================================================== */
/* 样品检测框更新                                                             */
/* ========================================================================== */
void ui_update_sample_box(const sample_roi_t *roi, bool show_result, bool final_good) {
    if (!part_detect_box) return;

    // 纯分类模式：不显示检测框，只通过日志/结果卡片展示 OK/NG
    lv_obj_add_flag(part_detect_box, LV_OBJ_FLAG_HIDDEN);
    (void)roi;
    (void)show_result;
    (void)final_good;

    if (part_detect_label) {
        if (!show_result) {
            lv_label_set_text(part_detect_label, "CHECKING");
            lv_obj_set_style_text_color(part_detect_label, lv_palette_main(LV_PALETTE_YELLOW), 0);
        } else if (final_good) {
            lv_label_set_text(part_detect_label, "GOOD");
            lv_obj_set_style_text_color(part_detect_label, lv_palette_main(LV_PALETTE_GREEN), 0);
        } else {
            lv_label_set_text(part_detect_label, "BAD");
            lv_obj_set_style_text_color(part_detect_label, lv_palette_main(LV_PALETTE_RED), 0);
        }
    }
}

void ui_reset_sample_box(void) {
    if (!part_detect_box) return;
    lv_obj_set_style_border_color(part_detect_box, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_add_flag(part_detect_box, LV_OBJ_FLAG_HIDDEN);
    if (part_detect_label) {
        lv_label_set_text(part_detect_label, "CHECKING");
        lv_obj_set_style_text_color(part_detect_label, lv_palette_main(LV_PALETTE_YELLOW), 0);
    }
}

/* ========================================================================== */
/* 按钮回调与 UI 事件任务                                                       */
/* ========================================================================== */
void btn_event_cb(lv_event_t *e) {
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    lv_obj_t *label = (lv_obj_t *)lv_obj_get_child(btn, 0);
    const char *txt = lv_label_get_text(label);

    system_event_t ev = {};
    if (ui_command_from_label(txt, &ev.type)) {
        xQueueSend(ui_event_queue, &ev, 0);
    }
}

void ui_event_task(void *pvParameters) {
    system_event_t ev;
    while (1) {
        if (xQueueReceive(ui_event_queue, &ev, portMAX_DELAY) == pdPASS) {

            // 处理拍照请求（内存拷贝在锁外完成）
            if (ev.type == CMD_BTN_CAPTURE) {
                uint8_t buf_idx;
                portENTER_CRITICAL(&ppa_spinlock);
                buf_idx = g_ppa_read_idx;
                portEXIT_CRITICAL(&ppa_spinlock);
                if (buf_idx >= NUM_BUFS || cam_buff[buf_idx] == nullptr) {
                    snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[ERR] No camera frame yet!");
                } else {
                    uint8_t *snap_copy = (uint8_t *)heap_caps_malloc(cam_buff_size, MALLOC_CAP_SPIRAM);
                    if (snap_copy) {
                        memcpy(snap_copy, cam_buff[buf_idx], cam_buff_size);
                        capture_msg_t c_msg = { .raw_img_buf = snap_copy };

                        if (xQueueSend(sd_write_queue, &c_msg, pdMS_TO_TICKS(200)) == pdPASS) {
                            snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[SD] Capturing...");
                        } else {
                            heap_caps_free(snap_copy);
                            snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[ERR] SD Queue Full!");
                        }
                    } else {
                        snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[ERR] Out of PSRAM!");
                    }
                }
            }

            // 获取 UI 锁，更新显示
            if (bsp_display_lock(portMAX_DELAY)) {
                switch (ev.type) {
                    case CMD_VISION_START:
                        update_vision_status(VISION_STATE_RUNNING);
                        update_ai_status_bar_bool(true);
                        lighting_set_brightness(80);  // 开始检测时打开照明
                        ui_reset_sample_box();
#if APP_ENABLE_SAMPLE_ROI
                        g_sample_background_capture_requested = true;
                        g_sample_background_delay_frames = 30;  // 等 ~1s 让灯和画面稳定
                        g_sample_present = false;
                        ESP_LOGI(TAG, "[ROI] Background capture requested on VISION START, stabilizing...");
#endif
                        if (b_vision_start) lv_obj_add_state(b_vision_start, LV_STATE_DISABLED);
                        if (b_vision_stop)  lv_obj_clear_state(b_vision_stop, LV_STATE_DISABLED);
                        if (b_audio_start) lv_obj_clear_state(b_audio_start, LV_STATE_DISABLED);
                        break;
                    case CMD_VISION_STOP:
                        update_vision_status(VISION_STATE_STOPPED);
                        update_ai_status_bar_bool(false);
                        lighting_set_brightness(0);  // 停止检测时关闭照明
#if APP_ENABLE_SAMPLE_ROI
                        g_sample_present = false;
                        g_sample_background_ready = false;
#endif
                        ui_reset_sample_box();
                        if (b_vision_start) lv_obj_clear_state(b_vision_start, LV_STATE_DISABLED);
                        if (b_vision_stop)  lv_obj_add_state(b_vision_stop, LV_STATE_DISABLED);
                        break;
                    case CMD_AUDIO_START:
                        update_audio_status(AUDIO_STATE_RUNNING);
                        if (b_audio_start) lv_obj_add_state(b_audio_start, LV_STATE_DISABLED);
                        // 手动启动 1 秒录音
                        {
                            system_event_t rec_ev = {};
                            rec_ev.type = EVENT_AUDIO_START_RECORD;
                            xQueueSend(audio_trigger_queue, &rec_ev, 0);
                            // 同步触发一次敲击（如有接敲击机构）
                            system_event_t hit_ev = {};
                            hit_ev.type = EVENT_HIT_TRIGGER;
                            xQueueSend(striker_trigger_queue, &hit_ev, 0);
                        }
                        break;
                    case CMD_SERVO_TOGGLE:
                        // 360° 连续舵机：按一下反转一小段，再按一下正转一小段
                        if (b_audio_toggle) {
                            lv_obj_t *lab = lv_obj_get_child(b_audio_toggle, 0);
                            if (g_servo_reversed) {
                                // 上一次为反转，这次正转回去
                                if (lab) lv_label_set_text(lab, LV_SYMBOL_DOWN " SERVO REV");
                                lv_obj_set_style_bg_color(b_audio_toggle, lv_color_hex(0x0088CC), 0);
                                lv_obj_set_style_bg_color(b_audio_toggle, lv_color_darken(lv_color_hex(0x0088CC), LV_OPA_30), LV_STATE_PRESSED);
                                servo_jog_continuous(SERVO_MANUAL_SPEED_PCT, SERVO_MANUAL_JOG_MS);
                                g_servo_reversed = false;
                            } else {
                                // 第一次按下先反转
                                if (lab) lv_label_set_text(lab, LV_SYMBOL_UP " SERVO FWD");
                                lv_obj_set_style_bg_color(b_audio_toggle, lv_color_hex(0xCC3333), 0);
                                lv_obj_set_style_bg_color(b_audio_toggle, lv_color_darken(lv_color_hex(0xCC3333), LV_OPA_30), LV_STATE_PRESSED);
                                servo_jog_continuous(-SERVO_MANUAL_SPEED_PCT, SERVO_MANUAL_JOG_MS);
                                g_servo_reversed = true;
                            }
                        }
                        break;
                    case CMD_BTN_EMERGENCY:
                        update_vision_status(VISION_STATE_EMERGENCY);
                        update_audio_status(AUDIO_STATE_EMERGENCY);
                        update_system_status(true);
                        update_ai_status_bar_bool(false);
                        lighting_set_brightness(0);  // 急停时关闭照明
#if APP_ENABLE_SAMPLE_ROI
                        g_sample_present = false;
                        g_sample_background_ready = false;
#endif
                        ui_reset_sample_box();
                        // 禁用所有按钮，仅保留视觉反馈
                        if (b_vision_start)  lv_obj_add_state(b_vision_start, LV_STATE_DISABLED);
                        if (b_vision_stop)   lv_obj_add_state(b_vision_stop, LV_STATE_DISABLED);
                        if (b_audio_start)   lv_obj_add_state(b_audio_start, LV_STATE_DISABLED);
                        if (b_audio_toggle)  lv_obj_add_state(b_audio_toggle, LV_STATE_DISABLED);
                        if (b_emergency)     lv_obj_add_state(b_emergency, LV_STATE_DISABLED);
                        if (b_capture)       lv_obj_add_state(b_capture, LV_STATE_DISABLED);
                        break;
                    case CMD_BTN_CAPTURE:
                        lv_textarea_add_text(log_box, ev.log_msg);
                        if (screenshot_log_label) {
                            if (strstr(ev.log_msg, "ERR")) {
                                s_capture_dropped_cnt++;
                                lv_label_set_text_fmt(screenshot_log_label, "Dropped:%lu Saved:%lu",
                                                      (unsigned long)s_capture_dropped_cnt,
                                                      (unsigned long)s_capture_saved_cnt);
                                lv_obj_set_style_text_color(screenshot_log_label, lv_palette_main(LV_PALETTE_RED), 0);
                            } else {
                                lv_label_set_text(screenshot_log_label, "Capturing...");
                                lv_obj_set_style_text_color(screenshot_log_label, lv_color_hex(0x00D2FF), 0);
                            }
                        }
                        break;
                    case EVENT_AI_PASS:
                        count_pass++;
                        update_yield_stats();
                        update_ai_result_card(g_ai_ng_pct, g_ai_ok_pct);
                        ui_update_sample_box(g_sample_present ? &g_sample_roi : NULL, true, true);
                        lv_textarea_add_text(log_box, "\n[AI] NORMAL OK");
                        break;
                    case EVENT_AI_DEFECT_FOUND:
                        count_fail++;
                        update_yield_stats();
                        update_ai_result_card(g_ai_ng_pct, g_ai_ok_pct);
                        ui_update_sample_box(g_sample_present ? &g_sample_roi : NULL, true, false);
                        lv_textarea_add_text(log_box, ev.log_msg);
                        break;
                    case EVENT_AI_NO_DETECTION:
                        update_ai_status_bar(AI_OVERLAY_DETECTING);
                        update_ai_result_card(g_ai_ng_pct, g_ai_ok_pct);
                        ui_reset_sample_box();
                        break;
                    case CMD_BTN_CAPTURE_DONE:
                        lv_textarea_add_text(log_box, "\n[SD] Saved to SD Card!");
                        s_capture_saved_cnt++;
                        if (screenshot_log_label) {
                            lv_label_set_text_fmt(screenshot_log_label, "Saved:%lu Dropped:%lu",
                                                  (unsigned long)s_capture_saved_cnt,
                                                  (unsigned long)s_capture_dropped_cnt);
                            lv_obj_set_style_text_color(screenshot_log_label, lv_palette_main(LV_PALETTE_GREEN), 0);
                        }
                        break;
                    case CMD_BTN_CAPTURE_FAIL:
                        lv_textarea_add_text(log_box, "\n[ERR] SD Write Failed!");
                        s_capture_dropped_cnt++;
                        if (screenshot_log_label) {
                            lv_label_set_text_fmt(screenshot_log_label, "Dropped:%lu Saved:%lu",
                                                  (unsigned long)s_capture_dropped_cnt,
                                                  (unsigned long)s_capture_saved_cnt);
                            lv_obj_set_style_text_color(screenshot_log_label, lv_palette_main(LV_PALETTE_RED), 0);
                        }
                        break;
                    case EVENT_AUDIO_CAP_DONE:
                        if (!g_emergency_active) {
                            update_audio_status(AUDIO_STATE_STOPPED);
                            if (b_audio_start) lv_obj_clear_state(b_audio_start, LV_STATE_DISABLED);
                        }
                        lv_textarea_add_text(log_box, "\n[AUDIO] Capture Saved");
                        break;
                    case EVENT_AUDIO_AI_GOOD:
                    case EVENT_AUDIO_AI_ANOMALY:
                        ui_update_audio_ai_score(ev.type == EVENT_AUDIO_AI_GOOD, ev.log_msg);
                        break;
                    default: break;
                }

                // 更新良率（需要访问 LVGL 对象，必须在锁内）
                update_yield_stats();

                bsp_display_unlock();
            }
        }
    }
}

/* ========================================================================== */
/* 工业面板 UI 搭建                                                            */
/* ========================================================================== */
void ui_create(void) {
    bsp_display_lock(0);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0A0C), 0); // 整体工业深底色

    /* ===================================================================== */
    /* 顶部全局状态栏                                                        */
    /* ===================================================================== */
    lv_obj_t *top_bar = lv_obj_create(scr);
    lv_obj_set_size(top_bar, SCREEN_W, TOP_H);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x141419), 0);
    lv_obj_set_style_radius(top_bar, 0, 0);
    lv_obj_set_style_border_width(top_bar, 0, 0);
    lv_obj_set_style_pad_all(top_bar, 0, 0);

    // 1) 系统状态
    sys_status_lab = lv_label_create(top_bar);
    lv_label_set_text(sys_status_lab, LV_SYMBOL_OK " SYSTEM NORMAL");
    lv_obj_set_style_text_font(sys_status_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sys_status_lab, lv_color_black(), 0);
    lv_obj_set_style_bg_color(sys_status_lab, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(sys_status_lab, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(sys_status_lab, 6, 0);
    lv_obj_set_style_pad_hor(sys_status_lab, 12, 0);
    lv_obj_set_style_pad_ver(sys_status_lab, 8, 0);
    lv_obj_align(sys_status_lab, LV_ALIGN_LEFT_MID, 10, 0);

    // 2) 视觉状态
    vision_status_lab = lv_label_create(top_bar);
    lv_label_set_text(vision_status_lab, LV_SYMBOL_VIDEO " VISION: IDLE");
    lv_obj_set_style_text_font(vision_status_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(vision_status_lab, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(vision_status_lab, LV_ALIGN_LEFT_MID, 300, 0);

    // 3) 音频状态
    audio_status_lab = lv_label_create(top_bar);
    lv_label_set_text(audio_status_lab, LV_SYMBOL_AUDIO " AUDIO: STOPPED");
    lv_obj_set_style_text_font(audio_status_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(audio_status_lab, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_align(audio_status_lab, LV_ALIGN_LEFT_MID, 500, 0);

    // 4) 紧急状态
    emergency_status_lab = lv_label_create(top_bar);
    lv_label_set_text(emergency_status_lab, LV_SYMBOL_STOP " E-STOP: INACTIVE");
    lv_obj_set_style_text_font(emergency_status_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(emergency_status_lab, lv_color_black(), 0);
    lv_obj_set_style_bg_color(emergency_status_lab, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_opa(emergency_status_lab, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(emergency_status_lab, 6, 0);
    lv_obj_set_style_pad_hor(emergency_status_lab, 12, 0);
    lv_obj_set_style_pad_ver(emergency_status_lab, 8, 0);
    lv_obj_align(emergency_status_lab, LV_ALIGN_RIGHT_MID, -8, 0);

    /* ===================================================================== */
    /* 左侧主视频画面区                                                      */
    /* ===================================================================== */
    camera_canvas = lv_canvas_create(scr);
    lv_obj_set_size(camera_canvas, CAM_W, CAM_H);
    lv_canvas_set_buffer(camera_canvas, cam_buff[0], CAM_W, CAM_H, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(camera_canvas, 0, TOP_H);
    lv_obj_set_style_pad_all(camera_canvas, 0, 0);
    lv_obj_set_style_border_width(camera_canvas, 0, 0);
    lv_obj_set_style_bg_opa(camera_canvas, LV_OPA_TRANSP, 0);

    // 画面顶部中央半透明状态条（摄像头右上角 AI 检测状态）
    ai_overlay_bar = lv_obj_create(scr);
    lv_obj_set_size(ai_overlay_bar, 300, 34);
    lv_obj_align(ai_overlay_bar, LV_ALIGN_TOP_MID, 0, TOP_H + 12);
    lv_obj_set_style_bg_color(ai_overlay_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ai_overlay_bar, LV_OPA_70, 0);
    lv_obj_set_style_radius(ai_overlay_bar, 17, 0);
    lv_obj_set_style_border_width(ai_overlay_bar, 0, 0);
    lv_obj_set_style_pad_all(ai_overlay_bar, 0, 0);

    ai_overlay_lab = lv_label_create(ai_overlay_bar);
    lv_label_set_text(ai_overlay_lab, LV_SYMBOL_MINUS " AI: IDLE");
    lv_obj_set_style_text_font(ai_overlay_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ai_overlay_lab, lv_color_hex(0x888888), 0);
    lv_obj_center(ai_overlay_lab);

    // 半透明十字线叠加（辅助对齐）
    lv_obj_t *cross_h = lv_obj_create(scr);
    lv_obj_set_size(cross_h, CAM_W, 1);
    lv_obj_set_pos(cross_h, 0, TOP_H + CAM_H / 2);
    lv_obj_set_style_bg_color(cross_h, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(cross_h, LV_OPA_20, 0);
    lv_obj_set_style_border_width(cross_h, 0, 0);

    lv_obj_t *cross_v = lv_obj_create(scr);
    lv_obj_set_size(cross_v, 1, CAM_H);
    lv_obj_set_pos(cross_v, CAM_W / 2, TOP_H);
    lv_obj_set_style_bg_color(cross_v, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(cross_v, LV_OPA_20, 0);
    lv_obj_set_style_border_width(cross_v, 0, 0);

    // 检测框（默认隐藏，检测到物体时显示）
    part_detect_box = lv_obj_create(scr);
    lv_obj_set_size(part_detect_box, 200, 200);
    lv_obj_center(part_detect_box);
    lv_obj_set_style_bg_opa(part_detect_box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(part_detect_box, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_set_style_border_width(part_detect_box, 3, 0);
    lv_obj_set_style_radius(part_detect_box, 4, 0);
    lv_obj_set_style_pad_all(part_detect_box, 0, 0);
    lv_obj_clear_flag(part_detect_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(part_detect_box, LV_OBJ_FLAG_HIDDEN);

    part_detect_label = lv_label_create(part_detect_box);
    lv_label_set_text(part_detect_label, "CHECKING");
    lv_obj_set_style_text_font(part_detect_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(part_detect_label, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_align(part_detect_label, LV_ALIGN_TOP_MID, 0, 4);

    /* ===================================================================== */
    /* 右侧操作面板                                                          */
    /* ===================================================================== */
    lv_obj_t *rp = lv_obj_create(scr);
    lv_obj_set_size(rp, PANEL_W, CAM_H);
    lv_obj_set_pos(rp, CAM_W, TOP_H);
    lv_obj_set_style_bg_color(rp, lv_color_hex(0x1A1A1F), 0);
    lv_obj_set_style_radius(rp, 0, 0);
    lv_obj_set_style_border_width(rp, 0, 0);
    lv_obj_set_style_pad_all(rp, 0, 0);

    auto create_btn = [](lv_obj_t *parent, const char *text, lv_color_t bg, lv_color_t fg, int y, int h) -> lv_obj_t* {
        lv_obj_t *btn = lv_btn_create(parent);
        lv_obj_set_size(btn, PANEL_W - 30, h);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, y);
        lv_obj_set_style_bg_color(btn, bg, 0);
        lv_obj_set_style_bg_color(btn, lv_color_darken(bg, LV_OPA_30), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_t *lab = lv_label_create(btn);
        lv_label_set_text(lab, text);
        lv_obj_set_style_text_font(lab, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lab, fg, 0);
        lv_obj_center(lab);
        return btn;
    };

    auto add_divider = [](lv_obj_t *parent, int y) {
        lv_obj_t *line = lv_obj_create(parent);
        lv_obj_set_size(line, PANEL_W - 40, 1);
        lv_obj_align(line, LV_ALIGN_TOP_MID, 0, y);
        lv_obj_set_style_bg_color(line, lv_color_hex(0x333333), 0);
        lv_obj_set_style_border_width(line, 0, 0);
    };

    // 1) 紧急停止（单独隔离，最大尺寸）
    b_emergency = create_btn(rp, LV_SYMBOL_STOP " EMERGENCY STOP", lv_color_hex(0xBC0000), lv_color_white(), 8, 76);
    lv_obj_set_style_text_font(lv_obj_get_child(b_emergency, 0), &lv_font_montserrat_14, 0);
    lv_obj_add_event_cb(b_emergency, btn_event_cb, LV_EVENT_CLICKED, NULL);

    add_divider(rp, 94);

    // 2) 视觉检测组
    lv_obj_t *vision_title = lv_label_create(rp);
    lv_label_set_text(vision_title, LV_SYMBOL_VIDEO " VISION");
    lv_obj_set_style_text_font(vision_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(vision_title, lv_color_hex(0x888888), 0);
    lv_obj_align(vision_title, LV_ALIGN_TOP_LEFT, 20, 102);

    b_vision_start = create_btn(rp, LV_SYMBOL_PLAY " VISION START", lv_palette_main(LV_PALETTE_GREEN), lv_color_white(), 120, 38);
    lv_obj_add_event_cb(b_vision_start, btn_event_cb, LV_EVENT_CLICKED, NULL);

    b_vision_stop = create_btn(rp, LV_SYMBOL_STOP " VISION STOP", lv_color_hex(0xCCAA00), lv_color_black(), 164, 38);
    lv_obj_add_state(b_vision_stop, LV_STATE_DISABLED);
    lv_obj_add_event_cb(b_vision_stop, btn_event_cb, LV_EVENT_CLICKED, NULL);

    add_divider(rp, 212);

    // 3) 音频分析组
    lv_obj_t *audio_title = lv_label_create(rp);
    lv_label_set_text(audio_title, LV_SYMBOL_AUDIO " AUDIO");
    lv_obj_set_style_text_font(audio_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(audio_title, lv_color_hex(0x888888), 0);
    lv_obj_align(audio_title, LV_ALIGN_TOP_LEFT, 20, 220);

    b_audio_start = create_btn(rp, LV_SYMBOL_PLAY " AUDIO START", lv_color_hex(0x0088CC), lv_color_white(), 238, 38);
    lv_obj_add_event_cb(b_audio_start, btn_event_cb, LV_EVENT_CLICKED, NULL);

    // 舵机手动反转按钮：一键反转 30°，速度 50%
    b_audio_toggle = create_btn(rp, LV_SYMBOL_DOWN " SERVO REV", lv_color_hex(0x0088CC), lv_color_white(), 282, 38);
    lv_obj_add_event_cb(b_audio_toggle, btn_event_cb, LV_EVENT_CLICKED, NULL);

    add_divider(rp, 332);

    // 4) 辅助操作
    b_capture = create_btn(rp, LV_SYMBOL_SAVE " CAPTURE", lv_color_hex(0x0088FF), lv_color_white(), 350, 38);
    lv_obj_add_event_cb(b_capture, btn_event_cb, LV_EVENT_CLICKED, NULL);

    /* ===================================================================== */
    /* 底部声纹分析区（左下）                                                */
    /* ===================================================================== */
    lv_obj_t *sp = lv_obj_create(scr);
    lv_obj_set_size(sp, ACOUSTIC_W, ACOUSTIC_H);
    lv_obj_align(sp, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_bg_color(sp, lv_color_hex(0x121216), 0);
    lv_obj_set_style_radius(sp, 0, 0);
    lv_obj_set_style_border_width(sp, 0, 0);
    lv_obj_set_style_pad_all(sp, 0, 0);

    lv_obj_t *sound_title = lv_label_create(sp);
    lv_label_set_text(sound_title, LV_SYMBOL_AUDIO " ACOUSTIC ANALYSIS");
    lv_obj_set_style_text_font(sound_title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sound_title, lv_color_hex(0x00D2FF), 0);
    lv_obj_align(sound_title, LV_ALIGN_TOP_LEFT, 10, 6);

    // 实时波形图
    acoustic_chart = lv_chart_create(sp);
    lv_obj_set_size(acoustic_chart, 520, 100);
    lv_obj_align(acoustic_chart, LV_ALIGN_TOP_LEFT, 10, 28);
    lv_chart_set_type(acoustic_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(acoustic_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 500);
    lv_chart_set_point_count(acoustic_chart, 120);
    lv_obj_set_style_bg_color(acoustic_chart, lv_color_hex(0x0A0A0C), LV_PART_MAIN);
    lv_obj_set_style_line_color(acoustic_chart, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_size(acoustic_chart, 0, 0, LV_PART_INDICATOR);
    acoustic_chart_ser = lv_chart_add_series(acoustic_chart, lv_color_hex(0x00FF88), LV_CHART_AXIS_PRIMARY_Y);

    // X 轴 / Y 轴刻度标签
    lv_obj_t *x_label = lv_label_create(sp);
    lv_label_set_text(x_label, "0s        5s        10s");
    lv_obj_set_style_text_font(x_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(x_label, lv_color_hex(0x666666), 0);
    lv_obj_align_to(x_label, acoustic_chart, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    lv_obj_t *y_label = lv_label_create(sp);
    lv_label_set_text(y_label, "500\n250\n0");
    lv_obj_set_style_text_font(y_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(y_label, lv_color_hex(0x666666), 0);
    lv_obj_align_to(y_label, acoustic_chart, LV_ALIGN_OUT_LEFT_TOP, -2, 0);

    // 状态提示区（右侧）
    lv_obj_t *result_bg = lv_obj_create(sp);
    lv_obj_set_size(result_bg, 170, 100);
    lv_obj_align_to(result_bg, acoustic_chart, LV_ALIGN_OUT_RIGHT_TOP, 12, 0);
    lv_obj_set_style_bg_color(result_bg, lv_color_hex(0x1A1A1F), 0);
    lv_obj_set_style_radius(result_bg, 8, 0);
    lv_obj_set_style_border_width(result_bg, 0, 0);

    acoustic_result_lab = lv_label_create(result_bg);
    lv_label_set_text(acoustic_result_lab, "WAIT NO HIT");
    lv_obj_set_style_text_font(acoustic_result_lab, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(acoustic_result_lab, lv_color_hex(0x888888), 0);
    lv_obj_center(acoustic_result_lab);

    acoustic_result_sub = lv_label_create(result_bg);
    lv_label_set_text(acoustic_result_sub, "--");
    lv_obj_set_style_text_font(acoustic_result_sub, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(acoustic_result_sub, lv_color_hex(0x888888), 0);
    lv_obj_align(acoustic_result_sub, LV_ALIGN_BOTTOM_MID, 0, -6);

    /* ===================================================================== */
    /* 右下角报警与 AI 结果区                                                */
    /* ===================================================================== */
    lv_obj_t *alarm_cont = lv_obj_create(scr);
    lv_obj_set_size(alarm_cont, ALARM_W, ALARM_H);
    lv_obj_set_pos(alarm_cont, CAM_W, SCREEN_H - ALARM_H);
    lv_obj_set_style_bg_color(alarm_cont, lv_color_hex(0x1A1A1F), 0);
    lv_obj_set_style_radius(alarm_cont, 0, 0);
    lv_obj_set_style_border_width(alarm_cont, 0, 0);
    lv_obj_set_style_pad_all(alarm_cont, 0, 0);

    // 统计条（PASS/FAIL/YIELD）
    lv_obj_t *stat_bar = lv_obj_create(alarm_cont);
    lv_obj_set_size(stat_bar, ALARM_W - 30, 24);
    lv_obj_align(stat_bar, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_bg_color(stat_bar, lv_color_hex(0x222228), 0);
    lv_obj_set_style_bg_opa(stat_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(stat_bar, 8, 0);
    lv_obj_set_style_border_width(stat_bar, 0, 0);
    lv_obj_set_style_pad_all(stat_bar, 0, 0);

    p_lab = lv_label_create(stat_bar);
    lv_label_set_text(p_lab, "PASS\n0");
    lv_obj_set_style_text_font(p_lab, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(p_lab, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(p_lab, LV_ALIGN_LEFT_MID, 10, 0);

    f_lab = lv_label_create(stat_bar);
    lv_label_set_text(f_lab, "FAIL\n0");
    lv_obj_set_style_text_font(f_lab, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(f_lab, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(f_lab, LV_ALIGN_CENTER, -10, 0);

    y_lab = lv_label_create(stat_bar);
    lv_label_set_text(y_lab, "YIELD\n0.00%");
    lv_obj_set_style_text_font(y_lab, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(y_lab, lv_color_hex(0x00D2FF), 0);
    lv_obj_align(y_lab, LV_ALIGN_RIGHT_MID, -10, 0);

    // 报警信息卡片
    alarm_card = lv_obj_create(alarm_cont);
    lv_obj_set_size(alarm_card, ALARM_W - 30, 30);
    lv_obj_align(alarm_card, LV_ALIGN_TOP_MID, 0, 32);
    lv_obj_set_style_bg_color(alarm_card, lv_color_hex(0x2A2A30), 0);
    lv_obj_set_style_bg_opa(alarm_card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(alarm_card, 8, 0);
    lv_obj_set_style_border_width(alarm_card, 0, 0);

    alarm_label = lv_label_create(alarm_card);
    lv_label_set_text(alarm_label, "NO ALARM");
    lv_obj_set_style_text_font(alarm_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(alarm_label, lv_color_hex(0x888888), 0);
    lv_obj_center(alarm_label);

    // AI 检测结果卡片
    ai_result_card = lv_obj_create(alarm_cont);
    lv_obj_set_size(ai_result_card, ALARM_W - 30, 52);
    lv_obj_align(ai_result_card, LV_ALIGN_TOP_MID, 0, 64);
    lv_obj_set_style_bg_color(ai_result_card, lv_color_hex(0x222228), 0);
    lv_obj_set_style_bg_opa(ai_result_card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ai_result_card, 8, 0);
    lv_obj_set_style_border_width(ai_result_card, 0, 0);

    ai_ng_label = lv_label_create(ai_result_card);
    lv_label_set_text(ai_ng_label, "AI NG: 0.0%");
    lv_obj_set_style_text_font(ai_ng_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(ai_ng_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(ai_ng_label, LV_ALIGN_TOP_LEFT, 8, 2);

    // NG 进度条：背景 + 填充矩形
    ai_ng_bar = lv_obj_create(ai_result_card);
    lv_obj_set_size(ai_ng_bar, ALARM_W - 46, 8);
    lv_obj_align(ai_ng_bar, LV_ALIGN_TOP_LEFT, 8, 16);
    lv_obj_set_style_bg_color(ai_ng_bar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(ai_ng_bar, 4, 0);
    lv_obj_set_style_border_width(ai_ng_bar, 0, 0);
    lv_obj_set_style_pad_all(ai_ng_bar, 0, 0);
    ai_ng_fill = lv_obj_create(ai_ng_bar);
    lv_obj_set_size(ai_ng_fill, 0, 8);
    lv_obj_align(ai_ng_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(ai_ng_fill, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_radius(ai_ng_fill, 4, 0);
    lv_obj_set_style_border_width(ai_ng_fill, 0, 0);

    ai_ok_label = lv_label_create(ai_result_card);
    lv_label_set_text(ai_ok_label, "AI OK: 0.0%");
    lv_obj_set_style_text_font(ai_ok_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(ai_ok_label, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_align(ai_ok_label, LV_ALIGN_TOP_LEFT, 8, 26);

    // OK 进度条：背景 + 填充矩形
    ai_ok_bar = lv_obj_create(ai_result_card);
    lv_obj_set_size(ai_ok_bar, ALARM_W - 46, 8);
    lv_obj_align(ai_ok_bar, LV_ALIGN_TOP_LEFT, 8, 40);
    lv_obj_set_style_bg_color(ai_ok_bar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(ai_ok_bar, 4, 0);
    lv_obj_set_style_border_width(ai_ok_bar, 0, 0);
    lv_obj_set_style_pad_all(ai_ok_bar, 0, 0);
    ai_ok_fill = lv_obj_create(ai_ok_bar);
    lv_obj_set_size(ai_ok_fill, 0, 8);
    lv_obj_align(ai_ok_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(ai_ok_fill, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_radius(ai_ok_fill, 4, 0);
    lv_obj_set_style_border_width(ai_ok_fill, 0, 0);

    // 截图状态日志卡片
    lv_obj_t *ss_log_card = lv_obj_create(alarm_cont);
    lv_obj_set_size(ss_log_card, ALARM_W - 30, 20);
    lv_obj_align(ss_log_card, LV_ALIGN_TOP_MID, 0, 118);
    lv_obj_set_style_bg_color(ss_log_card, lv_color_hex(0x1A1A1F), 0);
    lv_obj_set_style_bg_opa(ss_log_card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ss_log_card, 6, 0);
    lv_obj_set_style_border_width(ss_log_card, 0, 0);

    screenshot_log_label = lv_label_create(ss_log_card);
    lv_label_set_text(screenshot_log_label, "Ready");
    lv_obj_set_style_text_font(screenshot_log_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(screenshot_log_label, lv_color_hex(0x888888), 0);
    lv_obj_center(screenshot_log_label);

    // 日志框（保留句柄但不再显示）
    log_box = lv_textarea_create(alarm_cont);
    lv_obj_set_size(log_box, ALARM_W - 30, 0);
    lv_obj_add_flag(log_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(log_box, lv_color_black(), 0);
    lv_obj_set_style_text_color(log_box, lv_palette_main(LV_PALETTE_LIGHT_GREEN), 0);

    update_system_status(false);
    update_vision_status(VISION_STATE_STOPPED);
    update_audio_status(AUDIO_STATE_STOPPED);
    update_emergency_status(false);
    update_ai_status_bar_bool(false);
    bsp_display_unlock();
}

/* ========================================================================== */
/* 声纹分析区更新                                                             */
/* ========================================================================== */
void ui_update_acoustic_result(int health_score, int hit_pulse, float band_total, float bg_noise) {
    if (!acoustic_chart || !acoustic_chart_ser || !acoustic_result_lab) return;

    if (bsp_display_lock(pdMS_TO_TICKS(5))) {
        lv_chart_set_next_value(acoustic_chart, acoustic_chart_ser, (int32_t)health_score);
        lv_chart_refresh(acoustic_chart);

        // 只更新图表，文字由 ui_update_audio_ai_score 在敲击后更新
        lv_chart_refresh(acoustic_chart);

        bsp_display_unlock();
    }
}

/* ========================================================================== */
/* 声学 AI 敲击结果概率显示 — 每次敲击录音完成后调用一次                        */
/* ========================================================================== */
void ui_update_audio_ai_score(bool is_good, const char *zscore_str) {
    if (!acoustic_result_lab || !acoustic_result_sub || !zscore_str) return;

    float max_zscore = strtof(zscore_str, NULL);
    float good_pct = 100.0f * (1.0f - max_zscore / 6.0f);
    if (good_pct < 0.0f) good_pct = 0.0f;
    if (good_pct > 100.0f) good_pct = 100.0f;

    if (bsp_display_lock(pdMS_TO_TICKS(10))) {
        if (is_good) {
            lv_label_set_text_fmt(acoustic_result_lab, "GOOD %.0f%%", (double)good_pct);
            lv_obj_set_style_text_color(acoustic_result_lab, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_label_set_text_fmt(acoustic_result_sub, "Z=%.2f  NORMAL", (double)max_zscore);
            lv_obj_set_style_text_color(acoustic_result_sub, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_obj_set_style_line_color(acoustic_chart, lv_palette_main(LV_PALETTE_GREEN), LV_PART_ITEMS);
        } else {
            float bad_pct = 100.0f - good_pct;
            lv_label_set_text_fmt(acoustic_result_lab, "BAD %.0f%%", (double)bad_pct);
            lv_obj_set_style_text_color(acoustic_result_lab, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text_fmt(acoustic_result_sub, "Z=%.2f  ANOMALY", (double)max_zscore);
            lv_obj_set_style_text_color(acoustic_result_sub, lv_palette_main(LV_PALETTE_RED), 0);
            lv_obj_set_style_line_color(acoustic_chart, lv_palette_main(LV_PALETTE_RED), LV_PART_ITEMS);
        }
        bsp_display_unlock();
    }
}
