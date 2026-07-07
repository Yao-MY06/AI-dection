/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "video_pipeline.h"
#include "app_config.h"
#include "app_globals.h"
#include "ui_manager.h"
#if APP_ENABLE_SAMPLE_ROI
#include "sample_roi.hpp"
#endif

#include <string.h>
#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if SOC_PPA_SUPPORTED
#include "driver/ppa.h"
#endif

/* ========================================================================== */
/* PPA 与摄像头缓冲（模块私有）                                               */
/* ========================================================================== */
#if SOC_PPA_SUPPORTED
static ppa_client_handle_t ppa_srm_handle = NULL;
#endif

/* ========================================================================== */
/* 动态样品框跟踪状态（模块私有）                                             */
/* ========================================================================== */
#if APP_ENABLE_SAMPLE_ROI
static sample_roi_t s_sample_candidate_roi = {};
static int s_sample_stable_frames = 0;
static int s_sample_missing_frames = 0;

#define SAMPLE_STABLE_FRAMES    3
#define SAMPLE_MISSING_FRAMES   8
#define SAMPLE_ROI_MOVE_TOLERANCE   24
#define SAMPLE_ROI_SIZE_TOLERANCE   36

static bool sample_roi_is_similar(const sample_roi_t *a, const sample_roi_t *b)
{
    if (!a || !b || !a->present || !b->present) {
        return false;
    }
    return abs(a->x - b->x) <= SAMPLE_ROI_MOVE_TOLERANCE &&
           abs(a->y - b->y) <= SAMPLE_ROI_MOVE_TOLERANCE &&
           abs(a->w - b->w) <= SAMPLE_ROI_SIZE_TOLERANCE &&
           abs(a->h - b->h) <= SAMPLE_ROI_SIZE_TOLERANCE;
}
#endif

/* ========================================================================== */
/* PPA 初始化与图像处理                                                       */
/* ========================================================================== */
#if SOC_PPA_SUPPORTED
static bool app_ppa_done_cb(ppa_client_handle_t ppa_client, ppa_event_data_t *event_data, void *user_data) {
    portENTER_CRITICAL(&ppa_spinlock);
    g_ppa_read_idx = g_ppa_write_idx;
    g_ppa_write_idx = (g_ppa_write_idx + 1) % NUM_BUFS;
    g_ppa_busy = false;
    g_ppa_done_cnt = g_ppa_done_cnt + 1;
    g_has_camera_frame = true;
    portEXIT_CRITICAL(&ppa_spinlock);
    return false;
}

static void app_ppa_init(void) {
    ppa_client_config_t cfg = {};
    cfg.oper_type = PPA_OPERATION_SRM;
    ppa_register_client(&cfg, &ppa_srm_handle);

    ppa_event_callbacks_t cbs = {
        .on_trans_done = app_ppa_done_cb,
    };
    ppa_client_register_event_callbacks(ppa_srm_handle, &cbs);
}

static esp_err_t app_image_process(uint8_t *in_buf, uint32_t in_w, uint32_t in_h, uint8_t *out_buf) {
    uint32_t crop_w = in_w;
    uint32_t crop_h = in_h;

    uint32_t target_w = (uint32_t)((float)in_h * CAM_W / CAM_H);
    uint32_t target_h = (uint32_t)((float)in_w * CAM_H / CAM_W);
    if (target_w <= in_w) {
        crop_w = target_w;
    } else {
        crop_h = target_h;
    }

    crop_w &= ~1U;
    crop_h &= ~1U;
    if (crop_w == 0 || crop_h == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t offset_x = (in_w > crop_w) ? (in_w - crop_w) / 2 : 0;
    uint32_t offset_y = (in_h > crop_h) ? (in_h - crop_h) / 2 : 0;
    offset_x &= ~1;
    offset_y &= ~1;

    ppa_srm_oper_config_t srm_config;
    memset(&srm_config, 0, sizeof(srm_config));
    srm_config.in.buffer = in_buf;
    srm_config.in.pic_w = in_w;
    srm_config.in.pic_h = in_h;
    srm_config.in.block_w = crop_w;
    srm_config.in.block_h = crop_h;
    srm_config.in.block_offset_x = offset_x;
    srm_config.in.block_offset_y = offset_y;
    srm_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm_config.out.buffer = out_buf;
    srm_config.out.buffer_size = cam_buff_size;
    srm_config.out.pic_w = CAM_W;
    srm_config.out.pic_h = CAM_H;
    srm_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm_config.scale_x = (float)CAM_W / (float)crop_w;
    srm_config.scale_y = (float)CAM_H / (float)crop_h;
    srm_config.mode = PPA_TRANS_MODE_BLOCKING;

    return ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
}
#endif

void video_pipeline_init(void) {
#if SOC_PPA_SUPPORTED
    app_ppa_init();
#endif

#if APP_ENABLE_SAMPLE_ROI
    /* 分配样品背景灰度缓存，优先 PSRAM */
    if (!g_sample_background_gray) {
        g_sample_background_gray = (uint8_t *)heap_caps_malloc(CAM_W * CAM_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_sample_background_gray) {
            g_sample_background_gray = (uint8_t *)malloc(CAM_W * CAM_H);
        }
        if (!g_sample_background_gray) {
            ESP_LOGE(TAG, "Sample background buffer alloc failed");
        } else {
            memset(g_sample_background_gray, 0, CAM_W * CAM_H);
        }
    }
#endif
}

/* ========================================================================== */
/* 显示刷新任务                                                               */
/* ========================================================================== */
void display_refresh_task(void *pvParameters) {
    uint8_t last_read_idx = CAM_BUF_IDX_NONE;
    int64_t last_diag_us = esp_timer_get_time();
    while (1) {
        uint8_t read_idx;
        portENTER_CRITICAL(&ppa_spinlock);
        read_idx = g_ppa_read_idx;
        portEXIT_CRITICAL(&ppa_spinlock);

        if (read_idx != last_read_idx && read_idx < NUM_BUFS && cam_buff[read_idx] != nullptr) {
            last_read_idx = read_idx;
            if (bsp_display_lock(pdMS_TO_TICKS(5))) {
                lv_canvas_set_buffer(ui_get_camera_canvas(), cam_buff[read_idx], CAM_W, CAM_H, LV_COLOR_FORMAT_RGB565);
#if APP_ENABLE_SAMPLE_ROI
                ui_update_sample_box(g_sample_present ? &g_sample_roi : NULL, false, false);
#endif
                bsp_display_unlock();
            }
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_diag_us > 5000000) {
            uint32_t submit, done;
            bool busy;
            portENTER_CRITICAL(&ppa_spinlock);
            submit = g_ppa_submit_cnt;
            done = g_ppa_done_cnt;
            busy = g_ppa_busy;
            portEXIT_CRITICAL(&ppa_spinlock);
            ESP_LOGI(TAG, "[PPA_DIAG] busy=%d submit=%lu done=%lu diff=%ld",
                     busy, submit, done, (long)(submit - done));
            last_diag_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

/* ========================================================================== */
/* 摄像头帧回调                                                               */
/* ========================================================================== */
void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index,
                                  uint32_t camera_buf_hes, uint32_t camera_buf_ves,
                                  size_t camera_buf_len) {
    (void)camera_buf_index;
    (void)camera_buf_len;

    if (g_emergency_active) return;

    portENTER_CRITICAL(&ppa_spinlock);
    if (g_ppa_busy) {
        portEXIT_CRITICAL(&ppa_spinlock);
        g_has_camera_frame = true;
        return;
    }
    g_ppa_busy = true;
    uint8_t write_idx = g_ppa_write_idx;
    bool ai_reading = (write_idx == g_ai_reading_cam_buf_idx);  // 同一临界区内检查，消除竞态
    portEXIT_CRITICAL(&ppa_spinlock);

    if (ai_reading) {
        portENTER_CRITICAL(&ppa_spinlock);
        g_ppa_busy = false;
        portEXIT_CRITICAL(&ppa_spinlock);
        g_has_camera_frame = true;
        return;
    }

    int64_t t_start = esp_timer_get_time();
#if SOC_PPA_SUPPORTED
    int64_t t_ppa_start = esp_timer_get_time();
    if (ppa_srm_handle) {
        portENTER_CRITICAL(&ppa_spinlock);
        g_ppa_submit_cnt = g_ppa_submit_cnt + 1;
        portEXIT_CRITICAL(&ppa_spinlock);

        esp_err_t ret = app_image_process(camera_buf, camera_buf_hes, camera_buf_ves, cam_buff[write_idx]);
        if (ret != ESP_OK) {
            portENTER_CRITICAL(&ppa_spinlock);
            g_ppa_busy = false;
            portEXIT_CRITICAL(&ppa_spinlock);
            ESP_LOGW(TAG, "PPA frame process failed: %s", esp_err_to_name(ret));
            g_has_camera_frame = true;
            return;
        }
    }
    int64_t t_ppa_end = esp_timer_get_time();
#endif

    /* ================================================================= */
    /* 动态样品框检测                                                    */
    /* ================================================================= */
#if APP_ENABLE_SAMPLE_ROI
    if (g_sample_background_gray && g_vision_state == VISION_STATE_RUNNING) {
        const uint16_t *frame = (const uint16_t *)cam_buff[write_idx];

        if (g_sample_background_capture_requested || !g_sample_background_ready) {
            if (g_sample_background_delay_frames > 0) {
                g_sample_background_delay_frames--;
                ESP_LOGI(TAG, "[ROI] Stabilizing... %d", g_sample_background_delay_frames);
            } else {
                sample_roi_capture_background_rgb565(frame, g_sample_background_gray, CAM_W, CAM_H);
                g_sample_background_capture_requested = false;
                g_sample_background_ready = true;
                g_sample_present = false;
                s_sample_candidate_roi = {};
                s_sample_stable_frames = 0;
                s_sample_missing_frames = 0;
                ESP_LOGI(TAG, "[ROI] Background captured");
            }
        } else {
            sample_roi_t detected_roi = sample_roi_detect_rgb565(frame, g_sample_background_gray, CAM_W, CAM_H);
            bool was_present = g_sample_present;

            if (detected_roi.present) {
                s_sample_missing_frames = 0;
                if (sample_roi_is_similar(&detected_roi, &s_sample_candidate_roi)) {
                    s_sample_stable_frames++;
                } else {
                    s_sample_candidate_roi = detected_roi;
                    s_sample_stable_frames = 1;
                }

                if (s_sample_stable_frames >= SAMPLE_STABLE_FRAMES) {
                    g_sample_roi = detected_roi;
                    g_sample_present = true;
                }
            } else {
                s_sample_candidate_roi = {};
                s_sample_stable_frames = 0;
                if (s_sample_missing_frames < SAMPLE_MISSING_FRAMES) {
                    s_sample_missing_frames++;
                }
                if (s_sample_missing_frames >= SAMPLE_MISSING_FRAMES) {
                    g_sample_roi = {};
                    g_sample_present = false;
                }
            }

            if (!was_present && g_sample_present) {
                ESP_LOGI(TAG, "[ROI] Sample visible: x=%d y=%d w=%d h=%d",
                         g_sample_roi.x, g_sample_roi.y, g_sample_roi.w, g_sample_roi.h);
            } else if (was_present && !g_sample_present) {
                ESP_LOGI(TAG, "[ROI] Sample hidden");
            }
        }
    }
#endif

    int64_t t_total = esp_timer_get_time() - t_start;

    static int cam_frame_cnt = 0;
    cam_frame_cnt++;
    if (cam_frame_cnt % 60 == 0) {
        ESP_LOGI(TAG, "[CAM] total=%lldus PPA_submit=%lldus",
                 t_total,
                 (t_ppa_end - t_ppa_start));
    }
}
