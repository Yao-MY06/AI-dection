/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

/* ========================================================================== */
/* 头文件包含区                                                                */
/* ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <map>
#include <vector>       
#include <string>       
#include "sdkconfig.h"
#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_err.h"
#include "esp_log.h"
#include <fcntl.h>
#include <unistd.h>
#include "esp_private/esp_cache_private.h"
#include "app_video.h"
#include "esp_timer.h"

// 麦克风 FFT 与 GPIO 头文件
#include <math.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_dsp.h"

// FreeRTOS 任务与队列核心头文件
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#if SOC_PPA_SUPPORTED
#include "driver/ppa.h"
#endif

// [融合的 AI 核心头文件]
#include "dl_tensor_base.hpp"
#include "dl_model_base.hpp"
#include "dl_image.hpp"


// === SD卡与文件系统头文件 ===
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include <sys/stat.h>
#include <dirent.h>

// === 图像压缩头文件 ===
#include "esp_camera.h" // 包含 fmt2jpg 函数，用于 RGB565 转 JPEG
// 将 #include "esp_camera.h" 替换为或者追加：
#include "img_converters.h"
/* ========================================================================== */
/* 布局常量与宏定义区                                                          */
/* ========================================================================== */
#define SCREEN_W 1024                           // 屏幕总宽度
#define SCREEN_H 600                            // 屏幕总高度
#define TOP_H    60                             // 顶部全局状态栏高度
#define PANEL_W  300                            // 右侧操作面板宽度
#define BOTTOM_H 140                            // 底部面板总高度

// 动态计算各区域尺寸
#define CAM_W    (SCREEN_W - PANEL_W)           // 主视频区宽度 (724)
#define CAM_H    (SCREEN_H - TOP_H - BOTTOM_H)  // 主视频区高度 (400)
#define ACOUSTIC_W CAM_W                        // 声纹分析区宽度
#define ACOUSTIC_H BOTTOM_H                     // 声纹分析区高度
#define ALARM_W  PANEL_W                        // 报警结果区宽度
#define ALARM_H  BOTTOM_H                       // 报警结果区高度

#define NUM_BUFS 2                              // 摄像头缓冲帧数
#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1)) // 内存对齐宏

// AI 模型输入尺寸
#define AI_INPUT_W 640
#define AI_INPUT_H 640

/* ========================================================================== */
/* 系统状态与消息队列类型定义                                                   */
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

typedef enum {
    CMD_VISION_START = 0,
    CMD_VISION_STOP,
    CMD_AUDIO_START,
    CMD_AUDIO_STOP,
    CMD_BTN_EMERGENCY,
    CMD_BTN_CAPTURE,                            // UI指令: 发起抓拍
    CMD_BTN_CAPTURE_DONE,                       // 后台事件: 抓拍并保存完成
    CMD_BTN_CAPTURE_FAIL,                       // 后台事件: 抓拍保存失败
    EVENT_AI_DEFECT_FOUND,
    EVENT_AI_PASS                               // AI事件: 检测通过
} event_type_t;

typedef struct {
    event_type_t type;                          // 事件类型
    char log_msg[64];                           // 附带的日志信息
} system_event_t;

typedef struct {
    uint8_t *raw_img_buf;   // 数据长度
} capture_msg_t;

/* ========================================================================== */
/* 全局变量声明区                                                              */
/* ========================================================================== */
static const char *TAG = "AI_INSPECTOR";

static QueueHandle_t ui_event_queue = NULL;
static QueueHandle_t hw_cmd_queue = NULL;

static lv_obj_t *camera_canvas = NULL;      // 主视频 Canvas
static lv_obj_t *part_detect_box = NULL;      // 零件检测框（预留）
static lv_obj_t *log_box = NULL;              // 日志输出框（保留在右下面板内）

// 顶部全局状态栏句柄
static lv_obj_t *sys_status_lab = NULL;       // 系统状态
static lv_obj_t *vision_status_lab = NULL;    // 视觉状态
static lv_obj_t *audio_status_lab = NULL;     // 音频状态
static lv_obj_t *emergency_status_lab = NULL; // 紧急状态

// 视频画面顶部中央半透明状态条句柄
static lv_obj_t *ai_overlay_bar = NULL;
static lv_obj_t *ai_overlay_lab = NULL;

// 右侧操作面板按钮句柄（用于互斥禁用）
static lv_obj_t *b_vision_start = NULL;
static lv_obj_t *b_vision_stop = NULL;
static lv_obj_t *b_audio_start = NULL;
static lv_obj_t *b_audio_stop = NULL;
static lv_obj_t *b_emergency = NULL;
static lv_obj_t *b_capture = NULL;

// 底部声纹分析区句柄
static lv_obj_t *acoustic_chart = NULL;
static lv_chart_series_t *acoustic_chart_ser = NULL;
static lv_obj_t *acoustic_result_lab = NULL;     // 大状态文字
static lv_obj_t *acoustic_result_sub = NULL;     // 小状态说明

// 右下角报警与 AI 结果卡片句柄
static lv_obj_t *alarm_card = NULL;
static lv_obj_t *alarm_label = NULL;
static lv_obj_t *ai_result_card = NULL;
static lv_obj_t *ai_ng_bar = NULL;          // NG 进度条背景
static lv_obj_t *ai_ng_fill = NULL;         // NG 进度条填充
static lv_obj_t *ai_ok_bar = NULL;          // OK 进度条背景
static lv_obj_t *ai_ok_fill = NULL;         // OK 进度条填充
static lv_obj_t *ai_ng_label = NULL;
static lv_obj_t *ai_ok_label = NULL;

// 保留旧命名兼容（新布局中仍使用这些变量名，但指向新的 UI 对象）
static lv_obj_t *ai_status_bar = NULL;    // 兼容：实际指向视频顶部半透明状态条
static lv_obj_t *ai_status_lab = NULL;    // 兼容：实际指向视频顶部状态文字
static lv_obj_t *p_lab = NULL;            // 兼容：指向顶部 PASS 小卡片
static lv_obj_t *f_lab = NULL;            // 兼容：指向顶部 FAIL 小卡片
static lv_obj_t *y_lab = NULL;            // 兼容：指向顶部 YIELD 小卡片

// PPA 输出双缓冲索引（PPA 写 write_idx，LVGL/AI 读 read_idx）
#define CAM_BUF_IDX_NONE 0xFF
static volatile uint8_t g_ppa_write_idx = 0;
static volatile uint8_t g_ppa_read_idx = CAM_BUF_IDX_NONE;
static volatile uint8_t g_ai_reading_cam_buf_idx = CAM_BUF_IDX_NONE;
static volatile bool g_has_camera_frame = false;
static portMUX_TYPE cam_buf_spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_ppa_busy = false;

// === 麦克风 FFT 配置 ===
#define I2S_SCK_IO  ((gpio_num_t)21)
#define I2S_WS_IO   ((gpio_num_t)22)
#define I2S_SD_IO   ((gpio_num_t)20)
#define RELAY_PIN   ((gpio_num_t)23)

#define N_SAMPLES   1024
#define FFT_TAG     "MIC_FFT"

static i2s_chan_handle_t rx_chan = NULL;
__attribute__((aligned(16))) static float fft_y_cf[N_SAMPLES * 2];
__attribute__((aligned(16))) static float fft_window[N_SAMPLES];
static float audio_buffer[N_SAMPLES];

// 音频分析结果（用于 UI 显示，由 mic_fft_task 写）
static volatile float g_total_energy = 0.0f;
static volatile float g_bg_noise = 0.0f;
static volatile int   g_health_score = 0;
static volatile int   g_hit_pulse = 0;

static vision_state_t g_vision_state = VISION_STATE_STOPPED;
static audio_state_t g_audio_state = AUDIO_STATE_STOPPED;
static bool g_emergency_active = false;
static uint32_t count_pass = 0;
static uint32_t count_fail = 0;
static float g_ai_ng_pct = 0.0f;            // 最新 AI NG 概率
static float g_ai_ok_pct = 0.0f;            // 最新 AI OK 概率

static uint8_t *cam_buff[NUM_BUFS];
static uint32_t cam_buff_size = 0;

extern const uint8_t model_espdl_start[] asm("_binary_best_model_espdl_start");
static dl::Model *yolo_model = nullptr;

#if SOC_PPA_SUPPORTED
static ppa_client_handle_t ppa_srm_handle = NULL; // 提前声明 PPA 句柄
#endif


static QueueHandle_t sd_write_queue = NULL;
static const char *MOUNT_POINT = "/sdcard";

/* ========================================================================== */
/* LVGL 动画与基础回调                                                         */
/* ========================================================================== */
static void anim_opa_cb(void * var, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t *)var, v, 0);
}

/* ========================================================================== */
/* 视觉检测状态更新（顶部状态栏：图标+文字）                                     */
/* ========================================================================== */
static void update_vision_status(vision_state_t new_state) {
    g_vision_state = new_state;
    if (!vision_status_lab) return;

    lv_anim_delete(vision_status_lab, (lv_anim_exec_xcb_t)anim_opa_cb);

    switch (new_state) {
        case VISION_STATE_RUNNING:
            lv_label_set_text(vision_status_lab, LV_SYMBOL_VIDEO " VISION: RUNNING");
            lv_obj_set_style_text_color(vision_status_lab, lv_palette_main(LV_PALETTE_GREEN), 0);
            {
                lv_anim_t a;
                lv_anim_init(&a);
                lv_anim_set_var(&a, vision_status_lab);
                lv_anim_set_values(&a, LV_OPA_50, LV_OPA_COVER);
                lv_anim_set_duration(&a, 800);
                lv_anim_set_playback_duration(&a, 800);
                lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
                lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_opa_cb);
                lv_anim_start(&a);
            }
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
static void update_audio_status(audio_state_t new_state) {
    g_audio_state = new_state;
    if (!audio_status_lab) return;

    lv_anim_delete(audio_status_lab, (lv_anim_exec_xcb_t)anim_opa_cb);

    switch (new_state) {
        case AUDIO_STATE_RUNNING:
            lv_label_set_text(audio_status_lab, LV_SYMBOL_AUDIO " AUDIO: LISTENING");
            lv_obj_set_style_text_color(audio_status_lab, lv_color_hex(0x00D2FF), 0);
            {
                lv_anim_t a;
                lv_anim_init(&a);
                lv_anim_set_var(&a, audio_status_lab);
                lv_anim_set_values(&a, LV_OPA_50, LV_OPA_COVER);
                lv_anim_set_duration(&a, 800);
                lv_anim_set_playback_duration(&a, 800);
                lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
                lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_opa_cb);
                lv_anim_start(&a);
            }
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
/* 状态：IDLE / DETECTING / NG DETECTED / OK                                   */
/* ========================================================================== */
typedef enum {
    AI_OVERLAY_IDLE = 0,
    AI_OVERLAY_DETECTING,
    AI_OVERLAY_NG,
    AI_OVERLAY_OK
} ai_overlay_state_t;

static void update_ai_status_bar(ai_overlay_state_t state) {
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

static void update_ai_status_bar_bool(bool is_detecting) {
    update_ai_status_bar(is_detecting ? AI_OVERLAY_DETECTING : AI_OVERLAY_IDLE);
}

/* ========================================================================== */
/* 系统整体状态更新（顶部状态栏系统状态卡片）                                   */
/* ========================================================================== */
static void update_system_status(bool emergency) {
    g_emergency_active = emergency;
    if (!sys_status_lab) return;

    lv_anim_delete(sys_status_lab, (lv_anim_exec_xcb_t)anim_opa_cb);

    if (emergency) {
        lv_label_set_text(sys_status_lab, LV_SYMBOL_WARNING " SYSTEM ERROR");
        lv_obj_set_style_text_color(sys_status_lab, lv_color_white(), 0);
        lv_obj_set_style_bg_color(sys_status_lab, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_bg_opa(sys_status_lab, LV_OPA_COVER, 0);
        // 错误状态：红底白字+闪烁
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, sys_status_lab);
        lv_anim_set_values(&a, LV_OPA_60, LV_OPA_COVER);
        lv_anim_set_duration(&a, 400);
        lv_anim_set_playback_duration(&a, 400);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_opa_cb);
        lv_anim_start(&a);
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
static void update_emergency_status(bool active) {
    if (!emergency_status_lab) return;
    lv_anim_delete(emergency_status_lab, (lv_anim_exec_xcb_t)anim_opa_cb);

    if (active) {
        lv_label_set_text(emergency_status_lab, LV_SYMBOL_STOP " E-STOP: ACTIVE");
        lv_obj_set_style_text_color(emergency_status_lab, lv_color_white(), 0);
        lv_obj_set_style_bg_color(emergency_status_lab, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_set_style_bg_opa(emergency_status_lab, LV_OPA_COVER, 0);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, emergency_status_lab);
        lv_anim_set_values(&a, LV_OPA_50, LV_OPA_COVER);
        lv_anim_set_duration(&a, 400);
        lv_anim_set_playback_duration(&a, 400);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)anim_opa_cb);
        lv_anim_start(&a);
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
static void update_ai_result_card(float ng_pct, float ok_pct) {
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
static void update_yield_stats(void) {
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
/* FreeRTOS 多核任务区                                                         */
/* ========================================================================== */
static void btn_event_cb(lv_event_t * e) {
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
    lv_obj_t * label = (lv_obj_t *)lv_obj_get_child(btn, 0);
    const char * txt = lv_label_get_text(label);
    
    // CAPTURE 只发轻量事件到 ui_event_queue，重活由 ui_event_task 处理，不阻塞 UI
    if (strstr(txt, "CAPTURE")) {
        system_event_t ev = {};
        ev.type = CMD_BTN_CAPTURE;
        xQueueSend(ui_event_queue, &ev, 0);
        return;
    }

    // 其他常规指令继续走系统队列
    system_event_t ev = {};
    if (strstr(txt, "VISION") && strstr(txt, "START")) ev.type = CMD_VISION_START;
    else if (strstr(txt, "VISION") && strstr(txt, "STOP")) ev.type = CMD_VISION_STOP;
    else if (strstr(txt, "AUDIO") && strstr(txt, "START")) ev.type = CMD_AUDIO_START;
    else if (strstr(txt, "AUDIO") && strstr(txt, "STOP")) ev.type = CMD_AUDIO_STOP;
    else if (strstr(txt, "EMERGENCY")) ev.type = CMD_BTN_EMERGENCY;

    xQueueSend(ui_event_queue, &ev, 0);
}

void ui_event_task(void *pvParameters) 
{
    system_event_t ev;
    while (1) 
    {
        if (xQueueReceive(ui_event_queue, &ev, portMAX_DELAY) == pdPASS) {
            
            // 处理拍照请求（内存拷贝在锁外完成）
            if (ev.type == CMD_BTN_CAPTURE) {
                uint8_t buf_idx = g_ppa_read_idx;
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
                        if (b_vision_start) lv_obj_add_state(b_vision_start, LV_STATE_DISABLED);
                        if (b_vision_stop)  lv_obj_clear_state(b_vision_stop, LV_STATE_DISABLED);
                        if (b_audio_start) lv_obj_clear_state(b_audio_start, LV_STATE_DISABLED);
                        break;
                    case CMD_VISION_STOP:
                        update_vision_status(VISION_STATE_STOPPED);
                        update_ai_status_bar_bool(false);
                        if (b_vision_start) lv_obj_clear_state(b_vision_start, LV_STATE_DISABLED);
                        if (b_vision_stop)  lv_obj_add_state(b_vision_stop, LV_STATE_DISABLED);
                        break;
                    case CMD_AUDIO_START:
                        update_audio_status(AUDIO_STATE_RUNNING);
                        if (b_audio_start) lv_obj_add_state(b_audio_start, LV_STATE_DISABLED);
                        if (b_audio_stop)  lv_obj_clear_state(b_audio_stop, LV_STATE_DISABLED);
                        break;
                    case CMD_AUDIO_STOP:
                        update_audio_status(AUDIO_STATE_STOPPED);
                        if (b_audio_start) lv_obj_clear_state(b_audio_start, LV_STATE_DISABLED);
                        if (b_audio_stop)  lv_obj_add_state(b_audio_stop, LV_STATE_DISABLED);
                        break;
                    case CMD_BTN_EMERGENCY:
                        update_vision_status(VISION_STATE_EMERGENCY);
                        update_audio_status(AUDIO_STATE_EMERGENCY);
                        update_system_status(true);
                        update_ai_status_bar_bool(false);
                        // 禁用所有按钮，仅保留视觉反馈
                        if (b_vision_start) lv_obj_add_state(b_vision_start, LV_STATE_DISABLED);
                        if (b_vision_stop)  lv_obj_add_state(b_vision_stop, LV_STATE_DISABLED);
                        if (b_audio_start)  lv_obj_add_state(b_audio_start, LV_STATE_DISABLED);
                        if (b_audio_stop)   lv_obj_add_state(b_audio_stop, LV_STATE_DISABLED);
                        if (b_emergency)    lv_obj_add_state(b_emergency, LV_STATE_DISABLED);
                        if (b_capture)      lv_obj_add_state(b_capture, LV_STATE_DISABLED);
                        break;
                    case CMD_BTN_CAPTURE:
                        lv_textarea_add_text(log_box, ev.log_msg);
                        break;
                    case EVENT_AI_PASS:
                        count_pass++;
                        update_yield_stats();
                        update_ai_result_card(g_ai_ng_pct, g_ai_ok_pct);
                        break;
                    case EVENT_AI_DEFECT_FOUND:
                        count_fail++;
                        update_yield_stats();
                        update_ai_result_card(g_ai_ng_pct, g_ai_ok_pct);
                        lv_textarea_add_text(log_box, ev.log_msg);
                        break;
                    case CMD_BTN_CAPTURE_DONE:
                        lv_textarea_add_text(log_box, "\n[SD] Saved to SD Card!");
                        break;
                    case CMD_BTN_CAPTURE_FAIL:
                        lv_textarea_add_text(log_box, "\n[ERR] SD Write Failed!");
                        break;
                    default: break;
                }

                // 更新良率（需要访问 LVGL 对象，必须在锁内）
                update_yield_stats();

                bsp_display_unlock();   // ✅ 在这里解锁，与上面的 lock 配对
            }
        }
    }
}


/**
 * @brief [优化版] AI 视觉推理任务 - LUT 加速 + 缓存 input_map + 最新帧索引
 */
void ai_vision_task(void *pvParameters) {
    ESP_LOGI(TAG, "AI 任务启动，等待模型加载...");

    static int src_x_lut[AI_INPUT_W];
    static int src_y_lut[AI_INPUT_H];
    static uint8_t r5_to_r8[32];
    static uint8_t g6_to_g8[64];
    static uint8_t b5_to_b8[32];
    static bool lut_initialized = false;

    if (!lut_initialized) {
        float scale_x = (float)CAM_W / AI_INPUT_W;
        float scale_y = (float)CAM_H / AI_INPUT_H;
        for (int x = 0; x < AI_INPUT_W; x++) {
            src_x_lut[x] = (int)(x * scale_x);
            if (src_x_lut[x] >= CAM_W) src_x_lut[x] = CAM_W - 1;
        }
        for (int y = 0; y < AI_INPUT_H; y++) {
            src_y_lut[y] = (int)(y * scale_y);
            if (src_y_lut[y] >= CAM_H) src_y_lut[y] = CAM_H - 1;
        }
        for (int i = 0; i < 32; i++) r5_to_r8[i] = (i << 3) | (i >> 2);
        for (int i = 0; i < 64; i++) g6_to_g8[i] = (i << 2) | (i >> 4);
        for (int i = 0; i < 32; i++) b5_to_b8[i] = (i << 3) | (i >> 2);
        lut_initialized = true;
        ESP_LOGI(TAG, "AI LUT 预计算完成");
    }

    static std::map<std::string, dl::TensorBase*> s_input_map;
    static std::string s_input_name;
    static bool s_input_ready = false;

    while (1) {
        if (g_vision_state == VISION_STATE_RUNNING && yolo_model != nullptr && g_has_camera_frame) {
            auto &inputs = yolo_model->get_inputs();
            if (inputs.empty()) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            dl::TensorBase *input_tensor = inputs.begin()->second;

            if (!s_input_ready) {
                s_input_name = inputs.begin()->first;
                s_input_map[s_input_name] = input_tensor;
                s_input_ready = true;
                auto shape = input_tensor->get_shape();
                ESP_LOGI(TAG, "Model input shape: [%d,%d,%d,%d], dtype: %s",
                    shape[0], shape[1], shape[2], shape[3],
                    input_tensor->get_dtype() == dl::DATA_TYPE_FLOAT ? "FLOAT" : "OTHER");
            }

            uint8_t buf_idx = g_ppa_read_idx;
            if (buf_idx >= NUM_BUFS || cam_buff[buf_idx] == nullptr) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            bool is_float_input = (input_tensor->get_dtype() == dl::DATA_TYPE_FLOAT);
            portENTER_CRITICAL(&cam_buf_spinlock);
            g_ai_reading_cam_buf_idx = buf_idx;
            portEXIT_CRITICAL(&cam_buf_spinlock);
            uint16_t *src = (uint16_t *)cam_buff[buf_idx];

            int64_t t_preprocess_start = esp_timer_get_time();
            
            if (is_float_input) {
                float *dst_f = (float *)input_tensor->get_element_ptr();
                for (int y = 0; y < AI_INPUT_H; y++) {
                    int src_y = src_y_lut[y];
                    int dst_row_base = y * AI_INPUT_W * 3;
                    for (int x = 0; x < AI_INPUT_W; x++) {
                        int src_x = src_x_lut[x];
                        uint16_t pixel = src[src_y * CAM_W + src_x];
                        int dst_idx = dst_row_base + x * 3;
                        dst_f[dst_idx + 0] = r5_to_r8[(pixel >> 11) & 0x1F] / 255.0f;
                        dst_f[dst_idx + 1] = g6_to_g8[(pixel >> 5) & 0x3F] / 255.0f;
                        dst_f[dst_idx + 2] = b5_to_b8[pixel & 0x1F] / 255.0f;
                    }
                }
            } else {
                uint8_t *dst = (uint8_t *)input_tensor->get_element_ptr();
                for (int y = 0; y < AI_INPUT_H; y++) {
                    int src_y = src_y_lut[y];
                    int dst_row_base = y * AI_INPUT_W * 3;
                    for (int x = 0; x < AI_INPUT_W; x++) {
                        int src_x = src_x_lut[x];
                        uint16_t pixel = src[src_y * CAM_W + src_x];
                        int dst_idx = dst_row_base + x * 3;
                        dst[dst_idx + 0] = r5_to_r8[(pixel >> 11) & 0x1F];
                        dst[dst_idx + 1] = g6_to_g8[(pixel >> 5) & 0x3F];
                        dst[dst_idx + 2] = b5_to_b8[pixel & 0x1F];
                    }
                }
            }

            portENTER_CRITICAL(&cam_buf_spinlock);
            g_ai_reading_cam_buf_idx = CAM_BUF_IDX_NONE;
            portEXIT_CRITICAL(&cam_buf_spinlock);
            int64_t t_preprocess_end = esp_timer_get_time();

            int64_t t_infer_start = esp_timer_get_time();
            yolo_model->run(s_input_map);
            int64_t t_infer_end = esp_timer_get_time();

            static int ai_run_cnt = 0;
            ai_run_cnt++;
            if (ai_run_cnt % 10 == 0) {
                ESP_LOGI(TAG, "[AI] preprocess=%lldus inference=%lldus total=%lldus",
                         (t_preprocess_end - t_preprocess_start),
                         (t_infer_end - t_infer_start),
                         (t_infer_end - t_preprocess_start));
            }

            auto &outputs = yolo_model->get_outputs();
            if (!outputs.empty()) {
                dl::TensorBase *out_tensor = outputs.begin()->second;
                float *out_ptr = (float *)out_tensor->get_element_ptr();
                if (out_ptr) {
                    float normal_score = out_ptr[0];
                    float scratch_score = out_ptr[1];
                    
                    float max_val = fmaxf(normal_score, scratch_score);
                    float exp_normal = expf(normal_score - max_val);
                    float exp_scratch = expf(scratch_score - max_val);
                    float sum = exp_normal + exp_scratch;
                    float prob_normal = exp_normal / sum;
                    float prob_scratch = exp_scratch / sum;

                    ESP_LOGI(TAG, "AI: normal=%.3f scratch=%.3f", prob_normal, prob_scratch);

                    system_event_t ev = {};
                    g_ai_ng_pct = prob_scratch * 100.0f;
                    g_ai_ok_pct = prob_normal * 100.0f;
                    if (prob_scratch > 0.60f) {
                        ev.type = EVENT_AI_DEFECT_FOUND;
                        snprintf(ev.log_msg, sizeof(ev.log_msg),
                                 "\n[AI] NG=%.1f%% OK=%.1f%% → FAIL", 
                                 g_ai_ng_pct, g_ai_ok_pct);
                        xQueueSend(hw_cmd_queue, &ev, 0);
                    } else {
                        ev.type = EVENT_AI_PASS;
                        snprintf(ev.log_msg, sizeof(ev.log_msg),
                                 "\n[AI] NG=%.1f%% OK=%.1f%% → PASS",
                                 g_ai_ng_pct, g_ai_ok_pct);
                    }
                    xQueueSend(ui_event_queue, &ev, 0);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}


void hardware_control_task(void *pvParameters) {
    system_event_t hw_ev;
    while (1) {
        if (xQueueReceive(hw_cmd_queue, &hw_ev, portMAX_DELAY) == pdPASS) {
            if (hw_ev.type == EVENT_AI_DEFECT_FOUND) {
                ESP_LOGI("HARDWARE", "▶ Activating Rejecter Relay...");
                vTaskDelay(pdMS_TO_TICKS(200)); 
                ESP_LOGI("HARDWARE", "⏹ Rejecter Reset.");
            }
        }
    }
}

/* ========================================================================== */
/* 麦克风 I2S 与 FFT 声学分析任务                                               */
/* ========================================================================== */
static void i2s_mic_init(void)
{
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000), 
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO), 
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    
            .bclk = I2S_SCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI(FFT_TAG, "I2S mic initialized");
}

void striker_task(void *pvParameters) {
    bool relay_available = false;

    esp_err_t ret = gpio_reset_pin((gpio_num_t)RELAY_PIN);
    if (ret == ESP_OK) {
        ret = gpio_set_direction((gpio_num_t)RELAY_PIN, GPIO_MODE_OUTPUT);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_level((gpio_num_t)RELAY_PIN, 0);
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Relay not connected (GPIO %d init failed: %s)", RELAY_PIN, esp_err_to_name(ret));
    } else {
        relay_available = true;
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    while (1) {
        if (g_vision_state == VISION_STATE_RUNNING || g_audio_state == AUDIO_STATE_RUNNING) {
            if (relay_available) {
                gpio_set_level((gpio_num_t)RELAY_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(20));
                gpio_set_level((gpio_num_t)RELAY_PIN, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        } else {
            if (relay_available) {
                gpio_set_level((gpio_num_t)RELAY_PIN, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void mic_fft_task(void *pvParameters) {
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(FFT_TAG, "FFT init failed!");
        vTaskDelete(NULL);
    }
    dsps_wind_hann_f32(fft_window, N_SAMPLES);
    ESP_LOGI(FFT_TAG, "FFT engine ready");

    size_t bytes_read = 0;
    int32_t raw_samples[64];
    int64_t dc_offset = 0;
    int buffer_idx = 0;

    static float env_mid = 0;
    static float env_high = 0;
    static float bg_noise = 1.0f;
    static float health_score = 0;
    static int hit_pulse = 0;
    static int print_throttle = 0;

    // === 频谱质心 + 智能防抖状态机 ===
    static float locked_score = 0.0f;       // 当前锁定的材质分数
    static int stability_counter = 0;       // 连续接近计数器
    static int hit_cooldown = 0;            // 撞击冷却计时器
    static bool has_locked = false;         // 是否已完成首次锁定

    #define HIT_COOLDOWN_FRAMES     8       // 约 64ms 冷却窗口
    #define STABILITY_THRESHOLD     25.0f   // 判定为同一材质的差值阈值
    #define STABILITY_COUNT         3       // 连续 N 次接近才完全锁定
    #define JUMP_THRESHOLD          80.0f   // 突变阈值，立刻切换材质

    while (1) {
        if (i2s_channel_read(rx_chan, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY) == ESP_OK) {
            int sample_count = bytes_read / sizeof(int32_t);
            for (int i = 0; i < sample_count; i++) {
                int32_t raw_val = raw_samples[i] >> 8;
                dc_offset = (dc_offset * 63 + raw_val) / 64;
                int32_t clean_val = raw_val - (int32_t)dc_offset;
                audio_buffer[buffer_idx] = (float)clean_val;
                buffer_idx++;

                if (buffer_idx >= N_SAMPLES) {
                    int64_t t_fft_start = esp_timer_get_time();
                    for (int k = 0; k < N_SAMPLES; k++) {
                        fft_y_cf[k * 2 + 0] = audio_buffer[k] * fft_window[k];
                        fft_y_cf[k * 2 + 1] = 0;
                    }

                    dsps_fft2r_fc32(fft_y_cf, N_SAMPLES);
                    dsps_bit_rev_fc32(fft_y_cf, N_SAMPLES);

                    // === 1. 频谱质心计算 + 分频段统计 ===
                    float weighted_sum = 0.0f;
                    float total_energy = 0.0f;
                    float current_low = 0, current_mid = 0, current_high = 0;

                    for (int k = 0; k < N_SAMPLES / 2; k++) {
                        float re = fft_y_cf[k * 2 + 0];
                        float im = fft_y_cf[k * 2 + 1];
                        float magnitude = sqrtf(re * re + im * im);
                        float real_amp = magnitude / 512.0f;

                        // 保留原有分频段统计（用于撞击检测与调试）
                        if (k < 32)       current_low += real_amp;
                        else if (k < 192) current_mid += real_amp;
                        else              current_high += real_amp;

                        // 频谱质心：能量加权平均频率
                        weighted_sum += k * real_amp;
                        total_energy += real_amp;
                    }

                    current_low /= 32.0f;
                    current_mid /= 160.0f;
                    current_high /= 320.0f;
                    float band_total = current_low + current_mid + current_high;

                    // === 2. 包络线与环境底噪跟踪 ===
                    env_mid = (current_mid > env_mid) ? current_mid : env_mid * 0.90f;
                    env_high = (current_high > env_high) ? current_high : env_high * 0.90f;

                    if (band_total < bg_noise) {
                        bg_noise = bg_noise * 0.80f + band_total * 0.20f;
                    } else {
                        bg_noise = bg_noise * 0.98f + band_total * 0.02f;
                    }

                    // === 3. 撞击检测 ===
                    if (band_total > bg_noise * 3.0f && band_total > 0.5f) {
                        hit_pulse = 50;
                    } else {
                        hit_pulse = 0;
                    }

                    // === 4. 撞击瞬间抓取：频谱质心材质识别 ===
                    float raw_score = 0.0f;
                    bool score_valid = false;

                    if (hit_pulse > 0 && hit_cooldown == 0) {
                        if (total_energy > 1e-6f) {
                            raw_score = weighted_sum / total_energy;  // Spectral Centroid
                            score_valid = true;
                        }
                        hit_cooldown = HIT_COOLDOWN_FRAMES;  // 启动冷却，忽略声音拖尾
                    }

                    if (hit_cooldown > 0) {
                        hit_cooldown--;
                    }

                    // === 5. 阶跃式防抖锁定 ===
                    if (score_valid) {
                        if (!has_locked) {
                            // 首次有效撞击，直接锁定
                            locked_score = raw_score;
                            has_locked = true;
                            stability_counter = 0;
                        } else {
                            float diff = fabsf(raw_score - locked_score);
                            if (diff > JUMP_THRESHOLD) {
                                // 突变超过阈值：立刻判定换材质，瞬间切换并重新锁定
                                locked_score = raw_score;
                                stability_counter = 0;
                            } else if (diff < STABILITY_THRESHOLD) {
                                // 连续分数接近：判定为同一物体，UI 平滑锁定
                                stability_counter++;
                                locked_score = locked_score * 0.7f + raw_score * 0.3f;
                            } else {
                                // 中等变化：重置稳定计数，暂不切换
                                stability_counter = 0;
                            }
                        }
                        health_score = locked_score;
                    }

                    g_total_energy = band_total;
                    g_bg_noise = bg_noise;
                    g_health_score = (int)health_score;
                    g_hit_pulse = hit_pulse;

                    print_throttle++;
                    if (print_throttle >= 3) {
                        printf(">Total_Energy:%.2f\n", band_total);
                        printf(">Bg_Noise:%.2f\n", bg_noise);
                        printf(">Health_Score:%.1f (locked: %.1f, raw: %.1f)\n",
                               health_score, locked_score, score_valid ? raw_score : 0.0f);
                        printf(">Hit_Event:%d\n", hit_pulse);
                        size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
                        size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
                        printf(">PSRAM:%u/%uKB (free/total)\n",
                               (unsigned)(psram_free / 1024), (unsigned)(psram_total / 1024));
                        print_throttle = 0;
                        int64_t t_fft_end = esp_timer_get_time();
                        printf(">FFT_UI:%lldus\n", (t_fft_end - t_fft_start));

                        if (acoustic_chart && acoustic_chart_ser && acoustic_result_lab) {
                            
                            // 【关键修复】：只有成功获取到锁，才进行 UI 操作！
                            if (bsp_display_lock(pdMS_TO_TICKS(5))) {
                                
                                // 更新 Health Score 波形图
                                lv_chart_set_next_value(acoustic_chart, acoustic_chart_ser, (int32_t)health_score);
                                lv_chart_refresh(acoustic_chart);

                                // 声纹状态判定（文字 + 波形颜色）
                                if (hit_pulse > 0) {
                                    if (health_score > 50.0f && health_score < 320.0f) {
                                        lv_label_set_text(acoustic_result_lab, "SIGNAL OK");
                                        lv_obj_set_style_text_color(acoustic_result_lab, lv_palette_main(LV_PALETTE_GREEN), 0);
                                        lv_label_set_text(acoustic_result_sub, "ACOUSTIC NORMAL");
                                        lv_obj_set_style_text_color(acoustic_result_sub, lv_palette_main(LV_PALETTE_GREEN), 0);
                                        lv_obj_set_style_line_color(acoustic_chart, lv_palette_main(LV_PALETTE_GREEN), LV_PART_ITEMS);
                                    } else {
                                        lv_label_set_text(acoustic_result_lab, "SIGNAL NG");
                                        lv_obj_set_style_text_color(acoustic_result_lab, lv_palette_main(LV_PALETTE_RED), 0);
                                        lv_label_set_text(acoustic_result_sub, "ABNORMAL SOUND");
                                        lv_obj_set_style_text_color(acoustic_result_sub, lv_palette_main(LV_PALETTE_RED), 0);
                                        lv_obj_set_style_line_color(acoustic_chart, lv_palette_main(LV_PALETTE_RED), LV_PART_ITEMS);
                                    }
                                } else if (band_total < bg_noise * 1.5f) {
                                    lv_label_set_text(acoustic_result_lab, "WAIT NO HIT");
                                    lv_obj_set_style_text_color(acoustic_result_lab, lv_color_hex(0x888888), 0);
                                    lv_label_set_text(acoustic_result_sub, "MONITORING");
                                    lv_obj_set_style_text_color(acoustic_result_sub, lv_color_hex(0x888888), 0);
                                    lv_obj_set_style_line_color(acoustic_chart, lv_palette_main(LV_PALETTE_GREEN), LV_PART_ITEMS);
                                }

                                // 【关键修复】：解锁必须放在 if(bsp_display_lock) 的内部！
                                bsp_display_unlock();
                            }
                        }
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    buffer_idx = 0;
                }
            }
        }
    }
}


void sd_writer_task(void *pvParameters) {
    capture_msg_t msg;
    char file_path[64];

    while (1) {
        if (xQueueReceive(sd_write_queue, &msg, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "Saving RAW image to SD card...");

    // 扫描 SD 卡已有图片，从最大编号继续，避免覆盖
    static uint32_t img_cnt = 0;
    static bool cnt_initialized = false;
    if (!cnt_initialized) {
        cnt_initialized = true;
        char img_dir[32];
        snprintf(img_dir, sizeof(img_dir), "%s/images", MOUNT_POINT);
        DIR *dir = opendir(img_dir);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                // FatFS 可能返回大写的 8.3 格式文件名，先转小写再匹配
                char name_lower[64];
                strncpy(name_lower, entry->d_name, sizeof(name_lower) - 1);
                name_lower[sizeof(name_lower) - 1] = '\0';
                for (char *p = name_lower; *p; p++) {
                    if (*p >= 'A' && *p <= 'Z') *p += ('a' - 'A');
                }
                uint32_t num = 0;
                if (sscanf(name_lower, "pic%4lu.raw", &num) == 1) {
                    if (num > img_cnt) img_cnt = num;
                }
            }
            closedir(dir);
            ESP_LOGI(TAG, "SD card existing images found, starting from pic%04lu.raw", img_cnt + 1);
        } else {
            ESP_LOGW(TAG, "Failed to open %s for scanning", img_dir);
        }
    }
            snprintf(file_path, sizeof(file_path), "%s/images/pic%04lu.raw", MOUNT_POINT, ++img_cnt);
            
            FILE *f = fopen(file_path, "wb");

            if (f != NULL) {
                size_t write_size = CAM_W * CAM_H * 2;
                size_t chunk_size = 128 * 1024;
                size_t written = 0;
                uint8_t *ptr = msg.raw_img_buf;

                while (written < write_size) {
                    size_t to_write = (write_size - written < chunk_size) ? (write_size - written) : chunk_size;
                    size_t actually_written = fwrite(ptr, 1, to_write, f);
                    if (actually_written != to_write) {
                        ESP_LOGE(TAG, "SD write error at offset %u", written);
                        break;
                    }
                    ptr += to_write;
                    written += to_write;
                }

                // 确保数据从 C 库缓冲区和 FatFS 层全部刷入物理 SD 卡
                fflush(f);
                fsync(fileno(f));
                fclose(f);

                // SD 卡内部写缓存需要时间完成，否则下次 fopen 会失败
                vTaskDelay(pdMS_TO_TICKS(200));

                if (written == write_size) {
                    ESP_LOGI(TAG, "Image saved: %s", file_path);
                    system_event_t ui_ev = {};
                    ui_ev.type = CMD_BTN_CAPTURE_DONE;
                    xQueueSend(ui_event_queue, &ui_ev, 0);
                } else {
                    ESP_LOGE(TAG, "Image write incomplete: %s", file_path);
                    system_event_t ui_ev = {};
                    ui_ev.type = CMD_BTN_CAPTURE_FAIL;
                    xQueueSend(ui_event_queue, &ui_ev, 0);
                }
            } else {
                ESP_LOGE(TAG, "Failed to open file for writing: %s", file_path);
                system_event_t ui_ev = {};
                ui_ev.type = CMD_BTN_CAPTURE_FAIL;
                xQueueSend(ui_event_queue, &ui_ev, 0);

                // fopen 失败后也加延时，给 SD 卡恢复时间
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            if (msg.raw_img_buf != NULL) {
                heap_caps_free(msg.raw_img_buf);
                msg.raw_img_buf = NULL;
            }
        }
    }
}

/* ========================================================================== */
/* PPA 硬件图像处理 (给屏幕用的，完全保留 v2.txt 的正确逻辑)                      */
/* ========================================================================== */
#if SOC_PPA_SUPPORTED
static bool app_ppa_done_cb(ppa_client_handle_t ppa_client, ppa_event_data_t *event_data, void *user_data)
{
    // PPA 完成后，更新可读 buffer 索引，并交换写索引
    g_ppa_read_idx = g_ppa_write_idx;
    g_ppa_write_idx = (g_ppa_write_idx + 1) % NUM_BUFS;
    g_ppa_busy = false;
    g_has_camera_frame = true;
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
    // 保持 CAM_W/CAM_H 的宽高比，从摄像头画面中居中裁剪，避免无意义的 *2 大裁剪块
    uint32_t crop_w = in_w;
    uint32_t crop_h = in_h;

    // 计算保持显示区域宽高比的裁剪尺寸
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
    srm_config.mode = PPA_TRANS_MODE_NON_BLOCKING;
    
    return ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
}
#endif

static void display_refresh_task(void *pvParameters)
{
    uint8_t last_read_idx = CAM_BUF_IDX_NONE;
    while (1) {
        uint8_t read_idx = g_ppa_read_idx;
        if (read_idx != last_read_idx && read_idx < NUM_BUFS && cam_buff[read_idx] != nullptr) {
            last_read_idx = read_idx;
            if (bsp_display_lock(pdMS_TO_TICKS(5))) {
                if (camera_canvas) {
                    lv_canvas_set_buffer(camera_canvas, cam_buff[read_idx], CAM_W, CAM_H, LV_COLOR_FORMAT_RGB565);
                }
                bsp_display_unlock();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(33));  // 约 30fps 刷新
    }
}

static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, uint32_t camera_buf_hes,
                                        uint32_t camera_buf_ves, size_t camera_buf_len) {
    if (g_emergency_active) return;

    // 如果上一帧 PPA 还没完成，跳过本帧
    if (g_ppa_busy) {
        return;
    }

    // 如果 AI 正在读取目标 buffer，跳过本帧
    portENTER_CRITICAL(&cam_buf_spinlock);
    bool ai_reading = (g_ppa_write_idx == g_ai_reading_cam_buf_idx);
    portEXIT_CRITICAL(&cam_buf_spinlock);
    if (ai_reading) return;

    int64_t t_start = esp_timer_get_time();
    uint8_t *out_buf = camera_buf;
#if SOC_PPA_SUPPORTED
    int64_t t_ppa_start = esp_timer_get_time();
    if (ppa_srm_handle) {
        g_ppa_busy = true;
        esp_err_t ret = app_image_process(camera_buf, camera_buf_hes, camera_buf_ves, cam_buff[g_ppa_write_idx]);
        if (ret != ESP_OK) {
            g_ppa_busy = false;
            ESP_LOGW(TAG, "PPA frame process failed: %s", esp_err_to_name(ret));
            return;
        }
        out_buf = cam_buff[g_ppa_write_idx];
    }
    int64_t t_ppa_end = esp_timer_get_time();
#endif

    // PPA 非阻塞：实际显示由 display_refresh_task 在 done_cb 后更新
    int64_t t_lvgl_start = esp_timer_get_time();
    int64_t t_lvgl_end = esp_timer_get_time();
    int64_t t_total = esp_timer_get_time() - t_start;

    static int cam_frame_cnt = 0;
    cam_frame_cnt++;
    if (cam_frame_cnt % 60 == 0) {
        ESP_LOGI(TAG, "[CAM] total=%lldus PPA_submit=%lldus",
                 t_total,
                 (t_ppa_end - t_ppa_start));
    }
}




/* ========================================================================== */
/* 应用主入口 (app_main) [完整 UI 恢复]                                        */
/* ========================================================================== */
extern "C" void app_main(void) {
    // 在 app_main() 函数开头处：
    ui_event_queue = xQueueCreate(15, sizeof(system_event_t));

    // 【修改点 3】：把 5 改成 15。15张图约占用 11MB PSRAM，对于连续抓拍完全够用。
    sd_write_queue = xQueueCreate(15, sizeof(capture_msg_t)); 

    hw_cmd_queue = xQueueCreate(5, sizeof(system_event_t));
    if (!ui_event_queue || !hw_cmd_queue) {
        ESP_LOGE(TAG, "关键队列创建失败，系统终止");
        return;
    }

    // 启动时打印内存信息
    ESP_LOGI(TAG, "=== Memory Info ===");
    ESP_LOGI(TAG, "PSRAM total: %u KB, free: %u KB, largest free block: %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
    ESP_LOGI(TAG, "DRAM  total: %u KB, free: %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    ESP_LOGI(TAG, "Mounting SD Card via BSP...");
    esp_err_t sd_ret = bsp_sdcard_mount();
    
    if (sd_ret != ESP_OK) {
        // 如果挂载失败，只打印错误日志，不让系统崩溃
        ESP_LOGE(TAG, "SD Card mount failed (0x%x). System will continue without saving images.", sd_ret);
    } else {
        // 如果挂载成功，再执行文件夹创建操作
        ESP_LOGI(TAG, "SD card mounted at /sdcard");
        struct stat st;
        if (stat("/sdcard/images", &st) == -1) {
            mkdir("/sdcard/images", 0777);
        }
    }

    // 启动 SD 写入任务
    // Change the 4096 to 8192
    xTaskCreatePinnedToCore(sd_writer_task, "SD_Task", 8192, NULL, 5, NULL, 1);

    bsp_display_start();
    bsp_display_backlight_on();

    // 内存对齐与 PPA/摄像头双缓冲分配
    size_t data_cache_line_size = 0;
    ESP_ERROR_CHECK(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size));
    cam_buff_size = ALIGN_UP(CAM_W * CAM_H * 2, data_cache_line_size);
    for (int i = 0; i < NUM_BUFS; i++) {
        cam_buff[i] = (uint8_t *)heap_caps_aligned_calloc(data_cache_line_size, 1, cam_buff_size, MALLOC_CAP_SPIRAM);
        if (!cam_buff[i]) {
            ESP_LOGE(TAG, "摄像头 Buffer %d PSRAM 分配失败 (size=%lu)", i, (unsigned long)cam_buff_size);
            return;
        }
    }

#if SOC_PPA_SUPPORTED
    app_ppa_init();
#endif

    // [AI 脑部植入]
    yolo_model = new dl::Model((const char *)model_espdl_start, fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    if (!yolo_model) {
        ESP_LOGE(TAG, "AI 模型加载失败，系统终止");
        return;
    }

    // ----------------------------------------------------------------------
    // LVGL 工业面板 UI 搭建（生产检测设备优化布局）
    // ----------------------------------------------------------------------
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
    lv_obj_align(audio_status_lab, LV_ALIGN_LEFT_MID, 540, 0);

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
    lv_obj_align(emergency_status_lab, LV_ALIGN_RIGHT_MID, -10, 0);

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

    // 画面顶部中央半透明状态条
    ai_overlay_bar = lv_obj_create(scr);
    ai_status_bar = ai_overlay_bar; // 兼容
    lv_obj_set_size(ai_overlay_bar, 300, 34);
    lv_obj_align(ai_overlay_bar, LV_ALIGN_TOP_MID, 0, TOP_H + 12);
    lv_obj_set_style_bg_color(ai_overlay_bar, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ai_overlay_bar, LV_OPA_70, 0);
    lv_obj_set_style_radius(ai_overlay_bar, 17, 0);
    lv_obj_set_style_border_width(ai_overlay_bar, 0, 0);
    lv_obj_set_style_pad_all(ai_overlay_bar, 0, 0);

    ai_overlay_lab = lv_label_create(ai_overlay_bar);
    ai_status_lab = ai_overlay_lab; // 兼容
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
    lv_obj_set_style_border_color(part_detect_box, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_border_width(part_detect_box, 2, 0);
    lv_obj_set_style_radius(part_detect_box, 4, 0);
    lv_obj_add_flag(part_detect_box, LV_OBJ_FLAG_HIDDEN);

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

    b_audio_stop = create_btn(rp, LV_SYMBOL_STOP " AUDIO STOP", lv_color_hex(0xCC8800), lv_color_black(), 282, 38);
    lv_obj_add_state(b_audio_stop, LV_STATE_DISABLED);
    lv_obj_add_event_cb(b_audio_stop, btn_event_cb, LV_EVENT_CLICKED, NULL);

    add_divider(rp, 330);

    // 4) 辅助操作
    b_capture = create_btn(rp, LV_SYMBOL_SAVE " CAPTURE", lv_color_hex(0x0088FF), lv_color_white(), 346, 38);
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

    // 报警信息卡片
    alarm_card = lv_obj_create(alarm_cont);
    lv_obj_set_size(alarm_card, ALARM_W - 30, 56);
    lv_obj_align(alarm_card, LV_ALIGN_TOP_MID, 0, 8);
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
    lv_obj_set_size(ai_result_card, ALARM_W - 30, 62);
    lv_obj_align(ai_result_card, LV_ALIGN_TOP_MID, 0, 72);
    lv_obj_set_style_bg_color(ai_result_card, lv_color_hex(0x222228), 0);
    lv_obj_set_style_bg_opa(ai_result_card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ai_result_card, 8, 0);
    lv_obj_set_style_border_width(ai_result_card, 0, 0);

    ai_ng_label = lv_label_create(ai_result_card);
    lv_label_set_text(ai_ng_label, "AI NG: 0.0%");
    lv_obj_set_style_text_font(ai_ng_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(ai_ng_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(ai_ng_label, LV_ALIGN_TOP_LEFT, 8, 6);

    // NG 进度条：背景 + 填充矩形
    ai_ng_bar = lv_obj_create(ai_result_card);
    lv_obj_set_size(ai_ng_bar, ALARM_W - 46, 8);
    lv_obj_align(ai_ng_bar, LV_ALIGN_TOP_LEFT, 8, 24);
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
    lv_obj_align(ai_ok_label, LV_ALIGN_TOP_LEFT, 8, 36);

    // OK 进度条：背景 + 填充矩形
    ai_ok_bar = lv_obj_create(ai_result_card);
    lv_obj_set_size(ai_ok_bar, ALARM_W - 46, 8);
    lv_obj_align(ai_ok_bar, LV_ALIGN_TOP_LEFT, 8, 54);
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

    // 日志框（保留在右下角卡片下方，用于调试信息）
    log_box = lv_textarea_create(alarm_cont);
    lv_obj_set_size(log_box, ALARM_W - 30, 0); // 高度为 0：保留句柄但不再显示
    lv_obj_add_flag(log_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(log_box, lv_color_black(), 0);
    lv_obj_set_style_text_color(log_box, lv_palette_main(LV_PALETTE_LIGHT_GREEN), 0);

    update_system_status(false);
    update_vision_status(VISION_STATE_STOPPED);
    update_audio_status(AUDIO_STATE_STOPPED);
    update_emergency_status(false);
    update_ai_status_bar_bool(false);
    bsp_display_unlock();

    // ----------------------------------------------------------------------
    // 启动 FreeRTOS 多核硬件调度机制
    // ----------------------------------------------------------------------
    if (xTaskCreatePinnedToCore(display_refresh_task, "DispRefresh", 4096, NULL, 6, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Display refresh task create failed");
        return;
    }
    if (xTaskCreatePinnedToCore(ui_event_task, "UI_Task", 4096 * 2, NULL, 7, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "UI task create failed");
        return;
    }
    if (xTaskCreatePinnedToCore(ai_vision_task, "AI_Task", 8192, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "AI task create failed");
        return;
    }
    if (xTaskCreatePinnedToCore(hardware_control_task, "HW_Task", 2048, NULL, 3, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "HW task create failed");
        return;
    }

    // 启动麦克风 I2S 与 FFT 声学监测
    i2s_mic_init();
    if (xTaskCreatePinnedToCore(striker_task, "striker", 2048, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "striker task create failed");
        return;
    }
    if (xTaskCreatePinnedToCore(mic_fft_task, "mic_fft", 8192, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "mic FFT task create failed");
        return;
    }

    // 启动摄像头底层硬件中断驱动
    ESP_ERROR_CHECK(bsp_camera_start(NULL));
    int fd = app_video_open((char *)BSP_CAMERA_DEVICE, APP_VIDEO_FMT_RGB565);
    if (fd < 0) {
        ESP_LOGE(TAG, "摄像头设备打开失败，系统终止");
        return;
    }
    ESP_ERROR_CHECK(app_video_set_bufs(fd, NUM_BUFS, NULL));
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(camera_video_frame_operation));
    ESP_ERROR_CHECK(app_video_stream_task_start(fd, 0));
}
