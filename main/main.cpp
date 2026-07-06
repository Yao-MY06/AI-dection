/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "sdkconfig.h"
#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"
#include "app_video.h"
#include "audio_capture.h"
#include "audio_analysis.h"
#include "ai_vision.h"
#include "hardware_control.h"
#include "striker.h"
#include "sd_storage.h"
#include "ui_manager.h"
#include "video_pipeline.h"
#include "lighting.h"
#include "servo.h"

/* ========================================================================== */
/* 全局变量定义                                                               */
/* ========================================================================== */
const char *TAG = "AI_INSPECTOR";

QueueHandle_t ui_event_queue = NULL;
QueueHandle_t hw_cmd_queue = NULL;
QueueHandle_t audio_cap_queue = NULL;
QueueHandle_t audio_trigger_queue = NULL;
QueueHandle_t striker_trigger_queue = NULL;
QueueHandle_t sd_write_queue = NULL;

vision_state_t g_vision_state = VISION_STATE_STOPPED;
audio_state_t  g_audio_state = AUDIO_STATE_STOPPED;
bool           g_emergency_active = false;
bool           g_servo_reversed = false;

uint32_t count_pass = 0;
uint32_t count_fail = 0;
float    g_ai_ng_pct = 0.0f;
float    g_ai_ok_pct = 0.0f;

volatile float g_total_energy = 0.0f;
volatile float g_bg_noise = 0.0f;
volatile int   g_health_score = 0;
volatile int   g_hit_pulse = 0;

uint8_t  *cam_buff[NUM_BUFS] = {};
uint32_t  cam_buff_size = 0;

volatile uint8_t  g_ppa_write_idx = 0;
volatile uint8_t  g_ppa_read_idx = CAM_BUF_IDX_NONE;
volatile uint8_t  g_ai_reading_cam_buf_idx = CAM_BUF_IDX_NONE;
volatile bool     g_has_camera_frame = false;
portMUX_TYPE      cam_buf_spinlock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE      ppa_spinlock = portMUX_INITIALIZER_UNLOCKED;
volatile bool     g_ppa_busy = false;
volatile uint32_t g_ppa_submit_cnt = 0;
volatile uint32_t g_ppa_done_cnt = 0;

/* ========================================================================== */
/* 动态样品框检测状态                                                         */
/* ========================================================================== */
sample_roi_t g_sample_roi = {};
volatile bool g_sample_present = false;
volatile bool g_sample_background_ready = false;
volatile bool g_sample_background_capture_requested = false;
volatile int g_sample_background_delay_frames = 0;
uint8_t *g_sample_background_gray = NULL;

/* ========================================================================== */
/* 初始化阶段与资源清理                                                        */
/* ========================================================================== */
typedef enum {
    INIT_STAGE_NONE = 0,
    INIT_STAGE_QUEUES,          // 队列已创建
    INIT_STAGE_SD_MOUNTED,      // SD 卡已挂载
    INIT_STAGE_DISPLAY,         // 显示已启动
    INIT_STAGE_CAM_BUFFERS,     // 摄像头缓冲已分配
    INIT_STAGE_PPA,             // PPA 已初始化
    INIT_STAGE_AI_MODEL,        // AI 模型已加载
    INIT_STAGE_UI,              // UI 已创建
    INIT_STAGE_VIDEO_STREAM,    // 视频流已启动
} init_stage_t;

typedef struct {
    init_stage_t stage;
    bool         sd_mounted;
    int          video_fd;
} init_context_t;

static void system_cleanup(init_context_t *ctx) {
    ESP_LOGW(TAG, "Initialization failed at stage %d, cleaning up...", (int)ctx->stage);

    // 停止视频流并关闭设备
    if (ctx->stage >= INIT_STAGE_VIDEO_STREAM && ctx->video_fd >= 0) {
        app_video_stream_task_stop(ctx->video_fd);
        app_video_wait_video_stop();
        app_video_close(ctx->video_fd);
        ctx->video_fd = -1;
    }

    // AI 模型反初始化（如果 esp-dl 支持）
    // 当前 esp-dl Model 为 new 创建，无统一析构接口，暂不处理

    // 释放 PSRAM 摄像头缓冲区
    if (ctx->stage >= INIT_STAGE_CAM_BUFFERS) {
        for (int i = 0; i < NUM_BUFS; i++) {
            if (cam_buff[i]) {
                heap_caps_free(cam_buff[i]);
                cam_buff[i] = NULL;
            }
        }
    }

    // 关闭显示与背光
    if (ctx->stage >= INIT_STAGE_DISPLAY) {
        bsp_display_backlight_off();
        // bsp_display_stop() 部分 BSP 未提供，依赖系统复位
    }

    // 卸载 SD 卡
    if (ctx->sd_mounted) {
        bsp_sdcard_unmount();
        ctx->sd_mounted = false;
    }

    // 删除队列
    if (ctx->stage >= INIT_STAGE_QUEUES) {
        if (ui_event_queue)        { vQueueDelete(ui_event_queue);        ui_event_queue = NULL; }
        if (sd_write_queue)        { vQueueDelete(sd_write_queue);        sd_write_queue = NULL; }
        if (hw_cmd_queue)          { vQueueDelete(hw_cmd_queue);          hw_cmd_queue = NULL; }
        if (audio_cap_queue)       { vQueueDelete(audio_cap_queue);       audio_cap_queue = NULL; }
        if (audio_trigger_queue)   { vQueueDelete(audio_trigger_queue);   audio_trigger_queue = NULL; }
        if (striker_trigger_queue) { vQueueDelete(striker_trigger_queue); striker_trigger_queue = NULL; }
    }

    ESP_LOGE(TAG, "Cleanup complete. System halted.");
}

/* ========================================================================== */
/* 应用主入口                                                                 */
/* ========================================================================== */
extern "C" void app_main(void) {
    init_context_t ctx = {
        .stage = INIT_STAGE_NONE,
        .sd_mounted = false,
        .video_fd = -1,
    };

    // 创建系统事件队列
    ui_event_queue = xQueueCreate(UI_EVENT_QUEUE_SIZE, sizeof(system_event_t));
    sd_write_queue = xQueueCreate(SD_WRITE_QUEUE_SIZE, sizeof(capture_msg_t));
    hw_cmd_queue = xQueueCreate(HW_CMD_QUEUE_SIZE, sizeof(system_event_t));
    audio_cap_queue = xQueueCreate(AUDIO_CAP_QUEUE_SIZE, sizeof(audio_capture_t));
    audio_trigger_queue = xQueueCreate(AUDIO_TRIGGER_QUEUE_SIZE, sizeof(system_event_t));
    striker_trigger_queue = xQueueCreate(STRIKER_TRIGGER_QUEUE_SIZE, sizeof(system_event_t));
    ctx.stage = INIT_STAGE_QUEUES;  // 必须在校验前设置，确保失败时能清理已创建的队列
    if (!ui_event_queue || !hw_cmd_queue || !audio_cap_queue || !audio_trigger_queue || !striker_trigger_queue || !sd_write_queue) {
        ESP_LOGE(TAG, "关键队列创建失败，系统终止");
        system_cleanup(&ctx);
        return;
    }

    // 启动时打印内存信息
    // 初始化照明与舵机 PWM
    lighting_init();
    lighting_set_brightness(0);
    servo_init();
    servo_stop();

    ESP_LOGI(TAG, "=== Memory Info ===");
    ESP_LOGI(TAG, "PSRAM total: %u KB, free: %u KB, largest free block: %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
    ESP_LOGI(TAG, "DRAM  total: %u KB, free: %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    // 挂载 SD 卡
    ESP_LOGI(TAG, "Mounting SD Card via BSP...");
    esp_err_t sd_ret = bsp_sdcard_mount();
    if (sd_ret != ESP_OK) {
        ESP_LOGE(TAG, "SD Card mount failed (0x%x). System will continue without saving images.", sd_ret);
    } else {
        ESP_LOGI(TAG, "SD card mounted at /sdcard");
        ctx.sd_mounted = true;
        struct stat st;
        if (stat("/sdcard/images", &st) == -1) {
            mkdir("/sdcard/images", 0777);
        }
        if (stat("/sdcard/audio", &st) == -1) {
            mkdir("/sdcard/audio", 0777);
        }
    }
    ctx.stage = INIT_STAGE_SD_MOUNTED;

    // 启动 SD 写入任务
    if (xTaskCreatePinnedToCore(sd_writer_task, "SD_Task", 8192, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "SD writer task create failed");
        system_cleanup(&ctx);
        return;
    }

    // 启动显示
    bsp_display_start();
    bsp_display_backlight_on();
    ctx.stage = INIT_STAGE_DISPLAY;

    // 内存对齐与 PPA/摄像头双缓冲分配
    size_t data_cache_line_size = 0;
    ESP_ERROR_CHECK(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size));
    cam_buff_size = ALIGN_UP(CAM_W * CAM_H * 2, data_cache_line_size);
    for (int i = 0; i < NUM_BUFS; i++) {
        cam_buff[i] = (uint8_t *)heap_caps_aligned_calloc(data_cache_line_size, 1, cam_buff_size, MALLOC_CAP_SPIRAM);
        if (!cam_buff[i]) {
            ESP_LOGE(TAG, "摄像头 Buffer %d PSRAM 分配失败 (size=%lu)", i, (unsigned long)cam_buff_size);
            system_cleanup(&ctx);
            return;
        }
    }
    ctx.stage = INIT_STAGE_CAM_BUFFERS;

    // 初始化视频管道（PPA）
    video_pipeline_init();
    ctx.stage = INIT_STAGE_PPA;

    // 加载 AI 模型
    if (ai_vision_init() != 0) {
        ESP_LOGE(TAG, "AI 模型加载失败，系统终止");
        system_cleanup(&ctx);
        return;
    }
    ctx.stage = INIT_STAGE_AI_MODEL;

    // 创建 LVGL 工业面板 UI
    ui_create();
    ctx.stage = INIT_STAGE_UI;

    // 启动 FreeRTOS 多核硬件调度机制
    if (xTaskCreatePinnedToCore(display_refresh_task, "DispRefresh", 4096, NULL, 6, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Display refresh task create failed");
        system_cleanup(&ctx);
        return;
    }
    if (xTaskCreatePinnedToCore(ui_event_task, "UI_Task", 4096 * 2, NULL, 7, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "UI task create failed");
        system_cleanup(&ctx);
        return;
    }
    if (xTaskCreatePinnedToCore(ai_vision_task, "AI_Task", 8192, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "AI task create failed");
        system_cleanup(&ctx);
        return;
    }
    if (xTaskCreatePinnedToCore(hardware_control_task, "HW_Task", 2048, NULL, 3, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "HW task create failed");
        system_cleanup(&ctx);
        return;
    }

    // 启动麦克风 I2S 与 FFT 声学监测
    i2s_mic_init();
    if (xTaskCreatePinnedToCore(striker_task, "striker", 2048, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "striker task create failed");
        system_cleanup(&ctx);
        return;
    }
    if (xTaskCreatePinnedToCore(audio_capture_task, "audio_cap", 8192, NULL, 4, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "audio capture task create failed");
        system_cleanup(&ctx);
        return;
    }
    if (xTaskCreatePinnedToCore(audio_process_task, "audio_proc", 4096, NULL, 3, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "audio process task create failed");
        system_cleanup(&ctx);
        return;
    }

    // 启动摄像头底层硬件中断驱动
    ESP_ERROR_CHECK(bsp_camera_start(NULL));
    int fd = app_video_open((char *)BSP_CAMERA_DEVICE, APP_VIDEO_FMT_RGB565);
    if (fd < 0) {
        ESP_LOGE(TAG, "摄像头设备打开失败，系统终止");
        system_cleanup(&ctx);
        return;
    }
    ctx.video_fd = fd;
    ctx.stage = INIT_STAGE_VIDEO_STREAM;

    ESP_ERROR_CHECK(app_video_set_bufs(fd, NUM_BUFS, NULL));
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(camera_video_frame_operation));
    ESP_ERROR_CHECK(app_video_stream_task_start(fd, 0));

    ESP_LOGI(TAG, "=== System initialized successfully ===");
}
