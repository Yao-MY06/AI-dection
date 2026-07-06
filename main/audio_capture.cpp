/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "audio_capture.h"
#include "audio_analysis.h"
#include "audio_ai.h"
#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"
#include "ui_manager.h"

#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"

/* ========================================================================== */
/* I2S 麦克风句柄（模块私有）                                                 */
/* ========================================================================== */
static i2s_chan_handle_t rx_chan = NULL;

/* ========================================================================== */
/* 环形缓冲与音频截取状态（模块私有）                                         */
/* ========================================================================== */
__attribute__((aligned(16))) static int16_t audio_ring_buf[RING_BUF_SIZE];
static volatile int ring_wr_idx = 0;

static audio_cap_state_t cap_state = AUDIO_CAP_IDLE;
static int16_t *capture_psram = NULL;
static int cap_sample_idx = 0;
static int capture_target_count = TOTAL_CAP_SAMPLES;


static inline int16_t audio_apply_capture_gain(int32_t sample)
{
    int32_t scaled = (int32_t)((float)sample * AUDIO_CAPTURE_GAIN);
    if (scaled > 32767) {
        return 32767;
    }
    if (scaled < -32768) {
        return -32768;
    }
    return (int16_t)scaled;
}
/* ========================================================================== */
/* I2S 麦克风初始化                                                           */
/* ========================================================================== */
void i2s_mic_init(void) {
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
    ESP_LOGI(FFT_TAG, "I2S mic initialized, capture_gain=%.3f", (double)AUDIO_CAPTURE_GAIN);
}

/* ========================================================================== */
/* 麦克风采集与 FFT 分析任务                                                  */
/* ========================================================================== */
void audio_capture_task(void *pvParameters) {
    // 初始化 FFT 分析引擎
    if (audio_analysis_init() != 0) {
        ESP_LOGE(FFT_TAG, "Audio analysis init failed!");
        vTaskDelete(NULL);
    }

    size_t bytes_read = 0;
    int32_t raw_samples[64];
    int64_t dc_offset = 0;
    int buffer_idx = 0;
    float audio_buffer[N_SAMPLES];
    static int print_throttle = 0;

    while (1) {
        // 轮询触发信号 (非阻塞)：自动敲击触发 或 手动录音触发
        if (cap_state == AUDIO_CAP_IDLE) {
            system_event_t ev;
            if (xQueueReceive(audio_trigger_queue, &ev, 0) == pdPASS) {
                if (ev.type == EVENT_HIT_TRIGGER) {
                    capture_psram = (int16_t *)heap_caps_malloc(
                        TOTAL_CAP_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
                    if (capture_psram) {
                        for (int j = 0; j < RING_BUF_SIZE; j++) {
                            capture_psram[j] = audio_ring_buf[(ring_wr_idx + j) % RING_BUF_SIZE];
                        }
                        cap_sample_idx = RING_BUF_SIZE;
                        capture_target_count = TOTAL_CAP_SAMPLES;
                        cap_state = AUDIO_CAP_ACTIVE;
                        ESP_LOGI(FFT_TAG, "Audio capture started, ring_buf=%d samples copied", RING_BUF_SIZE);
                    } else {
                        ESP_LOGE(FFT_TAG, "PSRAM alloc failed for audio capture (%d bytes)",
                                 (int)(TOTAL_CAP_SAMPLES * sizeof(int16_t)));
                        // 发送空消息通知上层分配失败
                        audio_capture_t fail_msg = { .samples = NULL, .sample_count = 0 };
                        if (xQueueSend(audio_cap_queue, &fail_msg, pdMS_TO_TICKS(50)) != pdPASS) {
                            ESP_LOGW(FFT_TAG, "audio_cap_queue full, fail message dropped");
                        }
                    }
                } else if (ev.type == EVENT_AUDIO_START_RECORD) {
                    capture_psram = (int16_t *)heap_caps_malloc(
                        AUDIO_MANUAL_CAP_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
                    if (capture_psram) {
                        // 手动录音也复制 ring buffer 作为预触发，避免敲击事件被丢弃后录不到声音
                        for (int j = 0; j < RING_BUF_SIZE; j++) {
                            capture_psram[j] = audio_ring_buf[(ring_wr_idx + j) % RING_BUF_SIZE];
                        }
                        cap_sample_idx = RING_BUF_SIZE;
                        capture_target_count = AUDIO_MANUAL_CAP_SAMPLES;
                        cap_state = AUDIO_CAP_ACTIVE;
                        ESP_LOGI(FFT_TAG, "Manual audio capture started, %d samples (ring_buf=%d pre-trigger)",
                                 AUDIO_MANUAL_CAP_SAMPLES, RING_BUF_SIZE);
                    } else {
                        ESP_LOGE(FFT_TAG, "PSRAM alloc failed for manual audio capture (%d bytes)",
                                 (int)(AUDIO_MANUAL_CAP_SAMPLES * sizeof(int16_t)));
                        audio_capture_t fail_msg = { .samples = NULL, .sample_count = 0 };
                        if (xQueueSend(audio_cap_queue, &fail_msg, pdMS_TO_TICKS(50)) != pdPASS) {
                            ESP_LOGW(FFT_TAG, "audio_cap_queue full, fail message dropped");
                        }
                    }
                }
            }
        }

        if (i2s_channel_read(rx_chan, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY) == ESP_OK) {
            int sample_count = bytes_read / sizeof(int32_t);
            for (int i = 0; i < sample_count; i++) {
                int32_t raw_val = raw_samples[i] >> 8;
                dc_offset = (dc_offset * 63 + raw_val) / 64;
                int32_t clean_val = raw_val - (int32_t)dc_offset;
                int16_t pcm_val = audio_apply_capture_gain(clean_val);

                // 环形缓冲写入
                audio_ring_buf[ring_wr_idx] = pcm_val;
                ring_wr_idx = (ring_wr_idx + 1) % RING_BUF_SIZE;

                // 音频截取状态机
                if (cap_state == AUDIO_CAP_ACTIVE && capture_psram != NULL) {
                    capture_psram[cap_sample_idx] = pcm_val;
                    cap_sample_idx++;
                    if (cap_sample_idx >= capture_target_count) {
                        ESP_LOGI(FFT_TAG, "Audio capture done: first4=[%d,%d,%d,%d] last4=[%d,%d,%d,%d] addr=%p",
                                 capture_psram[0], capture_psram[1], capture_psram[2], capture_psram[3],
                                 capture_psram[capture_target_count - 4], capture_psram[capture_target_count - 3],
                                 capture_psram[capture_target_count - 2], capture_psram[capture_target_count - 1],
                                 capture_psram);
                        audio_capture_t msg = {
                            .samples = capture_psram,
                            .sample_count = (uint32_t)capture_target_count
                        };
                        if (xQueueSend(audio_cap_queue, &msg, pdMS_TO_TICKS(50)) != pdPASS) {
                            ESP_LOGW(FFT_TAG, "audio_cap_queue full, captured audio dropped");
                            heap_caps_free(capture_psram);
                        }
                        capture_psram = NULL;
                        cap_state = AUDIO_CAP_IDLE;
                        system_event_t done_ev = {};
                        done_ev.type = EVENT_AUDIO_CAP_DONE;
                        if (xQueueSend(ui_event_queue, &done_ev, pdMS_TO_TICKS(50)) != pdPASS) {
                            ESP_LOGW(FFT_TAG, "ui_event_queue full, audio done event dropped");
                        }
                    }
                }

                audio_buffer[buffer_idx] = (float)pcm_val;
                buffer_idx++;

                if (buffer_idx >= N_SAMPLES) {
#if AUDIO_FFT_MONITOR_LOGS
                    int64_t t_fft_start = esp_timer_get_time();
#endif

                    // 调用独立的频谱分析模块
                    audio_analysis_result_t analysis;
                    audio_analysis_process(audio_buffer, &analysis);

                    // 更新全局状态
                    g_total_energy = analysis.band_total;
                    g_bg_noise = analysis.bg_noise;
                    g_health_score = (int)analysis.health_score;
                    g_hit_pulse = analysis.hit_pulse;

                    print_throttle++;
                    if (print_throttle >= 3) {
#if AUDIO_FFT_MONITOR_LOGS
                        ESP_LOGI(FFT_TAG, "Total_Energy:%.2f Bg_Noise:%.2f Health_Score:%.1f (raw: %.1f) Hit_Event:%d",
                                 analysis.band_total, analysis.bg_noise,
                                 analysis.health_score, analysis.score_valid ? analysis.raw_score : 0.0f,
                                 analysis.hit_pulse);

                        int64_t t_fft_end = esp_timer_get_time();
                        ESP_LOGI(FFT_TAG, "FFT+UI elapsed: %lld us", (t_fft_end - t_fft_start));
#endif
                        print_throttle = 0;

                        ui_update_acoustic_result((int)analysis.health_score, analysis.hit_pulse,
                                                  analysis.band_total, analysis.bg_noise);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    buffer_idx = 0;
                }
            }
        }
    }
}

/* ========================================================================== */
/* 音频片段处理任务 - 接收截取完成的 PCM 数据                                   */
/*        可在此处接入声学 AI 模型推理，或保存到 SD 卡用于离线训练              */
/* ========================================================================== */
void audio_process_task(void *pvParameters) {
    audio_capture_t cap;
    char file_path[64];

    while (1) {
        if (xQueueReceive(audio_cap_queue, &cap, portMAX_DELAY) == pdPASS) {
            if (cap.samples == NULL || cap.sample_count == 0) {
                ESP_LOGW(TAG, "Audio cap: empty message (allocation may have failed), skipping");
                continue;
            }

            float duration_ms = (float)cap.sample_count / AUDIO_SAMPLE_RATE * 1000.0f;
            ESP_LOGI(TAG, "Audio captured: %lu samples (%.1f ms), addr=%p",
                     (unsigned long)cap.sample_count, duration_ms, cap.samples);

            // 计算简要统计
            int64_t sum = 0;
            int64_t sum_sq = 0;
            int32_t peak = 0;
            uint32_t clipped = 0;
            for (uint32_t i = 0; i < cap.sample_count; i++) {
                int16_t s = cap.samples[i];
                int32_t abs_s = (s == -32768) ? 32768 : abs((int)s);
                sum += s;
                sum_sq += (int64_t)s * (int64_t)s;
                if (abs_s > peak) peak = abs_s;
                if (s == 32767 || s == -32768) clipped++;
            }
            int16_t dc = (int16_t)(sum / (int64_t)cap.sample_count);
            float rms = sqrtf((float)((double)sum_sq / (double)cap.sample_count));
            ESP_LOGI(TAG, "Audio stats: DC=%d, peak=%ld, rms=%.1f, clipped=%lu/%lu, gain=%.3f, first4=[%d,%d,%d,%d]",
                     dc, (long)peak, (double)rms,
                     (unsigned long)clipped, (unsigned long)cap.sample_count,
                     (double)AUDIO_CAPTURE_GAIN,
                     cap.samples[0], cap.samples[1],
                     cap.samples[2], cap.samples[3]);

            float features[AUDIO_AI_FEATURE_DIM];
            audio_ai_extract_features(cap.samples, cap.sample_count, features);
            ESP_LOGI(TAG,
                     "Audio features: rms=%.1f rms20=%.1f rms50=%.1f decay=%.3f zcr=%.4f centroid=%.1fHz rolloff=%.1fHz low=%.4f mid=%.4f high=%.4f peak_dbfs=%.1f",
                     (double)features[0], (double)features[1], (double)features[2],
                     (double)features[5], (double)features[3],
                     (double)features[7], (double)features[8],
                     (double)features[9], (double)features[10], (double)features[11],
                     (double)features[6]);
            // 保存原始 PCM 到 SD 卡，文件名格式 X-Y.RAW：
            // X = 样品编号，Y = 同一部位的第几次敲击；每 3 次自动换下一个样品
            static uint32_t sample_idx = 1;
            static uint32_t hit_idx = 1;
            snprintf(file_path, sizeof(file_path), "%s/audio/%lu-%lu.RAW", MOUNT_POINT, sample_idx, hit_idx);
            hit_idx++;
            if (hit_idx > 3) {
                hit_idx = 1;
                sample_idx++;
            }
            FILE *f = fopen(file_path, "wb");
            if (f != NULL) {
                size_t total_bytes = cap.sample_count * sizeof(int16_t);
                size_t written = fwrite(cap.samples, 1, total_bytes, f);
                fflush(f);
                fsync(fileno(f));
                fclose(f);
                if (written == total_bytes) {
                    ESP_LOGI(TAG, "Audio saved: %s (%u bytes)", file_path, (unsigned)written);
                } else {
                    ESP_LOGE(TAG, "Audio save incomplete: %s (%u/%u)", file_path,
                             (unsigned)written, (unsigned)total_bytes);
                }
            } else {
                ESP_LOGW(TAG, "SD write skipped (file open failed): %s", file_path);
            }

            // 声学 AI 单分类异常检测
            float ai_score = 0.0f;
            audio_ai_result_t ai_result = audio_ai_predict_with_features_and_score(features, &ai_score);
            const char *ai_label = (ai_result == AUDIO_AI_GOOD) ? "GOOD" : "ANOMALY";
            ESP_LOGI(TAG, "Audio AI result: %s (max_zscore=%.3f)", ai_label, (double)ai_score);

            {
                system_event_t ai_ev = {};
                ai_ev.type = (ai_result == AUDIO_AI_GOOD) ? EVENT_AUDIO_AI_GOOD : EVENT_AUDIO_AI_ANOMALY;
                snprintf(ai_ev.log_msg, sizeof(ai_ev.log_msg), "%.3f", (double)ai_score);
                if (xQueueSend(ui_event_queue, &ai_ev, pdMS_TO_TICKS(50)) != pdPASS) {
                    ESP_LOGW(TAG, "UI event queue full, audio AI event dropped");
                }
            }

            heap_caps_free(cap.samples);
        }
    }
}
