/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sd_storage.h"
#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void sd_writer_task(void *pvParameters) {
    capture_msg_t msg;
    char file_path[64];
    static uint32_t img_cnt = 0;
    static bool cnt_initialized = false;

    while (1) {
        if (xQueueReceive(sd_write_queue, &msg, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "Saving RAW image to SD card...");

            // 扫描 SD 卡已有图片，从最大编号继续，避免覆盖
            if (!cnt_initialized) {
                cnt_initialized = true;
                char img_dir[32];
                snprintf(img_dir, sizeof(img_dir), "%s/images", MOUNT_POINT);
                DIR *dir = opendir(img_dir);
                if (dir) {
                    struct dirent *entry;
                    while ((entry = readdir(dir)) != NULL) {
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

                fflush(f);
                fsync(fileno(f));
                fclose(f);

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
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            if (msg.raw_img_buf != NULL) {
                heap_caps_free(msg.raw_img_buf);
                msg.raw_img_buf = NULL;
            }
        }
    }
}
