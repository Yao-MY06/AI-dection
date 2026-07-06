/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SD 卡图像写入任务
 */
void sd_writer_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // SD_STORAGE_H
