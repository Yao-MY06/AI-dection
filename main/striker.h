/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef STRIKER_H
#define STRIKER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 敲击执行机构任务：接收触发信号，驱动继电器敲击被测物
 */
void striker_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // STRIKER_H
