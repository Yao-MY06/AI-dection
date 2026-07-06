/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 硬件控制任务：处理 NG 信号，触发剔除/报警等执行动作
 */
void hardware_control_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // HARDWARE_CONTROL_H
