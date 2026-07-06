/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "hardware_control.h"
#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REJECTER_PULSE_MS   200

void hardware_control_task(void *pvParameters) {
    // 初始化剔除机构 GPIO
    bool rejecter_available = false;
    esp_err_t ret = gpio_reset_pin((gpio_num_t)REJECTER_PIN);
    if (ret == ESP_OK) {
        ret = gpio_set_direction((gpio_num_t)REJECTER_PIN, GPIO_MODE_OUTPUT);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_level((gpio_num_t)REJECTER_PIN, 0);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Rejecter pin (GPIO %d) init failed: %s", REJECTER_PIN, esp_err_to_name(ret));
    } else {
        rejecter_available = true;
    }

    system_event_t hw_ev;
    while (1) {
        if (xQueueReceive(hw_cmd_queue, &hw_ev, portMAX_DELAY) == pdPASS) {
            // TODO: 自动剔除暂时禁用，仅保留手动舵机控制
            // if (hw_ev.type == EVENT_AI_DEFECT_FOUND) {
            //     ESP_LOGI("HARDWARE", "▶ Activating Rejecter Relay (GPIO %d)...", REJECTER_PIN);
            //     if (rejecter_available) {
            //         gpio_set_level((gpio_num_t)REJECTER_PIN, 1);
            //         servo_move_relative(-30);
            //         vTaskDelay(pdMS_TO_TICKS(300));
            //         gpio_set_level((gpio_num_t)REJECTER_PIN, 0);
            //     }
            //     ESP_LOGI("HARDWARE", "⏹ Rejecter Reset.");
            // }
        }
    }
}
