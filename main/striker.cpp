/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "striker.h"
#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STRIKER_PULSE_MS    20

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
        ESP_LOGW(TAG, "Striker relay not connected (GPIO %d init failed: %s)", RELAY_PIN, esp_err_to_name(ret));
    } else {
        relay_available = true;
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    while (1) {
        system_event_t ev;
        if (xQueueReceive(striker_trigger_queue, &ev, portMAX_DELAY) == pdPASS) {
            if (ev.type == EVENT_HIT_TRIGGER && relay_available) {
                gpio_set_level((gpio_num_t)RELAY_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(STRIKER_PULSE_MS));
                gpio_set_level((gpio_num_t)RELAY_PIN, 0);
                ESP_LOGI(TAG, "Striker hit once");
            }
        }
    }
}
