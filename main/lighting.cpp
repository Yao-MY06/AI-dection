/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lighting.h"
#include "app_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LIGHT_LEDC_TIMER        LEDC_TIMER_3
#define LIGHT_LEDC_CHANNEL      LEDC_CHANNEL_3
#define LIGHT_LEDC_FREQ         5000        // 5kHz 无频闪
#define LIGHT_LEDC_RES          LEDC_TIMER_10_BIT
#define LIGHT_MAX_DUTY          1023
#define LIGHT_DEFAULT_PCT       10          // 默认 10% 亮度

static const char *LIGHT_TAG = "LIGHTING";
static bool s_lighting_initialized = false;

void lighting_init(void) {
    if (s_lighting_initialized) return;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LIGHT_LEDC_RES,
        .timer_num       = LIGHT_LEDC_TIMER,
        .freq_hz         = LIGHT_LEDC_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num       = (int)LIGHT_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LIGHT_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LIGHT_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .flags          = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    s_lighting_initialized = true;
    ESP_LOGI(LIGHT_TAG, "Lighting PWM init: GPIO%d, %dHz, %d-bit", LIGHT_PIN, LIGHT_LEDC_FREQ, LIGHT_LEDC_RES);
}

void lighting_set_brightness(uint8_t pct) {
    if (!s_lighting_initialized) {
        ESP_LOGW(LIGHT_TAG, "Lighting not initialized, call lighting_init() first");
        return;
    }
    if (pct > 100) pct = 100;
    uint32_t duty = (uint32_t)pct * LIGHT_MAX_DUTY / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LIGHT_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LIGHT_LEDC_CHANNEL));
    ESP_LOGI(LIGHT_TAG, "Brightness set to %d%% (duty=%lu)", pct, (unsigned long)duty);
}
