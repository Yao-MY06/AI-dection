/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "servo.h"
#include "app_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_LEDC_TIMER        LEDC_TIMER_2
#define SERVO_LEDC_CHANNEL      LEDC_CHANNEL_2
#define SERVO_FREQ              50          // 50Hz
#define SERVO_DUTY_RES          LEDC_TIMER_12_BIT
#define SERVO_MAX_DUTY          4095

// 舵机全速角速度 (°/s)，SG90 典型值约 0.1s/60° = 600°/s @4.8V
#define SERVO_FULL_SPEED_DPS    600

static const char *SERVO_TAG = "SERVO";
static bool s_servo_initialized = false;
static int16_t s_current_angle = SERVO_CENTER_ANGLE_DEG;

static uint32_t servo_pulse_to_duty(uint32_t pulse_us)
{
    return pulse_us * SERVO_MAX_DUTY / SERVO_PWM_PERIOD_US;
}

static int16_t servo_round_interpolated_angle(int16_t start_angle, int16_t delta_deg, int step, int total_steps)
{
    int32_t numerator = (int32_t)delta_deg * step;
    int32_t half = total_steps / 2;
    int32_t rounded_delta = (numerator >= 0)
        ? (numerator + half) / total_steps
        : (numerator - half) / total_steps;
    return servo_clamp_angle(start_angle + rounded_delta);
}

void servo_init(void) {
    if (s_servo_initialized) return;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = SERVO_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num       = (int)SERVO_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = SERVO_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = SERVO_LEDC_TIMER,
        .duty           = servo_pulse_to_duty(servo_angle_to_pulse_us(SERVO_CENTER_ANGLE_DEG)),
        .hpoint         = 0,
        .flags          = { .output_invert = 0 },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    s_servo_initialized = true;
    s_current_angle = SERVO_CENTER_ANGLE_DEG;
    ESP_LOGI(SERVO_TAG, "180° Servo PWM init: GPIO%d, %dHz, center=%dus",
             SERVO_PIN, SERVO_FREQ, (int)servo_angle_to_pulse_us(SERVO_CENTER_ANGLE_DEG));
}

void servo_set_pulse_us(uint32_t pulse_us) {
    if (!s_servo_initialized) {
        ESP_LOGW(SERVO_TAG, "Servo not initialized, call servo_init() first");
        return;
    }
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;

    uint32_t duty = servo_pulse_to_duty(pulse_us);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL));
    ESP_LOGI(SERVO_TAG, "Pulse set to %lu us (duty=%lu/4095)",
             (unsigned long)pulse_us, (unsigned long)duty);
}

void servo_set_angle(int16_t angle) {
    if (!s_servo_initialized) {
        ESP_LOGW(SERVO_TAG, "Servo not initialized, call servo_init() first");
        return;
    }

    angle = servo_clamp_angle(angle);
    s_current_angle = angle;

    uint32_t pulse_us = servo_angle_to_pulse_us(angle);
    servo_set_pulse_us((uint32_t)pulse_us);
    ESP_LOGI(SERVO_TAG, "Angle set to %d° (pulse=%lu us)", (int)angle, (unsigned long)pulse_us);
}

void servo_move_relative(int16_t delta_deg) {
    servo_set_angle(servo_apply_delta_angle(s_current_angle, delta_deg));
}

void servo_move_relative_speed(int16_t delta_deg, uint8_t speed_pct) {
    if (!s_servo_initialized) {
        ESP_LOGW(SERVO_TAG, "Servo not initialized, call servo_init() first");
        return;
    }

    if (speed_pct > 100) speed_pct = 100;
    if (speed_pct < 1)   speed_pct = 1;

    int16_t start_angle = s_current_angle;
    int16_t target_angle = servo_apply_delta_angle(start_angle, delta_deg);
    int16_t move_delta = target_angle - start_angle;
    if (move_delta == 0) return;

    int abs_delta = (move_delta > 0) ? move_delta : -move_delta;

    // 全速时间 = 角度 / 角速度，实际时间按 speed_pct 缩放
    float total_time_s  = (float)abs_delta / (float)SERVO_FULL_SPEED_DPS;
    float total_time_ms = total_time_s * 1000.0f * 100.0f / (float)speed_pct;

    // 每步约 10ms（FreeRTOS tick 最小粒度），计算步数和步长
    int num_steps = (int)(total_time_ms / 10.0f);
    if (num_steps < 1) num_steps = 1;
    if (num_steps > abs_delta) num_steps = abs_delta;  // 每步至少 1°

    ESP_LOGI(SERVO_TAG, "Relative move: %d° -> %d° at %d%% speed (%d steps, %.1fms total)",
             (int)start_angle, (int)target_angle, speed_pct, num_steps, total_time_ms);

    for (int i = 1; i <= num_steps; i++) {
        servo_set_angle(servo_round_interpolated_angle(start_angle, move_delta, i, num_steps));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    servo_set_angle(target_angle);
}

void servo_set_continuous_speed(int16_t speed_pct) {
    if (!s_servo_initialized) {
        ESP_LOGW(SERVO_TAG, "Servo not initialized, call servo_init() first");
        return;
    }

    uint32_t pulse_us = servo_continuous_speed_to_pulse_us(speed_pct);
    servo_set_pulse_us(pulse_us);
    ESP_LOGI(SERVO_TAG, "Continuous speed set to %d%% (pulse=%lu us)",
             (int)speed_pct, (unsigned long)pulse_us);
}

void servo_jog_continuous(int16_t speed_pct, uint16_t duration_ms) {
    if (duration_ms == 0) {
        servo_set_continuous_speed(0);
        return;
    }

    servo_set_continuous_speed(speed_pct);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    servo_set_continuous_speed(0);
}

void servo_stop(void) {
    // 位置舵机：停止 = 保持当前角度（持续输出 PWM 维持位置）
    servo_set_continuous_speed(0);
}

void servo_rotate_for(int8_t speed_pct, uint16_t duration_ms) {
    (void)speed_pct;
    (void)duration_ms;
    ESP_LOGW(SERVO_TAG, "servo_rotate_for is deprecated for position servo, use servo_move_relative()");
}
