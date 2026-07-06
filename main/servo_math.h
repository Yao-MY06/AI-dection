#ifndef SERVO_MATH_H
#define SERVO_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_MIN_ANGLE_DEG     0
#define SERVO_CENTER_ANGLE_DEG  90
#define SERVO_MAX_ANGLE_DEG     180
#define SERVO_MIN_PULSE_US      500
#define SERVO_MAX_PULSE_US      2500
#define SERVO_PWM_PERIOD_US     20000

int16_t servo_clamp_angle(int16_t angle);
uint32_t servo_angle_to_pulse_us(int16_t angle);
int16_t servo_apply_delta_angle(int16_t current_angle, int16_t delta_deg);
uint32_t servo_continuous_speed_to_pulse_us(int16_t speed_pct);

#ifdef __cplusplus
}
#endif

#endif // SERVO_MATH_H
