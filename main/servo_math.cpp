#include "servo_math.h"

int16_t servo_clamp_angle(int16_t angle)
{
    if (angle < SERVO_MIN_ANGLE_DEG) return SERVO_MIN_ANGLE_DEG;
    if (angle > SERVO_MAX_ANGLE_DEG) return SERVO_MAX_ANGLE_DEG;
    return angle;
}

uint32_t servo_angle_to_pulse_us(int16_t angle)
{
    int16_t clamped_angle = servo_clamp_angle(angle);
    int32_t pulse_span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    int32_t angle_span = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
    return SERVO_MIN_PULSE_US + ((int32_t)clamped_angle * pulse_span) / angle_span;
}

int16_t servo_apply_delta_angle(int16_t current_angle, int16_t delta_deg)
{
    return servo_clamp_angle(current_angle + delta_deg);
}

uint32_t servo_continuous_speed_to_pulse_us(int16_t speed_pct)
{
    if (speed_pct > 100) speed_pct = 100;
    if (speed_pct < -100) speed_pct = -100;

    int32_t half_span = (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 2;
    int32_t center = servo_angle_to_pulse_us(SERVO_CENTER_ANGLE_DEG);
    return center - ((int32_t)speed_pct * half_span) / 100;
}
