#include "../main/servo_math.h"

#include <cassert>
#include <cstdint>

static void maps_physical_angle_to_expected_pulse()
{
    assert(servo_angle_to_pulse_us(0) == 500);
    assert(servo_angle_to_pulse_us(10) == 611);
    assert(servo_angle_to_pulse_us(90) == 1500);
    assert(servo_angle_to_pulse_us(180) == 2500);
}

static void clamps_physical_angle_before_mapping()
{
    assert(servo_angle_to_pulse_us(-20) == 500);
    assert(servo_angle_to_pulse_us(220) == 2500);
}

static void applies_relative_moves_from_physical_center()
{
    assert(servo_apply_delta_angle(90, -10) == 80);
    assert(servo_apply_delta_angle(80, 10) == 90);
    assert(servo_apply_delta_angle(5, -10) == 0);
    assert(servo_apply_delta_angle(175, 10) == 180);
}

static void maps_continuous_servo_speed_to_reference_pulse()
{
    assert(servo_continuous_speed_to_pulse_us(100) == 500);
    assert(servo_continuous_speed_to_pulse_us(50) == 1000);
    assert(servo_continuous_speed_to_pulse_us(0) == 1500);
    assert(servo_continuous_speed_to_pulse_us(-50) == 2000);
    assert(servo_continuous_speed_to_pulse_us(-100) == 2500);
}

int main()
{
    maps_physical_angle_to_expected_pulse();
    clamps_physical_angle_before_mapping();
    applies_relative_moves_from_physical_center();
    maps_continuous_servo_speed_to_reference_pulse();
    return 0;
}
