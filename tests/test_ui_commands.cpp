#include "../main/ui_commands.h"

#include <cassert>

static void maps_servo_labels_to_servo_toggle_command()
{
    event_type_t type = CMD_VISION_START;

    assert(ui_command_from_label("SERVO REV", &type));
    assert(type == CMD_SERVO_TOGGLE);

    type = CMD_VISION_START;
    assert(ui_command_from_label("SERVO FWD", &type));
    assert(type == CMD_SERVO_TOGGLE);

    type = CMD_VISION_START;
    assert(ui_command_from_label("SERVO ZERO", &type));
    assert(type == CMD_SERVO_TOGGLE);
}

static void rejects_unknown_labels_without_defaulting_to_vision_start()
{
    event_type_t type = CMD_VISION_START;

    assert(!ui_command_from_label("UNKNOWN", &type));
    assert(type == CMD_VISION_START);
}

int main()
{
    maps_servo_labels_to_servo_toggle_command();
    rejects_unknown_labels_without_defaulting_to_vision_start();
    return 0;
}
