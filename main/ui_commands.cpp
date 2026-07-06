#include "ui_commands.h"

#include <string.h>

bool ui_command_from_label(const char *txt, event_type_t *out_type)
{
    if (!txt || !out_type) {
        return false;
    }

    if (strstr(txt, "CAPTURE")) {
        *out_type = CMD_BTN_CAPTURE;
        return true;
    }
    if (strstr(txt, "VISION") && strstr(txt, "START")) {
        *out_type = CMD_VISION_START;
        return true;
    }
    if (strstr(txt, "VISION") && strstr(txt, "STOP")) {
        *out_type = CMD_VISION_STOP;
        return true;
    }
    if (strstr(txt, "AUDIO") && strstr(txt, "START")) {
        *out_type = CMD_AUDIO_START;
        return true;
    }
    if (strstr(txt, "SERVO") &&
        (strstr(txt, "REV") || strstr(txt, "FWD") || strstr(txt, "ZERO"))) {
        *out_type = CMD_SERVO_TOGGLE;
        return true;
    }
    if (strstr(txt, "EMERGENCY")) {
        *out_type = CMD_BTN_EMERGENCY;
        return true;
    }

    return false;
}
