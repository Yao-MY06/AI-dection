#ifndef UI_COMMANDS_H
#define UI_COMMANDS_H

#include "app_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ui_command_from_label(const char *txt, event_type_t *out_type);

#ifdef __cplusplus
}
#endif

#endif // UI_COMMANDS_H
