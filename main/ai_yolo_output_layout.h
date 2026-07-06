/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AI_YOLO_OUTPUT_LAYOUT_H
#define AI_YOLO_OUTPUT_LAYOUT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int anchors;
    bool channels_first;
} ai_yolo_tensor_layout_t;

bool ai_yolo_detect_layout_from_shape(const int *shape,
                                      int dims,
                                      int tensor_size,
                                      int expected_channels,
                                      ai_yolo_tensor_layout_t *layout);

#ifdef __cplusplus
}
#endif

#endif // AI_YOLO_OUTPUT_LAYOUT_H
