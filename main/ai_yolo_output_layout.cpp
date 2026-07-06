/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "ai_yolo_output_layout.h"

bool ai_yolo_detect_layout_from_shape(const int *shape,
                                      int dims,
                                      int tensor_size,
                                      int expected_channels,
                                      ai_yolo_tensor_layout_t *layout)
{
    if (!layout || expected_channels <= 0 || tensor_size <= 0) {
        return false;
    }
    layout->anchors = 0;
    layout->channels_first = true;

    if (shape && dims == 3) {
        if (shape[1] == expected_channels) {
            layout->anchors = shape[2];
            layout->channels_first = true;
            return layout->anchors > 0;
        }
        if (shape[2] == expected_channels) {
            layout->anchors = shape[1];
            layout->channels_first = false;
            return layout->anchors > 0;
        }
        return false;
    }

    if (shape && dims == 2) {
        if (shape[0] == expected_channels) {
            layout->anchors = shape[1];
            layout->channels_first = true;
            return layout->anchors > 0;
        }
        if (shape[1] == expected_channels) {
            layout->anchors = shape[0];
            layout->channels_first = false;
            return layout->anchors > 0;
        }
        return false;
    }

    if (tensor_size % expected_channels == 0) {
        layout->anchors = tensor_size / expected_channels;
        layout->channels_first = true;
        return layout->anchors > 0;
    }

    return false;
}
