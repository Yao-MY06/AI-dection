/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AI_VISION_MONITOR_H
#define AI_VISION_MONITOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frame_id;
    bool detected;
    int class_id;
    float score;
    float normal_score;
    float scratch_score;
    int x;
    int y;
    int w;
    int h;
    int output_dims;
    int output_size;
    int output_shape[4];
    const char *output_dtype;
    int output_exponent;
    float output_max;
    const char *parse_status;
    int anchor_count;
    int candidate_count;
    int valid_candidate_count;
    int best_anchor;
    float best_raw_cx;
    float best_raw_cy;
    float best_raw_w;
    float best_raw_h;
    float best_normal;
    float best_scratch;
    int preprocess_us;
    int inference_us;
    int total_us;
    uint8_t input_luma_mean;
    uint8_t input_luma_range;
} ai_vision_monitor_sample_t;

const char *ai_vision_monitor_class_name(int class_id, bool detected);

int ai_vision_monitor_format(char *buffer,
                             size_t buffer_size,
                             const ai_vision_monitor_sample_t *sample,
                             int frame_width,
                             int frame_height);

#ifdef __cplusplus
}
#endif

#endif // AI_VISION_MONITOR_H
