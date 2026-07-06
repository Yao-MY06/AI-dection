/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "ai_vision_monitor.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

const char *ai_vision_monitor_class_name(int class_id, bool detected)
{
    if (!detected || class_id < 0) {
        return "none";
    }
    if (class_id == 0) {
        return "normal";
    }
    if (class_id == 1) {
        return "scratch";
    }
    return "unknown";
}

static float ai_vision_monitor_box_area_pct(const ai_vision_monitor_sample_t *sample,
                                            int frame_width,
                                            int frame_height)
{
    if (!sample || !sample->detected || sample->w <= 0 || sample->h <= 0 ||
        frame_width <= 0 || frame_height <= 0) {
        return 0.0f;
    }
    return ((float)sample->w * (float)sample->h * 100.0f) /
           ((float)frame_width * (float)frame_height);
}

int ai_vision_monitor_format(char *buffer,
                             size_t buffer_size,
                             const ai_vision_monitor_sample_t *sample,
                             int frame_width,
                             int frame_height)
{
    if (!buffer || buffer_size == 0 || !sample) {
        return -1;
    }

    float margin = fabsf(sample->scratch_score - sample->normal_score);
    float box_area = ai_vision_monitor_box_area_pct(sample, frame_width, frame_height);
    int x = sample->detected ? sample->x : 0;
    int y = sample->detected ? sample->y : 0;
    int w = sample->detected ? sample->w : 0;
    int h = sample->detected ? sample->h : 0;
    const char *dtype = sample->output_dtype ? sample->output_dtype : "?";
    const char *parse_status = sample->parse_status ? sample->parse_status : "?";

    char shape[32] = {};
    int offset = 0;
    int dims = sample->output_dims;
    if (dims < 0) dims = 0;
    if (dims > 4) dims = 4;
    if (dims == 0) {
        snprintf(shape, sizeof(shape), "-");
    } else {
        for (int i = 0; i < dims; ++i) {
            int written = snprintf(shape + offset, sizeof(shape) - (size_t)offset,
                                   "%s%d", i == 0 ? "" : "x", sample->output_shape[i]);
            if (written < 0) {
                break;
            }
            offset += written;
            if (offset >= (int)sizeof(shape)) {
                offset = (int)sizeof(shape) - 1;
                shape[offset] = '\0';
                break;
            }
        }
    }

    return snprintf(buffer, buffer_size,
                    "[AI_MON] frame=%lu det=%d class=%s "
                    "score=%.3f normal=%.3f scratch=%.3f margin=%.3f "
                    "box=%d,%d,%d,%d box_area=%.3f "
                    "pre_us=%d infer_us=%d total_us=%d "
                    "luma_mean=%u luma_range=%u "
                    "out_dims=%d out_size=%d out_max=%.3f "
                    "shape=%s dtype=%s exp=%d parse=%s anchors=%d cand=%d valid=%d "
                    "best_anchor=%d best_n=%.3f best_s=%.3f raw_box=%.1f,%.1f,%.1f,%.1f",
                    (unsigned long)sample->frame_id,
                    sample->detected ? 1 : 0,
                    ai_vision_monitor_class_name(sample->class_id, sample->detected),
                    sample->detected ? sample->score : 0.0f,
                    sample->detected ? sample->normal_score : 0.0f,
                    sample->detected ? sample->scratch_score : 0.0f,
                    sample->detected ? margin : 0.0f,
                    x, y, w, h, box_area,
                    sample->preprocess_us,
                    sample->inference_us,
                    sample->total_us,
                    (unsigned)sample->input_luma_mean,
                    (unsigned)sample->input_luma_range,
                    sample->output_dims,
                    sample->output_size,
                    sample->output_max,
                    shape,
                    dtype,
                    sample->output_exponent,
                    parse_status,
                    sample->anchor_count,
                    sample->candidate_count,
                    sample->valid_candidate_count,
                    sample->best_anchor,
                    sample->best_normal,
                    sample->best_scratch,
                    sample->best_raw_cx,
                    sample->best_raw_cy,
                    sample->best_raw_w,
                    sample->best_raw_h);
}
