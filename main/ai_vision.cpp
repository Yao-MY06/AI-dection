/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#include "ai_vision.h"
#include "ai_vision_monitor.h"
#include "ai_yolo_output_layout.h"
#include "app_config.h"
#include "app_globals.h"
#include "app_types.h"
#include "ui_manager.h"

#include <math.h>
#include <stdint.h>
#include <string>
#include <map>
#include <vector>
#include "esp_log.h"
#include "esp_timer.h"
#include "dl_define.hpp"
#include "dl_tensor_base.hpp"
#include "dl_model_base.hpp"

/* ========================================================================== */
/* 模型与 LUT（模块私有）                                                     */
/* ========================================================================== */
extern const uint8_t model_espdl_start[] asm("_binary_best_model_espdl_start");

static dl::Model *yolo_model = nullptr;

static int src_x_lut[AI_INPUT_W];
static int src_y_lut[AI_INPUT_H];
static uint8_t r5_to_r8[32];
static uint8_t g6_to_g8[64];
static uint8_t b5_to_b8[32];
static bool lut_initialized = false;

static std::map<std::string, dl::TensorBase*> s_input_map;
static std::string s_input_name;
static bool s_input_ready = false;
static bool s_detection_output_warned = false;

typedef enum {
    AI_INPUT_LAYOUT_NHWC = 0,
    AI_INPUT_LAYOUT_NCHW,
} ai_input_layout_t;

typedef struct {
    bool present;
    int class_id;        // 0=normal, 1=scratch
    float score;
    float normal_score;
    float scratch_score;
    int x;
    int y;
    int w;
    int h;
} ai_detection_t;

static ai_input_layout_t s_input_layout = AI_INPUT_LAYOUT_NHWC;
static float s_letterbox_scale = 1.0f;
static float s_letterbox_pad_x = 0.0f;
static float s_letterbox_pad_y = 0.0f;

static void ai_log_monitor_sample(const ai_vision_monitor_sample_t *sample)
{
#if AI_MONITOR_LOG_EVERY_N > 0
    if (!sample || (sample->frame_id % AI_MONITOR_LOG_EVERY_N) != 0) {
        return;
    }

    char line[384] = {};
    int written = ai_vision_monitor_format(line, sizeof(line), sample, CAM_W, CAM_H);
    if (written > 0) {
        ESP_LOGI(TAG, "%s", line);
    }
#else
    (void)sample;
#endif
}

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int round_to_int(float value)
{
    return (int)lroundf(value);
}

static float maybe_sigmoid_score(float value)
{
    if (value >= 0.0f && value <= 1.0f) {
        return value;
    }
    return 1.0f / (1.0f + expf(-value));
}

static int ai_input_index(int channel, int y, int x)
{
    if (s_input_layout == AI_INPUT_LAYOUT_NCHW) {
        return channel * AI_INPUT_W * AI_INPUT_H + y * AI_INPUT_W + x;
    }
    return (y * AI_INPUT_W + x) * 3 + channel;
}

static int quantize_to_int(float value, int exponent, int low, int high)
{
    float inv_scale = DL_RESCALE(exponent);
    int q = round_to_int(value * inv_scale);
    return clamp_int(q, low, high);
}

static void ai_write_input_value(dl::TensorBase *tensor, int index, uint8_t value_u8)
{
    float value = value_u8 / 255.0f;
    switch (tensor->get_dtype()) {
        case dl::DATA_TYPE_FLOAT:
            tensor->get_element_ptr<float>()[index] = value;
            break;
        case dl::DATA_TYPE_INT8:
            tensor->get_element_ptr<int8_t>()[index] =
                (int8_t)quantize_to_int(value, tensor->get_exponent(), -128, 127);
            break;
        case dl::DATA_TYPE_UINT8:
            tensor->get_element_ptr<uint8_t>()[index] =
                (tensor->get_exponent() == 0)
                    ? value_u8
                    : (uint8_t)quantize_to_int(value, tensor->get_exponent(), 0, 255);
            break;
        case dl::DATA_TYPE_INT16:
            tensor->get_element_ptr<int16_t>()[index] =
                (int16_t)quantize_to_int(value, tensor->get_exponent(), -32768, 32767);
            break;
        default:
            break;
    }
}

static float ai_tensor_value_as_float(dl::TensorBase *tensor, int index)
{
    float scale = DL_SCALE(tensor->get_exponent());
    switch (tensor->get_dtype()) {
        case dl::DATA_TYPE_FLOAT:
            return tensor->get_element_ptr<float>()[index];
        case dl::DATA_TYPE_INT8:
            return (float)tensor->get_element_ptr<int8_t>()[index] * scale;
        case dl::DATA_TYPE_UINT8:
            return (float)tensor->get_element_ptr<uint8_t>()[index] * scale;
        case dl::DATA_TYPE_INT16:
            return (float)tensor->get_element_ptr<int16_t>()[index] * scale;
        case dl::DATA_TYPE_INT32:
            return (float)tensor->get_element_ptr<int32_t>()[index] * scale;
        default:
            return 0.0f;
    }
}

static ai_input_layout_t detect_input_layout(const std::vector<int> &shape)
{
    if (shape.size() == 4) {
        if (shape[1] == 3) {
            return AI_INPUT_LAYOUT_NCHW;
        }
        if (shape[3] == 3) {
            return AI_INPUT_LAYOUT_NHWC;
        }
    }
    return AI_INPUT_LAYOUT_NHWC;
}


static bool detect_yolo_tensor_layout(dl::TensorBase *tensor,
                                      int expected_channels,
                                      ai_yolo_tensor_layout_t *layout)
{
    if (!tensor || !layout) {
        return false;
    }

    std::vector<int> shape = tensor->get_shape();
    return ai_yolo_detect_layout_from_shape(shape.empty() ? nullptr : shape.data(),
                                            (int)shape.size(),
                                            tensor->get_size(),
                                            expected_channels,
                                            layout);
}

static float ai_tensor_max_value(dl::TensorBase *tensor)
{
    if (!tensor || !tensor->get_element_ptr()) {
        return 0.0f;
    }
    float out_max = 0.0f;
    int out_size = tensor->get_size();
    for (int k = 0; k < out_size && k < 10000; k++) {
        float v = ai_tensor_value_as_float(tensor, k);
        if (isfinite(v) && v > out_max) {
            out_max = v;
        }
    }
    return out_max;
}

static void fill_monitor_tensor_context(ai_vision_monitor_sample_t *mon,
                                        dl::TensorBase *tensor)
{
    if (!mon || !tensor) {
        return;
    }
    std::vector<int> shape = tensor->get_shape();
    mon->output_dims = (int)shape.size();
    for (int i = 0; i < mon->output_dims && i < 4; ++i) {
        mon->output_shape[i] = shape[i];
    }
    mon->output_size = tensor->get_size();
    mon->output_dtype = tensor->get_dtype_string();
    mon->output_exponent = tensor->get_exponent();
    mon->output_max = ai_tensor_max_value(tensor);
}

static void log_output_tensor_once(const std::string &name,
                                   dl::TensorBase *tensor,
                                   uint32_t frame_id)
{
    if (frame_id != 1 || !tensor) {
        return;
    }

    std::vector<int> shape = tensor->get_shape();
    if (shape.size() == 3) {
        ESP_LOGI(TAG, "[AI] output %s shape=[%d,%d,%d] dtype=%s exp=%d",
                 name.c_str(), shape[0], shape[1], shape[2],
                 tensor->get_dtype_string(), tensor->get_exponent());
    } else {
        ESP_LOGI(TAG, "[AI] output %s dims=%u size=%d dtype=%s exp=%d",
                 name.c_str(), (unsigned)shape.size(), tensor->get_size(),
                 tensor->get_dtype_string(), tensor->get_exponent());
    }
}

static bool parse_yolo_output(dl::TensorBase *out_tensor,
                              ai_detection_t *det,
                              ai_vision_monitor_sample_t *mon)
{
    if (!out_tensor || !det) return false;
    *det = {};
    if (mon) {
        mon->parse_status = "start";
        mon->anchor_count = 0;
        mon->candidate_count = 0;
        mon->valid_candidate_count = 0;
        mon->best_anchor = -1;
    }

    std::vector<int> shape = out_tensor->get_shape();
    int output_size = out_tensor->get_size();
    int channels = 0;
    int anchors = 0;
    bool channels_first = true;

    if (shape.size() == 3) {
        if (shape[1] == 6) {
            channels = shape[1];
            anchors = shape[2];
            channels_first = true;
        } else if (shape[2] == 6) {
            channels = shape[2];
            anchors = shape[1];
            channels_first = false;
        }
    } else if (shape.size() == 2) {
        if (shape[0] == 6) {
            channels = shape[0];
            anchors = shape[1];
            channels_first = true;
        } else if (shape[1] == 6) {
            channels = shape[1];
            anchors = shape[0];
            channels_first = false;
        }
    }

    if (channels != 6 || anchors <= 0) {
        if (output_size % 6 == 0) {
            channels = 6;
            anchors = output_size / 6;
            channels_first = true;
        } else {
            if (!s_detection_output_warned) {
                ESP_LOGW(TAG, "[AI] Unsupported YOLO output: size=%d shape_dims=%u",
                         output_size, (unsigned)shape.size());
                s_detection_output_warned = true;
            }
            if (mon) {
                mon->parse_status = "unsupported_shape";
            }
            return false;
        }
    }
    if (mon) {
        mon->anchor_count = anchors;
    }

    auto get_value = [&](int anchor, int channel) -> float {
        int index = channels_first ? (channel * anchors + anchor) : (anchor * channels + channel);
        return ai_tensor_value_as_float(out_tensor, index);
    };

    float best_score = -1.0f;
    int best_anchor = -1;
    int best_class = 0;
    float best_normal = 0.0f;
    float best_scratch = 0.0f;

    // 诊断统计
    float score_min = 1e9f, score_max = -1e9f, score_sum = 0.0f;
    int score_finite_count = 0;
    struct score_rec_t { float normal; float scratch; float raw_normal; float raw_scratch; float score; int idx; };
    score_rec_t top_scores[10] = {};

    for (int i = 0; i < anchors; ++i) {
        float raw_normal = get_value(i, 4);
        float raw_scratch = get_value(i, 5);
        float normal = maybe_sigmoid_score(raw_normal) * AI_DET_SCORE_GAIN;
        float scratch = maybe_sigmoid_score(raw_scratch) * AI_DET_SCORE_GAIN;
        if (!isfinite(normal) || !isfinite(scratch)) {
            continue;
        }

        int class_id = (scratch > normal * AI_NORMAL_BIAS) ? 1 : 0;
        float score = (class_id == 1) ? scratch : normal;

        if (isfinite(score)) {
            score_sum += score;
            if (score < score_min) score_min = score;
            if (score > score_max) score_max = score;
            score_finite_count++;
            for (int k = 0; k < 10; ++k) {
                if (score > top_scores[k].score) {
                    for (int m = 9; m > k; --m) {
                        top_scores[m] = top_scores[m - 1];
                    }
                    top_scores[k].score = score;
                    top_scores[k].normal = normal;
                    top_scores[k].scratch = scratch;
                    top_scores[k].raw_normal = raw_normal;
                    top_scores[k].raw_scratch = raw_scratch;
                    top_scores[k].idx = i;
                    break;
                }
            }
        }

        if (score > best_score) {
            best_score = score;
            best_anchor = i;
            best_class = class_id;
            best_normal = normal;
            best_scratch = scratch;
            if (mon) {
                mon->best_anchor = i;
                mon->best_normal = normal;
                mon->best_scratch = scratch;
                mon->best_raw_cx = get_value(i, 0);
                mon->best_raw_cy = get_value(i, 1);
                mon->best_raw_w = get_value(i, 2);
                mon->best_raw_h = get_value(i, 3);
            }
        }

        if (score >= AI_DET_CONF_THRESHOLD && mon) {
            mon->candidate_count++;
            float cand_cx = get_value(i, 0);
            float cand_cy = get_value(i, 1);
            float cand_bw = get_value(i, 2);
            float cand_bh = get_value(i, 3);
            if (isfinite(cand_cx) && isfinite(cand_cy) &&
                isfinite(cand_bw) && isfinite(cand_bh) &&
                cand_bw > 0.0f && cand_bh > 0.0f) {
                if (cand_cx <= 2.0f && cand_cy <= 2.0f && cand_bw <= 2.0f && cand_bh <= 2.0f) {
                    cand_cx *= AI_INPUT_W;
                    cand_bw *= AI_INPUT_W;
                    cand_cy *= AI_INPUT_H;
                    cand_bh *= AI_INPUT_H;
                }

                float cand_x1 = (cand_cx - cand_bw * 0.5f - s_letterbox_pad_x) / s_letterbox_scale;
                float cand_y1 = (cand_cy - cand_bh * 0.5f - s_letterbox_pad_y) / s_letterbox_scale;
                float cand_x2 = (cand_cx + cand_bw * 0.5f - s_letterbox_pad_x) / s_letterbox_scale;
                float cand_y2 = (cand_cy + cand_bh * 0.5f - s_letterbox_pad_y) / s_letterbox_scale;
                cand_x1 = clamp_float(cand_x1, 0.0f, (float)(CAM_W - 1));
                cand_y1 = clamp_float(cand_y1, 0.0f, (float)(CAM_H - 1));
                cand_x2 = clamp_float(cand_x2, 0.0f, (float)(CAM_W - 1));
                cand_y2 = clamp_float(cand_y2, 0.0f, (float)(CAM_H - 1));
                if (round_to_int(cand_x2) > round_to_int(cand_x1) &&
                    round_to_int(cand_y2) > round_to_int(cand_y1)) {
                    mon->valid_candidate_count++;
                }
            }
        }
    }

    if (best_anchor < 0 || best_score < AI_DET_CONF_THRESHOLD) {
        if (mon) {
            mon->parse_status = (best_anchor < 0) ? "no_finite_score" : "below_threshold";
        }
        float score_mean = score_finite_count > 0 ? score_sum / score_finite_count : 0.0f;
        ESP_LOGI(TAG, "[AI_DIAG] score stats: min=%.4f max=%.4f mean=%.4f finite=%d/%d",
                 score_min, score_max, score_mean, score_finite_count, anchors);
        for (int k = 0; k < 10 && k < score_finite_count; ++k) {
            ESP_LOGI(TAG, "[AI_DIAG] top[%d] idx=%d normal=%.4f scratch=%.4f score=%.4f raw=(%.3f,%.3f)",
                     k, top_scores[k].idx, top_scores[k].normal, top_scores[k].scratch,
                     top_scores[k].score, top_scores[k].raw_normal, top_scores[k].raw_scratch);
        }
        return false;
    }

    float cx = get_value(best_anchor, 0);
    float cy = get_value(best_anchor, 1);
    float bw = get_value(best_anchor, 2);
    float bh = get_value(best_anchor, 3);

    if (!isfinite(cx) || !isfinite(cy) || !isfinite(bw) || !isfinite(bh) || bw <= 0.0f || bh <= 0.0f) {
        if (mon) {
            mon->parse_status = "best_box_invalid";
        }
        return false;
    }

    // Ultralytics ONNX 通常输出 320 输入空间内的像素坐标；如果是归一化坐标则放大。
    if (cx <= 2.0f && cy <= 2.0f && bw <= 2.0f && bh <= 2.0f) {
        cx *= AI_INPUT_W;
        bw *= AI_INPUT_W;
        cy *= AI_INPUT_H;
        bh *= AI_INPUT_H;
    }

    float x1_model = cx - bw * 0.5f;
    float y1_model = cy - bh * 0.5f;
    float x2_model = cx + bw * 0.5f;
    float y2_model = cy + bh * 0.5f;

    float x1 = (x1_model - s_letterbox_pad_x) / s_letterbox_scale;
    float y1 = (y1_model - s_letterbox_pad_y) / s_letterbox_scale;
    float x2 = (x2_model - s_letterbox_pad_x) / s_letterbox_scale;
    float y2 = (y2_model - s_letterbox_pad_y) / s_letterbox_scale;

    x1 = clamp_float(x1, 0.0f, (float)(CAM_W - 1));
    y1 = clamp_float(y1, 0.0f, (float)(CAM_H - 1));
    x2 = clamp_float(x2, 0.0f, (float)(CAM_W - 1));
    y2 = clamp_float(y2, 0.0f, (float)(CAM_H - 1));

    int left = round_to_int(x1);
    int top = round_to_int(y1);
    int right = round_to_int(x2);
    int bottom = round_to_int(y2);
    if (right <= left || bottom <= top) {
        if (mon) {
            mon->parse_status = "best_box_collapsed";
        }
        return false;
    }

    if (mon) {
        mon->parse_status = "ok";
    }
    det->present = true;
    det->class_id = best_class;
    det->score = best_score;
    det->normal_score = best_normal;
    det->scratch_score = best_scratch;
    det->x = left;
    det->y = top;
    det->w = right - left;
    det->h = bottom - top;
    return true;
}

/* ========================================================================== */
/* 初始化                                                                     */
/* ========================================================================== */

static bool parse_yolo_outputs(dl::TensorBase *box_tensor,
                               dl::TensorBase *score_tensor,
                               ai_detection_t *det,
                               ai_vision_monitor_sample_t *mon)
{
    if (!box_tensor || !score_tensor || !det) return false;
    *det = {};
    if (mon) {
        mon->parse_status = "two_output_start";
        mon->anchor_count = 0;
        mon->candidate_count = 0;
        mon->valid_candidate_count = 0;
        mon->best_anchor = -1;
    }

    ai_yolo_tensor_layout_t box_layout = {};
    ai_yolo_tensor_layout_t score_layout = {};
    if (!detect_yolo_tensor_layout(box_tensor, 4, &box_layout)) {
        if (mon) mon->parse_status = "bad_box_shape";
        return false;
    }
    if (!detect_yolo_tensor_layout(score_tensor, 2, &score_layout)) {
        if (mon) mon->parse_status = "bad_score_shape";
        return false;
    }
    if (box_layout.anchors != score_layout.anchors) {
        if (mon) mon->parse_status = "anchor_mismatch";
        return false;
    }
    if (mon) {
        mon->anchor_count = score_layout.anchors;
    }

    auto get_box = [&](int anchor, int channel) -> float {
        int index = box_layout.channels_first
            ? (channel * box_layout.anchors + anchor)
            : (anchor * 4 + channel);
        return ai_tensor_value_as_float(box_tensor, index);
    };

    auto get_score = [&](int anchor, int channel) -> float {
        int index = score_layout.channels_first
            ? (channel * score_layout.anchors + anchor)
            : (anchor * 2 + channel);
        return ai_tensor_value_as_float(score_tensor, index);
    };

    float best_score = -1.0f;
    int best_anchor = -1;
    int best_class = 0;
    float best_normal = 0.0f;
    float best_scratch = 0.0f;

    // 诊断统计
    float score_min = 1e9f, score_max = -1e9f, score_sum = 0.0f;
    int score_finite_count = 0;
    struct score_rec_t { float normal; float scratch; float raw_normal; float raw_scratch; float score; int idx; };
    score_rec_t top_scores[10] = {};

    for (int i = 0; i < score_layout.anchors; ++i) {
        float raw_normal = get_score(i, 0);
        float raw_scratch = get_score(i, 1);
        float normal = maybe_sigmoid_score(raw_normal) * AI_DET_SCORE_GAIN;
        float scratch = maybe_sigmoid_score(raw_scratch) * AI_DET_SCORE_GAIN;
        if (!isfinite(normal) || !isfinite(scratch)) {
            continue;
        }

        int class_id = (scratch > normal * AI_NORMAL_BIAS) ? 1 : 0;
        float score = (class_id == 1) ? scratch : normal;

        if (isfinite(score)) {
            score_sum += score;
            if (score < score_min) score_min = score;
            if (score > score_max) score_max = score;
            score_finite_count++;
            for (int k = 0; k < 10; ++k) {
                if (score > top_scores[k].score) {
                    for (int m = 9; m > k; --m) {
                        top_scores[m] = top_scores[m - 1];
                    }
                    top_scores[k].score = score;
                    top_scores[k].normal = normal;
                    top_scores[k].scratch = scratch;
                    top_scores[k].raw_normal = raw_normal;
                    top_scores[k].raw_scratch = raw_scratch;
                    top_scores[k].idx = i;
                    break;
                }
            }
        }

        if (score > best_score) {
            best_score = score;
            best_anchor = i;
            best_class = class_id;
            best_normal = normal;
            best_scratch = scratch;
            if (mon) {
                mon->best_anchor = i;
                mon->best_normal = normal;
                mon->best_scratch = scratch;
                mon->best_raw_cx = get_box(i, 0);
                mon->best_raw_cy = get_box(i, 1);
                mon->best_raw_w = get_box(i, 2);
                mon->best_raw_h = get_box(i, 3);
            }
        }

        if (score >= AI_DET_CONF_THRESHOLD && mon) {
            mon->candidate_count++;
            float cand_cx = get_box(i, 0);
            float cand_cy = get_box(i, 1);
            float cand_bw = get_box(i, 2);
            float cand_bh = get_box(i, 3);
            if (isfinite(cand_cx) && isfinite(cand_cy) &&
                isfinite(cand_bw) && isfinite(cand_bh) &&
                cand_bw > 0.0f && cand_bh > 0.0f) {
                if (cand_cx <= 2.0f && cand_cy <= 2.0f && cand_bw <= 2.0f && cand_bh <= 2.0f) {
                    cand_cx *= AI_INPUT_W;
                    cand_bw *= AI_INPUT_W;
                    cand_cy *= AI_INPUT_H;
                    cand_bh *= AI_INPUT_H;
                }

                float cand_x1 = (cand_cx - cand_bw * 0.5f - s_letterbox_pad_x) / s_letterbox_scale;
                float cand_y1 = (cand_cy - cand_bh * 0.5f - s_letterbox_pad_y) / s_letterbox_scale;
                float cand_x2 = (cand_cx + cand_bw * 0.5f - s_letterbox_pad_x) / s_letterbox_scale;
                float cand_y2 = (cand_cy + cand_bh * 0.5f - s_letterbox_pad_y) / s_letterbox_scale;
                cand_x1 = clamp_float(cand_x1, 0.0f, (float)(CAM_W - 1));
                cand_y1 = clamp_float(cand_y1, 0.0f, (float)(CAM_H - 1));
                cand_x2 = clamp_float(cand_x2, 0.0f, (float)(CAM_W - 1));
                cand_y2 = clamp_float(cand_y2, 0.0f, (float)(CAM_H - 1));
                if (round_to_int(cand_x2) > round_to_int(cand_x1) &&
                    round_to_int(cand_y2) > round_to_int(cand_y1)) {
                    mon->valid_candidate_count++;
                }
            }
        }
    }

    if (best_anchor < 0 || best_score < AI_DET_CONF_THRESHOLD) {
        if (mon) {
            mon->parse_status = (best_anchor < 0) ? "no_finite_score" : "below_threshold";
        }
        float score_mean = score_finite_count > 0 ? score_sum / score_finite_count : 0.0f;
        ESP_LOGI(TAG, "[AI_DIAG] score stats: min=%.4f max=%.4f mean=%.4f finite=%d/%d",
                 score_min, score_max, score_mean, score_finite_count, score_layout.anchors);
        for (int k = 0; k < 10 && k < score_finite_count; ++k) {
            ESP_LOGI(TAG, "[AI_DIAG] top[%d] idx=%d normal=%.4f scratch=%.4f score=%.4f raw=(%.3f,%.3f)",
                     k, top_scores[k].idx, top_scores[k].normal, top_scores[k].scratch,
                     top_scores[k].score, top_scores[k].raw_normal, top_scores[k].raw_scratch);
        }
        return false;
    }

    float cx = get_box(best_anchor, 0);
    float cy = get_box(best_anchor, 1);
    float bw = get_box(best_anchor, 2);
    float bh = get_box(best_anchor, 3);

    if (!isfinite(cx) || !isfinite(cy) || !isfinite(bw) || !isfinite(bh) || bw <= 0.0f || bh <= 0.0f) {
        if (mon) mon->parse_status = "best_box_invalid";
        return false;
    }

    if (cx <= 2.0f && cy <= 2.0f && bw <= 2.0f && bh <= 2.0f) {
        cx *= AI_INPUT_W;
        bw *= AI_INPUT_W;
        cy *= AI_INPUT_H;
        bh *= AI_INPUT_H;
    }

    float x1_model = cx - bw * 0.5f;
    float y1_model = cy - bh * 0.5f;
    float x2_model = cx + bw * 0.5f;
    float y2_model = cy + bh * 0.5f;

    float x1 = (x1_model - s_letterbox_pad_x) / s_letterbox_scale;
    float y1 = (y1_model - s_letterbox_pad_y) / s_letterbox_scale;
    float x2 = (x2_model - s_letterbox_pad_x) / s_letterbox_scale;
    float y2 = (y2_model - s_letterbox_pad_y) / s_letterbox_scale;

    x1 = clamp_float(x1, 0.0f, (float)(CAM_W - 1));
    y1 = clamp_float(y1, 0.0f, (float)(CAM_H - 1));
    x2 = clamp_float(x2, 0.0f, (float)(CAM_W - 1));
    y2 = clamp_float(y2, 0.0f, (float)(CAM_H - 1));

    int left = round_to_int(x1);
    int top = round_to_int(y1);
    int right = round_to_int(x2);
    int bottom = round_to_int(y2);
    if (right <= left || bottom <= top) {
        if (mon) mon->parse_status = "best_box_collapsed";
        return false;
    }

    if (mon) {
        mon->parse_status = "ok";
    }
    det->present = true;
    det->class_id = best_class;
    det->score = best_score;
    det->normal_score = best_normal;
    det->scratch_score = best_scratch;
    det->x = left;
    det->y = top;
    det->w = right - left;
    det->h = bottom - top;
    return true;
}

static bool is_classification_output(dl::TensorBase *tensor)
{
    if (!tensor || !tensor->get_element_ptr()) {
        return false;
    }
    std::vector<int> shape = tensor->get_shape();
    int size = tensor->get_size();
    // 支持 [1,2], [2], [1,1,2], [1,2,1] 等常见分类输出
    if (size != 2) {
        return false;
    }
    for (int dim : shape) {
        if (dim == 2) {
            return true;
        }
    }
    return shape.empty() || (shape.size() == 1 && shape[0] == 2);
}

static void softmax_2class(float a, float b, float *out_a, float *out_b)
{
    float max_val = (a > b) ? a : b;
    float exp_a = expf(a - max_val);
    float exp_b = expf(b - max_val);
    float sum = exp_a + exp_b;
    if (sum < 1e-9f) {
        *out_a = 0.5f;
        *out_b = 0.5f;
        return;
    }
    *out_a = exp_a / sum;
    *out_b = exp_b / sum;
}

static bool parse_classification_output(dl::TensorBase *out_tensor,
                                        ai_detection_t *det,
                                        ai_vision_monitor_sample_t *mon)
{
    if (!out_tensor || !det) return false;
    *det = {};
    if (mon) {
        mon->parse_status = "classification";
        mon->anchor_count = 1;
        mon->candidate_count = 1;
        mon->valid_candidate_count = 1;
        mon->best_anchor = 0;
    }

    int size = out_tensor->get_size();
    if (size != 2) {
        if (mon) mon->parse_status = "class_bad_size";
        return false;
    }

    float normal_logit = ai_tensor_value_as_float(out_tensor, 0);
    float scratch_logit = ai_tensor_value_as_float(out_tensor, 1);
    if (!isfinite(normal_logit) || !isfinite(scratch_logit)) {
        if (mon) mon->parse_status = "class_nonfinite";
        return false;
    }

    // 打印 raw int8 / float 值，方便确认 parser 读取正确
    if (out_tensor->get_dtype() == dl::DATA_TYPE_INT8) {
        const int8_t *raw = out_tensor->get_element_ptr<int8_t>();
        float scale = DL_SCALE(out_tensor->get_exponent());
        ESP_LOGI(TAG, "[CLASS_RAW] raw=[%d,%d] exp=%d scale=%.6f float=[%.4f,%.4f]",
                 (int)raw[0], (int)raw[1], out_tensor->get_exponent(), scale,
                 normal_logit, scratch_logit);
    }

    float normal_score = 0.0f;
    float scratch_score = 0.0f;
    softmax_2class(normal_logit, scratch_logit, &normal_score, &scratch_score);

    int class_id = (scratch_score > normal_score) ? 1 : 0;
    float score = (class_id == 1) ? scratch_score : normal_score;

    if (mon) {
        mon->best_normal = normal_score;
        mon->best_scratch = scratch_score;
    }

    // 分类模型不画框，框设为 0
    det->present = true;
    det->class_id = class_id;
    det->score = score;
    det->normal_score = normal_score;
    det->scratch_score = scratch_score;
    det->x = 0;
    det->y = 0;
    det->w = 0;
    det->h = 0;
    return true;
}

static bool parse_yolo_model_outputs(const std::map<std::string, dl::TensorBase*> &outputs,
                                     ai_detection_t *det,
                                     ai_vision_monitor_sample_t *mon,
                                     uint32_t frame_id)
{
    if (!det || !mon) {
        return false;
    }

    dl::TensorBase *single_tensor = nullptr;
    dl::TensorBase *box_tensor = nullptr;
    dl::TensorBase *score_tensor = nullptr;
    dl::TensorBase *class_tensor = nullptr;

    for (auto &kv : outputs) {
        dl::TensorBase *tensor = kv.second;
        if (!tensor || !tensor->get_element_ptr()) {
            continue;
        }
        log_output_tensor_once(kv.first, tensor, frame_id);

        if (!class_tensor && is_classification_output(tensor)) {
            class_tensor = tensor;
            continue;
        }

        ai_yolo_tensor_layout_t layout = {};
        if (!box_tensor && detect_yolo_tensor_layout(tensor, 4, &layout)) {
            box_tensor = tensor;
        } else if (!score_tensor && detect_yolo_tensor_layout(tensor, 2, &layout)) {
            score_tensor = tensor;
        } else if (!single_tensor) {
            single_tensor = tensor;
        }
    }

    // 优先处理分类输出
    if (class_tensor) {
        fill_monitor_tensor_context(mon, class_tensor);
        return parse_classification_output(class_tensor, det, mon);
    }

    dl::TensorBase *monitor_tensor = score_tensor ? score_tensor : single_tensor;
    if (!monitor_tensor) {
        monitor_tensor = box_tensor;
    }
    fill_monitor_tensor_context(mon, monitor_tensor);

    if (box_tensor && score_tensor) {
        return parse_yolo_outputs(box_tensor, score_tensor, det, mon);
    }
    if (single_tensor) {
        return parse_yolo_output(single_tensor, det, mon);
    }

    mon->parse_status = "no_parser_output";
    return false;
}

int ai_vision_init(void) {
    yolo_model = new dl::Model((const char *)model_espdl_start, fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    if (!yolo_model) {
        ESP_LOGE(TAG, "AI 模型加载失败");
        return -1;
    }
    ESP_LOGI(TAG, "AI 模型加载成功");
    return 0;
}

/* ========================================================================== */
/* AI 视觉推理任务 - LUT 加速 + 缓存 input_map + 最新帧索引                   */
/* ========================================================================== */
void ai_vision_task(void *pvParameters) {
    ESP_LOGI(TAG, "AI 任务启动，等待模型加载...");

    if (!lut_initialized) {
        s_letterbox_scale = fminf((float)AI_INPUT_W / (float)CAM_W, (float)AI_INPUT_H / (float)CAM_H);
        int resized_w = round_to_int((float)CAM_W * s_letterbox_scale);
        int resized_h = round_to_int((float)CAM_H * s_letterbox_scale);
        s_letterbox_pad_x = ((float)AI_INPUT_W - (float)resized_w) * 0.5f;
        s_letterbox_pad_y = ((float)AI_INPUT_H - (float)resized_h) * 0.5f;

        for (int x = 0; x < AI_INPUT_W; x++) {
            float src_x = ((float)x - s_letterbox_pad_x) / s_letterbox_scale;
            if (src_x < 0.0f || src_x > (float)(CAM_W - 1)) {
                src_x_lut[x] = -1;
            } else {
                src_x_lut[x] = clamp_int(round_to_int(src_x), 0, CAM_W - 1);
            }
        }
        for (int y = 0; y < AI_INPUT_H; y++) {
            float src_y = ((float)y - s_letterbox_pad_y) / s_letterbox_scale;
            if (src_y < 0.0f || src_y > (float)(CAM_H - 1)) {
                src_y_lut[y] = -1;
            } else {
                src_y_lut[y] = clamp_int(round_to_int(src_y), 0, CAM_H - 1);
            }
        }
        for (int i = 0; i < 32; i++) r5_to_r8[i] = (i << 3) | (i >> 2);
        for (int i = 0; i < 64; i++) g6_to_g8[i] = (i << 2) | (i >> 4);
        for (int i = 0; i < 32; i++) b5_to_b8[i] = (i << 3) | (i >> 2);
        lut_initialized = true;
        ESP_LOGI(TAG, "AI letterbox LUT ready: %dx%d -> %dx%d scale=%.4f pad=(%.1f, %.1f)",
                 CAM_W, CAM_H, resized_w, resized_h, s_letterbox_scale,
                 s_letterbox_pad_x, s_letterbox_pad_y);
    }

    while (1) {
        if (g_vision_state == VISION_STATE_RUNNING && yolo_model != nullptr && g_has_camera_frame) {
            auto &inputs = yolo_model->get_inputs();
            if (inputs.empty()) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            dl::TensorBase *input_tensor = inputs.begin()->second;

            if (!s_input_ready) {
                s_input_name = inputs.begin()->first;
                s_input_map[s_input_name] = input_tensor;
                s_input_ready = true;
                s_input_layout = detect_input_layout(input_tensor->get_shape());
                auto shape = input_tensor->get_shape();
                if (shape.size() == 4) {
                    ESP_LOGI(TAG, "Model input shape: [%d,%d,%d,%d], dtype: %s, exponent=%d, layout=%s",
                        shape[0], shape[1], shape[2], shape[3],
                        input_tensor->get_dtype_string(),
                        input_tensor->get_exponent(),
                        s_input_layout == AI_INPUT_LAYOUT_NCHW ? "NCHW" : "NHWC");
                    system_event_t dbg_ev = {};
                    dbg_ev.type = EVENT_AI_DEBUG;
                    snprintf(dbg_ev.log_msg, sizeof(dbg_ev.log_msg),
                             "IN [%d,%d,%d,%d] %s",
                             shape[0], shape[1], shape[2], shape[3],
                             s_input_layout == AI_INPUT_LAYOUT_NCHW ? "NCHW" : "NHWC");
                    xQueueSend(ui_event_queue, &dbg_ev, 0);
                } else {
                    ESP_LOGW(TAG, "Model input shape dims=%u, dtype: %s, exponent=%d",
                        (unsigned)shape.size(),
                        input_tensor->get_dtype_string(),
                        input_tensor->get_exponent());
                }
            }

            uint8_t buf_idx;
            bool buf_valid = false;
            portENTER_CRITICAL(&ppa_spinlock);
            buf_idx = g_ppa_read_idx;
            if (buf_idx < NUM_BUFS && cam_buff[buf_idx] != nullptr) {
                buf_valid = true;
                g_ai_reading_cam_buf_idx = buf_idx;  // 仅在确认有效后标记占用
            }
            portEXIT_CRITICAL(&ppa_spinlock);
            if (!buf_valid) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            uint16_t *src = (uint16_t *)cam_buff[buf_idx];

            int64_t t_preprocess_start = esp_timer_get_time();
            uint64_t luma_sum = 0;
            uint8_t luma_min = 255;
            uint8_t luma_max = 0;

            for (int y = 0; y < AI_INPUT_H; y++) {
                int src_y = src_y_lut[y];
                for (int x = 0; x < AI_INPUT_W; x++) {
                    int src_x = src_x_lut[x];
                    uint8_t r = 114;
                    uint8_t g = 114;
                    uint8_t b = 114;
                    if (src_x >= 0 && src_y >= 0) {
                        uint16_t pixel = src[src_y * CAM_W + src_x];
                        r = r5_to_r8[(pixel >> 11) & 0x1F];
                        g = g6_to_g8[(pixel >> 5) & 0x3F];
                        b = b5_to_b8[pixel & 0x1F];
                    }
                    uint8_t luma = (uint8_t)(((uint16_t)77 * r + (uint16_t)150 * g + (uint16_t)29 * b) >> 8);
                    luma_sum += luma;
                    if (luma < luma_min) luma_min = luma;
                    if (luma > luma_max) luma_max = luma;
                    ai_write_input_value(input_tensor, ai_input_index(0, y, x), r);
                    ai_write_input_value(input_tensor, ai_input_index(1, y, x), g);
                    ai_write_input_value(input_tensor, ai_input_index(2, y, x), b);
                }
            }

            portENTER_CRITICAL(&ppa_spinlock);
            g_ai_reading_cam_buf_idx = CAM_BUF_IDX_NONE;
            portEXIT_CRITICAL(&ppa_spinlock);
            int64_t t_preprocess_end = esp_timer_get_time();
            uint32_t input_pixels = AI_INPUT_W * AI_INPUT_H;
            uint8_t input_luma_mean = (uint8_t)(luma_sum / input_pixels);
            uint8_t input_luma_range = (uint8_t)(luma_max - luma_min);

            static uint32_t ai_run_cnt = 0;
            uint32_t ai_frame_id = ++ai_run_cnt;
            int preprocess_us = (int)(t_preprocess_end - t_preprocess_start);
            if (ai_frame_id <= 3) {
                ESP_LOGI(TAG, "[AI] run start frame=%lu pre_us=%d", (unsigned long)ai_frame_id, preprocess_us);
            }

            int64_t t_infer_start = esp_timer_get_time();
            yolo_model->run(s_input_map);
            int64_t t_infer_end = esp_timer_get_time();

            int inference_us = (int)(t_infer_end - t_infer_start);
            int total_us = (int)(t_infer_end - t_preprocess_start);
            if (ai_frame_id <= 3) {
                ESP_LOGI(TAG, "[AI] run done frame=%lu infer_us=%d total_us=%d", (unsigned long)ai_frame_id, inference_us, total_us);
            }
            if (ai_frame_id % 10 == 0) {
                ESP_LOGI(TAG, "[AI] preprocess=%lldus inference=%lldus total=%lldus",
                         (t_preprocess_end - t_preprocess_start),
                         (t_infer_end - t_infer_start),
                         (t_infer_end - t_preprocess_start));
            }

            auto &outputs = yolo_model->get_outputs();
            if (ai_frame_id <= 3) {
                ESP_LOGI(TAG, "[AI] output count frame=%lu count=%u",
                         (unsigned long)ai_frame_id, (unsigned)outputs.size());
            }
            if (!outputs.empty()) {
                ai_detection_t det = {};
                system_event_t ev = {};

                ai_vision_monitor_sample_t mon = {};
                mon.frame_id = ai_frame_id;
                mon.detected = false;
                mon.class_id = -1;
                mon.preprocess_us = preprocess_us;
                mon.inference_us = inference_us;
                mon.total_us = total_us;
                mon.input_luma_mean = input_luma_mean;
                mon.input_luma_range = input_luma_range;

                if (!parse_yolo_model_outputs(outputs, &det, &mon, ai_frame_id)) {
                    ai_log_monitor_sample(&mon);

                    g_sample_present = false;
                    g_sample_roi = {};
                    g_ai_ng_pct = 0.0f;
                    g_ai_ok_pct = 0.0f;
                    ev.type = EVENT_AI_NO_DETECTION;
                    snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[AI] NO SAMPLE");
                    if (xQueueSend(ui_event_queue, &ev, pdMS_TO_TICKS(20)) != pdPASS) {
                        ESP_LOGW(TAG, "UI event queue full, AI no-detection event dropped!");
                    }

                    system_event_t dbg_ev = {};
                    dbg_ev.type = EVENT_AI_DEBUG;
                    snprintf(dbg_ev.log_msg, sizeof(dbg_ev.log_msg),
                             "OUT dims=%d size=%d max=%.3f parse=%s",
                             mon.output_dims, mon.output_size, mon.output_max,
                             mon.parse_status ? mon.parse_status : "?");
                    xQueueSend(ui_event_queue, &dbg_ev, 0);

                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }

                sample_roi_t roi = {};
                roi.present = true;
                roi.x = det.x;
                roi.y = det.y;
                roi.w = det.w;
                roi.h = det.h;
                g_sample_roi = roi;
                g_sample_present = true;

                float biased_normal = det.normal_score * AI_NORMAL_BIAS;
                float score_sum = biased_normal + det.scratch_score;
                if (score_sum > 0.0001f) {
                    g_ai_ok_pct = biased_normal / score_sum * 100.0f;
                    g_ai_ng_pct = det.scratch_score / score_sum * 100.0f;
                } else {
                    g_ai_ok_pct = (det.class_id == 0) ? det.score * 100.0f : 0.0f;
                    g_ai_ng_pct = (det.class_id == 1) ? det.score * 100.0f : 0.0f;
                }

                ESP_LOGI(TAG,
                         "AI DET: class=%s score=%.3f normal=%.3f scratch=%.3f box=[%d,%d,%d,%d]",
                         det.class_id == 1 ? "scratch" : "normal",
                         det.score, det.normal_score, det.scratch_score,
                         det.x, det.y, det.w, det.h);

                mon.detected = true;
                mon.class_id = det.class_id;
                mon.score = det.score;
                mon.normal_score = det.normal_score;
                mon.scratch_score = det.scratch_score;
                mon.x = det.x;
                mon.y = det.y;
                mon.w = det.w;
                mon.h = det.h;
                ai_log_monitor_sample(&mon);

                {
                    system_event_t dbg_ev = {};
                    dbg_ev.type = EVENT_AI_DEBUG;
                    snprintf(dbg_ev.log_msg, sizeof(dbg_ev.log_msg),
                             "DET %s s=%.2f n=%.2f sc=%.2f box=%dx%d",
                             det.class_id == 1 ? "NG" : "OK",
                             det.score, det.normal_score, det.scratch_score,
                             det.w, det.h);
                    xQueueSend(ui_event_queue, &dbg_ev, 0);
                }

                if (det.class_id == 1) {
                    ev.type = EVENT_AI_DEFECT_FOUND;
                    snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[AI] SCRATCH NG");
                    if (xQueueSend(hw_cmd_queue, &ev, pdMS_TO_TICKS(100)) != pdPASS) {
                        ESP_LOGW(TAG, "HW cmd queue full, NG event dropped!");
                    }
                } else {
                    ev.type = EVENT_AI_PASS;
                    snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[AI] NORMAL OK");
                }
                if (xQueueSend(ui_event_queue, &ev, pdMS_TO_TICKS(100)) != pdPASS) {
                    ESP_LOGW(TAG, "UI event queue full, AI result event dropped!");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
