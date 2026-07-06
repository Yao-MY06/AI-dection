#include "../main/ai_vision_monitor.h"

#include <cassert>
#include <cstring>
#include <string>

static std::string format_sample(const ai_vision_monitor_sample_t &sample)
{
    char line[640] = {};
    int written = ai_vision_monitor_format(line, sizeof(line), &sample, 724, 400);
    assert(written > 0);
    assert(written < (int)sizeof(line));
    return std::string(line);
}

static void formats_detected_sample_with_scores_timing_and_box_area()
{
    ai_vision_monitor_sample_t sample = {};
    sample.frame_id = 42;
    sample.detected = true;
    sample.class_id = 1;
    sample.score = 0.83f;
    sample.normal_score = 0.12f;
    sample.scratch_score = 0.83f;
    sample.x = 10;
    sample.y = 20;
    sample.w = 30;
    sample.h = 40;
    sample.output_dims = 3;
    sample.output_size = 5040;
    sample.output_shape[0] = 1;
    sample.output_shape[1] = 840;
    sample.output_shape[2] = 6;
    sample.output_dtype = "uint8";
    sample.output_exponent = 0;
    sample.output_max = 0.95f;
    sample.parse_status = "ok";
    sample.anchor_count = 840;
    sample.candidate_count = 6;
    sample.valid_candidate_count = 2;
    sample.best_anchor = 12;
    sample.best_raw_cx = 160.0f;
    sample.best_raw_cy = 100.0f;
    sample.best_raw_w = 60.0f;
    sample.best_raw_h = 80.0f;
    sample.best_normal = 0.12f;
    sample.best_scratch = 0.83f;
    sample.preprocess_us = 100;
    sample.inference_us = 200;
    sample.total_us = 320;
    sample.input_luma_mean = 128;
    sample.input_luma_range = 90;

    std::string line = format_sample(sample);

    assert(line.find("[AI_MON] frame=42 det=1 class=scratch") != std::string::npos);
    assert(line.find("score=0.830 normal=0.120 scratch=0.830 margin=0.710") != std::string::npos);
    assert(line.find("box=10,20,30,40 box_area=0.414") != std::string::npos);
    assert(line.find("pre_us=100 infer_us=200 total_us=320") != std::string::npos);
    assert(line.find("luma_mean=128 luma_range=90") != std::string::npos);
    assert(line.find("out_dims=3 out_size=5040 out_max=0.950") != std::string::npos);
    assert(line.find("shape=1x840x6 dtype=uint8 exp=0 parse=ok anchors=840 cand=6 valid=2") != std::string::npos);
    assert(line.find("best_anchor=12 best_n=0.120 best_s=0.830 raw_box=160.0,100.0,60.0,80.0") != std::string::npos);
}

static void formats_no_detection_sample_with_none_class_and_output_context()
{
    ai_vision_monitor_sample_t sample = {};
    sample.frame_id = 7;
    sample.detected = false;
    sample.output_dims = 2;
    sample.output_size = 6;
    sample.output_shape[0] = 6;
    sample.output_shape[1] = 1;
    sample.output_dtype = "float";
    sample.output_exponent = -8;
    sample.output_max = 0.41f;
    sample.parse_status = "below_threshold";
    sample.anchor_count = 1;
    sample.candidate_count = 0;
    sample.valid_candidate_count = 0;
    sample.best_anchor = 0;
    sample.best_raw_cx = 1.0f;
    sample.best_raw_cy = 2.0f;
    sample.best_raw_w = 3.0f;
    sample.best_raw_h = 4.0f;
    sample.best_normal = 0.22f;
    sample.best_scratch = 0.41f;
    sample.preprocess_us = 111;
    sample.inference_us = 222;
    sample.total_us = 333;
    sample.input_luma_mean = 64;
    sample.input_luma_range = 12;

    std::string line = format_sample(sample);

    assert(line.find("[AI_MON] frame=7 det=0 class=none") != std::string::npos);
    assert(line.find("score=0.000 normal=0.000 scratch=0.000 margin=0.000") != std::string::npos);
    assert(line.find("box=0,0,0,0 box_area=0.000") != std::string::npos);
    assert(line.find("luma_mean=64 luma_range=12") != std::string::npos);
    assert(line.find("out_dims=2 out_size=6 out_max=0.410") != std::string::npos);
    assert(line.find("shape=6x1 dtype=float exp=-8 parse=below_threshold anchors=1 cand=0 valid=0") != std::string::npos);
    assert(line.find("best_anchor=0 best_n=0.220 best_s=0.410 raw_box=1.0,2.0,3.0,4.0") != std::string::npos);
}

static void returns_none_for_unknown_class_ids()
{
    assert(std::strcmp(ai_vision_monitor_class_name(-1, true), "none") == 0);
    assert(std::strcmp(ai_vision_monitor_class_name(0, true), "normal") == 0);
    assert(std::strcmp(ai_vision_monitor_class_name(1, true), "scratch") == 0);
    assert(std::strcmp(ai_vision_monitor_class_name(2, true), "unknown") == 0);
    assert(std::strcmp(ai_vision_monitor_class_name(1, false), "none") == 0);
}

int main()
{
    formats_detected_sample_with_scores_timing_and_box_area();
    formats_no_detection_sample_with_none_class_and_output_context();
    returns_none_for_unknown_class_ids();
    return 0;
}
