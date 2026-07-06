#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool present;
    int x;
    int y;
    int w;
    int h;
} sample_roi_t;

typedef struct {
    uint8_t diff_threshold;
    int min_changed_pixels;
    int min_width;
    int min_height;
} sample_roi_config_t;

uint8_t sample_roi_rgb565_to_gray(uint16_t pixel);
sample_roi_config_t sample_roi_default_config(int width, int height);
void sample_roi_capture_background_rgb565(const uint16_t *frame, uint8_t *background, int width, int height);
sample_roi_t sample_roi_detect_rgb565(const uint16_t *frame, const uint8_t *background, int width, int height);
sample_roi_t sample_roi_detect_rgb565_with_config(const uint16_t *frame,
                                                  const uint8_t *background,
                                                  int width,
                                                  int height,
                                                  sample_roi_config_t config);
