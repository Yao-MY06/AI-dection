#include "sample_roi.hpp"

#include <stdlib.h>

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int max_int(int a, int b)
{
    return a > b ? a : b;
}

uint8_t sample_roi_rgb565_to_gray(uint16_t pixel)
{
    uint8_t r5 = (pixel >> 11) & 0x1F;
    uint8_t g6 = (pixel >> 5) & 0x3F;
    uint8_t b5 = pixel & 0x1F;

    uint8_t r8 = (r5 << 3) | (r5 >> 2);
    uint8_t g8 = (g6 << 2) | (g6 >> 4);
    uint8_t b8 = (b5 << 3) | (b5 >> 2);

    return (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
}

sample_roi_config_t sample_roi_default_config(int width, int height)
{
    int frame_area = (width > 0 && height > 0) ? width * height : 0;

    sample_roi_config_t config = {};
    config.diff_threshold = 18;
    config.min_changed_pixels = max_int(frame_area / 200, 80);
    config.min_width = max_int(width / 20, 32);
    config.min_height = max_int(height / 20, 32);
    return config;
}

void sample_roi_capture_background_rgb565(const uint16_t *frame, uint8_t *background, int width, int height)
{
    if (!frame || !background || width <= 0 || height <= 0) {
        return;
    }

    int pixel_count = width * height;
    for (int i = 0; i < pixel_count; ++i) {
        background[i] = sample_roi_rgb565_to_gray(frame[i]);
    }
}

sample_roi_t sample_roi_detect_rgb565(const uint16_t *frame, const uint8_t *background, int width, int height)
{
    return sample_roi_detect_rgb565_with_config(frame, background, width, height,
                                                sample_roi_default_config(width, height));
}

sample_roi_t sample_roi_detect_rgb565_with_config(const uint16_t *frame,
                                                  const uint8_t *background,
                                                  int width,
                                                  int height,
                                                  sample_roi_config_t config)
{
    sample_roi_t roi = {};
    if (!frame || !background || width <= 0 || height <= 0) {
        return roi;
    }

    config.diff_threshold = config.diff_threshold == 0 ? 30 : config.diff_threshold;
    config.min_changed_pixels = clamp_int(config.min_changed_pixels, 1, width * height);
    config.min_width = config.min_width <= 0 ? 1 : config.min_width;
    config.min_height = config.min_height <= 0 ? 1 : config.min_height;

    int min_x = width;
    int min_y = height;
    int max_x = -1;
    int max_y = -1;
    int changed_pixels = 0;

    for (int y = 0; y < height; ++y) {
        int row_base = y * width;
        for (int x = 0; x < width; ++x) {
            int idx = row_base + x;
            uint8_t gray = sample_roi_rgb565_to_gray(frame[idx]);
            int diff = abs((int)gray - (int)background[idx]);
            if (diff > config.diff_threshold) {
                ++changed_pixels;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }
        }
    }

    if (changed_pixels < config.min_changed_pixels || max_x < min_x || max_y < min_y) {
        return roi;
    }

    int roi_w = max_x - min_x + 1;
    int roi_h = max_y - min_y + 1;
    if (roi_w < config.min_width || roi_h < config.min_height) {
        return roi;
    }

    roi.present = true;
    roi.x = min_x;
    roi.y = min_y;
    roi.w = roi_w;
    roi.h = roi_h;
    return roi;
}
