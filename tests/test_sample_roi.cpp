#include "../main/sample_roi.hpp"

#include <cassert>
#include <cstdint>
#include <vector>

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static std::vector<uint8_t> make_background_gray(int width, int height, uint8_t gray)
{
    std::vector<uint8_t> bg(width * height);
    for (uint8_t &v : bg) {
        v = gray;
    }
    return bg;
}

static std::vector<uint16_t> make_frame(int width, int height, uint16_t color)
{
    std::vector<uint16_t> frame(width * height);
    for (uint16_t &v : frame) {
        v = color;
    }
    return frame;
}

static void draws_no_roi_when_frame_matches_background()
{
    constexpr int width = 100;
    constexpr int height = 80;
    std::vector<uint8_t> bg = make_background_gray(width, height, 120);
    std::vector<uint16_t> frame = make_frame(width, height, rgb565(120, 120, 120));

    sample_roi_t roi = sample_roi_detect_rgb565(frame.data(), bg.data(), width, height);

    assert(!roi.present);
}

static void draws_roi_around_large_changed_object()
{
    constexpr int width = 200;
    constexpr int height = 160;
    std::vector<uint8_t> bg = make_background_gray(width, height, 40);
    std::vector<uint16_t> frame = make_frame(width, height, rgb565(40, 40, 40));

    for (int y = 30; y < 120; ++y) {
        for (int x = 25; x < 125; ++x) {
            frame[y * width + x] = rgb565(230, 230, 230);
        }
    }

    sample_roi_t roi = sample_roi_detect_rgb565(frame.data(), bg.data(), width, height);

    assert(roi.present);
    assert(roi.x == 25);
    assert(roi.y == 30);
    assert(roi.w == 100);
    assert(roi.h == 90);
}

static void ignores_small_changed_region()
{
    constexpr int width = 100;
    constexpr int height = 80;
    std::vector<uint8_t> bg = make_background_gray(width, height, 40);
    std::vector<uint16_t> frame = make_frame(width, height, rgb565(40, 40, 40));

    for (int y = 10; y < 20; ++y) {
        for (int x = 10; x < 20; ++x) {
            frame[y * width + x] = rgb565(230, 230, 230);
        }
    }

    sample_roi_t roi = sample_roi_detect_rgb565(frame.data(), bg.data(), width, height);

    assert(!roi.present);
}

static void draws_roi_around_demo_sized_sample()
{
    constexpr int width = 200;
    constexpr int height = 160;
    std::vector<uint8_t> bg = make_background_gray(width, height, 70);
    std::vector<uint16_t> frame = make_frame(width, height, rgb565(70, 70, 70));

    for (int y = 45; y < 95; ++y) {
        for (int x = 60; x < 110; ++x) {
            frame[y * width + x] = rgb565(120, 120, 120);
        }
    }

    sample_roi_t roi = sample_roi_detect_rgb565(frame.data(), bg.data(), width, height);

    assert(roi.present);
    assert(roi.x == 60);
    assert(roi.y == 45);
    assert(roi.w == 50);
    assert(roi.h == 50);
}

static void copies_rgb565_frame_to_grayscale_background()
{
    constexpr int width = 4;
    constexpr int height = 2;
    std::vector<uint16_t> frame = {
        rgb565(0, 0, 0),       rgb565(255, 255, 255),
        rgb565(255, 0, 0),     rgb565(0, 255, 0),
        rgb565(0, 0, 255),     rgb565(120, 120, 120),
        rgb565(200, 80, 30),   rgb565(30, 80, 200),
    };
    std::vector<uint8_t> bg(width * height, 0);

    sample_roi_capture_background_rgb565(frame.data(), bg.data(), width, height);

    assert(bg[0] < 5);
    assert(bg[1] > 245);
    assert(bg[5] >= 115 && bg[5] <= 125);
}

int main()
{
    draws_no_roi_when_frame_matches_background();
    draws_roi_around_large_changed_object();
    ignores_small_changed_region();
    draws_roi_around_demo_sized_sample();
    copies_rgb565_frame_to_grayscale_background();
    return 0;
}
