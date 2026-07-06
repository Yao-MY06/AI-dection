#include "ai_yolo_output_layout.h"

#include <cassert>

static void detects_channels_first_three_dim_shape()
{
    int shape[] = {1, 4, 2100};
    ai_yolo_tensor_layout_t layout = {};

    bool ok = ai_yolo_detect_layout_from_shape(shape, 3, 1 * 4 * 2100, 4, &layout);

    assert(ok);
    assert(layout.anchors == 2100);
    assert(layout.channels_first);
}

static void detects_channels_last_three_dim_shape()
{
    int shape[] = {1, 2100, 2};
    ai_yolo_tensor_layout_t layout = {};

    bool ok = ai_yolo_detect_layout_from_shape(shape, 3, 1 * 2100 * 2, 2, &layout);

    assert(ok);
    assert(layout.anchors == 2100);
    assert(!layout.channels_first);
}

static void falls_back_to_flat_size_when_shape_is_not_specific()
{
    int shape[] = {8400};
    ai_yolo_tensor_layout_t layout = {};

    bool ok = ai_yolo_detect_layout_from_shape(shape, 1, 8400, 4, &layout);

    assert(ok);
    assert(layout.anchors == 2100);
    assert(layout.channels_first);
}

static void rejects_shape_that_cannot_match_expected_channels()
{
    int shape[] = {1, 5, 2100};
    ai_yolo_tensor_layout_t layout = {};

    bool ok = ai_yolo_detect_layout_from_shape(shape, 3, 1 * 5 * 2100, 4, &layout);

    assert(!ok);
}

int main()
{
    detects_channels_first_three_dim_shape();
    detects_channels_last_three_dim_shape();
    falls_back_to_flat_size_when_shape_is_not_specific();
    rejects_shape_that_cannot_match_expected_channels();
    return 0;
}
