/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AUDIO_AI_MODEL_H
#define AUDIO_AI_MODEL_H

#include <math.h>
#include <stdint.h>

#define AUDIO_AI_FEATURE_DIM      12
#define AUDIO_AI_ZSCORE_THRESHOLD 3.000f

typedef enum {
    AUDIO_AI_GOOD = 0,
    AUDIO_AI_ANOMALY = 1,
} audio_ai_result_t;

static const char * const audio_ai_feature_names[AUDIO_AI_FEATURE_DIM] = {
    "rms",
    "rms_20ms",
    "rms_50ms",
    "zero_crossing_rate",
    "abs_mean",
    "energy_decay",
    "peak_dbfs",
    "spectral_centroid_hz",
    "spectral_rolloff_hz",
    "band_ratio_low",
    "band_ratio_mid",
    "band_ratio_high",
};

static const float audio_ai_mean[AUDIO_AI_FEATURE_DIM] = {
    17546.619627f, 18512.178765f, 18095.268725f, 0.226100f, 14859.195980f, 0.973724f, -0.000667f, 3347.017647f, 6245.409804f, 0.040357f, 0.169133f, 0.790512f
};

static const float audio_ai_inv_std[AUDIO_AI_FEATURE_DIM] = {
    0.001908f, 0.000762f, 0.000872f, 16.881036f, 0.001761f, 19.382272f, 1.000000f, 0.001929f, 0.002120f, 50.000000f, 16.031131f, 13.129229f
};

static inline audio_ai_result_t audio_ai_predict(const float *features)
{
    if (!features) return AUDIO_AI_ANOMALY;
    float max_zscore = 0.0f;
    for (int i = 0; i < AUDIO_AI_FEATURE_DIM; i++) {
        float diff = features[i] - audio_ai_mean[i];
        float zscore = fabsf(diff) * audio_ai_inv_std[i];
        if (zscore > max_zscore) {
            max_zscore = zscore;
        }
    }
    return (max_zscore <= AUDIO_AI_ZSCORE_THRESHOLD) ? AUDIO_AI_GOOD : AUDIO_AI_ANOMALY;
}

#endif // AUDIO_AI_MODEL_H
