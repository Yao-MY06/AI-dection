/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AUDIO_AI_MODEL_H
#define AUDIO_AI_MODEL_H

#include <math.h>
#include <stdint.h>

#define AUDIO_AI_FEATURE_DIM      12
#define AUDIO_AI_ZSCORE_THRESHOLD 3.725f

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
    16849.310703f, 18313.924243f, 18069.007485f, 0.272563f, 14144.214911f, 0.990293f, -0.000842f, 3577.344059f, 6446.083663f, 0.037267f, 0.145225f, 0.817510f
};

static const float audio_ai_inv_std[AUDIO_AI_FEATURE_DIM] = {
    0.000720f, 0.000736f, 0.000732f, 13.389665f, 0.000704f, 8.819455f, 1.000000f, 0.001847f, 0.001942f, 45.498580f, 15.163422f, 11.995970f
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
