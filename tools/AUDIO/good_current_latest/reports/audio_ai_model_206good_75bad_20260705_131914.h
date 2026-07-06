/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: CC0-1.0
 */

#ifndef AUDIO_AI_MODEL_H
#define AUDIO_AI_MODEL_H

#include <math.h>
#include <stdint.h>

#define AUDIO_AI_FEATURE_DIM      12
#define AUDIO_AI_ZSCORE_THRESHOLD 3.759f

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
    16825.752782f, 18232.911786f, 17983.806248f, 0.271537f, 14119.934714f, 0.989199f, -0.000840f, 3547.692718f, 6404.502913f, 0.038850f, 0.149800f, 0.811351f
};

static const float audio_ai_inv_std[AUDIO_AI_FEATURE_DIM] = {
    0.000716f, 0.000664f, 0.000669f, 13.403756f, 0.000700f, 8.817027f, 1.000000f, 0.001720f, 0.001633f, 40.555149f, 13.135419f, 10.434606f
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
