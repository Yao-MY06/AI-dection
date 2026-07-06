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
    1910.965000f, 13486.799000f, 8546.096500f, 0.000250f, 193.627000f, 0.000000f, -0.029000f, 1825.700000f, 4468.750000f, 0.194100f, 0.348300f, 0.457650f
};

static const float audio_ai_inv_std[AUDIO_AI_FEATURE_DIM] = {
    0.007937f, 0.001122f, 0.001775f, 8000.000000f, 0.042488f, 0.000000f, 125.000000f, 0.019048f, 0.010667f, 65.359477f, 3333.333333f, 66.889632f
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
