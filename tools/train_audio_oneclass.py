#!/usr/bin/env python3
"""
训练单分类音频异常检测模型（支持 good/bad 样本和频域特征）。

用法：
    python tools/train_audio_oneclass.py \
        --features-dir tools/AUDIO/features \
        --good-label good \
        --bad-label bad \
        --out-c main/audio_ai_model.h

数据组织方式（二选一）：

方式 A - 按文件名前缀：
    features/good_*.json  -> 正常样本
    features/bad_*.json   -> 异常样本（用于阈值校准）

方式 B - 按 JSON 内 label 字段：
    features/*.json 中每个样本包含 "label": "good" 或 "label": "bad"

说明：
    --good-label 和 --bad-label 可以指定前缀或 label 字段值。
    如果只提供 good 样本，bad 样本用于校准阈值；不提供 bad 样本时退化为 zscore_threshold。
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# 用于模型训练的特征（与板端 audio_ai_extract_features 保持一致）
AI_FEATURES = [
    # 时域特征
    "rms",
    "rms_20ms",
    "rms_50ms",
    "zero_crossing_rate",
    "abs_mean",
    "energy_decay",
    "peak_dbfs",
    # 频域特征
    "spectral_centroid_hz",
    "spectral_rolloff_hz",
    "band_ratio_low",
    "band_ratio_mid",
    "band_ratio_high",
]

DEFAULT_ZSCORE_THRESHOLD = 3.0

# Per-feature std floors prevent tiny sample variance from over-amplifying normal drift.
FEATURE_STD_FLOORS = {
    "rms": 100.0,
    "rms_20ms": 500.0,
    "rms_50ms": 300.0,
    "zero_crossing_rate": 0.001,
    "abs_mean": 20.0,
    "energy_decay": 0.02,
    "peak_dbfs": 1.0,
    "spectral_centroid_hz": 50.0,
    "spectral_rolloff_hz": 100.0,
    "band_ratio_low": 0.02,
    "band_ratio_mid": 0.02,
    "band_ratio_high": 0.02,
}


def load_features(features_dir: Path, good_label: str, bad_label: str) -> Tuple[List[Dict], List[Dict]]:
    features_dir = Path(features_dir)
    if not features_dir.exists():
        raise FileNotFoundError(f"features dir not found: {features_dir}")

    good_samples = []
    bad_samples = []

    for json_path in sorted(features_dir.glob("*.json")):
        data = json.loads(json_path.read_text(encoding="utf-8"))
        name = data.get("name", json_path.stem)
        label = data.get("label", None)

        # 优先用 JSON 内的 label 字段
        if label == good_label:
            good_samples.append(data)
        elif label == bad_label:
            bad_samples.append(data)
        # 其次按文件名前缀匹配
        elif name.startswith(good_label):
            good_samples.append(data)
        elif name.startswith(bad_label):
            bad_samples.append(data)

    return good_samples, bad_samples


def extract_vector(sample: Dict) -> List[float]:
    return [float(sample.get(k, 0.0)) for k in AI_FEATURES]


def compute_mean_std(vectors: List[List[float]]) -> Tuple[List[float], List[float]]:
    dim = len(AI_FEATURES)
    n = len(vectors)
    means = [0.0] * dim
    stds = [0.0] * dim

    for vec in vectors:
        for i in range(dim):
            means[i] += vec[i]
    means = [m / max(1, n) for m in means]

    for vec in vectors:
        for i in range(dim):
            stds[i] += (vec[i] - means[i]) ** 2
    stds = [math.sqrt(s / max(1, n)) for s in stds]

    return means, stds


def max_zscore(vec: List[float], means: List[float], inv_stds: List[float]) -> float:
    return max(abs(v - m) * inv for v, m, inv in zip(vec, means, inv_stds))


def find_threshold(good_vectors: List[List[float]], bad_vectors: List[List[float]],
                   means: List[float], inv_stds: List[float]) -> float:
    """用 bad 样本找阈值：让所有 good 通过，所有 bad 拦截。"""
    if not bad_vectors:
        return DEFAULT_ZSCORE_THRESHOLD

    good_z = [max_zscore(v, means, inv_stds) for v in good_vectors]
    bad_z = [max_zscore(v, means, inv_stds) for v in bad_vectors]

    good_max = max(good_z) if good_z else 0.0
    bad_min = min(bad_z) if bad_z else float("inf")

    print(f"  good max zscore: {good_max:.3f}")
    print(f"  bad  min zscore: {bad_min:.3f}")

    # 理想阈值在 good_max 和 bad_min 之间，留 10% margin
    if good_max < bad_min:
        threshold = good_max + (bad_min - good_max) * 0.5
        print(f"  auto threshold: {threshold:.3f} (between good/bad)")
        return threshold

    # 有重叠时，取 good 的 95 分位数再稍微放宽
    sorted_good = sorted(good_z)
    p95_idx = int(len(sorted_good) * 0.95)
    threshold = sorted_good[min(p95_idx, len(sorted_good) - 1)] * 1.2
    print(f"  warning: good/bad overlap, fallback threshold: {threshold:.3f}")
    return threshold


def train_oneclass(good_samples: List[Dict], bad_samples: List[Dict], zscore_threshold: float = None):
    if len(good_samples) < 2:
        print(f"[WARN] only {len(good_samples)} good sample(s); model will be weak. collect more.")

    good_vectors = [extract_vector(s) for s in good_samples]
    bad_vectors = [extract_vector(s) for s in bad_samples]

    means, raw_stds = compute_mean_std(good_vectors)
    stds = [
        max(raw_stds[i], FEATURE_STD_FLOORS.get(AI_FEATURES[i], 1e-9))
        for i in range(len(AI_FEATURES))
    ]
    inv_stds = [1.0 / s if s > 1e-9 else 0.0 for s in stds]

    if zscore_threshold is None and bad_vectors:
        zscore_threshold = find_threshold(good_vectors, bad_vectors, means, inv_stds)
    elif zscore_threshold is None:
        zscore_threshold = DEFAULT_ZSCORE_THRESHOLD

    print(f"trained on {len(good_vectors)} good samples, {len(bad_vectors)} bad samples, dim={len(AI_FEATURES)}, threshold={zscore_threshold:.3f}")
    print("feature means:", [round(m, 4) for m in means])
    print("feature stds: ", [round(s, 4) for s in stds], "(floored)")
    return means, inv_stds, zscore_threshold


def generate_c_header(means: List[float], inv_stds: List[float], threshold: float, out_path: Path):
    dim = len(AI_FEATURES)
    lines = [
        "/*",
        " * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD",
        " * SPDX-License-Identifier: CC0-1.0",
        " */",
        "",
        "#ifndef AUDIO_AI_MODEL_H",
        "#define AUDIO_AI_MODEL_H",
        "",
        "#include <math.h>",
        "#include <stdint.h>",
        "",
        f"#define AUDIO_AI_FEATURE_DIM      {dim}",
        f"#define AUDIO_AI_ZSCORE_THRESHOLD {threshold:.3f}f",
        "",
        "typedef enum {",
        "    AUDIO_AI_GOOD = 0,",
        "    AUDIO_AI_ANOMALY = 1,",
        "} audio_ai_result_t;",
        "",
        "static const char * const audio_ai_feature_names[AUDIO_AI_FEATURE_DIM] = {",
    ]
    for name in AI_FEATURES:
        lines.append(f'    "{name}",')
    lines.append("};")
    lines.append("")

    lines.append("static const float audio_ai_mean[AUDIO_AI_FEATURE_DIM] = {")
    lines.append("    " + ", ".join(f"{m:.6f}f" for m in means))
    lines.append("};")
    lines.append("")

    lines.append("static const float audio_ai_inv_std[AUDIO_AI_FEATURE_DIM] = {")
    lines.append("    " + ", ".join(f"{s:.6f}f" for s in inv_stds))
    lines.append("};")
    lines.append("")

    lines.extend([
        "static inline audio_ai_result_t audio_ai_predict(const float *features)",
        "{",
        "    if (!features) return AUDIO_AI_ANOMALY;",
        "    float max_zscore = 0.0f;",
        "    for (int i = 0; i < AUDIO_AI_FEATURE_DIM; i++) {",
        "        float diff = features[i] - audio_ai_mean[i];",
        "        float zscore = fabsf(diff) * audio_ai_inv_std[i];",
        "        if (zscore > max_zscore) {",
        "            max_zscore = zscore;",
        "        }",
        "    }",
        "    return (max_zscore <= AUDIO_AI_ZSCORE_THRESHOLD) ? AUDIO_AI_GOOD : AUDIO_AI_ANOMALY;",
        "}",
        "",
        "#endif // AUDIO_AI_MODEL_H",
        "",
    ])

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"generated C header: {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--features-dir", type=Path, required=True)
    parser.add_argument("--good-label", type=str, default="good",
                        help="正常样本标识：文件名前缀或 JSON label 字段值")
    parser.add_argument("--bad-label", type=str, default="bad",
                        help="异常样本标识：文件名前缀或 JSON label 字段值")
    parser.add_argument("--zscore-threshold", type=float, default=None,
                        help="指定阈值；不指定则用 bad 样本自动校准")
    parser.add_argument("--out-c", type=Path, default=Path("main/audio_ai_model.h"))
    args = parser.parse_args()

    good_samples, bad_samples = load_features(args.features_dir, args.good_label, args.bad_label)

    if not good_samples:
        print("[ERROR] no good samples found")
        sys.exit(1)

    print(f"loaded {len(good_samples)} good samples, {len(bad_samples)} bad samples")

    means, inv_stds, threshold = train_oneclass(good_samples, bad_samples, args.zscore_threshold)
    generate_c_header(means, inv_stds, threshold, args.out_c)

    # 自检
    print("\nself-check:")
    for s in good_samples:
        vec = extract_vector(s)
        max_z = max_zscore(vec, means, inv_stds)
        result = "GOOD" if max_z <= threshold else "ANOMALY"
        print(f"  {s['name']}: max_z={max_z:.3f} -> {result}")
    for s in bad_samples:
        vec = extract_vector(s)
        max_z = max_zscore(vec, means, inv_stds)
        result = "GOOD" if max_z <= threshold else "ANOMALY"
        print(f"  {s['name']}: max_z={max_z:.3f} -> {result}")


if __name__ == "__main__":
    main()
