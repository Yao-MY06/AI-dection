#!/usr/bin/env python3
"""Batch convert ESP32-P4 raw PCM audio and extract quick inspection features.

Default workspace:
  qw/       original .raw PCM16LE files
  wav/      converted .wav files for listening
  features/ per-file JSON feature records
  plots/    optional waveform/spectrum PNGs
  reports/  CSV summary
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import struct
import wave
from pathlib import Path
from typing import Dict, Iterable, List, Optional


DEFAULT_SAMPLE_RATE = 16000
DEFAULT_CHANNELS = 1
DEFAULT_SAMPLE_WIDTH = 2

RAW_DIR_NAME = "qw"
WAV_DIR_NAME = "wav"
FEATURE_DIR_NAME = "features"
PLOT_DIR_NAME = "plots"
REPORT_DIR_NAME = "reports"

SUMMARY_FIELDS = [
    "name",
    "raw_path",
    "wav_path",
    "sample_rate",
    "sample_count",
    "duration_s",
    "mean",
    "dc_offset",
    "peak",
    "peak_dbfs",
    "rms",
    "rms_dbfs",
    "abs_mean",
    "zero_crossing_rate",
    "clipped_ratio",
    "peak_time_ms",
    "rms_20ms",
    "rms_50ms",
    "energy_decay",
    "dominant_freq_hz",
    "spectral_centroid_hz",
    "spectral_rolloff_hz",
    "band_ratio_low",
    "band_ratio_mid",
    "band_ratio_high",
]


def workspace_paths(base_dir: Path) -> Dict[str, Path]:
    base_dir = Path(base_dir)
    return {
        "base": base_dir,
        "raw": base_dir / RAW_DIR_NAME,
        "wav": base_dir / WAV_DIR_NAME,
        "features": base_dir / FEATURE_DIR_NAME,
        "plots": base_dir / PLOT_DIR_NAME,
        "reports": base_dir / REPORT_DIR_NAME,
    }


def ensure_workspace(base_dir: Path) -> Dict[str, Path]:
    paths = workspace_paths(base_dir)
    for key in ("raw", "wav", "features", "plots", "reports"):
        paths[key].mkdir(parents=True, exist_ok=True)
    return paths


def iter_raw_files(raw_dir: Path) -> Iterable[Path]:
    return sorted(path for path in Path(raw_dir).glob("*.raw") if path.is_file())


def read_pcm16le(raw_path: Path) -> List[int]:
    data = Path(raw_path).read_bytes()
    if len(data) % DEFAULT_SAMPLE_WIDTH != 0:
        raise ValueError(f"{raw_path} byte length is not aligned to int16 samples: {len(data)}")
    if not data:
        return []
    return list(struct.unpack("<" + "h" * (len(data) // DEFAULT_SAMPLE_WIDTH), data))


def write_wav(wav_path: Path, samples: List[int], sample_rate: int) -> None:
    wav_path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(wav_path), "wb") as wav_file:
        wav_file.setnchannels(DEFAULT_CHANNELS)
        wav_file.setsampwidth(DEFAULT_SAMPLE_WIDTH)
        wav_file.setframerate(sample_rate)
        frames = struct.pack("<" + "h" * len(samples), *samples) if samples else b""
        wav_file.writeframes(frames)


def safe_dbfs(value: float) -> float:
    if value <= 0.0:
        return -120.0
    return round(20.0 * math.log10(value / 32768.0), 3)


def compute_frequency_features(samples: List[int], sample_rate: int, fft_size: int = None) -> Dict[str, float]:
    """提取频域特征。

    默认使用整段音频做 FFT；指定 fft_size 时，取峰值附近 fft_size 个样本做 FFT，
    与 ESP32 板端 1024 点 FFT 保持一致。
    """
    defaults = {
        "dominant_freq_hz": 0.0,
        "spectral_centroid_hz": 0.0,
        "spectral_rolloff_hz": 0.0,
        "band_ratio_low": 0.0,
        "band_ratio_mid": 0.0,
        "band_ratio_high": 0.0,
    }
    if len(samples) < 2:
        return defaults

    try:
        import numpy as np

        arr_full = np.asarray(samples, dtype=np.float64)
        effective_fft_size = int(fft_size) if fft_size else int(arr_full.size)
        if effective_fft_size <= 1:
            return defaults

        # With fft_size, mirror the board path: peak-centered window, zero-padded to the fixed FFT size.
        if fft_size:
            peak_idx = int(np.argmax(np.abs(arr_full)))
            half = effective_fft_size // 2
            start = max(0, peak_idx - half)
            end = start + effective_fft_size
            if end > len(arr_full):
                end = len(arr_full)
                start = max(0, end - effective_fft_size)
            window_samples = arr_full[start:end].copy()
            arr = np.zeros(effective_fft_size, dtype=np.float64)
            if window_samples.size:
                arr[: window_samples.size] = window_samples - float(np.mean(window_samples))
        else:
            arr = arr_full.copy()
            arr = arr - float(np.mean(arr))

        if not np.any(arr):
            return defaults

        window = np.hanning(effective_fft_size)
        spectrum = np.abs(np.fft.rfft(arr * window))
        freqs = np.fft.rfftfreq(effective_fft_size, d=1.0 / sample_rate)
        bins = effective_fft_size // 2
        if bins <= 1 or spectrum.size <= bins:
            return defaults

        # Match the board path: skip DC and leave the Nyquist bin out of energy totals.
        valid_spectrum = spectrum[1:bins]
        valid_freqs = freqs[1:bins]
        total = float(np.sum(valid_spectrum))
        if total <= 0.0:
            return defaults

        dominant_freq = float(valid_freqs[int(np.argmax(valid_spectrum))])
        centroid = float(np.sum(valid_freqs * valid_spectrum) / total)
        cumsum = np.cumsum(valid_spectrum)
        rolloff_idx = int(np.searchsorted(cumsum, 0.85 * total))
        rolloff = float(valid_freqs[min(rolloff_idx, len(valid_freqs) - 1)])

        low_mask = valid_freqs < 200.0
        mid_mask = (valid_freqs >= 200.0) & (valid_freqs < 1000.0)
        high_mask = valid_freqs >= 1000.0
        low_energy = float(np.sum(valid_spectrum[low_mask]))
        mid_energy = float(np.sum(valid_spectrum[mid_mask]))
        high_energy = float(np.sum(valid_spectrum[high_mask]))

        return {
            "dominant_freq_hz": round(dominant_freq, 1),
            "spectral_centroid_hz": round(centroid, 1),
            "spectral_rolloff_hz": round(rolloff, 1),
            "band_ratio_low": round(low_energy / total, 4),
            "band_ratio_mid": round(mid_energy / total, 4),
            "band_ratio_high": round(high_energy / total, 4),
        }
    except ImportError:
        return defaults


def compute_peak_time_ms(samples: List[int], sample_rate: int) -> float:
    if not samples:
        return 0.0
    peak_idx = max(range(len(samples)), key=lambda i: abs(samples[i]))
    return round(peak_idx / sample_rate * 1000.0, 3)


def compute_window_rms(samples: List[int], center_idx: int, window_ms: float, sample_rate: int) -> float:
    if not samples or center_idx < 0:
        return 0.0
    half_samples = int((window_ms / 1000.0) * sample_rate / 2)
    start = max(0, center_idx - half_samples)
    end = min(len(samples), center_idx + half_samples)
    if start >= end:
        return 0.0
    window = samples[start:end]
    return round(math.sqrt(sum(float(v) * float(v) for v in window) / len(window)), 3)


def compute_energy_decay(samples: List[int]) -> float:
    if len(samples) < 2:
        return 1.0
    mid = len(samples) // 2
    first = samples[:mid]
    second = samples[mid:]
    rms_first = math.sqrt(sum(float(v) * float(v) for v in first) / len(first)) if first else 0.0
    rms_second = math.sqrt(sum(float(v) * float(v) for v in second) / len(second)) if second else 0.0
    if rms_first <= 0.0:
        return 1.0
    return round(rms_second / rms_first, 4)


def compute_features(raw_path: Path, samples: List[int], sample_rate: int, wav_path: Path) -> Dict[str, object]:
    sample_count = len(samples)
    duration_s = round(sample_count / float(sample_rate), 6) if sample_rate > 0 else 0.0
    if sample_count == 0:
        base = {
            "name": raw_path.stem,
            "raw_path": str(raw_path),
            "wav_path": str(wav_path),
            "sample_rate": sample_rate,
            "sample_count": 0,
            "duration_s": 0.0,
            "mean": 0.0,
            "dc_offset": 0.0,
            "peak": 0,
            "peak_dbfs": -120.0,
            "rms": 0.0,
            "rms_dbfs": -120.0,
            "abs_mean": 0.0,
            "zero_crossing_rate": 0.0,
            "clipped_ratio": 0.0,
        }
        base.update({
            "peak_time_ms": 0.0,
            "rms_20ms": 0.0,
            "rms_50ms": 0.0,
            "energy_decay": 1.0,
            "dominant_freq_hz": 0.0,
            "spectral_centroid_hz": 0.0,
            "spectral_rolloff_hz": 0.0,
            "band_ratio_low": 0.0,
            "band_ratio_mid": 0.0,
            "band_ratio_high": 0.0,
        })
        return base

    total = sum(samples)
    mean = total / sample_count
    abs_values = [abs(v) for v in samples]
    peak = max(abs_values)
    rms = math.sqrt(sum(float(v) * float(v) for v in samples) / sample_count)
    zero_crossings = 0
    previous = samples[0]
    for current in samples[1:]:
        if (previous < 0 <= current) or (previous >= 0 > current):
            zero_crossings += 1
        previous = current
    clipped = sum(1 for v in samples if v <= -32768 or v >= 32767)

    peak_idx = max(range(sample_count), key=lambda i: abs(samples[i]))

    features: Dict[str, object] = {
        "name": raw_path.stem,
        "raw_path": str(raw_path),
        "wav_path": str(wav_path),
        "sample_rate": sample_rate,
        "sample_count": sample_count,
        "duration_s": duration_s,
        "mean": round(mean, 3),
        "dc_offset": round(mean, 3),
        "peak": int(peak),
        "peak_dbfs": safe_dbfs(float(peak)),
        "rms": round(rms, 3),
        "rms_dbfs": safe_dbfs(rms),
        "abs_mean": round(sum(abs_values) / sample_count, 3),
        "zero_crossing_rate": round(zero_crossings / max(1, sample_count - 1), 6),
        "clipped_ratio": round(clipped / sample_count, 6),
        "peak_time_ms": compute_peak_time_ms(samples, sample_rate),
        "rms_20ms": compute_window_rms(samples, peak_idx, 20.0, sample_rate),
        "rms_50ms": compute_window_rms(samples, peak_idx, 50.0, sample_rate),
        "energy_decay": compute_energy_decay(samples),
    }
    features.update(compute_frequency_features(samples, sample_rate, fft_size=1024))
    return features


def write_feature_json(feature_path: Path, features: Dict[str, object]) -> None:
    feature_path.parent.mkdir(parents=True, exist_ok=True)
    feature_path.write_text(json.dumps(features, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_summary_csv(report_path: Path, rows: List[Dict[str, object]]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in SUMMARY_FIELDS})


def write_plot(plot_path: Path, samples: List[int], sample_rate: int) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        return False

    if not samples:
        return False

    arr = np.asarray(samples, dtype=np.float64)
    time_axis = np.arange(arr.size) / float(sample_rate)
    centered = arr - np.mean(arr)
    spectrum = np.abs(np.fft.rfft(centered * np.hanning(arr.size)))
    freqs = np.fft.rfftfreq(arr.size, d=1.0 / sample_rate)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax_wave, ax_spec) = plt.subplots(2, 1, figsize=(10, 6))
    ax_wave.plot(time_axis, arr, linewidth=0.7)
    ax_wave.set_title("Waveform")
    ax_wave.set_xlabel("Time (s)")
    ax_wave.set_ylabel("Amplitude")
    ax_wave.grid(True, alpha=0.25)

    ax_spec.plot(freqs, spectrum, linewidth=0.7)
    ax_spec.set_title("Spectrum")
    ax_spec.set_xlabel("Frequency (Hz)")
    ax_spec.set_ylabel("Magnitude")
    ax_spec.set_xlim(0, min(8000, sample_rate / 2))
    ax_spec.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=140)
    plt.close(fig)
    return True


def import_raw_files(source_dir: Path, raw_dir: Path, copy: bool = True) -> int:
    source_dir = Path(source_dir)
    raw_dir = Path(raw_dir)
    if not source_dir.exists():
        raise FileNotFoundError(f"import source does not exist: {source_dir}")
    raw_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    for source in iter_raw_files(source_dir):
        target = raw_dir / source.name
        if source.resolve() == target.resolve():
            continue
        if copy:
            shutil.copy2(source, target)
        else:
            shutil.move(str(source), str(target))
        count += 1
    return count


def process_directory(base_dir: Path, sample_rate: int = DEFAULT_SAMPLE_RATE, make_plots: bool = True) -> Dict[str, object]:
    paths = ensure_workspace(base_dir)
    rows: List[Dict[str, object]] = []
    plot_count = 0

    for raw_path in iter_raw_files(paths["raw"]):
        samples = read_pcm16le(raw_path)
        wav_path = paths["wav"] / f"{raw_path.stem}.wav"
        feature_path = paths["features"] / f"{raw_path.stem}.json"
        plot_path = paths["plots"] / f"{raw_path.stem}.png"

        write_wav(wav_path, samples, sample_rate)
        features = compute_features(raw_path, samples, sample_rate, wav_path)
        write_feature_json(feature_path, features)
        rows.append(features)
        if make_plots and write_plot(plot_path, samples, sample_rate):
            plot_count += 1

    report_path = paths["reports"] / "audio_summary.csv"
    write_summary_csv(report_path, rows)
    return {
        "processed": len(rows),
        "plots": plot_count,
        "report": str(report_path),
        "base_dir": str(paths["base"]),
    }


def parse_args() -> argparse.Namespace:
    default_base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Convert qw/*.raw PCM16LE audio to wav and feature reports.")
    parser.add_argument("--base-dir", type=Path, default=default_base, help="Audio workspace directory.")
    parser.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE, help="Raw PCM sample rate.")
    parser.add_argument("--no-plots", action="store_true", help="Skip waveform/spectrum PNG generation.")
    parser.add_argument("--import-from", type=Path, help="Copy .raw files from another folder into qw before processing.")
    parser.add_argument("--move-imported", action="store_true", help="Move imported .raw files instead of copying.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = ensure_workspace(args.base_dir)
    imported = 0
    if args.import_from:
        imported = import_raw_files(args.import_from, paths["raw"], copy=not args.move_imported)
    summary = process_directory(args.base_dir, sample_rate=args.sample_rate, make_plots=not args.no_plots)
    print(f"workspace: {summary['base_dir']}")
    if args.import_from:
        action = "copied" if not args.move_imported else "moved"
        print(f"imported: {imported} raw files {action} into {paths['raw']}")
    print(f"processed: {summary['processed']} raw files")
    print(f"plots: {summary['plots']}")
    print(f"report: {summary['report']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
