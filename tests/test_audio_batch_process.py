import importlib.util
import json
import math
import struct
import tempfile
import wave
from pathlib import Path


def load_audio_batch_module():
    script_path = Path(__file__).resolve().parents[1] / "tools" / "AUDIO" / "audio_batch_process.py"
    spec = importlib.util.spec_from_file_location("audio_batch_process", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_raw_pcm16(path: Path, sample_rate: int, sample_count: int) -> None:
    samples = []
    for i in range(sample_count):
        value = int(12000 * math.sin(2.0 * math.pi * 440.0 * i / sample_rate))
        samples.append(struct.pack("<h", value))
    path.write_bytes(b"".join(samples))


def test_process_directory_converts_raw_to_wav_and_features():
    audio_batch = load_audio_batch_module()

    with tempfile.TemporaryDirectory() as tmp:
        base_dir = Path(tmp)
        raw_dir = base_dir / "qw"
        raw_dir.mkdir()
        raw_path = raw_dir / "aud0001.raw"
        write_raw_pcm16(raw_path, sample_rate=16000, sample_count=16000)

        summary = audio_batch.process_directory(base_dir=base_dir, sample_rate=16000, make_plots=False)

        wav_path = base_dir / "wav" / "aud0001.wav"
        feature_path = base_dir / "features" / "aud0001.json"
        report_path = base_dir / "reports" / "audio_summary.csv"

        assert summary["processed"] == 1
        assert wav_path.exists()
        assert feature_path.exists()
        assert report_path.exists()

        with wave.open(str(wav_path), "rb") as wav_file:
            assert wav_file.getnchannels() == 1
            assert wav_file.getsampwidth() == 2
            assert wav_file.getframerate() == 16000
            assert wav_file.getnframes() == 16000

        features = json.loads(feature_path.read_text(encoding="utf-8"))
        assert features["sample_count"] == 16000
        assert features["duration_s"] == 1.0
        assert features["peak"] > 10000
        assert features["rms"] > 5000
        assert features["dominant_freq_hz"] == 437.5


def test_frequency_features_match_board_band_boundaries():
    audio_batch = load_audio_batch_module()

    sample_rate = 16000
    sample_count = 16000
    bin_12_frequency = 187.5
    samples = [
        int(10000 * math.sin(2.0 * math.pi * bin_12_frequency * i / sample_rate))
        for i in range(sample_count)
    ]

    features = audio_batch.compute_frequency_features(samples, sample_rate, fft_size=1024)

    assert features["dominant_freq_hz"] == 187.5
    assert features["band_ratio_low"] > 0.7
    assert features["band_ratio_mid"] < 0.3


if __name__ == "__main__":
    test_process_directory_converts_raw_to_wav_and_features()
