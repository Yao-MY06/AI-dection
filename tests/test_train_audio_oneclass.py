import importlib.util
from pathlib import Path


def load_train_module():
    script_path = Path(__file__).resolve().parents[1] / "tools" / "train_audio_oneclass.py"
    spec = importlib.util.spec_from_file_location("train_audio_oneclass", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample(**overrides):
    values = {
        "name": "good_sample",
        "rms": 1000.0,
        "rms_20ms": 8000.0,
        "rms_50ms": 5000.0,
        "zero_crossing_rate": 0.001,
        "abs_mean": 100.0,
        "energy_decay": 0.1,
        "peak_dbfs": -1.0,
        "spectral_centroid_hz": 1500.0,
        "spectral_rolloff_hz": 4000.0,
        "band_ratio_low": 0.2,
        "band_ratio_mid": 0.35,
        "band_ratio_high": 0.45,
    }
    values.update(overrides)
    return values


def test_train_oneclass_applies_feature_std_floors():
    train_audio = load_train_module()
    good_samples = [
        sample(name="good_001", band_ratio_mid=0.3500),
        sample(name="good_002", band_ratio_mid=0.3506),
    ]

    _means, inv_stds, _threshold = train_audio.train_oneclass(good_samples, bad_samples=[])
    inv_by_name = dict(zip(train_audio.AI_FEATURES, inv_stds))

    assert inv_by_name["band_ratio_mid"] <= 50.0
    assert inv_by_name["spectral_centroid_hz"] <= 0.02
