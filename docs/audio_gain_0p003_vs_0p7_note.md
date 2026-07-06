# Audio Gain Version Note: 0.003f vs 0.7f

Date: 2026-07-05

## Current diagnostic version before rollback

Code state before rollback:

- `AUDIO_CAPTURE_GAIN = 0.003f`
- Gain is applied centrally in `audio_apply_capture_gain()` before writing both ring buffer and capture buffer.
- Monitor prints `Audio stats` with `rms`, `clipped`, and `gain`.
- Monitor also prints `Audio features`:
  - `rms`, `rms20`, `rms50`
  - `decay`, `zcr`
  - `centroid`, `rolloff`
  - `low`, `mid`, `high`
  - `peak_dbfs`

Observed behavior with `0.003f`:

- Clipping is much lower, usually around `0~15/16000` in recent monitor logs.
- RMS is reduced to a range where frequency features are readable.
- Good/bad should be judged mainly by frequency shape, not by peak:
  - dull/bad-like: lower `centroid`, lower `rolloff`, lower `high`, higher `low/mid`
  - crisp/good-like: higher `centroid`, higher `rolloff`, higher `high`, lower `low`
- The current old audio model may still output `ANOMALY`; this is expected until data is recollected and retrained with the same gain setting.

## 0.7f rollback behavior

Code state after rollback:

- `AUDIO_CAPTURE_GAIN = 0.7f`
- The feature monitor logging is kept, so logs can still show `Audio stats` and `Audio features`.

Expected behavior with `0.7f`:

- Signal level will be much larger than `0.003f`.
- Clipping may become heavy again because transient impact samples can hit `32767/-32768`.
- `peak` and `peak_dbfs` should not be used for good/bad judgment under clipping.
- Frequency features may be less reliable if clipping is high, but the extra log line can still help compare whether the old gain path matches the previous dataset/model better.

## Practical comparison rule

When comparing logs, use this order:

1. Check `gain` to confirm firmware version.
2. Check `clipped`; if it is high, do not trust `peak`.
3. Use `centroid`, `rolloff`, `low`, `mid`, `high`, and `decay` to judge sound character.
4. Ignore `Audio AI result` until the model is retrained using data captured with the same gain.