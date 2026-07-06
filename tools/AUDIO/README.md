# AUDIO Workspace

Directory layout:

- `qw/` - original ESP32-P4 `.raw` audio captures, PCM16 little-endian, mono, 16 kHz.
- `wav/` - converted `.wav` files for listening.
- `features/` - one JSON feature file per raw clip.
- `plots/` - optional waveform and spectrum PNGs.
- `reports/` - batch CSV summaries.

Typical use:

```powershell
python tools\AUDIO\audio_batch_process.py
```

Import raw captures from another folder first:

```powershell
python tools\AUDIO\audio_batch_process.py --import-from G:\AUDIO
```

Use `--no-plots` when you only need WAV and feature files.
