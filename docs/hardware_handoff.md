# Hardware Hand-off (for the rig team)

The software side is ready for real camera input. This is what we need from the rig, and how
to check it.

## What the software expects
- One **fixed** USB camera at the laptop (ground receiver), looking at the drone from about 1.5-2 m.
- A **green LED** beacon on the drone (520 nm). It must be the brightest green thing in view.
- **Locked exposure and gain.** Auto-exposure hides intensity changes and breaks the calibration.
- 60 FPS if possible (30 works). Note: `jitter` is frame-to-frame motion x fps, so sensor noise
  scales with fps. Calibrate at the fps you will demo at.
- Run the camera code with the Python that **owns the camera** (Windows Python, not WSL).

## Setup on the laptop
```bash
python -m venv .venv && .venv\Scripts\activate      # Windows
pip install -r requirements.txt
```

## Step 1: tune the detector
```bash
python tools/camera_tune.py --device 0
```
- Adjust the HSV sliders until only the LED is circled (yellow circle) and the state reaches `TRACK`.
- Press `c` with the drone in **calm hover**; this saves `calibration.json` (boresight pixel and
  full-signal brightness).
- Copy the final HSV values into `LEDDetector.__init__` (`hsv_low`, `hsv_high`) in `src/camera_provider.py`.
- If exposure does not lock, change `--exposure` or the property used in `VideoCaptureSource`
  (V4L2 and DirectShow differ).

## Step 2: record three runs (about 30 s each, same distance, same lighting)
```bash
python src/demo_runner.py --source camera --calibration calibration.json --duration 30 --quiet --record calm.csv
python src/demo_runner.py --source camera --calibration calibration.json --duration 30 --quiet --record fan.csv
python src/demo_runner.py --source camera --calibration calibration.json --duration 20 --quiet --record obstruct.csv
```
- `calm.csv`: fan off, drone hovering.
- `fan.csv`: fan on at the setting you plan to demo.
- `obstruct.csv`: hand over the lens for a few seconds.
- Send us the CSVs. We will fit the transfer-function constants (`python tools/summarize_telemetry.py --skip 5 calm.csv fan.csv obstruct.csv`, then `tools/tune.py`).

## Step 3: rehearse the demo
```bash
python src/demo_runner.py --source camera --calibration calibration.json --duration 120 --log run.jsonl --record demo.csv
python tools/make_dashboard.py run.jsonl dashboard.html
```
Targets: first `TRACK` lock under 60 s (printed at the end), tracking held for the full
120 s with the fan on, and keys issued while the fan is at the demo setting. Keep `demo.csv` as
the fallback: `--source csv --csv demo.csv`.

## Checklist
- [ ] Camera hard-mounted and does not move
- [ ] Exposure and gain locked (intensity at calm hover stays steady)
- [ ] Only the LED is detected (no reflections, screens, or other green lights)
- [ ] Calm hover gives drift about a few px, jitter low, intensity about 1.0 after calibration
- [ ] Fan shows up as higher jitter and drift but the LED stays in frame
- [ ] Lens cover drops intensity to about 0 and tracking returns when removed
- [ ] Fallback CSV recorded
