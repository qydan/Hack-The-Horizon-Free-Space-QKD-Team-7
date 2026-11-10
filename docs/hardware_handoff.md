# Hardware Hand-off (for the rig team)

The software runs on a real camera in **fixed-camera mode** (camera mounted still, no gimbal).
This is what it needs from the rig and how to check it. A steered gimbal is not wired in yet
(see "Not implemented" in `docs/system_architecture.md`).

## What the software expects
- One **fixed** camera at the laptop looking at the drone from about 3 m (`nominal_range_m`).
  It must not move after calibration.
- A **blinking beacon** on the drone: 100% / 40% at 13 Hz, never fully off (IR 850 nm or red LED,
  or retroreflective tape with a blinking ring light). A constantly-lit LED also works with
  `--beacon steady`, but only in a dark scene with no other small bright spots.
- **Frame rate matched to the blink:** at least 4 frames per blink. 13 Hz needs about 60 fps or
  more (the design assumes 120); at 30 fps slow the blink to about 7 Hz and pass `--blink-hz`.
- **Locked exposure, gain and white balance.** Auto-exposure hides intensity changes and breaks
  the calibration. `tracker.py` does not set exposure itself: lock it in the camera or driver
  (`tools/camera_check.py --exposure -9` or `--dialog`) and confirm with the check below.
- Run the camera code with the Python that **owns the camera** (Windows Python, not WSL).

## Setup on the laptop
```bash
python -m venv .venv && .venv\Scripts\activate      # Windows
pip install -r requirements.txt
```

## Step 1: check the camera
```bash
python tools/camera_check.py --device 0 --width 1920 --height 1080 --fps 60
python tools/camera_check.py --device 0 --fps 60 --steady          # constantly-lit LED instead
```
Three phases (frame rate, noise floor with the beacon off, beacon on). It prints OK / FIX lines
and the settings to use: real fps, `min_amplitude`, and a first guess for `i_ref`. Fix every FIX
line before going on.

## Step 2: calibrate in calm hover
```bash
python src/tracker.py --source camera --device 0 --fps 60 --hfov 70 --calibrate 15 --duration 1
```
Hold the drone in calm hover with the beacon on. This saves `calibration.json` (boresight
direction and full-signal beacon brightness); later runs load it automatically. Pass the fps,
`--min-amplitude` and `--blink-hz` that step 1 reported, and the lens's real `--hfov`.

Useful options: `--zone 0.25,0.1,0.75,0.9` (only look for the beacon in the hover area, so
lamps and windows cannot steal the lock), `--proc-width` (downscale for speed),
`--max-jump-mrad`.

## Step 3: record three runs (about 30 s each, same distance, same lighting)
Two terminals, sandbox first. The sandbox stops after 5 s without telemetry (`--timeout`).
```bash
python src/demo_runner.py --source zmq --quiet --record calm.csv
python src/tracker.py --source camera --fps 60 --duration 30
```
- `calm.csv`: fan off, drone hovering.
- `fan.csv`: fan on at the setting you plan to demo.
- `obstruct.csv`: hand over the lens for a few seconds (20 s is enough).

Send us the CSVs. They replay through the same pipeline
(`python src/demo_runner.py --source csv --csv calm.csv`), and the measured residual and drift
tell us where the operating point sits (`python tools/tune.py`).

## Step 4: rehearse the demo
```bash
python src/demo_runner.py --source zmq --log run.jsonl --record demo.csv --keys-out keys.jsonl
python src/tracker.py --source camera --fps 60 --duration 120
python tools/make_dashboard.py run.jsonl dashboard.html
```
Targets: first lock under 60 s (the tracker prints it at the end), tracking held for the full
120 s with the fan on, and keys issued while the fan is at the demo setting. Keep `demo.csv` as
the fallback: `--source csv --csv demo.csv`.

## Known limits of fixed-camera mode
- **Residual is the beacon's distance from the image centre.** Nothing steers, so the whole of
  the drone's offset counts as ground pointing error against the 10 mrad virtual receiver.
  10 mrad is 3 cm at 3 m: aim the camera so the calm-hover position sits on the image centre,
  and expect aborts whenever the drone wanders more than a few centimetres off it.
- **Lean and range are not measured** (no AprilTag pose in this mode): `lean_deg` is 0 and
  `range_m` is 3.0.

## Checklist
- [ ] Camera hard-mounted and does not move
- [ ] `camera_check.py` reports READY (frame rate, dropped frames, brightness drift, SNR, no clipping)
- [ ] Exposure and gain locked (intensity at calm hover stays steady)
- [ ] Only the beacon is detected (no reflections, screens or lamps inside the zone)
- [ ] Calm hover: state `TRACK`, intensity about 1.0 after calibration, residual a few mrad
- [ ] Fan shows up as higher jitter and drift but the beacon stays in frame
- [ ] Lens cover drops the state to `COAST` then `SEARCH`, and tracking returns when removed
- [ ] Fallback CSV recorded
