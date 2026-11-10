# Hardware Hand-off (for the rig team)

The software runs on a real camera in **fixed-camera mode** (camera mounted still, no gimbal).
This is what it needs from the rig and how to check it. A steered gimbal is not wired in yet
(see "Not implemented" in `docs/system_architecture.md`).

## What the software expects
- One **fixed** camera at the laptop looking at the drone from about 3 m (`nominal_range_m`).
  It must not move after calibration.
- A **constantly-lit red LED** on the drone, tracked with `--beacon steady` (every `tracker.py`
  command passes it; `camera_check.py` uses `--steady`). It is found as the brightest small
  spot, so the scene must be dim with no other small bright spots in view (lamps, screens,
  windows, reflections), and `--zone` should cover only where the drone hovers. A blinking
  beacon would tolerate room lights, but this rig does not use one.
- **Frame rate:** the rig runs the Action 5 Pro at 30 fps, so every command passes `--fps 30`.
  There is no blink rate to match, so `--blink-hz` is not used. (The simulated rig defaults to
  120 fps and a 13 Hz blink.)
- **Locked exposure, gain and white balance.** Auto-exposure hides intensity changes and breaks
  the calibration. `tracker.py` does not set exposure itself: it is locked on the Action 5 Pro (or in the driver)
  (`tools/camera_check.py --exposure -9` or `--dialog`) and confirm with the check below.
- Run the camera code with the Python that **owns the camera** (Windows Python, not WSL).

## Setup on the laptop
```bash
python -m venv .venv && .venv\Scripts\activate      # Windows
pip install -r requirements.txt
```

## Step 0: bench test with a hand-held LED (no drone)
Hold the red LED about 3 m from the camera, roughly at the height of the lens, and move it the
way the drone would: steady for calibration, then a slow sway and a faster shake. This exercises
the whole chain (lock, drift, jitter, residual, key decisions); only the real drone motion and the
fan are missing.
- Use the commands in steps 1-4 as written (steady LED). Dim the room and add `--zone` around
  where you will hold it, because anything else small and bright in view can steal the lock.
- Hold it still for the 15 s calibration (`--calibrate 15`), then run the tracker with
  `demo_runner.py --source zmq --log run.jsonl` and `tools/live_dashboard.py run.jsonl` open.
  Sway slowly and the drift should rise; shake it and the jitter should rise. Cover the LED with a
  hand and the state should drop to `COAST` then `SEARCH`.
- A hand moves differently from a drone in wind, so do not take the thresholds from this test.

## Step 1: check the camera
```bash
python tools/camera_check.py --device 0 --width 1920 --height 1080 --fps 30
python tools/camera_check.py --device 0 --fps 30 --steady          # our beacon: a constantly-lit LED
```
Three phases (frame rate, noise floor with the beacon off, beacon on). It prints OK / FIX lines
and the settings to use: real fps, `min_amplitude`, and a first guess for `i_ref`. Fix every FIX
line before going on.

## Step 2: calibrate in calm hover
```bash
python src/tracker.py --source camera --device 0 --fps 30 --beacon steady --hfov 70 --calibrate 15 --duration 1
```
Hold the drone in calm hover with the beacon on. This saves `calibration.json` (boresight
direction and full-signal beacon brightness); later runs load it automatically. Pass the fps and
`--min-amplitude` (grey levels the LED must stand above the background) that step 1 reported, and the lens's real `--hfov`.

Useful options: `--zone 0.25,0.1,0.75,0.9` (only look for the beacon in the hover area, so
lamps and windows cannot steal the lock), `--proc-width` (downscale for speed),
`--max-jump-mrad`.

## Step 3: record three runs (about 30 s each, same distance, same lighting)
Two terminals, sandbox first. The sandbox waits for the first message, then stops after 5 s without telemetry (`--timeout`).
```bash
python src/demo_runner.py --source zmq --quiet --record calm.csv
python src/tracker.py --source camera --fps 30 --beacon steady --duration 30
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
python src/tracker.py --source camera --fps 30 --beacon steady --duration 120
python tools/live_dashboard.py run.jsonl        # optional, live charts at http://127.0.0.1:8000 while it runs
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
