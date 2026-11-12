"""Composite the tracker's overlay video with a dashboard that fills in as the flight plays.

Each output frame shows the tracker view next to the state of the run *at that moment*: keys and
aborts so far, the gate's latest decision, the QBER trace drawn up to now against the 11% gate,
and one cell per key block. Everything comes from the run log, so the numbers match the
static dashboard (tools/make_dashboard.py) at the end of the clip.

    python src/tracker.py --source video --video flight.mp4 ... --save-video overlay.mp4
    python src/demo_runner.py --source zmq --log run.jsonl ...        (running alongside)
    python tools/make_demo_video.py overlay.mp4 run.jsonl demo.mp4 [--title "..."]

The log's ``t`` is the clip's own clock (VideoFileSource), so frame j of the overlay is t = (j + 1) / fps
(the tracker reads the first frame to size the camera before it starts writing). With the tracker's
``--start S``, pass the same ``--start S`` here; times on screen then count from the start of the demo.
"""
import argparse
import json

import cv2
import numpy as np

W, H = 1920, 1080
VW, VH = 1280, 720                      # tracker view, top left
GATE = 0.11
BG, PANEL, GRID, FG, MUTED = (22, 20, 18), (34, 30, 27), (60, 54, 50), (236, 233, 230), (150, 140, 132)
OK, BAD, WARN, LINK = (110, 200, 40), (80, 80, 235), (40, 190, 235), (120, 120, 120)      # BGR
OUTCOME = {"key": ("KEY ISSUED", OK), "abort": ("ABORT: QBER OVER GATE", BAD),
           "decoy_abort": ("ABORT: DECOY CHECK FAILED", BAD), "no_key": ("NO KEY (TOO FEW BITS)", WARN),
           "pooling": ("POOLING", WARN), "no_link": ("NO LINK", LINK)}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def text(img, s, org, scale, colour, thick=1, right=False):
    if right:
        (w, _), _ = cv2.getTextSize(s, FONT, scale, thick)
        org = (org[0] - w, org[1])
    cv2.putText(img, s, org, FONT, scale, colour, thick, cv2.LINE_AA)


def load(path):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    iv = [r for r in rows if r.get("kind") == "interval"]
    bl = [r for r in rows if r.get("kind") == "block"]
    return iv, bl


class Chart:
    """QBER over the whole flight, drawn incrementally up to the current time."""

    def __init__(self, x0, y0, w, h, t_end, ymax=0.5):
        self.x0, self.y0, self.w, self.h, self.t_end, self.ymax = x0, y0, w, h, t_end, ymax
        self.base = np.zeros((h, w, 3), np.uint8)
        self.base[:] = PANEL
        for v in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
            y = self.Y(v)
            cv2.line(self.base, (0, y), (w, y), GRID, 1)
            text(self.base, f"{v * 100:.0f}%", (6, y - 4 if y > 24 else y + 16), 0.45, MUTED)
        for s in range(0, int(t_end) + 1, 10):
            x = self.X(s)
            text(self.base, f"{s}s", (x + 3, h - 6), 0.42, MUTED)
        gy = self.Y(GATE)
        for x in range(0, w, 14):
            cv2.line(self.base, (x, gy), (min(x + 8, w), gy), BAD, 2)
        text(self.base, "11% abort gate", (w - 8, gy - 8), 0.55, BAD, 1, right=True)
        self.layer = self.base.copy()
        self.last = None
        self.i = 0

    def X(self, t):
        return int(round(8 + (self.w - 16) * t / self.t_end))

    def Y(self, q):
        return int(round(10 + (self.h - 34) * (1 - min(max(q, 0.0), self.ymax) / self.ymax)))

    def advance(self, iv, t):
        while self.i < len(iv) and iv[self.i]["t"] <= t:
            r = iv[self.i]
            p = (self.X(r["t"]), self.Y(r["qber_measured"]))
            colour = OK if r["qber_measured"] < GATE else (WARN if r["qber_measured"] < 0.25 else BAD)
            if self.last is not None:
                cv2.line(self.layer, self.last, p, colour, 2, cv2.LINE_AA)
            self.last = p
            self.i += 1

    def draw(self, img, t):
        img[self.y0:self.y0 + self.h, self.x0:self.x0 + self.w] = self.layer
        x = self.x0 + self.X(t)
        cv2.line(img, (x, self.y0), (x, self.y0 + self.h - 22), FG, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("overlay")
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--title", default="Drone-to-ground QKD link: live run")
    ap.add_argument("--start", type=float, default=0.0,
                    help="the --start the tracker was run with (seconds into the source clip)")
    ap.add_argument("--subtitle", default="Tracker on recorded flight footage -> link model -> BB84 + 11% QBER gate")
    args = ap.parse_args()

    iv, bl = load(args.log)
    cap = cv2.VideoCapture(args.overlay)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    t0 = args.start
    for r in iv + bl:                                  # show time from the start of this demo, not of the clip
        r["t"] -= t0
    t_end = max(iv[-1]["t"], n / fps)
    out = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))

    chart = Chart(16, VH + 46, W - 32, 230, t_end)
    strip_y, strip_h = VH + 46 + 230 + 32, 26
    bi = ii = 0
    keys = aborts = bits = 0
    last_block = None
    j = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = (j + 1) / fps
        j += 1
        while bi < len(bl) and bl[bi]["t"] <= t:
            b = bl[bi]
            keys += b["outcome"] == "key"
            aborts += b["outcome"] in ("abort", "decoy_abort")
            bits += (b["secret_bits"] or 0) if b["outcome"] == "key" else 0
            last_block = b
            bi += 1
        while ii + 1 < len(iv) and iv[ii + 1]["t"] <= t:
            ii += 1
        cur = iv[ii] if iv and iv[0]["t"] <= t else None
        chart.advance(iv, t)

        img = np.zeros((H, W, 3), np.uint8)
        img[:] = BG
        img[:VH, :VW] = cv2.resize(frame, (VW, VH), interpolation=cv2.INTER_AREA)

        # ---- right panel
        px = VW + 28
        text(img, args.title, (px, 44), 0.72, FG, 2)
        text(img, args.subtitle, (px, 70), 0.42, MUTED)
        label, colour = OUTCOME.get(last_block["outcome"], ("", MUTED)) if last_block else ("WAITING FOR FIRST BLOCK", MUTED)
        cv2.rectangle(img, (px, 92), (W - 28, 150), colour, -1)
        text(img, label, (px + 16, 131), 0.85, (20, 20, 20), 2)
        if last_block is not None:
            text(img, f"last block QBER {last_block['qber'] * 100:.1f}%", (W - 44, 131), 0.55, (20, 20, 20), 1, right=True)

        tiles = [("Keys issued", f"{keys}", OK), ("Aborts", f"{aborts}", BAD),
                 ("Secret bits", f"{bits:,}", FG), ("Blocks", f"{bi}", FG)]
        tw, th_ = (W - 28 - px - 12) // 2, 96
        for k, (name, val, c) in enumerate(tiles):
            x, y = px + (k % 2) * (tw + 12), 170 + (k // 2) * (th_ + 12)
            cv2.rectangle(img, (x, y), (x + tw, y + th_), PANEL, -1)
            text(img, name, (x + 14, y + 28), 0.5, MUTED)
            text(img, val, (x + 14, y + 78), 1.4, c, 3)

        y = 170 + 2 * (th_ + 12) + 26
        if cur is not None:
            q = cur["qber_measured"]
            rows = [("Tracker", cur["state"]), ("QBER now", f"{q * 100:.1f}%"),
                    ("Drift (pointing)", f"{cur['drift_mrad']:.0f} mrad"),
                    ("Residual", f"{cur['residual_mrad']:.1f} mrad"),
                    ("Channel loss", "no link" if cur["eta_db"] >= 299 else f"{cur['eta_db']:.0f} dB")]
            for name, val in rows:
                text(img, name, (px, y), 0.55, MUTED)
                text(img, val, (W - 28, y), 0.6, FG, 1, right=True)
                y += 34
        text(img, f"t = {t:5.1f} s", (px, VH - 18), 0.55, MUTED)

        # ---- bottom: QBER chart + block strip
        text(img, "Measured QBER (every 100 ms)", (16, VH + 32), 0.6, FG, 1)
        chart.draw(img, t)
        text(img, "Gate decision per 1 s key block", (16, strip_y - 8), 0.5, MUTED)
        n_bl = max(len(bl), 1)
        cw = (W - 32) / n_bl
        for k in range(bi):
            c = OUTCOME.get(bl[k]["outcome"], ("", MUTED))[1]
            x0 = int(16 + k * cw)
            cv2.rectangle(img, (x0, strip_y), (max(x0 + 1, int(16 + (k + 1) * cw) - 2), strip_y + strip_h), c, -1)
        out.write(img)
    out.release()
    print(f"wrote {args.out}: {j} frames, {keys} keys, {aborts} aborts")


if __name__ == "__main__":
    main()
