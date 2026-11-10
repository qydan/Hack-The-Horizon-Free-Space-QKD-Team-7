"""Render a demo run log (JSONL from `demo_runner.py --log`) as a self-contained HTML page.

Shows the live QBER trace (100 ms steps) against the 11% gate, the measured motion, the two
error bars (ground residual vs drone lean), and the gate's decision for every key block.

    python src/demo_runner.py --source sim --quiet --log run.jsonl
    python tools/make_dashboard.py run.jsonl dashboard.html
"""
import html
import json
import sys

GATE = 0.11
COLORS = {"key": "var(--ok)", "abort": "var(--bad)", "decoy_abort": "var(--bad)",
          "pooling": "var(--warn)", "no_key": "var(--warn)", "no_link": "var(--muted)"}


def load(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def line_chart(rows, series, ymax, title, gate=None, h=170, w=640):
    """series: [(key, label, css_color)]. Returns an inline SVG string."""
    pad_l, pad_r, pad_t, pad_b = 40, 10, 10, 22
    n = max(1, len(rows) - 1)
    X = lambda i: pad_l + (w - pad_l - pad_r) * i / n
    Y = lambda v: pad_t + (h - pad_t - pad_b) * (1 - min(max(v, 0), ymax) / ymax)
    out = [f'<svg viewBox="0 0 {w} {h}" role="img" aria-label="{html.escape(title)}">']
    for frac in (0, 0.5, 1):
        v = ymax * frac
        out.append(f'<line x1="{pad_l}" x2="{w-pad_r}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" class="grid"/>'
                   f'<text x="{pad_l-6}" y="{Y(v)+4:.1f}" text-anchor="end" class="ax">{v:g}</text>')
    if gate is not None:
        out.append(f'<line x1="{pad_l}" x2="{w-pad_r}" y1="{Y(gate):.1f}" y2="{Y(gate):.1f}" class="gate"/>'
                   f'<text x="{w-pad_r}" y="{Y(gate)-4:.1f}" text-anchor="end" class="gate-t">11% abort gate</text>')
    for key, label, color in series:
        pts = " ".join(f"{X(i):.1f},{Y(r[key]):.1f}" for i, r in enumerate(rows))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>')
    t_end = rows[-1]["t"]
    out.append(f'<text x="{pad_l}" y="{h-5}" class="ax">{rows[0]["t"]:g} s</text>'
               f'<text x="{w-pad_r}" y="{h-5}" text-anchor="end" class="ax">{t_end:g} s</text></svg>')
    legend = "".join(f'<span><i style="background:{c}"></i>{html.escape(l)}</span>' for _, l, c in series)
    return f'<figure><figcaption>{html.escape(title)}</figcaption>{"".join(out)}<div class="legend">{legend}</div></figure>'


def strip(rows, w=640, h=26):
    cw = (w) / len(rows)
    cells = "".join(
        f'<rect x="{i*cw:.2f}" y="0" width="{max(cw-0.6,0.4):.2f}" height="{h}" fill="{COLORS.get(r["outcome"], "var(--muted)")}">'
        f'<title>t={r["t"]:g}s {r["outcome"]}</title></rect>' for i, r in enumerate(rows))
    return f'<svg viewBox="0 0 {w} {h}" role="img" aria-label="Gate decision per key block">{cells}</svg>'


def stats(rows, trace):
    keys = [r for r in rows if r["outcome"] == "key"]
    best = run = 0
    for r in rows:
        run = run + 1 if r["outcome"] == "key" else 0
        best = max(best, run)
    first_lock = next((r["t"] for r in trace if r.get("state") == "TRACK"), None)
    return [
        ("Key blocks", len(rows)),
        ("Keys issued", len(keys)),
        ("Key yield", f"{100*len(keys)/len(rows):.0f}%"),
        ("Longest key streak", f"{best} blocks"),
        ("Secret bits issued", sum(r["secret_bits"] or 0 for r in keys)),
        ("Max QBER while issuing keys", f"{100*max((r['qber'] for r in keys), default=0):.1f}%"),
        ("Aborts (QBER / decoy)", sum(r["outcome"] in ("abort", "decoy_abort") for r in rows)),
        ("First tracking lock", f"{first_lock:g} s" if first_lock is not None else "n/a"),
    ]


PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>QKD Run Dashboard</title>
<style>
:root{--bg:#fff;--fg:#1a1d21;--muted:#9aa3ad;--card:#f4f6f8;--ok:#1f9d6b;--bad:#d64545;--warn:#d9a21b;
--c1:#2f6fdb;--c2:#c2410c;--c3:#7c3aed;--grid:#dde2e7}
@media (prefers-color-scheme:dark){:root{--bg:#14171a;--fg:#e6e9ec;--muted:#6b7680;--card:#1d2226;--ok:#3ecf94;
--bad:#ff6b6b;--warn:#f0c04a;--c1:#6ea2ff;--c2:#ff9a5c;--c3:#b794ff;--grid:#2c343a}}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
main{max-width:760px;margin:auto}h1{font-size:22px;margin:0 0 12px}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px;margin-bottom:16px}
.stat{background:var(--card);border-radius:8px;padding:10px}.stat b{display:block;font-size:20px}
.stat span{color:var(--muted);font-size:12px}
figure{margin:0 0 16px;background:var(--card);border-radius:8px;padding:10px}
figcaption{font-weight:600;margin-bottom:4px}svg{width:100%;height:auto;display:block}
.grid{stroke:var(--grid)}.ax{fill:var(--muted);font-size:10px}.gate{stroke:var(--bad);stroke-dasharray:4 3}
.gate-t{fill:var(--bad);font-size:10px}.legend{display:flex;gap:14px;font-size:12px;margin-top:4px;flex-wrap:wrap}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px}
</style></head><body><main id="m">__BODY__</main>__SCRIPT__</body></html>"""

BODY = """<h1>QKD Run Dashboard__SUFFIX__</h1>
<div class="stats">__STATS__</div>
__CHARTS__
<figure><figcaption>Gate decision per key block</figcaption>__STRIP__
<div class="legend"><span><i style="background:var(--ok)"></i>key</span><span><i style="background:var(--bad)"></i>abort / decoy abort</span>
<span><i style="background:var(--warn)"></i>pooling / no key / no link</span></div></figure>"""


def body_html(allrows, window_s=None, suffix=""):
    """The dashboard content for a list of run-log records. ``window_s`` keeps only the last
    N seconds of the traces (live view). Returns None if there is nothing to draw yet."""
    trace = [r for r in allrows if r.get("kind") == "interval"]
    rows = [r for r in allrows if r.get("kind") == "block"]
    if not rows or not trace:
        return None
    all_rows = rows
    if window_s is not None:
        t0 = trace[-1]["t"] - window_s
        trace = [r for r in trace if r["t"] >= t0]
        rows = [r for r in rows if r["t"] >= t0] or rows[-1:]
    for r in trace:
        r["lean_mrad"] = r["lean_deg"] * 17.453     # degrees -> mrad, to compare with the ground residual
    top = max(10, max(max(r["residual_mrad"], r["lean_mrad"]) for r in trace))
    charts = [
        line_chart(trace, [("qber_expected", "error budget QBER", "var(--c1)"),
                           ("qber_measured", "measured QBER (100 ms)", "var(--c2)")],
                   0.55, "Live QBER trace", gate=GATE),
        line_chart(trace, [("residual_mrad", "ground residual (gimbal + tracker)", "var(--c1)"),
                           ("lean_mrad", "drone lean (needs a drone-side mirror)", "var(--c2)")],
                   top, "Two error bars (mrad)", h=150),
        line_chart(trace, [("drift_mrad", "drift", "var(--c1)"), ("jitter_mrad", "jitter", "var(--c2)")],
                   max(10, max(max(r["drift_mrad"], r["jitter_mrad"]) for r in trace)), "Drift and jitter (mrad)", h=130),
        line_chart(trace, [("intensity", "beacon intensity", "var(--c3)")], 1.0, "Intensity", h=110),
        line_chart(trace, [("eta_db", "transmittance loss (dB)", "var(--c3)")],
                   max(30, min(60, max(r["eta_db"] for r in trace))), "Loss (dB)", h=110),
    ]
    stat_html = "".join(f'<div class="stat"><b>{html.escape(str(v))}</b><span>{html.escape(k)}</span></div>'
                        for k, v in stats(all_rows, trace))
    return (BODY.replace("__SUFFIX__", html.escape(suffix)).replace("__STATS__", stat_html)
            .replace("__CHARTS__", "".join(charts)).replace("__STRIP__", strip(rows)))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    body = body_html(load(sys.argv[1]))
    if body is None:
        sys.exit("log has no interval/block records (use a log written by demo_runner.py)")
    page = PAGE.replace("__BODY__", body).replace("__SCRIPT__", "")
    with open(sys.argv[2], "w") as f:
        f.write(page)
    print(f"wrote {sys.argv[2]}")


if __name__ == "__main__":
    main()
