"""Render a demo run log (JSONL from `demo_runner.py --log`) as a self-contained HTML page.

    python src/demo_runner.py --source synthetic --quiet --log run.jsonl
    python tools/make_dashboard.py run.jsonl dashboard.html
"""
import html
import json
import sys

GATE = 0.11
COLORS = {"key": "var(--ok)", "abort": "var(--bad)", "decoy_reject": "var(--bad)",
          "pooling": "var(--warn)", "no_key": "var(--warn)"}


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
    out.append(f'<text x="{pad_l}" y="{h-5}" class="ax">0 s</text>'
               f'<text x="{w-pad_r}" y="{h-5}" text-anchor="end" class="ax">{t_end:g} s</text></svg>')
    legend = "".join(f'<span><i style="background:{c}"></i>{html.escape(l)}</span>' for _, l, c in series)
    return f'<figure><figcaption>{html.escape(title)}</figcaption>{"".join(out)}<div class="legend">{legend}</div></figure>'


def strip(rows, w=640, h=26):
    cw = (w) / len(rows)
    cells = "".join(
        f'<rect x="{i*cw:.2f}" y="0" width="{max(cw-0.6,0.4):.2f}" height="{h}" fill="{COLORS.get(r["outcome"], "var(--muted)")}">'
        f'<title>t={r["t"]:g}s {r["outcome"]}</title></rect>' for i, r in enumerate(rows))
    return f'<svg viewBox="0 0 {w} {h}" role="img" aria-label="Outcome per window">{cells}</svg>'


def stats(rows):
    keys = [r for r in rows if r["outcome"] == "key"]
    best = run = 0
    for r in rows:
        run = run + 1 if r["outcome"] == "key" else 0
        best = max(best, run)
    first_lock = next((r["t"] for r in rows if r.get("tracking_state") == "TRACK"), None)
    return [
        ("Windows", len(rows)),
        ("Keys issued", len(keys)),
        ("Key yield", f"{100*len(keys)/len(rows):.0f}%"),
        ("Longest key streak", f"{best} windows"),
        ("Max QBER while issuing keys", f"{100*max((r['qber'] for r in keys), default=0):.1f}%"),
        ("Aborts", sum(r["outcome"] == "abort" for r in rows)),
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
</style></head><body><main><h1>QKD Run Dashboard</h1>
<div class="stats">__STATS__</div>
__CHARTS__
<figure><figcaption>Outcome per window</figcaption>__STRIP__
<div class="legend"><span><i style="background:var(--ok)"></i>key</span><span><i style="background:var(--bad)"></i>abort / decoy reject</span>
<span><i style="background:var(--warn)"></i>pooling / no key</span></div></figure>
</main></body></html>"""


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    rows = load(sys.argv[1])
    if not rows:
        sys.exit("empty log")
    charts = [
        line_chart(rows, [("p_error", "channel error P", "var(--c1)"), ("qber", "sampled QBER", "var(--c2)")],
                   0.55, "Channel error and QBER", gate=GATE),
        line_chart(rows, [("drift", "drift (px)", "var(--c1)")], max(10, max(r["drift"] for r in rows)),
                   "Drift", h=130),
        line_chart(rows, [("jitter", "jitter (px/s)", "var(--c2)")], max(10, max(r["jitter"] for r in rows)),
                   "Jitter", h=130),
        line_chart(rows, [("intensity", "intensity", "var(--c3)")], 1.0, "Intensity", h=130),
    ]
    stat_html = "".join(f'<div class="stat"><b>{html.escape(str(v))}</b><span>{html.escape(k)}</span></div>'
                        for k, v in stats(rows))
    page = PAGE.replace("__STATS__", stat_html).replace("__CHARTS__", "".join(charts)).replace("__STRIP__", strip(rows))
    with open(sys.argv[2], "w") as f:
        f.write(page)
    print(f"wrote {sys.argv[2]}")


if __name__ == "__main__":
    main()
