"""Print a Markdown table of every # ESTIMATE / # MEASURE value in config/*.yaml (for the README).

    python tools/list_estimates.py [--measure-only]
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FILES = ("world", "drone", "beacon", "fan", "camera_action5", "rigs", "gimbal_storm32", "lighting", "bus")


def rows(measure_only: bool = False) -> list[str]:
    out = []
    for name in FILES:
        stack: list[tuple[int, str]] = []
        for line in (ROOT / "config" / f"{name}.yaml").read_text().splitlines():
            if not line.strip() or line.strip().startswith("#"):
                continue
            ind = len(line) - len(line.lstrip())
            code, _, com = line.partition("#")
            m = re.match(r"\s*([\"\w]+):\s*(.*)", code)
            if not m:
                continue
            key, val = m.group(1).strip('"'), m.group(2).strip()
            while stack and stack[-1][0] >= ind:
                stack.pop()
            stack.append((ind, key))
            t = re.match(r"\s*(ESTIMATE|MEASURE)\b(.*)", com)
            if val and t and (not measure_only or "MEASURE" in com):
                note = t.group(2).strip().replace("|", "/")
                out.append(f"| `{name}.yaml` | `{'.'.join(k for _, k in stack)}` | {val} | {t.group(1)} | {note} |")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--measure-only", action="store_true")
    a = ap.parse_args()
    print("| File | Key | Value | Tag | Note |\n|---|---|---|---|---|")
    print("\n".join(rows(a.measure_only)))


if __name__ == "__main__":
    main()
