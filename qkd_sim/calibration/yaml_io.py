"""Write calibration results into config/calibration.yaml in place (keeps the file's comments and layout)."""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

CALIBRATION_YAML = Path(__file__).resolve().parent.parent / "config" / "calibration.yaml"


def _fmt(v: Any) -> str:
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, (int, str)):
        return str(v) if not isinstance(v, str) or re.fullmatch(r"[\w.\-]+", v) else json.dumps(v)
    return json.dumps(v)


def update_section(section: str, values: dict[str, Any], path: Path | None = None) -> None:
    """Set ``section.key`` for every key in ``values``; each key must already exist in the section."""
    path = Path(path or CALIBRATION_YAML)
    lines = path.read_text(encoding="utf-8").splitlines()
    out, in_sec, done = [], False, set()
    for line in lines:
        if re.match(r"^\S", line):
            in_sec = line.split(":", 1)[0].strip() == section
        m = re.match(r"^(\s+)([\w]+):(\s*)([^#]*?)(\s*#.*)?$", line) if in_sec else None
        if m and m.group(2) in values:
            key = m.group(2)
            line = f"{m.group(1)}{key}: {_fmt(values[key])}" + (f"  {m.group(5).strip()}" if m.group(5) else "")
            done.add(key)
        out.append(line)
    missing = set(values) - done
    if missing:
        raise KeyError(f"{path.name}: section {section!r} has no key(s) {sorted(missing)}")
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
