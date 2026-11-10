"""Generate the ArUco tag texture (cv2.aruco.generateImageMarker) with its matte white margin.

    python tools/make_tag_texture.py            # writes models/tag_aruco/materials/textures/aruco_<dict>_<id>.png

The texture covers marker + white margin, matching the plate size used by tools/world_builder.py:
plate edge = outer_edge_m + 2 * white_margin_m.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from sim_bridge import config as cfgmod  # noqa: E402


def tag_texture_path(cfg_beacon: dict) -> Path:
    t = cfg_beacon["tag"]
    return ROOT / "models" / "tag_aruco" / "materials" / "textures" / f"aruco_{t['dictionary']}_{t['id']}.png"


def make_tag_texture(cfg_beacon: dict) -> Path:
    import cv2
    t = cfg_beacon["tag"]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, t["dictionary"]))
    edge, margin, px = float(t["outer_edge_m"]), float(t["white_margin_m"]), int(t["texture_px"])
    marker_px = int(round(px * edge / (edge + 2 * margin)))
    marker = cv2.aruco.generateImageMarker(dictionary, int(t["id"]), marker_px, borderBits=int(t["border_bits"]))
    img = np.full((px, px), 255, np.uint8)
    o = (px - marker_px) // 2
    img[o:o + marker_px, o:o + marker_px] = marker
    if cfg_beacon.get("texture", {}).get("flip_horizontal"):
        img = img[:, ::-1]
    out = tag_texture_path(cfg_beacon)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    return out


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()
    print(make_tag_texture(cfgmod.load("beacon")))


if __name__ == "__main__":
    main()
