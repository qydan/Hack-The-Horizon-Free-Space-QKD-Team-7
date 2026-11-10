#!/usr/bin/env bash
# Copy the qkd_sim model, airframe and worlds into a PX4-Autopilot tree.
#
#   px4/install_px4_assets.sh /path/to/PX4-Autopilot
#
# After this, either:
#   PX4_GZ_WORLD=qkd_lab make px4_sitl gz_mini2se               # PX4 starts Gazebo and spawns mini2se
# or (what tools/run_scenario.py does): start Gazebo yourself with the generated world, then
#   PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=mini2se PX4_SYS_AUTOSTART=4099 PX4_GZ_WORLD=qkd_lab \
#     ./build/px4_sitl_default/bin/px4
set -euo pipefail

PX4_DIR="${1:-${PX4_DIR:-$HOME/PX4-Autopilot}}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

[ -d "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes" ] || { echo "not a PX4 tree: $PX4_DIR" >&2; exit 1; }

AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
GZ_DIR="$PX4_DIR/Tools/simulation/gz"

# Regenerate assets from config so the copies match the YAML.
python3 "$REPO/tools/world_builder.py" --write-assets

for af in "$HERE"/airframes/*_gz_*; do
  name="$(basename "$af")"
  num="${name%%_*}"
  clash="$(ls "$AF_DIR" | grep -E "^${num}_" | grep -v -x "$name" || true)"
  if [ -n "$clash" ]; then
    echo "airframe number $num is already used by: $clash" >&2
    echo "pick a free number, rename px4/airframes/$name and update drone.yaml px4.airframe_id" >&2
    exit 2
  fi
  install -m 0755 "$af" "$AF_DIR/$name"
  if ! grep -q "$name" "$AF_DIR/CMakeLists.txt"; then
    # register it in the px4_add_romfs_files(...) list (insert before the closing parenthesis)
    python3 - "$AF_DIR/CMakeLists.txt" "$name" <<'EOF'
import sys, re
path, name = sys.argv[1], sys.argv[2]
s = open(path).read()
i = s.rfind(")")
s = s[:i] + f"\t{name}\n" + s[i:]
open(path, "w").write(s)
EOF
    echo "registered $name in $AF_DIR/CMakeLists.txt"
  fi
done

mkdir -p "$GZ_DIR/models" "$GZ_DIR/worlds"
for m in "$REPO"/models/*/; do
  rm -rf "$GZ_DIR/models/$(basename "$m")"
  cp -r "$m" "$GZ_DIR/models/"
done
cp "$REPO"/worlds/*.sdf "$GZ_DIR/worlds/"

echo "Installed qkd_sim assets into $PX4_DIR"
echo "Rebuild once: (cd $PX4_DIR && make px4_sitl)"
