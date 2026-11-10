#!/usr/bin/env bash
# Start the qkd_sim container with GPU passthrough (camera rendering needs a GPU) and optional X11.
#
#   docker/run.sh                 # interactive shell, repo mounted at /qkd_sim
#   docker/run.sh python3 tools/run_scenario.py t3_turbulence --rig rig_gimbal
#   NO_X11=1 docker/run.sh ...    # headless only (EGL)
#
# NVIDIA: needs the NVIDIA Container Toolkit. Other GPUs: replace --gpus all with --device /dev/dri.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
IMAGE="${IMAGE:-qkd_sim}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  docker build -t "$IMAGE" -f "$REPO/docker/Dockerfile" "$REPO"
fi

X11_ARGS=()
if [ -z "${NO_X11:-}" ] && [ -n "${DISPLAY:-}" ]; then
  xhost +local:docker >/dev/null 2>&1 || true
  X11_ARGS=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
if [ -n "${DRI_GPU:-}" ]; then
  GPU_ARGS=(--device /dev/dri)
fi

exec docker run --rm -it \
  "${GPU_ARGS[@]}" "${X11_ARGS[@]}" \
  --net host --ipc host \
  -e QKD_RUN_SIM="${QKD_RUN_SIM:-1}" \
  -v "$REPO":/qkd_sim \
  -w /qkd_sim \
  "$IMAGE" "${@:-bash}"
