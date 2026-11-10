# qkd_sim: Ubuntu 24.04 + Gazebo Harmonic + PX4 SITL toolchain + Python libs.
#
#   docker build -t qkd_sim -f docker/Dockerfile .
#   docker/run.sh                      # GPU passthrough, repo mounted at /qkd_sim
#
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
# Pin to the latest stable PX4 release tag (v1.16+ supports Ubuntu 24.04).
ARG PX4_TAG=v1.16.0
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Etc/UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg lsb-release sudo git wget unzip \
        python3 python3-pip python3-venv python3-dev build-essential cmake ninja-build \
        libegl1 libgl1 libgles2 libglvnd0 mesa-utils x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic (LTS) from packages.osrfoundation.org, with the Python bindings
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic python3-gz-transport13 python3-gz-msgs10 \
 && rm -rf /var/lib/apt/lists/*

# PX4 SITL toolchain (pinned release tag; update PX4_TAG to the latest stable release)
RUN git clone --depth 1 --branch ${PX4_TAG} --recursive https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
 && rm -rf /var/lib/apt/lists/*
ENV PX4_DIR=/opt/PX4-Autopilot

# Python libraries (system site-packages so the apt gz bindings stay importable)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

# Install the qkd_sim assets into PX4 and build SITL once (the repo is copied for the build; at run time the
# live repo is mounted at /qkd_sim and run_scenario.py uses it directly)
COPY . /tmp/qkd_sim
RUN bash /tmp/qkd_sim/px4/install_px4_assets.sh /opt/PX4-Autopilot \
 && cd /opt/PX4-Autopilot && make px4_sitl \
 && rm -rf /tmp/qkd_sim

ENV GZ_SIM_RESOURCE_PATH=/qkd_sim/models
ENV PYTHONPATH=/qkd_sim
WORKDIR /qkd_sim
CMD ["bash"]
