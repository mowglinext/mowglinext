# syntax=docker/dockerfile:1.7
# =============================================================================
# MowgliNext Development Container
#
# All-in-one image for GitHub Codespaces and local devcontainer usage.
# Includes: ROS2 Lyrical, Nav2, Gazebo (headless), Go 1.24, Node 22,
# Python linting, build tools, and developer utilities.
#
# Gazebo runs headless (CPU rendering) — connect Foxglove Studio to
# ws://localhost:8765 for visualization.
#
# Fast-iterate optimisation: pass --build-arg BASE_IMAGE=<per-branch CI image>
# to skip ~10 minutes of apt-install (Nav2 / ROS plugins / slam_toolbox are
# already baked in) and let post-create.sh detect that /ros2_ws/install/
# is already populated, turning colcon build into a fast incremental.
#
# Example:
#   docker build \
#     --build-arg BASE_IMAGE=ghcr.io/mowglinext/mowglinext/mowgli-ros2:feat-my-branch \
#     -f .devcontainer/Dockerfile -t mowglinext-devcontainer .
#
# Default (ros:lyrical-ros-base) reproduces the upstream Codespaces flow
# bit-for-bit; that is what CI uses so contributors get an identical
# environment.
# =============================================================================
# ---------------------------------------------------------------------------
# Stage 0: gtsam-builder
# Mirrors ros2/Dockerfile stage 0. Builds GTSAM 4.3a1 from source —
# the Ubuntu Noble libgtsam-dev (4.2) ships a broken GTSAMConfig.cmake
# and uses the legacy boost::optional<Matrix&> custom-factor API. Lyrical's
# ROS binary is a different 4.3 snapshot with TBB enabled; retain the existing
# source pin and TBB setting used by fusion_graph.
# ---------------------------------------------------------------------------
ARG BASE_IMAGE=ros:lyrical-ros-base

# -Wno-error=array-bounds below mirrors ros2/Dockerfile + ros2-ci.yml (same
# GTSAM recipe, keep in lockstep): GCC 15 on an x86-64-v3 default toolchain
# (GitHub's amd64v3 `ubuntu-26.04` runner) trips a false-positive
# -Warray-bounds in Eigen's AVX loads, and GTSAM 4.3a1 hardcodes -Werror.
FROM ros:lyrical-ros-base AS gtsam-builder
ARG TARGETARCH

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      libboost-all-dev libeigen3-dev libmetis-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 -b 4.3a1 https://github.com/borglab/gtsam.git

RUN mkdir -p /opt/gtsam && chown -R ubuntu:ubuntu /opt/gtsam /tmp/gtsam
USER ubuntu
RUN --mount=type=cache,id=mowgli-gtsam-lyrical-4.3a1-${TARGETARCH},target=/tmp/gtsam/build,uid=1000,gid=1000,sharing=locked \
    cmake -S /tmp/gtsam -B /tmp/gtsam/build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=/opt/gtsam \
      -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=ON \
      -DGTSAM_WITH_TBB=OFF -DGTSAM_WITH_EIGEN_MKL=OFF \
      -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_CXX_FLAGS=-Wno-error=array-bounds \
 && cmake --build /tmp/gtsam/build --parallel 3 \
 && cmake --install /tmp/gtsam/build

FROM ros:lyrical-ros-base AS fields2cover-v3-builder
ARG TARGETARCH

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config \
      libeigen3-dev libgdal-dev libgeos++-dev libgeos-dev libgsl-dev \
      libtinyxml2-dev libpython3-dev python3-dev libtbb-dev nlohmann-json3-dev \
      ros-lyrical-ortools-vendor \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
# Pinned to v3.0 @ 884d895 (2026-03-02) — do NOT track the moving branch head.
# Patch an upstream bug in this commit: visualizer.cpp uses std::setw/setfill
# without #include <iomanip> (modern GCC/libstdc++ no longer pulls it in
# transitively), which fails the build with "'setw' is not a member of 'std'".
RUN git clone https://github.com/Fields2Cover/Fields2Cover.git f2c3 && \
    cd f2c3 && git checkout 884d895b59192882476e986ba44ea9143a06a6a9
RUN cd /tmp/f2c3 && \
    sed -i '1i #include <iomanip>' src/fields2cover/utils/visualizer.cpp && \
    sed -i '1i #include <algorithm>' src/fields2cover/types/Graph.cpp

RUN mkdir -p /opt/fields2cover-300 && chown -R ubuntu:ubuntu /opt/fields2cover-300 /tmp/f2c3
USER ubuntu
RUN --mount=type=cache,id=mowgli-f2c-lyrical-884d895-${TARGETARCH},target=/tmp/f2c3/build,uid=1000,gid=1000,sharing=locked \
    . /opt/ros/lyrical/setup.sh \
 && cmake -S /tmp/f2c3 -B /tmp/f2c3/build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_INSTALL_PREFIX=/opt/fields2cover-300 \
      -DBUILD_TESTS=OFF -DBUILD_PYTHON=OFF -DBUILD_TUTORIALS=OFF \
      -DBUILD_DOC=OFF -DALLOW_PARALLELIZATION=ON -DBUILD_SHARED_LIBS=ON \
 && cmake --build /tmp/f2c3/build --parallel 3 \
 && cmake --install /tmp/f2c3/build


# Source overlays for dependencies not released as Lyrical binaries.
FROM ros:lyrical-ros-base AS lyrical-vendor
ARG TARGETARCH
ARG DEBIAN_FRONTEND=noninteractive
COPY ros2/lyrical-vendor-packages.txt /tmp/vendor-packages.txt
RUN apt-get update && xargs apt-get install -y --no-install-recommends < /tmp/vendor-packages.txt \
 && mkdir -p /opt/lyrical_vendor && chown ubuntu:ubuntu /opt/lyrical_vendor
COPY ros2/scripts/build_lyrical_vendor.sh /tmp/build_lyrical_vendor.sh
USER ubuntu
RUN --mount=type=cache,id=mowgli-vendor-lyrical-${TARGETARCH},target=/tmp/lyrical-vendor,uid=1000,gid=1000,sharing=locked \
    VENDOR_WORKSPACE=/tmp/lyrical-vendor bash /tmp/build_lyrical_vendor.sh

FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
# A cached project image must use the same distribution as these dependencies.
RUN test "${ROS_DISTRO}" = lyrical

# ---------------------------------------------------------------------------
# 1. ROS2 runtime + build + simulation dependencies
#    (apt layer — heaviest; cache by keeping this mostly stable)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── Navigation ──
    ros-lyrical-nav2-bt-navigator \
    ros-lyrical-nav2-controller \
    ros-lyrical-nav2-planner \
    ros-lyrical-nav2-smoother \
    ros-lyrical-nav2-collision-monitor \
    ros-lyrical-nav2-behaviors \
    ros-lyrical-nav2-regulated-pure-pursuit-controller \
    ros-lyrical-nav2-costmap-2d \
    ros-lyrical-nav2-map-server \
    ros-lyrical-nav2-lifecycle-manager \
    ros-lyrical-nav2-waypoint-follower \
    ros-lyrical-nav2-velocity-smoother \
    ros-lyrical-nav2-rotation-shim-controller \
    ros-lyrical-nav2-core \
    ros-lyrical-nav2-util \
    ros-lyrical-nav2-msgs \
    # ── DDS (Cyclone is required — FastRTPS has stale shm issues on ARM) ──
    ros-lyrical-rmw-cyclonedds-cpp \
    # ── Docking ──
    ros-lyrical-opennav-docking \
    # ── Motion control ──
    ros-lyrical-twist-mux \
    ros-lyrical-topic-tools \
    # ── Behavior trees ──
    ros-lyrical-behaviortree-cpp \
    # ── Robot description / TF ──
    ros-lyrical-xacro \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-joint-state-publisher \
    ros-lyrical-tf2-ros \
    ros-lyrical-tf2-eigen \
    ros-lyrical-tf2-geometry-msgs \
    # ── Grid map ──
    # ── Diagnostics ──
    ros-lyrical-diagnostic-msgs \
    ros-lyrical-diagnostic-updater \
    ros-lyrical-rclcpp-components \
    ros-lyrical-rclcpp-lifecycle \
    # ── Bridges ──
    ros-lyrical-foxglove-bridge \
    ros-lyrical-rosbridge-suite \
    # ── Message definitions ──
    ros-lyrical-ublox-msgs \
    ros-lyrical-nmea-msgs \
    ros-lyrical-rtcm-msgs \
    ros-lyrical-geometry-msgs \
    ros-lyrical-nav-msgs \
    ros-lyrical-sensor-msgs \
    ros-lyrical-std-msgs \
    # ── Gazebo (headless simulation) ──
    ros-lyrical-ros-gz-sim \
    ros-lyrical-ros-gz-bridge \
    ros-lyrical-ros-gz-interfaces \
    # ── Build tools ──
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-lyrical-ament-cmake \
    ros-lyrical-rosidl-default-generators \
    build-essential \
    cmake \
    git \
    libcurl4-openssl-dev \
    libusb-1.0-0-dev \
    # ── Runtime deps ──
    libtinyxml2-dev \
    python3-argcomplete \
    # ── GTSAM runtime deps — libgtsam itself is COPYed from gtsam-builder.
    #    libmetis-dev provides both libmetis5 runtime and the unversioned
    #    libmetis.so symlink GTSAM's exported CMake target links against.
    #    libboost-* runtime libs come in transitively. ──
    libmetis-dev \
    # ── Developer tools ──
    gdb \
    htop \
    vim \
    curl \
    wget \
    bash-completion \
    clang-format-18 \
    cppcheck \
    # ── Python runtime + linting (matches mowgli_localization runtime deps) ──
    python3 \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-yaml \
    # ── Headless rendering (Gazebo sensors need a virtual display on ARM) ──
    xvfb \
    # ── GitHub CLI ──
    gpg \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/* \
 && curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
      | gpg --dearmor -o /usr/share/keyrings/githubcli-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
      > /etc/apt/sources.list.d/github-cli.list \
 && apt-get update && apt-get install -y --no-install-recommends gh \
 && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/clang-format-18 /usr/local/bin/clang-format
COPY ros2/lyrical-vendor-packages.txt /tmp/vendor-packages.txt
RUN apt-get update && xargs apt-get install -y --no-install-recommends < /tmp/vendor-packages.txt
COPY --from=fields2cover-v3-builder /opt/fields2cover-300 /opt/fields2cover-300
COPY --from=lyrical-vendor /opt/lyrical_vendor /opt/lyrical_vendor
RUN echo '/opt/fields2cover-300/lib' > /etc/ld.so.conf.d/fields2cover.conf && ldconfig
ENV CMAKE_PREFIX_PATH=/opt/lyrical_vendor:/opt/fields2cover-300:/opt/gtsam

# ---------------------------------------------------------------------------
# 2. Python linters + pre-commit (system-wide, PEP 668 override OK inside
#    a dev container). Versions track .pre-commit-config.yaml if the repo
#    adds one later.
# ---------------------------------------------------------------------------
RUN pip install --break-system-packages --no-cache-dir \
    ruff==0.8.0 \
    pre-commit==4.0.1

# ---------------------------------------------------------------------------
# 3. Go 1.24 — matches gui/go.mod + gui/Dockerfile build stage.
#    Installed under /usr/local/go, put on PATH.
# ---------------------------------------------------------------------------
ENV GO_VERSION=1.24.0
# Python extraction also works under amd64/Rosetta, where Resolute's GNU tar
# fails on nested paths (orbstack/orbstack#2588).
RUN set -eux; \
    arch="$(dpkg --print-architecture)"; \
    case "$arch" in \
      amd64) go_arch=amd64 ;; \
      arm64) go_arch=arm64 ;; \
      *) echo "Unsupported arch: $arch" >&2; exit 1 ;; \
    esac; \
    curl -fsSL "https://go.dev/dl/go${GO_VERSION}.linux-${go_arch}.tar.gz" -o /tmp/go.tar.gz; \
    python3 -m tarfile --filter data -e /tmp/go.tar.gz /usr/local; \
    rm /tmp/go.tar.gz
ENV PATH="/usr/local/go/bin:${PATH}"

# ---------------------------------------------------------------------------
# 4. Node 22 + yarn — matches gui/Dockerfile build stage (gui/web build).
# ---------------------------------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
 && apt-get install -y --no-install-recommends nodejs \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /opt/node-tools \
 && chown ubuntu:ubuntu /opt/node-tools \
 && runuser -u ubuntu -- npm install --prefix /opt/node-tools -g --no-fund --no-audit yarn
ENV PATH="/opt/node-tools/bin:${PATH}"

# ---------------------------------------------------------------------------
# 5. Pull source-built GTSAM 4.3a1 into /opt/gtsam, register its lib
#    directory with ldconfig, and expose the CMake config dir so
#    find_package(GTSAM) resolves without extra hints. Mirrors
#    ros2/Dockerfile so the devcontainer matches production exactly.
# ---------------------------------------------------------------------------
COPY --from=gtsam-builder /opt/gtsam /opt/gtsam
RUN echo '/opt/gtsam/lib' > /etc/ld.so.conf.d/gtsam.conf && ldconfig
ENV CMAKE_PREFIX_PATH=/opt/gtsam:${CMAKE_PREFIX_PATH}

# ---------------------------------------------------------------------------
# 6. rosdep init (needed for resolving deps on first build)
# ---------------------------------------------------------------------------
RUN runuser -u ubuntu -- rosdep update --rosdistro lyrical

# ---------------------------------------------------------------------------
# 7. Workspace directory
# ---------------------------------------------------------------------------
WORKDIR /ros2_ws

# ---------------------------------------------------------------------------
# 8. Environment setup — auto-source ROS2 + workspace in every shell
# ---------------------------------------------------------------------------
RUN echo 'source /opt/ros/lyrical/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /opt/lyrical_vendor/local_setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /etc/bash.bashrc && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
# 9. Cyclone DDS config (localhost-only discovery for containers)
# ---------------------------------------------------------------------------
COPY install/config/cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ---------------------------------------------------------------------------
# NOT installed by default (opt in if you need them):
#   PlatformIO      — firmware builds (~500 MB). Install with:
#                       pip install --break-system-packages platformio
#                     Works on x86 for ARM-target cross-compile.
#   Docker-in-Docker — multi-container sim/hardware topologies. Add the
#                     docker-in-docker feature to devcontainer.json.
# ---------------------------------------------------------------------------

# Builds and generated files belong to the normal project user.
RUN apt-get update && apt-get install -y --no-install-recommends sudo \
 && usermod -aG dialout ubuntu \
 && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
 && chown -R ubuntu:ubuntu /ros2_ws
USER ubuntu
