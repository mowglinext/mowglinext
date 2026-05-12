# =============================================================================
# MowgliNext Development Container
#
# All-in-one image for GitHub Codespaces and local devcontainer usage.
# Includes: ROS2 Kilted, Nav2, Gazebo Harmonic (headless), Go 1.24, Node 22,
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
#     --build-arg BASE_IMAGE=ghcr.io/cedbossneo/mowglinext/mowgli-ros2:feat-my-branch \
#     -f .devcontainer/Dockerfile -t mowglinext-devcontainer .
#
# Default (ros:kilted-ros-base) reproduces the upstream Codespaces flow
# bit-for-bit; that is what CI uses so contributors get an identical
# environment.
# =============================================================================
# ---------------------------------------------------------------------------
# Stage 0: gtsam-builder
# Mirrors ros2/Dockerfile stage 0. Builds GTSAM 4.3a1 from source —
# the Ubuntu Noble libgtsam-dev (4.2) ships a broken GTSAMConfig.cmake
# and uses the legacy boost::optional<Matrix&> custom-factor API, and
# there is no ros-kilted-gtsam package. Source build gives the modern
# OptionalMatrixType API needed by fusion_graph's GnssLeverArmFactor.
# ---------------------------------------------------------------------------
ARG BASE_IMAGE=ros:kilted-ros-base

FROM ros:kilted-ros-base AS gtsam-builder

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      libboost-all-dev libeigen3-dev libmetis-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 -b 4.3a1 https://github.com/borglab/gtsam.git

RUN cd gtsam && mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/gtsam \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_USE_SYSTEM_METIS=ON \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_WITH_EIGEN_MKL=OFF \
        -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
        -DBUILD_SHARED_LIBS=ON \
 && make -j$(nproc) install

FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. ROS2 runtime + build + simulation dependencies
#    (apt layer — heaviest; cache by keeping this mostly stable)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── Navigation ──
    ros-kilted-nav2-bringup \
    ros-kilted-nav2-bt-navigator \
    ros-kilted-nav2-controller \
    ros-kilted-nav2-planner \
    ros-kilted-nav2-behaviors \
    ros-kilted-nav2-regulated-pure-pursuit-controller \
    ros-kilted-nav2-smac-planner \
    ros-kilted-nav2-costmap-2d \
    ros-kilted-nav2-map-server \
    ros-kilted-nav2-lifecycle-manager \
    ros-kilted-nav2-waypoint-follower \
    ros-kilted-nav2-velocity-smoother \
    ros-kilted-nav2-rotation-shim-controller \
    ros-kilted-nav2-core \
    ros-kilted-nav2-util \
    ros-kilted-nav2-msgs \
    # ── DDS (Cyclone is required — FastRTPS has stale shm issues on ARM) ──
    ros-kilted-rmw-cyclonedds-cpp \
    # ── Docking ──
    ros-kilted-opennav-docking \
    # ── Motion control ──
    ros-kilted-twist-mux \
    ros-kilted-topic-tools \
    # ── Behavior trees ──
    ros-kilted-behaviortree-cpp \
    # ── Robot description / TF ──
    ros-kilted-xacro \
    ros-kilted-robot-state-publisher \
    ros-kilted-joint-state-publisher \
    ros-kilted-tf2-ros \
    ros-kilted-tf2-eigen \
    ros-kilted-tf2-geometry-msgs \
    # ── Grid map ──
    ros-kilted-grid-map-core \
    ros-kilted-grid-map-ros \
    ros-kilted-grid-map-msgs \
    # ── Diagnostics ──
    ros-kilted-diagnostic-msgs \
    ros-kilted-diagnostic-updater \
    # ── Bridges ──
    ros-kilted-foxglove-bridge \
    ros-kilted-rosbridge-suite \
    # ── Message definitions ──
    ros-kilted-ublox-msgs \
    ros-kilted-nmea-msgs \
    ros-kilted-rtcm-msgs \
    # ── Gazebo Harmonic (headless simulation) ──
    ros-kilted-ros-gz-sim \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-interfaces \
    # ── Build tools ──
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-kilted-ament-cmake \
    ros-kilted-rosidl-default-generators \
    build-essential \
    cmake \
    git \
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
    clang-format \
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
RUN set -eux; \
    arch="$(dpkg --print-architecture)"; \
    case "$arch" in \
      amd64) go_arch=amd64 ;; \
      arm64) go_arch=arm64 ;; \
      *) echo "Unsupported arch: $arch" >&2; exit 1 ;; \
    esac; \
    curl -fsSL "https://go.dev/dl/go${GO_VERSION}.linux-${go_arch}.tar.gz" \
      | tar -C /usr/local -xz
ENV PATH="/usr/local/go/bin:${PATH}"

# ---------------------------------------------------------------------------
# 4. Node 22 + yarn — matches gui/Dockerfile build stage (gui/web build).
# ---------------------------------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
 && apt-get install -y --no-install-recommends nodejs \
 && rm -rf /var/lib/apt/lists/* \
 && npm install -g --no-fund --no-audit yarn

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
RUN rosdep update --rosdistro kilted || true

# ---------------------------------------------------------------------------
# 7. Workspace directory
# ---------------------------------------------------------------------------
WORKDIR /ros2_ws

# ---------------------------------------------------------------------------
# 8. Environment setup — auto-source ROS2 + workspace in every shell
# ---------------------------------------------------------------------------
RUN echo 'source /opt/ros/kilted/setup.bash' >> /etc/bash.bashrc && \
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
