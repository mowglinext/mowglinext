# syntax=docker/dockerfile:1.7
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

# =============================================================================
# Stage 0: gtsam-builder
# Builds GTSAM 4.3a1 from source. The Ubuntu Noble apt package
# (libgtsam-dev 4.2) ships a broken GTSAMConfig.cmake (references files
# that aren't installed) AND uses the legacy boost::optional<Matrix&>
# API for custom factors. Keep the existing 4.3a1 pin during this migration.
# Building from source gives us a clean install + the modern
# OptionalMatrixType API + a single pinned version we control.
#
# Install lands in /opt/gtsam; runtime stages COPY it across, set up
# ldconfig, and export CMAKE_PREFIX_PATH so find_package(GTSAM) works.
#
# -Wno-error=array-bounds mirrors .github/workflows/ros2-ci.yml (same recipe,
# keep in lockstep): GTSAM 4.3a1 hardcodes -Werror, and on the GitHub
# `ubuntu-26.04` runner (Ubuntu `amd64v3` variant, GCC 15 targeting x86-64-v3
# by default) GCC 15 emits a false-positive -Warray-bounds on Eigen's AVX
# loads of fixed-size vectors. This baseline-amd64/arm64 image never enables
# AVX, so the flag is inert here.
# =============================================================================
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

# =============================================================================
# Stage 0b2: fields2cover-v3-builder  (coverage migration — Phase 1b: live)
# Builds Fields2Cover v3 into /opt/fields2cover-300. v3 is UNRELEASED (no tag/
# release); it lives on the moving `v3.0` branch, so we PIN a commit SHA. The win
# is native headland routing (rp::RoutePlannerBase::genRoute, graph via OR-Tools)
# to fix the headland-perimeter freeze. OR-Tools comes from
# ros-lyrical-ortools-vendor; nlohmann-json3-dev is an
# added v3 dep. mowgli_coverage now find_package()es 3.0.0 at
# /opt/fields2cover-300 and adapts the one API break (generateHeadlandSwaths
# returns vector<F2CMultiLineString>, was vector<F2CCells>).
# =============================================================================
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

# =============================================================================
# Stage 0c: ublox-msgs-builder
# Builds ONLY the ublox_ubx_msgs + ublox_ubx_interfaces interface packages
# from the cedbossneo/ublox_dgnss fork. The GPS driver runs in the separate
# sensors/gps image and publishes ~25 /ubx_* topics typed with
# ublox_ubx_msgs/* and ublox_dgnss services typed with ublox_ubx_interfaces/*.
# foxglove_bridge in THIS image introspects every topic/service on the graph
# and was logging one "package not found" error per topic on every reconnect
# because the schemas weren't installed here. We only need the .msg/.srv
# typesupport for schema resolution, NOT the driver node — so build just the
# two interface packages with --packages-up-to (pulls in nothing else; both
# depend only on builtin_interfaces/std_msgs/rosidl, present in the base).
#
# The pinned ref MUST stay identical to sensors/gps/Dockerfile's
# UBLOX_DGNSS_REF/SHA — a schema mismatch between what the driver publishes
# and what foxglove loads here would be worse than the current log spam.
# Bump both Dockerfiles together when the fork moves.
# =============================================================================
FROM ros:lyrical-ros-base AS ublox-msgs-builder

ARG DEBIAN_FRONTEND=noninteractive
ARG UBLOX_DGNSS_REF=fix/hp-fix-stale-status-cov
ARG UBLOX_DGNSS_SHA=5e1d0cf1e8ec0e2638c44a59f57d15f5dd04c9af
ARG UBLOX_DGNSS_URL=https://github.com/cedbossneo/ublox_dgnss.git

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      python3-colcon-common-extensions \
      ros-lyrical-rosidl-default-generators \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
RUN git clone --branch ${UBLOX_DGNSS_REF} ${UBLOX_DGNSS_URL} ublox_dgnss \
 && cd ublox_dgnss && git checkout ${UBLOX_DGNSS_SHA}

WORKDIR /ws
RUN chown -R ubuntu:ubuntu /ws
USER ubuntu
RUN . /opt/ros/lyrical/setup.sh \
 && colcon build --merge-install \
      --packages-up-to ublox_ubx_msgs ublox_ubx_interfaces \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

# =============================================================================
# Stage 0d: universal-gnss-interfaces-builder
# Builds ONLY Universal GNSS's public interface package from the pinned
# universal-gnss submodule. foxglove_bridge and mowgli_gnss_bridge need these
# schemas, while receiver/NTRIP/driver/tools remain exclusively in the
# external Universal GNSS sidecar.
# =============================================================================
FROM ros:lyrical-ros-base AS universal-gnss-interfaces-builder

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake \
      python3-colcon-common-extensions \
      ros-lyrical-ament-cmake \
      ros-lyrical-rosidl-default-generators \
      ros-lyrical-rosidl-default-runtime \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
COPY ros2/src/external/universal-gnss/universal_gnss_msgs/ universal_gnss_msgs/

WORKDIR /ws
RUN chown -R ubuntu:ubuntu /ws
USER ubuntu
RUN . /opt/ros/lyrical/setup.sh \
 && colcon build --merge-install \
      --packages-select universal_gnss_msgs \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

# =============================================================================
# Stage 1: base
# Installs all runtime ROS2 dependencies. Both runtime and simulation stages
# inherit from here, keeping the image lean.
# =============================================================================
FROM ros:lyrical-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive
COPY ros2/lyrical-vendor-packages.txt /tmp/vendor-packages.txt
RUN apt-get update && xargs apt-get install -y --no-install-recommends < /tmp/vendor-packages.txt
COPY --from=lyrical-vendor /opt/lyrical_vendor /opt/lyrical_vendor
ENV AMENT_PREFIX_PATH=/opt/lyrical_vendor
ENV CMAKE_PREFIX_PATH=/opt/lyrical_vendor
ENV LD_LIBRARY_PATH=/opt/lyrical_vendor/lib

# LOCAL-ONLY (do not commit): this Docker host has no IPv6 route, so apt's
# default IPv6-first to ports.ubuntu.com times out. Force IPv4 for all apt in
# the base-derived stages.
RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Navigation
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
    # Motion control
    ros-lyrical-twist-mux \
    # Behavior trees
    ros-lyrical-behaviortree-cpp \
    # Robot description / TF
    ros-lyrical-xacro \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-joint-state-publisher \
    ros-lyrical-tf2-ros \
    ros-lyrical-tf2-eigen \
    ros-lyrical-tf2-geometry-msgs \
    # Grid map
    # Diagnostics
    ros-lyrical-diagnostic-msgs \
    ros-lyrical-diagnostic-updater \
    # Foxglove Bridge (WebSocket bridge for GUI and Foxglove Studio)
    ros-lyrical-foxglove-bridge \
    # RTCM message definitions for the public /rtcm topic
    ros-lyrical-rtcm-msgs \
    # tinyxml2 (required at runtime by BehaviorTree.CPP)
    libtinyxml2-dev \
    # Topic tools (relay, throttle — used in simulation for topic remapping)
    ros-lyrical-topic-tools \
    # Docking (opennav_docking — dock/undock action server)
    # DockRobot/UndockRobot actions are in nav2_msgs (already installed).
    # The docking_server executable is in opennav_docking.
    ros-lyrical-opennav-docking \
    # Rotation shim controller for transit navigation
    ros-lyrical-nav2-rotation-shim-controller \
    # Nav2 core headers (FTC controller plugin depends on nav2_core)
    ros-lyrical-nav2-core \
    ros-lyrical-nav2-util \
    # Nav2 msgs (CostmapFilterInfo for costmap filters)
    ros-lyrical-nav2-msgs \
    # DDS middleware — Cyclone DDS is more reliable than FastRTPS on ARM
    # (no shared memory, no stale /dev/shm segments, better service discovery)
    ros-lyrical-rmw-cyclonedds-cpp \
    # GTSAM runtime deps — libgtsam itself is COPYed from the
    # gtsam-builder stage below. libmetis-dev provides both the
    # runtime libmetis5 and the unversioned libmetis.so symlink
    # that GTSAM's exported CMake target references at link time.
    # libboost-* runtime libs are pulled in transitively.
    libmetis-dev \
    # Fields2Cover runtime deps. libFields2Cover.so links
    # against libgdal, libgeos_c, libortools, libtinyxml2 + bundles
    # its own libsteering_functions and libmatplot under
    # /opt/fields2cover-300/lib. -dev variants are needed because
    # mowgli_coverage's find_package pulls in transitive headers via
    # the exported Fields2Cover CMake target.
    libgdal-dev libgeos++-dev libtinyxml2-dev libtbb12 \
    ros-lyrical-ortools-vendor \
    # LiDAR map anchor: Beluga MCL (header-only) + its ROS adapters
    # MQTT bridge (mowgli_monitoring/mqtt_bridge_node) runtime dependency.
    # libmosquitto-dev (headers + .so, needed to link) is installed in the
    # deps stage below; this is the runtime .so the final `runtime` image
    # (built FROM base, not FROM deps) actually needs to load
    # MosquittoMqttClient. Not a rosdep key (checked ros/rosdistro
    # rosdep/base.yaml — absent), so it's a plain apt-get here rather than
    # a package.xml <depend>.
    libmosquitto1 \
    # Utilities
    python3-argcomplete \
    # WebSocket server for cmd_vel_ws_relay (manual mowing low-latency path)
    python3-websockets \
 && rm -rf /var/lib/apt/lists/*

# Pull source-built GTSAM 4.3a1 into /opt/gtsam and register its lib
# directory with ldconfig. The full workspace build re-exports the
# external CMake prefixes explicitly before colcon so the overlay setup
# scripts cannot drop them.
COPY --from=gtsam-builder /opt/gtsam /opt/gtsam
RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-serialization-dev libboost-filesystem-dev libboost-thread-dev \
    libboost-program-options-dev libboost-date-time-dev libboost-timer-dev \
    libboost-chrono-dev libboost-regex-dev \
 && rm -rf /var/lib/apt/lists/* \
 && echo '/opt/gtsam/lib' > /etc/ld.so.conf.d/gtsam.conf && ldconfig

# Coverage uses the pinned Fields2Cover v3 install. Its OR-Tools dependency
# is supplied by the ROS environment.
COPY --from=fields2cover-v3-builder /opt/fields2cover-300 /opt/fields2cover-300
RUN echo '/opt/fields2cover-300/lib' > /etc/ld.so.conf.d/fields2cover-300.conf && ldconfig

# =============================================================================
# Stage 2: deps
# Installs build tools and resolves rosdep dependencies. Cached as long as
# package.xml files don't change.
# =============================================================================
FROM base AS deps

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ccache \
    ros-lyrical-ament-cmake \
    ros-lyrical-rosidl-default-generators \
    build-essential \
    cmake \
    git \
    # mowgli_monitoring's mqtt_bridge_node links against libmosquitto at
    # build time (CMakeLists.txt's pkg_check_modules/find_library(mosquitto)
    # gate on this to define MOWGLI_HAS_MOSQUITTO). Installed directly here,
    # not via rosdep, because libmosquitto-dev has no rosdep key. The
    # runtime-only libmosquitto1 is installed in the `base` stage above so
    # the final `runtime` image (FROM base) has it too.
    libmosquitto-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy only package.xml + CMakeLists.txt to resolve rosdep deps (cache layer).
# This ensures the expensive rosdep install is only re-run when package
# definitions change, NOT when source code changes. The Docker build context
# is the monorepo root so we can also pull the sidecar tools/motor package.
COPY ros2/src/mowgli_interfaces/package.xml    src/mowgli_interfaces/package.xml
COPY ros2/src/mowgli_interfaces/CMakeLists.txt src/mowgli_interfaces/CMakeLists.txt
COPY ros2/src/mowgli_hardware/package.xml      src/mowgli_hardware/package.xml
COPY ros2/src/mowgli_hardware/CMakeLists.txt   src/mowgli_hardware/CMakeLists.txt
COPY ros2/src/mowgli_localization/package.xml  src/mowgli_localization/package.xml
COPY ros2/src/mowgli_localization/CMakeLists.txt src/mowgli_localization/CMakeLists.txt
COPY ros2/src/mowgli_map/package.xml           src/mowgli_map/package.xml
COPY ros2/src/mowgli_map/CMakeLists.txt        src/mowgli_map/CMakeLists.txt
COPY ros2/src/mowgli_nav2_plugins/package.xml  src/mowgli_nav2_plugins/package.xml
COPY ros2/src/mowgli_nav2_plugins/CMakeLists.txt src/mowgli_nav2_plugins/CMakeLists.txt
COPY ros2/src/mowgli_behavior/package.xml      src/mowgli_behavior/package.xml
COPY ros2/src/mowgli_behavior/CMakeLists.txt   src/mowgli_behavior/CMakeLists.txt
COPY ros2/src/mowgli_monitoring/package.xml    src/mowgli_monitoring/package.xml
COPY ros2/src/mowgli_monitoring/CMakeLists.txt src/mowgli_monitoring/CMakeLists.txt
COPY ros2/src/mowgli_leds/package.xml          src/mowgli_leds/package.xml
COPY ros2/src/mowgli_leds/CMakeLists.txt      src/mowgli_leds/CMakeLists.txt
COPY ros2/src/mowgli_simulation/package.xml    src/mowgli_simulation/package.xml
COPY ros2/src/mowgli_simulation/CMakeLists.txt src/mowgli_simulation/CMakeLists.txt
COPY ros2/src/mowgli_bringup/package.xml       src/mowgli_bringup/package.xml
COPY ros2/src/mowgli_bringup/CMakeLists.txt    src/mowgli_bringup/CMakeLists.txt
COPY ros2/src/fusion_graph/package.xml         src/fusion_graph/package.xml
COPY ros2/src/fusion_graph/CMakeLists.txt      src/fusion_graph/CMakeLists.txt
# fusion_graph is the sole localizer and uses the pinned source GTSAM install.
COPY ros2/src/mowgli_coverage/package.xml      src/mowgli_coverage/package.xml
COPY ros2/src/mowgli_coverage/CMakeLists.txt   src/mowgli_coverage/CMakeLists.txt
# opennav_coverage submodule provides opennav_coverage_msgs (action types
# consumed by mowgli_coverage). Pulled by checkout submodules: recursive.
COPY ros2/src/opennav_coverage/opennav_coverage_msgs/package.xml    src/opennav_coverage/opennav_coverage_msgs/package.xml
COPY ros2/src/opennav_coverage/opennav_coverage_msgs/CMakeLists.txt src/opennav_coverage/opennav_coverage_msgs/CMakeLists.txt
# universal-gnss is vendored as a git submodule under ros2/src/external.
# It is built separately as an interface-only overlay for foxglove_bridge.
# operator-side drivetrain tools live outside ros2/src but must still be built
# into the runtime workspace as the ROS 2 package mowgli_tools.
COPY tools/motor/package.xml    src/mowgli_tools/package.xml
COPY tools/motor/CMakeLists.txt src/mowgli_tools/CMakeLists.txt

# Resolve rosdep dependencies (cached until package.xml changes). The skipped
# libraries come from lyrical-vendor; Webots belongs to the simulation image.
RUN apt-get update \
 && rosdep update --rosdistro lyrical \
 && rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro lyrical \
      --skip-keys "grid_map_core grid_map_ros grid_map_msgs beluga_ros nav2_smac_planner webots_ros2_driver webots_ros2_control" \
      -y

# =============================================================================
# Stage 3: build-interfaces
# Build only mowgli_interfaces (msg/srv/action definitions). These rarely
# change and are a dependency for all other packages.
# =============================================================================
FROM deps AS build-interfaces

# Copy the full mowgli_interfaces package
COPY ros2/src/mowgli_interfaces/ src/mowgli_interfaces/

RUN chown -R ubuntu:ubuntu /ros2_ws
USER ubuntu
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && colcon build --packages-select mowgli_interfaces \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --parallel-workers 2'

# =============================================================================
# Stage 4: build
# Compiles all remaining workspace packages from source.
# =============================================================================
FROM build-interfaces AS build
USER root

ARG BUILD_TYPE=Release

# Install only the public Universal GNSS typesupport into the build environment
# so mowgli_gnss_bridge can compile without exposing the UG runtime packages.
COPY --from=universal-gnss-interfaces-builder /ws/install /opt/universal_gnss_interfaces

# Now copy the rest of the source tree plus the sidecar tools package
COPY ros2/src/ src/
COPY tools/motor/ src/mowgli_tools/
COPY sensors/gps/mowgli_gnss_bridge/ src/mowgli_gnss_bridge/

# The vendored root is a generic CMake project named universal_gnss, not a ROS
# package. Letting colcon discover it shadows its nested ROS package and builds
# unwanted driver tools; the schema-only overlay is built in Stage 0d instead.
RUN touch src/external/universal-gnss/COLCON_IGNORE

# Skip every opennav_coverage subpackage except _msgs. The upstream server
# is pinned to F2C 1.2.1 (no TrapezoidalDecomp / generateHeadlandSwaths /
# planPathForConnection); we use mowgli_coverage with F2C v3 instead, and
# only need the action types from opennav_coverage_msgs. Marker files
# aren't tracked in the submodule, so create them here.
RUN touch \
      src/opennav_coverage/opennav_coverage/COLCON_IGNORE \
      src/opennav_coverage/opennav_coverage_bt/COLCON_IGNORE \
      src/opennav_coverage/opennav_coverage_demo/COLCON_IGNORE \
      src/opennav_coverage/opennav_coverage_navigator/COLCON_IGNORE \
      src/opennav_coverage/opennav_row_coverage/COLCON_IGNORE

RUN /bin/bash -c "\
    f2c_config=\$(find /opt/fields2cover-300 -type f \( -name 'Fields2CoverConfig.cmake' -o -name 'fields2cover-config.cmake' \) | head -n 1) && \
    test -n \"\${f2c_config}\" && \
    echo \"Fields2Cover config: \${f2c_config}\" \
    "

# Build all packages (mowgli_interfaces is already built and cached)
RUN chown -R ubuntu:ubuntu /ros2_ws
ARG TARGETARCH
USER ubuntu
RUN --mount=type=cache,id=mowgli-ros2-lyrical-${TARGETARCH},target=/home/ubuntu/.cache/ccache,uid=1000,gid=1000,sharing=locked \
    /bin/bash -c 'source /opt/ros/lyrical/setup.bash \
 && source /opt/lyrical_vendor/local_setup.bash \
 && source /opt/universal_gnss_interfaces/setup.bash \
 && source install/setup.bash \
 && export CMAKE_PREFIX_PATH=/opt/lyrical_vendor:/opt/fields2cover-300:/opt/gtsam:${CMAKE_PREFIX_PATH:-} \
 && MAKEFLAGS=-j3 colcon build --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DBUILD_TESTING=OFF \
      -DCMAKE_CXX_STANDARD=20 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DMOWGLI_REQUIRE_MOSQUITTO=ON \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      --event-handlers console_cohesion+'

# Tests are built and gated by ros2-ci.yml (BUILD_TESTING=ON).

# =============================================================================
# Stage 5: runtime
# Minimal image containing only what is needed to run the robot.
# =============================================================================
FROM base AS runtime
LABEL garden.mowgli.maintenance-api="1"

WORKDIR /ros2_ws

# Pull compiled install tree from the build stage
COPY --from=build /ros2_ws/install/ install/

# Pull the ublox_ubx_msgs + ublox_ubx_interfaces typesupport built from the
# cedbossneo/ublox_dgnss fork (see ublox-msgs-builder stage). The GPS driver
# runs in the separate sensors/gps image, so these message/service packages
# aren't otherwise present here — without them foxglove_bridge logs one
# "package not found" schema error per /ubx_* topic + ublox_dgnss service on
# every client reconnect. Sourced by ros2_entrypoint.sh as a ROS overlay.
COPY --from=ublox-msgs-builder /ws/install /opt/ublox_msgs

# Pull the schema-only universal_gnss_msgs overlay (see
# universal-gnss-interfaces-builder stage). The full Universal GNSS runtime
# remains outside this image; foxglove_bridge and mowgli_gnss_bridge only need
# the generated public typesupport.
COPY --from=universal-gnss-interfaces-builder /ws/install /opt/universal_gnss_interfaces

# Copy all launch files and config from every package
COPY ros2/src/mowgli_bringup/launch/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/launch/
COPY ros2/src/mowgli_bringup/config/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/
# mowgli_localization has no launch files; nodes are brought up from
# mowgli_bringup/navigation.launch.py. Keep the config copy so the YAML
# files still land in the install tree.
COPY ros2/src/mowgli_localization/config/  /ros2_ws/install/mowgli_localization/share/mowgli_localization/config/

# fusion_graph ships its own launch + config used by navigation.launch.py
COPY ros2/src/fusion_graph/launch/  /ros2_ws/install/fusion_graph/share/fusion_graph/launch/
COPY ros2/src/fusion_graph/config/  /ros2_ws/install/fusion_graph/share/fusion_graph/config/

# Use Cyclone DDS — FastRTPS shared memory causes stale /dev/shm segments
# and unreliable service discovery on ARM boards
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Entrypoint script sources both ROS and workspace setups before exec'ing CMD
COPY ros2/scripts/ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

# Mow-session monitor — runs alongside the live stack to record a full
# state timeline per mowing test (see CLAUDE.md "Mowing Session Monitoring").
COPY ros2/scripts/mow_session_monitor.py /ros2_ws/scripts/mow_session_monitor.py
RUN chmod +x /ros2_ws/scripts/mow_session_monitor.py

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "mowgli_bringup", "mowgli.launch.py"]

# Webots has no Lyrical binary in the ROS main repository yet.
FROM base AS webots-driver-builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake python3-colcon-common-extensions python3-dev \
    python3-collada python3-lark libxml2-dev libzip-dev libssl-dev libcurl4-openssl-dev \
    ros-lyrical-vision-msgs ros-lyrical-python-cmake-module ros-lyrical-tinyxml2-vendor \
    ros-lyrical-controller-manager
WORKDIR /webots_ws
RUN mkdir src && chown -R ubuntu:ubuntu /webots_ws
USER ubuntu
RUN git init -q src/webots_ros2 \
 && git -C src/webots_ros2 fetch --depth 1 https://github.com/cyberbotics/webots_ros2.git db4a77c84ee91445d39de9a631e5b57a888814e2 \
 && git -C src/webots_ros2 checkout -q --detach FETCH_HEAD \
 && git -C src/webots_ros2 submodule update --init --recursive \
 && sed -i 's/CMAKE_CXX_STANDARD 17/CMAKE_CXX_STANDARD 20/' src/webots_ros2/webots_ros2_driver/CMakeLists.txt
RUN /bin/bash -c 'source /opt/ros/lyrical/setup.bash && MAKEFLAGS=-j3 colcon build \
    --merge-install --packages-up-to webots_ros2_control --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_CXX_STANDARD=20 -DCMAKE_POLICY_VERSION_MINIMUM=3.5'

# =============================================================================
# Stage 6: simulation
# Extends runtime with the Webots simulator for headless/web simulation.
#
# Webots R2025a is the minimum version required by the ROS 2 Lyrical driver.
# Pin the vendor package so a clean checkout builds the same simulator image.
# Cyberbotics publishes this Linux package for amd64 only; the regular runtime
# target remains architecture-neutral, while this simulator target fails early
# and clearly on unsupported Linux architectures.
# =============================================================================
FROM runtime AS simulation

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG WEBOTS_VERSION=R2025a

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Headless rendering (sensors need a virtual display when no GPU)
    xvfb \
    # VNC + lightweight desktop for sim GUI access from macOS
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    openbox \
    xterm \
    x11-utils \
    # Required by Qt 6's xcb platform plugin in the headless Linux image.
    libxcb-cursor0 \
    # Webots' bundled FFmpeg still links libsndio.so.7 on Ubuntu 26.04.
    libsndio7.0 \
    wget \
    ca-certificates \
    # Webots ROS 2 launcher, controller and driver
    python3-collada python3-lark python3-pil ros-lyrical-vision-msgs \
    ros-lyrical-controller-manager ros-lyrical-diff-drive-controller \
    ros-lyrical-joint-state-broadcaster \
 && test "${TARGETARCH}" = "amd64" \
 && wget -q "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots_2025a_amd64.deb" -O /tmp/webots.deb \
 && apt-get install -y --no-install-recommends /tmp/webots.deb \
 && rm -f /tmp/webots.deb \
 && rm -rf /var/lib/apt/lists/*

COPY --from=webots-driver-builder /webots_ws/install /opt/webots_ros2
ENV WEBOTS_HOME=/usr/local/webots
ENV USER=root

# Docker Desktop's Linux VM kernel identifies as microsoft-standard. The
# upstream driver therefore selects its native-WSL path and attempts to use a
# Windows-side Webots install. In this Linux container Webots is installed at
# WEBOTS_HOME, so only the driver's Docker execution needs to opt out of that
# detection. Native WSL remains unchanged.
RUN python3 - <<'PY'
from pathlib import Path

path = next(Path('/opt/webots_ros2').glob('**/site-packages/webots_ros2_driver/utils.py'))
old = "def is_wsl():\n    return 'microsoft-standard' in uname().release\n"
new = (
    "def is_wsl():\n"
    "    return False if os.path.exists('/.dockerenv') else "
    "'microsoft-standard' in uname().release\n"
)
text = path.read_text()
if old not in text:
    raise SystemExit(f'Unexpected webots_ros2_driver is_wsl() implementation: {path}')
path.write_text(text.replace(old, new))
PY

# Windows checkouts can retain CRLF in Python script shebangs.  These
# executable simulation helpers are copied into the colcon install tree before
# this stage; Linux would otherwise interpret their interpreter as `python3\r`.
RUN find /ros2_ws/install/mowgli_simulation/lib/mowgli_simulation \
      -type f -name '*.py' -exec sed -i 's/\r$//' {} +

# Copy VNC startup script
COPY ros2/scripts/start_vnc.sh /ros2_ws/scripts/start_vnc.sh
RUN chmod +x /ros2_ws/scripts/start_vnc.sh


# Copy Foxglove layout for easy import
COPY ros2/foxglove/ /ros2_ws/foxglove/

# Foxglove Bridge WebSocket port
EXPOSE 8765
# noVNC web port (sim GUI via browser)
EXPOSE 6080

CMD ["ros2", "launch", "mowgli_bringup", "sim_full_system.launch.py", "headless:=true", "use_rviz:=false"]
