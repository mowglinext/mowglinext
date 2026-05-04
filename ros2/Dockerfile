# =============================================================================
# Stage 0: gtsam-builder
# Builds GTSAM 4.3a1 from source. The Ubuntu Noble apt package
# (libgtsam-dev 4.2) ships a broken GTSAMConfig.cmake (references files
# that aren't installed) AND uses the legacy boost::optional<Matrix&>
# API for custom factors. There is no ros-kilted-gtsam yet (only Jazzy).
# Building from source gives us a clean install + the modern
# OptionalMatrixType API + a single pinned version we control.
#
# Install lands in /opt/gtsam; runtime stages COPY it across, set up
# ldconfig, and export CMAKE_PREFIX_PATH so find_package(GTSAM) works.
# =============================================================================
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

# =============================================================================
# Stage 1: base
# Installs all runtime ROS2 dependencies. Both runtime and simulation stages
# inherit from here, keeping the image lean.
# =============================================================================
FROM ros:kilted-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Navigation
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
    # Motion control
    ros-kilted-twist-mux \
    # Behavior trees
    ros-kilted-behaviortree-cpp \
    # Robot description / TF
    ros-kilted-xacro \
    ros-kilted-robot-state-publisher \
    ros-kilted-joint-state-publisher \
    ros-kilted-tf2-ros \
    ros-kilted-tf2-eigen \
    ros-kilted-tf2-geometry-msgs \
    # Grid map
    ros-kilted-grid-map-core \
    ros-kilted-grid-map-ros \
    ros-kilted-grid-map-msgs \
    # Diagnostics
    ros-kilted-diagnostic-msgs \
    ros-kilted-diagnostic-updater \
    # Foxglove Bridge (WebSocket bridge for GUI and Foxglove Studio)
    ros-kilted-foxglove-bridge \
    # u-blox and RTCM message definitions (for foxglove_bridge schema resolution)
    ros-kilted-ublox-msgs \
    ros-kilted-nmea-msgs \
    ros-kilted-rtcm-msgs \
    # tinyxml2 (required at runtime by BehaviorTree.CPP)
    libtinyxml2-dev \
    # Topic tools (relay, throttle — used in simulation for topic remapping)
    ros-kilted-topic-tools \
    # Docking (opennav_docking — dock/undock action server)
    # DockRobot/UndockRobot actions are in nav2_msgs (already installed).
    # The docking_server executable is in opennav_docking.
    ros-kilted-opennav-docking \
    # Rotation shim controller for transit navigation
    ros-kilted-nav2-rotation-shim-controller \
    # Nav2 core headers (FTC controller plugin depends on nav2_core)
    ros-kilted-nav2-core \
    ros-kilted-nav2-util \
    # Nav2 msgs (CostmapFilterInfo for costmap filters)
    ros-kilted-nav2-msgs \
    # Simulation bridge (needed at runtime for ros_gz_bridge topic bridging)
    ros-kilted-ros-gz-sim \
    ros-kilted-ros-gz-bridge \
    # DDS middleware — Cyclone DDS is more reliable than FastRTPS on ARM
    # (no shared memory, no stale /dev/shm segments, better service discovery)
    ros-kilted-rmw-cyclonedds-cpp \
    # GTSAM runtime deps — libgtsam itself is COPYed from the
    # gtsam-builder stage below. libmetis-dev provides both the
    # runtime libmetis5 and the unversioned libmetis.so symlink
    # that GTSAM's exported CMake target references at link time.
    # libboost-* runtime libs are pulled in transitively.
    libmetis-dev \
    # Utilities
    python3-argcomplete \
 && rm -rf /var/lib/apt/lists/*

# Pull source-built GTSAM 4.3a1 into /opt/gtsam, register its lib
# directory with ldconfig, and expose the CMake config dir so
# find_package(GTSAM) resolves without extra hints. GTSAM's runtime
# .so links against libboost-* and libmetis5, both already present.
COPY --from=gtsam-builder /opt/gtsam /opt/gtsam
RUN echo '/opt/gtsam/lib' > /etc/ld.so.conf.d/gtsam.conf && ldconfig
ENV CMAKE_PREFIX_PATH=/opt/gtsam:${CMAKE_PREFIX_PATH}

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
    ros-kilted-ament-cmake \
    ros-kilted-rosidl-default-generators \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy only package.xml + CMakeLists.txt to resolve rosdep deps (cache layer).
# This ensures the expensive rosdep install is only re-run when package
# definitions change, NOT when source code changes.
COPY src/mowgli_interfaces/package.xml    src/mowgli_interfaces/package.xml
COPY src/mowgli_interfaces/CMakeLists.txt src/mowgli_interfaces/CMakeLists.txt
COPY src/mowgli_hardware/package.xml      src/mowgli_hardware/package.xml
COPY src/mowgli_hardware/CMakeLists.txt   src/mowgli_hardware/CMakeLists.txt
COPY src/mowgli_localization/package.xml  src/mowgli_localization/package.xml
COPY src/mowgli_localization/CMakeLists.txt src/mowgli_localization/CMakeLists.txt
COPY src/mowgli_map/package.xml           src/mowgli_map/package.xml
COPY src/mowgli_map/CMakeLists.txt        src/mowgli_map/CMakeLists.txt
COPY src/mowgli_nav2_plugins/package.xml  src/mowgli_nav2_plugins/package.xml
COPY src/mowgli_nav2_plugins/CMakeLists.txt src/mowgli_nav2_plugins/CMakeLists.txt
COPY src/mowgli_behavior/package.xml      src/mowgli_behavior/package.xml
COPY src/mowgli_behavior/CMakeLists.txt   src/mowgli_behavior/CMakeLists.txt
COPY src/mowgli_monitoring/package.xml    src/mowgli_monitoring/package.xml
COPY src/mowgli_monitoring/CMakeLists.txt src/mowgli_monitoring/CMakeLists.txt
COPY src/mowgli_simulation/package.xml    src/mowgli_simulation/package.xml
COPY src/mowgli_simulation/CMakeLists.txt src/mowgli_simulation/CMakeLists.txt
COPY src/mowgli_bringup/package.xml       src/mowgli_bringup/package.xml
COPY src/mowgli_bringup/CMakeLists.txt    src/mowgli_bringup/CMakeLists.txt
COPY src/fusion_graph/package.xml         src/fusion_graph/package.xml
COPY src/fusion_graph/CMakeLists.txt      src/fusion_graph/CMakeLists.txt
# fusion_graph is the opt-in factor-graph replacement for ekf_map_node;
# depends on libgtsam-dev installed in the base stage.

# Resolve rosdep dependencies (cached until package.xml changes)
RUN rosdep update --rosdistro kilted \
 && rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro kilted \
      -y \
 || true

# =============================================================================
# Stage 3: build-interfaces
# Build only mowgli_interfaces (msg/srv/action definitions). These rarely
# change and are a dependency for all other packages.
# =============================================================================
FROM deps AS build-interfaces

# Copy the full mowgli_interfaces package
COPY src/mowgli_interfaces/ src/mowgli_interfaces/

RUN /bin/bash -c "\
    source /opt/ros/kilted/setup.bash && \
    colcon build \
      --packages-select mowgli_interfaces \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers \$(nproc) \
    "

# =============================================================================
# Stage 4: build
# Compiles all remaining workspace packages from source.
# =============================================================================
FROM build-interfaces AS build

ARG BUILD_TYPE=Release

# Now copy the rest of the source tree
COPY src/ src/

# Build all packages (mowgli_interfaces is already built and cached)
RUN /bin/bash -c "\
    source /opt/ros/kilted/setup.bash && \
    source install/setup.bash && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DBUILD_TESTING=OFF \
      --parallel-workers \$(nproc) \
      --event-handlers console_cohesion+ \
    "

# Run unit tests (non-blocking)
RUN /bin/bash -c "\
    source /opt/ros/kilted/setup.bash && \
    source install/setup.bash && \
    colcon test \
      --ctest-args -L 'gtest' --output-on-failure \
    ; colcon test-result --verbose \
    " || true

# =============================================================================
# Stage 5: runtime
# Minimal image containing only what is needed to run the robot.
# =============================================================================
FROM base AS runtime

WORKDIR /ros2_ws

# Ensure libtinyxml2 .so.9 and .so.10 both resolve. The build stage may
# compile against a different minor version than the runtime base provides
# when Docker layer caching spans a base image update.
RUN reallib=$(find /usr/lib -name 'libtinyxml2.so.*.*.*' | head -1) && \
    dir=$(dirname "$reallib") && \
    [ ! -e "$dir/libtinyxml2.so.9" ] && ln -sf "$(basename "$reallib")" "$dir/libtinyxml2.so.9" || true && \
    [ ! -e "$dir/libtinyxml2.so.10" ] && ln -sf "$(basename "$reallib")" "$dir/libtinyxml2.so.10" || true && \
    ldconfig

# Pull compiled install tree from the build stage
COPY --from=build /ros2_ws/install/ install/

# Copy all launch files and config from every package
COPY src/mowgli_bringup/launch/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/launch/
COPY src/mowgli_bringup/config/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/
# mowgli_localization has no launch files; nodes are brought up from
# mowgli_bringup/navigation.launch.py. Keep the config copy so the YAML
# files still land in the install tree.
COPY src/mowgli_localization/config/  /ros2_ws/install/mowgli_localization/share/mowgli_localization/config/

# fusion_graph ships its own launch + config used by navigation.launch.py
# when use_fusion_graph:=true.
COPY src/fusion_graph/launch/  /ros2_ws/install/fusion_graph/share/fusion_graph/launch/
COPY src/fusion_graph/config/  /ros2_ws/install/fusion_graph/share/fusion_graph/config/

# Use Cyclone DDS — FastRTPS shared memory causes stale /dev/shm segments
# and unreliable service discovery on ARM boards
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Entrypoint script sources both ROS and workspace setups before exec'ing CMD
COPY scripts/ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

# Mow-session monitor — runs alongside the live stack to record a full
# state timeline per mowing test (see CLAUDE.md "Mowing Session Monitoring").
COPY scripts/mow_session_monitor.py /ros2_ws/scripts/mow_session_monitor.py
RUN chmod +x /ros2_ws/scripts/mow_session_monitor.py

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "mowgli_bringup", "mowgli.launch.py"]

# =============================================================================
# Stage 6: simulation
# Extends runtime with a full Gazebo installation for headless/web simulation.
# =============================================================================
FROM runtime AS simulation

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic comes via ros-kilted-ros-gz-sim (Jazzy default pairing)
    ros-kilted-ros-gz-sim \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-interfaces \
    # Headless rendering (sensors need a virtual display when no GPU)
    xvfb \
    # VNC + lightweight desktop for Gazebo GUI access from macOS
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    openbox \
    xterm \
    x11-utils \
 && rm -rf /var/lib/apt/lists/*

# Copy VNC startup script
COPY scripts/start_vnc.sh /ros2_ws/scripts/start_vnc.sh
RUN chmod +x /ros2_ws/scripts/start_vnc.sh


# Copy Foxglove layout for easy import
COPY foxglove/ /ros2_ws/foxglove/

# Foxglove Bridge WebSocket port
EXPOSE 8765
# noVNC web port (Gazebo GUI via browser)
EXPOSE 6080

CMD ["ros2", "launch", "mowgli_bringup", "sim_full_system.launch.py", "headless:=true", "use_rviz:=false"]
