# syntax=docker/dockerfile:1.7

# Pinned to the cedbossneo fork while the upstream PR for
# (b) Ntrip-GGA forwarding, (c) PVTSLNA position-type → fix_quality,
# (d) per-constellation GSV sat counts, plus the GUI-compatible
# 'GPS: fix/satellites/NTRIP/RTCM' diagnostics restructure is open.
# Once merged into mowglifrenchtouch/UM982Driver@mowgli, flip back to
# the upstream URL.
ARG UM982_REPO=https://github.com/cedbossneo/unicore-ros2
ARG UM982_REF=main
# Bump UM982_SHA to bust the docker layer cache when the fork moves.
# `git clone --branch` is cacheable per ARG value, so without a SHA
# pin we rebuild against stale code.
ARG UM982_SHA=8b9e25e6027039a52f6fc4d6932678b422084ffa

FROM ros:kilted-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive
ARG UM982_REPO
ARG UM982_REF
ARG UM982_SHA

ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:${PATH}"

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      git \
      ccache \
      build-essential \
      cmake \
      libcurl4-openssl-dev \
      python3-colcon-common-extensions \
      ros-kilted-rclcpp \
      ros-kilted-rclcpp-components \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rosidl-default-generators \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src

# Clone the branch then checkout the pinned SHA. The SHA is part of the
# RUN command so the layer cache invalidates whenever UM982_SHA bumps,
# even though `git clone --branch` would otherwise be cacheable.
RUN git clone --branch "${UM982_REF}" "${UM982_REPO}" um982_driver \
 && cd um982_driver && git checkout "${UM982_SHA}" && cd ..

WORKDIR /ws

RUN --mount=type=cache,target=/root/.ccache,sharing=locked \
    . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --base-paths src/um982_driver/compass_msgs \
                   src/um982_driver/ntrip_client_node \
                   src/um982_driver \
      --packages-up-to ntrip_client_node unicore_gnss \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libcurl4 \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rclcpp-components \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY --from=builder /ws/install /ws/install
COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps.sh /start_gps.sh
COPY configure_receiver.sh /configure_receiver.sh

RUN chmod +x /ros2_entrypoint.sh /start_gps.sh /configure_receiver.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
