# syntax=docker/dockerfile:1.7
# =============================================================================
# u-blox ZED-F9P driver (cedbossneo/ublox_dgnss feat/serial-transport)
# + NTRIP client
#
# Built from source — ublox_dgnss is not in the kilted apt index.
# Default transport is "serial" (kernel CDC ACM at /dev/ttyACM1); the
# upstream libusb path introduced ~3.5 s pipeline lag and was replaced
# 2026-04-26 (see sensors/gps/ublox_dgnss.yaml). Bundles its own NTRIP
# client so RTCM gets injected over the same channel correctly.
#
# Publishes:
#   /fix                    sensor_msgs/NavSatFix (high-precision)
#   /ubx_nav_hpposllh       ublox_ubx_msgs — 0.1mm lat/lon/alt
#   /ubx_nav_status         — fix type, flags, diffSoln, carrSoln
#   /ubx_rxm_rtcm           — RTCM correction ingestion counter
#
# Config read from /config/mowgli_robot.yaml (bind-mounted).
# =============================================================================

# ─── Builder ────────────────────────────────────────────────────────────────
FROM ros:kilted-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive
ARG UBLOX_DGNSS_REF=fix/hp-fix-stale-status-cov
# Bump UBLOX_DGNSS_SHA to the current branch tip to bust the docker
# layer cache when upstream moves. `git clone --branch` accepts only
# branch/tag names so we clone shallow then checkout the pinned SHA;
# the SHA also ensures cache-from doesn't reuse a stale layer when
# the branch moves under us (which is what we hit on 2026-04-26).
# Currently riding fix/hp-fix-stale-status-cov (PR cedbossneo/ublox_dgnss#2),
# which is fix/serial-reconnect-on-eio (#1, CDC ACM FD reopen on EIO) +
# a per-epoch h_acc fallback in ublox_nav_sat_fix_hp_node so /gps/fix no
# longer publishes positions with status/covariance from a different
# epoch. Measured loss on the prior branch: NAV-STATUS ~57 %, NAV-COV
# ~80 % at 7 Hz nav rate, which silently published Float positions with
# stale Fixed status during cycle slips. Switch back to the parent
# branches once #1 and #2 land.
ARG UBLOX_DGNSS_SHA=5e1d0cf1e8ec0e2638c44a59f57d15f5dd04c9af
# cedbossneo fork (itself a fork of privatedocs000/ublox_dgnss). Layered
# customisations over upstream:
#   * privatedocs000 base — RTK-Fixed detection via UBX-NAV-STATUS.carr_soln:
#     status=2 (STATUS_GBAS_FIX) for RTK Fixed, status=1 (STATUS_SBAS_FIX)
#     for RTK Float / RTCM-corrected, status=0 (STATUS_FIX) for plain GPS.
#     Downstream consumers use this to classify fixes correctly instead of
#     conflating every RTCM-corrected fix into DGPS_FIX.
#   * ntrip_client_node exit-check cadence exposed as a ROS parameter
#     (stream_exit_check_records, default 10000). Upstream's hardcoded 10
#     forced a full TCP+HTTP reconnect every ~0.6 s at 17 Hz RTCM, which
#     prevented the F9P from recovering RTK Fixed after sky obstructions.
ARG UBLOX_DGNSS_URL=https://github.com/cedbossneo/ublox_dgnss.git

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libusb-1.0-0-dev \
      libcurl4-openssl-dev \
      python3-colcon-common-extensions \
      ros-kilted-rclcpp \
      ros-kilted-rclcpp-components \
      ros-kilted-rclcpp-lifecycle \
      ros-kilted-nav-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
      ros-kilted-rtcm-msgs \
      ros-kilted-geometry-msgs \
      ros-kilted-tf2-ros

WORKDIR /ws/src
RUN git clone --branch ${UBLOX_DGNSS_REF} ${UBLOX_DGNSS_URL} ublox_dgnss \
 && cd ublox_dgnss && git checkout ${UBLOX_DGNSS_SHA} && cd ..

WORKDIR /ws
RUN . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
                   -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

# ─── Runtime ────────────────────────────────────────────────────────────────
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libusb-1.0-0 \
      libcurl4 \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-rtcm-msgs \
      ros-kilted-nmea-navsat-driver

COPY --from=builder /ws/install /opt/ublox_dgnss

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps.sh /start_gps.sh
COPY ublox_dgnss.yaml /ublox_dgnss.yaml
COPY gps_health_aggregator.py /gps_health_aggregator.py
COPY rtcm_serial_bridge.py /rtcm_serial_bridge.py
RUN chmod +x /ros2_entrypoint.sh /start_gps.sh /gps_health_aggregator.py /rtcm_serial_bridge.py

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
