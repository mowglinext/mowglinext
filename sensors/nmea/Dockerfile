# syntax=docker/dockerfile:1.7
# =============================================================================
# Generic NMEA-0183 GPS driver
#
# Truly generic NMEA driver for any receiver that emits standard NMEA
# sentences (GGA, RMC, GSA, VTG) over UART or USB-serial. Examples:
# Emlid Reach, generic u-blox in NMEA mode, BN-220, LC29H, NEO-M8N
# in NMEA mode, etc.
#
# Uses the upstream ros-drivers `nmea_navsat_driver` package from the
# Kilted apt index — no source build required, no UBX dependency.
#
# Publishes (remapped to the project's /gps/ namespace by the launch):
#   /gps/fix    sensor_msgs/NavSatFix      (lat/lon/alt + HDOP-derived cov)
#   /gps/vel    geometry_msgs/TwistStamped (from RMC/VTG ground speed+track)
#   no /gps/azimuth output — this generic driver does not publish
#   compass_msgs/Azimuth-compatible heading data
#
# Caveats vs the ublox backend:
#   * Covariance comes from HDOP only (no UBX position_accuracy), so the
#     map-frame EKF / fusion_graph see a more pessimistic σ than RTK-Fixed
#     would imply. RTK still works if the receiver outputs RTK-quality
#     GGA fix-quality codes (4=Fix, 5=Float).
#   * No carrier-solution flag or rich diagnostics stream: the shared
#     /gps/status runtime builder can publish typed basic GNSS state, but
#     only from the coarse fix semantics and covariance that
#     nmea_navsat_driver exposes.
#   * NTRIP injection is NOT bundled here. If your receiver supports NTRIP
#     internally (Emlid, most modern modules), configure it on the device.
#     If you need ROS-side RTCM relay over the same serial port, run a
#     separate ntrip_client_node and write RTCM bytes back to the device
#     — non-trivial and out of scope for this fragment.
# =============================================================================
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-kilted-nmea-navsat-driver \
      ros-kilted-rmw-cyclonedds-cpp

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_nmea.sh /start_nmea.sh
RUN chmod +x /ros2_entrypoint.sh /start_nmea.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_nmea.sh"]
