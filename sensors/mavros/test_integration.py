#!/usr/bin/env python3
"""Source-level contract checks for the external MowgliMAVROS sidecar."""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IMAGE_ENV = ROOT / "sensors/mavros/image.env"
README = ROOT / "sensors/mavros/README.md"
CONFIG = ROOT / "install/lib/config.sh"
COMPOSE = ROOT / "install/compose/docker-compose.mavros.yml"
ENV = ROOT / "install/lib/env.sh"
LAUNCH = ROOT / "ros2/src/mowgli_bringup/launch/mowgli.launch.py"
SETTINGS_BACKEND = ROOT / "gui/pkg/api/settings_backend.go"
PARAMS_API = ROOT / "gui/pkg/api/params.go"
FRONTEND_BACKENDS = ROOT / "gui/web/src/constants/hardwareBackends.ts"

EXPECTED_IMAGE = (
    "ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar:latest"
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    image_env = read(IMAGE_ENV)
    readme = read(README)
    config = read(CONFIG)
    compose = read(COMPOSE)
    env = read(ENV)
    launch = read(LAUNCH)
    settings_backend = read(SETTINGS_BACKEND)
    params_api = read(PARAMS_API)
    frontend_backends = read(FRONTEND_BACKENDS)

    require(EXPECTED_IMAGE in image_env, "external development image reference changed or is missing")
    require(
        'source "$_mavros_image_env"' in config,
        "installer no longer sources the MAVROS image contract",
    )
    require(
        "MAVROS_IMAGE_DEFAULT=\"${MOWGLI_MAVROS_IMAGE_DEFAULT}\"" in config,
        "installer no longer consumes the externally pinned image",
    )
    require("MOWGLI_ROS2_IMAGE_DEFAULT" not in image_env, "image pin must not follow IMAGE_TAG")

    for value in (
        "image: ${MAVROS_IMAGE}",
        "- /dev:/dev",
        "./docker/config/mavros:/ros2_ws/config:ro",
        "./docker/config/cyclonedds.xml:/cyclonedds.xml:ro",
        "MAVROS_PORT: ${MAVROS_PORT:-/dev/mavros}",
        "MAVROS_BAUD: ${MAVROS_BAUD:-921600}",
    ):
        require(value in compose, f"MAVROS compose contract missing: {value}")

    require(
        "./docker/config/mowgli:/ros2_ws/config:ro" not in compose,
        "MAVROS must not mount the operator-facing Mowgli config",
    )
    for value in ("UNIVERSAL_GNSS_IMAGE", "GNSS_NTRIP", "/dev/gnss-receiver"):
        require(
            value not in compose,
            f"MAVROS compose must not claim Universal GNSS ownership: {value}",
        )

    mavros_config_writer = re.search(
        r"write_mavros_runtime_config\(\) \{(?P<body>.*?)^\}",
        config,
        flags=re.DOTALL | re.MULTILINE,
    )
    require(mavros_config_writer is not None, "MAVROS derived-config writer missing")
    mavros_config_body = mavros_config_writer.group("body")
    require(
        'source="$DOCKER_DIR/config/mowgli/mowgli_robot.yaml"' in mavros_config_body
        and 'target="$DOCKER_DIR/config/mavros/mowgli_robot.yaml"' in mavros_config_body
        and 'cp "$source" "$target"' in mavros_config_body,
        "MAVROS config must be derived from the Mowgli config",
    )
    require(
        '_yaml_patch_key "$target" ntrip_enabled false' in mavros_config_body,
        "MAVROS derived config must disable only sidecar NTRIP ownership",
    )
    for expected in (
        'odometry_target="$DOCKER_DIR/config/mavros/esc_wheel_odometry.yaml"',
        'robot_template="$REPO_DIR/ros2/src/mowgli_bringup/config/mowgli_robot.yaml"',
        '"ticks_per_meter": effective.get("ticks_per_meter")',
        '"track_width_m": effective.get("wheel_track")',
        'payload = {"/**/esc_wheel_odometry": {"ros__parameters": values}}',
    ):
        require(expected in config, f"MAVROS odometry config contract missing: {expected}")
    require(
        'write_mavros_runtime_config' in config and 'esc_wheel_odometry.yaml' in config,
        "MAVROS odometry runtime config is not generated",
    )

    require(
        '"$MAVROS_BY_ID" == /dev/serial/by-id/*' in env,
        "stable /dev/serial/by-id MAVROS paths are not promoted to MAVROS_PORT",
    )
    require(
        'GNSS_BACKEND="disabled"' not in env and 'GNSS_STACK="disabled"' not in env,
        "MAVROS mode must keep Universal GNSS independent",
    )

    require('executable="hardware_bridge_node"' in launch, "native bridge declaration missing")
    # The native STM32 bridge must run only for the Mowgli backend.
    # The launch file uses a named constant so the same condition also
    # excludes the OpenMower sidecar.
    require(
        'NATIVE_BRIDGE_BACKEND = "mowgli"' in launch
        and 'condition=IfCondition(EqualsSubstitution(hardware_backend, NATIVE_BRIDGE_BACKEND))' in launch,
        "native bridge must be excluded in MAVROS and OpenMower modes",
    )
    for executable in ('executable="robot_state_publisher"', 'executable="twist_mux"'):
        require(executable in launch, f"main-stack node missing: {executable}")

    for expected in (
        "mavros_hardware_bridge_node",
        "/mavros/state",
        "/mavros/global_position/global",
        "HARDWARE_PENDING",
        "command and blade paths disabled by default",
        "Pepeuch/mowglimavros@b92925b",
        "mavros/esc_wheel_odometry.ticks_per_meter",
        "mavros/esc_wheel_odometry.track_width_m",
        "wheel-odometry calibration",
    ):
        require(expected in readme, f"MAVROS integration documentation missing: {expected}")

    for key, parameter in (
        ("ticks_per_meter", "mavros/esc_wheel_odometry.ticks_per_meter"),
        ("wheel_track", "mavros/esc_wheel_odometry.track_width_m"),
    ):
        expected = rf'"{key}":\s*\{{Parameter: "{re.escape(parameter)}", Runtime: "available"\}}'
        require(re.search(expected, settings_backend), f"GUI backend MAVROS route missing: {key}")
    require(
        'route.Runtime == "pending_image"' not in params_api,
        "generic parameter API must not retain pending MAVROS route rejection",
    )
    require(
        'route.runtime === "available"' in frontend_backends,
        "frontend must not apply pending MAVROS parameter routes",
    )

    # Document the canonical GNSS ownership contract without binding the
    # integration test to one exact prose sentence in the README.
    for expected in (
        "Universal GNSS",
        "NTRIP",
        "GNSS_SOURCE=mavros",
        "GNSS_MAVROS_SOURCE=gps1|gps2",
        "MAVROS_GPS1_CANONICAL",
    ):
        require(
            expected in readme,
            f"MAVROS integration documentation missing: {expected}",
        )

    require(
        not re.search(r"MAVROS_(?:ENABLE|COMMAND|BLADE)[A-Z_]*:\s*(?:true|1)", compose),
        "MowgliNext must not enable external command or blade paths",
    )
    for path in (IMAGE_ENV, README, COMPOSE):
        require(
            not re.search(r"usb-[A-Za-z0-9]", read(path)),
            f"machine-specific Pixhawk identity committed in {path.relative_to(ROOT)}",
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as error:
        print(f"MAVROS integration contract failed: {error}", file=sys.stderr)
        raise SystemExit(1)
