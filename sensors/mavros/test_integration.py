#!/usr/bin/env python3
"""Source-level contract checks for the external MowgliMAVROS sidecar."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
IMAGE_ENV = ROOT / "sensors/mavros/image.env"
README = ROOT / "sensors/mavros/README.md"
CONFIG = ROOT / "install/lib/config.sh"
COMPOSE = ROOT / "install/compose/docker-compose.mavros.yml"
ENV = ROOT / "install/lib/env.sh"
LAUNCH = ROOT / "ros2/src/mowgli_bringup/launch/mowgli.launch.py"

EXPECTED_IMAGE = (
    "ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar:"
    "kilted@sha256:04e4eb17b0f5ce38f882f68346b1694774fa87e1945b38b57c94f90da34dd560"
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

    require(EXPECTED_IMAGE in image_env, "external image digest changed or is missing")
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
        "./docker/config/mowgli:/ros2_ws/config:ro",
        "./docker/config/cyclonedds.xml:/cyclonedds.xml:ro",
        "MAVROS_PORT: ${MAVROS_PORT:-/dev/mavros}",
        "MAVROS_BAUD: ${MAVROS_BAUD:-921600}",
    ):
        require(value in compose, f"MAVROS compose contract missing: {value}")

    require(
        '"$MAVROS_BY_ID" == /dev/serial/by-id/*' in env,
        "stable /dev/serial/by-id MAVROS paths are not promoted to MAVROS_PORT",
    )
    require('GNSS_BACKEND="disabled"' in env and 'GNSS_STACK="disabled"' in env,
            "MAVROS mode must exclude the direct GNSS sidecar")

    require('executable="hardware_bridge_node"' in launch, "native bridge declaration missing")
    require(
        'condition=IfCondition(EqualsSubstitution(hardware_backend, "mowgli"))' in launch,
        "native bridge must be excluded in MAVROS mode",
    )
    for executable in ('executable="robot_state_publisher"', 'executable="twist_mux"'):
        require(executable in launch, f"main-stack node missing: {executable}")

    for expected in (
        "mavros_hardware_bridge_node",
        "/mavros/state",
        "/mavros/global_position/global",
        "HARDWARE_PENDING",
        "command and blade paths disabled by default",
    ):
        require(expected in readme, f"MAVROS integration documentation missing: {expected}")

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
