"""Execute the installer writer against sparse robot settings, without ROS."""
from pathlib import Path
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("opt_in", [False, True])
@pytest.mark.parametrize("ticks_per_meter", [512.125, 300])
def test_sparse_template_merge_and_traction_opt_in(tmp_path, opt_in, ticks_per_meter):
    source = tmp_path / "config/mowgli/mowgli_robot.yaml"
    source.parent.mkdir(parents=True)
    params = {"ticks_per_meter": ticks_per_meter, "ntrip_enabled": True}
    if opt_in:
        params.update(mavros_manual_control_enabled=True, mavros_wheel_lift_safety_enabled=False)
    payload = yaml.safe_dump({"mowgli": {"ros__parameters": params}})
    source.write_text(payload)
    subprocess.run(
        ["bash", "-c", 'source "$1/install/lib/config.sh"; REPO_DIR="$1"; DOCKER_DIR="$2"; HARDWARE_BACKEND=mavros; write_mavros_runtime_config',
         "test", str(ROOT), str(tmp_path)], check=True,
    )
    template = yaml.safe_load((ROOT / "ros2/src/mowgli_bringup/config/mowgli_robot.yaml").read_text())["mowgli"]["ros__parameters"]
    odometry = yaml.safe_load((tmp_path / "config/mavros/esc_wheel_odometry.yaml").read_text())["/**/esc_wheel_odometry"]["ros__parameters"]
    assert odometry == {"ticks_per_meter": float(ticks_per_meter), "track_width_m": template["wheel_track"]}
    assert isinstance(odometry["ticks_per_meter"], float)
    assert isinstance(odometry["track_width_m"], float)
    bridge = yaml.safe_load((tmp_path / "config/mavros/hardware_bridge.yaml").read_text())["hardware_bridge"]["ros__parameters"]
    assert bridge == {"manual_control_enabled": opt_in, "wheel_lift_safety_enabled": not opt_in, "blade_control_enabled": False}
    assert source.read_text() == payload
