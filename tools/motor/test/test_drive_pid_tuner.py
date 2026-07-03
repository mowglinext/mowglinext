import yaml

from mowgli_tools.robot_hardware_config import extract_robot_hardware_config


def test_extract_robot_hardware_config_reads_nested_mowgli_robot_yaml_mass() -> None:
    payload = yaml.safe_load(
        """
        mowgli:
          ros__parameters:
            chassis_mass_kg: 8.76
            wheel_radius: 0.04475
            ticks_per_revolution: 84
        """
    )

    config = extract_robot_hardware_config(payload, "/ros2_ws/config/mowgli_robot.yaml")

    assert config.source_path == "/ros2_ws/config/mowgli_robot.yaml"
    assert config.chassis_mass_kg == 8.76
    assert config.wheel_radius_m == 0.04475
    assert config.ticks_per_revolution == 84


def test_extract_robot_hardware_config_accepts_flat_mass_aliases() -> None:
    payload = {"mass": 8.76, "wheel_radius": 0.04475}

    config = extract_robot_hardware_config(payload, "/tmp/mowgli_robot.yaml")

    assert config.chassis_mass_kg == 8.76
    assert config.wheel_radius_m == 0.04475
