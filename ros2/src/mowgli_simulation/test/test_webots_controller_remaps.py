import ast
from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1] / 'launch' / 'webots_minimal.launch.py'
)


def test_diffdrive_topics_are_remapped_on_spawned_controller():
    source = LAUNCH_FILE.read_text()
    string_literals = {
        node.value
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }

    assert (
        '--controller-ros-args=-r '
        '/diffdrive_controller/cmd_vel:=/cmd_vel_wheels'
    ) in string_literals
    assert (
        '--controller-ros-args=-r '
        '/diffdrive_controller/odom:=/wheel_odom_raw'
    ) in string_literals


def test_diffdrive_topics_are_not_remapped_on_webots_driver():
    source = LAUNCH_FILE.read_text()
    driver_block = source.split(
        'mowgli_driver = WebotsController(', 1
    )[1].split('controller_manager_timeout', 1)[0]

    assert "'/diffdrive_controller/cmd_vel'" not in driver_block
    assert "'/diffdrive_controller/odom'" not in driver_block


def test_controller_spawners_start_without_webots_connection_event():
    source = LAUNCH_FILE.read_text()

    assert 'wait_for_controller_connection' not in source
    assert 'diffdrive_spawner,' in source
    assert 'joint_state_spawner,' in source
    assert "'--param-file'," in source
