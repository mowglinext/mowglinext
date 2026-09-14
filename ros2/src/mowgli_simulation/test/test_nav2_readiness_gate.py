# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

from pathlib import Path
import subprocess


def _gate_script() -> Path:
    return Path(__file__).resolve().parents[3] / 'scripts' / 'wait_for_nav2_active.sh'


def test_gate_accepts_lifecycle_ready_message(tmp_path: Path) -> None:
    launch_log = tmp_path / 'sim.log'
    launch_log.write_text('Managed nodes are active\n')
    process = subprocess.Popen(['sleep', '10'])
    try:
        result = subprocess.run(
            [str(_gate_script()), str(process.pid), str(launch_log), '2'],
            capture_output=True,
            text=True,
            timeout=5,
        )
    finally:
        process.terminate()
        process.wait(timeout=5)

    assert result.returncode == 0
    assert 'Managed nodes are active' in result.stdout


def test_gate_fails_when_launch_exits_before_activation(tmp_path: Path) -> None:
    launch_log = tmp_path / 'sim.log'
    launch_log.write_text('lifecycle startup failed\n')
    process = subprocess.Popen(['true'])
    process.wait(timeout=5)

    result = subprocess.run(
        [str(_gate_script()), str(process.pid), str(launch_log), '2'],
        capture_output=True,
        text=True,
        timeout=5,
    )

    assert result.returncode == 1
    assert 'exited before Nav2 became active' in result.stderr
    assert 'lifecycle startup failed' in result.stderr
