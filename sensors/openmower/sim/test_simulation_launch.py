# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The simulation must observe launch output before its child is killed."""

import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from run_simulation import start_bridge


class LaunchOutputTest(unittest.TestCase):
    def test_calibration_log_is_visible_while_child_is_still_running(self):
        # Exercise the real C logger used by the bridge, with stdout redirected.
        # The pipe handshake orders the assertion AFTER logging but BEFORE exit;
        # no timing sleep or successful child teardown can flush it for us.
        ready_r, ready_w = os.pipe()
        release_r, release_w = os.pipe()
        real_popen = subprocess.Popen
        process = None
        try:
            with tempfile.TemporaryDirectory() as workdir:
                logfile = Path(workdir) / 'bridge.log'
                rig = SimpleNamespace(paths={n: '/fake/' + n for n in ('ll', 'left', 'right', 'mow')})

                def launch_child(command, **kwargs):
                    self.assertEqual(command[:3], ['ros2', 'launch', 'mowgli_openmower_bridge'])
                    code = ("import os, rclpy, rclpy.logging; rclpy.init(); "
                            "rclpy.logging.get_logger('om_sim_test').info("
                            "'IMU bias ready: actual observation'); "
                            f"os.write({ready_w}, b'1'); os.read({release_r}, 1)")
                    # A C++ node does not interpret Python's buffering flag.
                    kwargs['env'].pop('PYTHONUNBUFFERED', None)
                    try:
                        return real_popen([sys.executable, '-c', code],
                                          pass_fds=(ready_w, release_r), **kwargs)
                    finally:
                        kwargs['stdout'].close()

                with patch.dict(os.environ, {'RCUTILS_LOGGING_USE_STDOUT': '1',
                                             'RCUTILS_LOGGING_BUFFERED_STREAM': '',
                                             'ROS_LOG_DIR': workdir}), \
                        patch('run_simulation.subprocess.Popen', side_effect=launch_child):
                    process = start_bridge(rig, 'xesc_mini', workdir, str(logfile))
                self.assertTrue(select.select([ready_r], [], [], 5)[0],
                                'child did not reach logger: ' + logfile.read_text())
                self.assertEqual(os.read(ready_r, 1), b'1')
                self.assertIsNone(process.poll())
                self.assertIn('IMU bias ready: actual observation', logfile.read_text())
        finally:
            os.write(release_w, b'1')
            if process is not None:
                process.wait(timeout=5)
            for fd in (ready_r, ready_w, release_r, release_w):
                os.close(fd)


if __name__ == '__main__':
    unittest.main()
