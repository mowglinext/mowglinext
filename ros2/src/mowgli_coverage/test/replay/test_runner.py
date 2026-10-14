#!/usr/bin/env python3
"""Tests for the comparator: detect geometry/order changes and ignore wall clocks."""

import copy
import os
import pathlib
import subprocess
import sys
import time
import unittest
from unittest import mock

from run_replay import (
    check_controls,
    digest,
    first_difference,
    input_identity,
    run_command,
    stable,
)


def child_has_exited(pid, start_time, timeout):
    """Wait for this child, without confusing a reused PID with the original."""
    deadline = time.monotonic() + timeout
    while True:
        try:
            # comm can contain spaces or ')'; the remaining fields start at state.
            fields = pathlib.Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        except (FileNotFoundError, ProcessLookupError):
            return True
        if fields[19] != start_time or fields[0] == 'Z':
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.01)


class ComparatorTests(unittest.TestCase):

    def record(self):
        return {
            'transported_polygons': [[[['0000', '0001']]]],
            'normalized_polygons': [[[['0000', '0001']]]],
            'parameters': {'width': 0.16},
            'stages': [{'name': 'execution', 'geometry': [[['0000', '0001']]]}],
            'fields': [],
            'metrics': {'poses': 1},
            'followstrip_fingerprint': 'abc',
            'timing': {'planning_ms': 1},
            'build': {'revision': 'baseline'},
        }

    def test_exact_geometry_change_is_detected(self):
        before = self.record()
        after = copy.deepcopy(before)
        after['stages'][0]['geometry'][0][0][0] = '0002'
        self.assertNotEqual(digest(stable(before)), digest(stable(after)))
        self.assertEqual(
            first_difference(stable(before), stable(after)), '/stages/0/geometry/0/0/0'
        )

    def test_subpath_order_change_is_detected(self):
        before = self.record()
        before['stages'][0]['geometry'].append([['0002', '0003']])
        after = copy.deepcopy(before)
        after['stages'][0]['geometry'].reverse()
        self.assertNotEqual(digest(stable(before)), digest(stable(after)))

    def test_clocks_do_not_change_geometry_identity(self):
        before = self.record()
        after = copy.deepcopy(before)
        after['timing']['planning_ms'] = 200
        self.assertEqual(stable(before), stable(after))

    def test_effective_parameter_and_version_change_input_identity(self):
        before = self.record()
        for key, replacement in (
            ('parameters', {'width': 0.18}),
            ('build', {'revision': 'another'}),
            ('build', {'revision': 'baseline', 'requested_cxx_flags_sha256': 'other'}),
        ):
            after = copy.deepcopy(before)
            after[key] = replacement
            self.assertNotEqual(input_identity(before), input_identity(after))

    def test_identity_controls_reject_same_geometry_fingerprint(self):
        a = {'input_identity': 'a', 'followstrip_fingerprint': 'same'}
        b = {'input_identity': 'b', 'followstrip_fingerprint': 'same'}
        self.assertEqual(len(check_controls({'square_fixed': a, 'square_changed_angle': b})), 1)

    def test_command_timeout_is_bounded(self):
        started = time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired):
            run_command([sys.executable, '-c', 'import time; time.sleep(60)'], 0.1)
        self.assertLess(time.monotonic() - started, 5)

    def test_child_exit_waits_for_signal_teardown(self):
        running = '123 (child with ) in name) R ' + '0 ' * 18 + '456'
        zombie = running.replace(') R ', ') Z ')
        with mock.patch.object(pathlib.Path, 'read_text', side_effect=[running, zombie]), \
                mock.patch.object(time, 'sleep'), \
                mock.patch.object(time, 'monotonic', return_value=0):
            self.assertTrue(child_has_exited(123, '456', 1))

    def test_child_exit_accepts_reaped_process(self):
        for error in (FileNotFoundError, ProcessLookupError):
            with self.subTest(error=error), \
                    mock.patch.object(pathlib.Path, 'read_text', side_effect=error):
                self.assertTrue(child_has_exited(123, '456', 1))

    def test_child_exit_does_not_track_reused_pid(self):
        reused = '123 (another child) R ' + '0 ' * 18 + '789'
        with mock.patch.object(pathlib.Path, 'read_text', return_value=reused):
            self.assertTrue(child_has_exited(123, '456', 1))

    def test_child_exit_rejects_live_child_at_deadline(self):
        running = '123 (child) R ' + '0 ' * 18 + '456'
        with mock.patch.object(pathlib.Path, 'read_text', return_value=running), \
                mock.patch.object(time, 'monotonic', side_effect=[0, 1]), \
                mock.patch.object(time, 'sleep'):
            self.assertFalse(child_has_exited(123, '456', 1))

    @unittest.skipUnless(
        os.name == 'posix' and pathlib.Path('/proc/self/stat').exists(), 'requires Linux procfs'
    )
    def test_timeout_kills_wrapper_and_child(self):
        child = (
            'import os,pathlib,time; '
            "start=pathlib.Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]; "
            'print(os.getpid(),start,flush=True); time.sleep(60)'
        )
        wrapper = "import subprocess,sys; subprocess.run([sys.executable,'-c',sys.argv[1]])"
        started = time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired) as result:
            run_command([sys.executable, '-c', wrapper, child], 1)
        pid, start_time = result.exception.output.split()
        # EOF/reaping the wrapper can precede its child's exit_notify/Z state.
        self.assertTrue(child_has_exited(int(pid), start_time, 1), 'original child still live')
        self.assertLess(time.monotonic() - started, 5)


if __name__ == '__main__':
    unittest.main()
