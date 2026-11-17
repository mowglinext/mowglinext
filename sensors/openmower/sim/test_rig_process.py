"""RemoteRig: the real rig in a child process, behind the scenarios' API."""

import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rig_process import RemoteRig, RemoteRigError  # noqa: E402


class RemoteRigTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.rig = RemoteRig(tempfile.mkdtemp(prefix='omrig-test-'), 'xesc_2040')
        cls.rig.start()

    @classmethod
    def tearDownClass(cls):
        cls.rig.close()

    def test_runs_in_another_process(self):
        self.assertNotEqual(self.rig._process.pid, os.getpid())
        self.assertTrue(self.rig._process.is_alive())

    def test_plain_data_comes_back_as_copies(self):
        self.assertEqual(sorted(self.rig.paths), ['left', 'll', 'mow', 'right'])
        self.assertEqual(self.rig.esc_type, 'xesc_2040')
        snap = self.rig.snapshot()
        self.assertIn('latch', snap)
        self.assertEqual(self.rig.ll.config['lift_period'], 100)
        self.assertEqual(self.rig.escs['mow'].commands.maxlen, 20000)

    def test_rig_clock_shares_the_monotonic_time_base(self):
        before = time.monotonic()
        now_s = self.rig._t0 + self.rig.now_ms() / 1000.0
        self.assertLessEqual(before - 0.05, now_s)
        self.assertLessEqual(now_s, time.monotonic() + 0.05)

    def test_mapping_of_objects_yields_remote_objects(self):
        self.assertEqual(self.rig.escs.keys(), ['left', 'right', 'mow'])
        names = [name for name, esc in self.rig.escs.items() if esc.commands is not None]
        self.assertEqual(names, ['left', 'right', 'mow'])

    def test_assignment_reaches_the_rig(self):
        self.rig.ll.paused = True
        try:
            self.assertTrue(self.rig.ll.paused)
        finally:
            self.rig.ll.paused = False
        self.assertFalse(self.rig.ll.paused)

    def test_lock_holds_the_rig_lock_and_allows_nested_calls(self):
        with self.rig.lock:
            first = self.rig.motion.rows
            snap = self.rig.snapshot()   # takes the same RLock inside the child
            second = self.rig.motion.rows
        self.assertEqual(first, second)  # the rig thread could not append meanwhile
        self.assertIn('x', snap)

    def test_methods_run_in_the_rig(self):
        t0 = self.rig.now_ms()
        self.assertIsNotNone(self.rig.first_time_effective_zero(('left', 'right'), t0, 200))
        self.rig.escs['left'].reboot()

    def test_remote_exceptions_carry_the_child_traceback(self):
        with self.assertRaises(RemoteRigError) as raised:
            self.rig.ll.no_such_attribute
        self.assertIn("no attribute 'no_such_attribute'", str(raised.exception))


class RemoteRigLifecycleTest(unittest.TestCase):

    def test_close_stops_the_process(self):
        rig = RemoteRig(tempfile.mkdtemp(prefix='omrig-close-'), 'xesc_mini')
        rig.start()
        rig.close()
        self.assertFalse(rig._process.is_alive())

    def test_construction_errors_surface(self):
        with self.assertRaises(RemoteRigError):
            RemoteRig(tempfile.mkdtemp(prefix='omrig-bad-'), 'no_such_esc')


if __name__ == '__main__':
    unittest.main()
