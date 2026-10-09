#!/usr/bin/env python3
"""Tests for the comparator: detect geometry/order changes and ignore wall clocks."""

import copy
import os
import pathlib
import subprocess
import sys
import time
import unittest

from run_replay import check_controls, digest, first_difference, input_identity, run_command, stable


class ComparatorTests(unittest.TestCase):
    def record(self):
        return {"transported_polygons": [[[["0000", "0001"]]]],
                "normalized_polygons": [[[["0000", "0001"]]]], "parameters": {"width": 0.16},
                "stages": [{"name": "execution", "geometry": [[["0000", "0001"]]]}],
                "fields": [], "metrics": {"poses": 1}, "followstrip_fingerprint": "abc",
                "timing": {"planning_ms": 1}, "build": {"revision": "baseline"}}

    def test_exact_geometry_change_is_detected(self):
        before = self.record()
        after = copy.deepcopy(before)
        after["stages"][0]["geometry"][0][0][0] = "0002"
        self.assertNotEqual(digest(stable(before)), digest(stable(after)))
        self.assertEqual(first_difference(stable(before), stable(after)),
                         "/stages/0/geometry/0/0/0")

    def test_subpath_order_change_is_detected(self):
        before = self.record()
        before["stages"][0]["geometry"].append([["0002", "0003"]])
        after = copy.deepcopy(before)
        after["stages"][0]["geometry"].reverse()
        self.assertNotEqual(digest(stable(before)), digest(stable(after)))

    def test_clocks_do_not_change_geometry_identity(self):
        before = self.record()
        after = copy.deepcopy(before)
        after["timing"]["planning_ms"] = 200
        self.assertEqual(stable(before), stable(after))

    def test_effective_parameter_and_version_change_input_identity(self):
        before = self.record()
        for key, replacement in (("parameters", {"width": 0.18}),
                                 ("build", {"revision": "another"})):
            after = copy.deepcopy(before)
            after[key] = replacement
            self.assertNotEqual(input_identity(before), input_identity(after))

    def test_identity_controls_reject_same_geometry_fingerprint(self):
        a = {"input_identity": "a", "followstrip_fingerprint": "same"}
        b = {"input_identity": "b", "followstrip_fingerprint": "same"}
        self.assertEqual(len(check_controls({"square_fixed": a, "square_changed_angle": b})), 1)

    def test_command_timeout_is_bounded(self):
        started = time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired):
            run_command([sys.executable, "-c", "import time; time.sleep(60)"], 0.1)
        self.assertLess(time.monotonic() - started, 5)

    @unittest.skipUnless(os.name == "posix", "planner wrapper runs only on POSIX")
    def test_timeout_kills_wrapper_and_child(self):
        child = "import os,time; print(os.getpid(),flush=True); time.sleep(60)"
        wrapper = "import subprocess,sys; subprocess.run([sys.executable,'-c',sys.argv[1]])"
        started = time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired) as result:
            run_command([sys.executable, "-c", wrapper, child], 1)
        self.assertLess(time.monotonic() - started, 5)
        pid = int(result.exception.output.strip())
        status = pathlib.Path(f"/proc/{pid}/stat")
        # A killed orphan can briefly remain a zombie until init reaps it.
        if status.exists():
            self.assertEqual(status.read_text().split()[2], "Z")


if __name__ == "__main__":
    unittest.main()
