#!/usr/bin/env python3
"""Exercise the real CMake provenance writer, without compiling a planner."""

import hashlib
import os
import pathlib
import shutil
import subprocess
import tempfile
import unittest


class ProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.TemporaryDirectory()
        self.addCleanup(self.work.cleanup)
        self.root = pathlib.Path(self.work.name)
        self.source = self.root / "planner.cpp"
        self.header = self.root / "planner.hpp"
        self.output = self.root / "provenance.hpp"
        self.source.write_text("source baseline\n")
        self.header.write_text("header baseline\n")
        self.cmake = os.environ.get("REPLAY_CMAKE") or shutil.which("cmake")
        self.assertTrue(self.cmake, "CMake is required for provenance tests")
        self.git("init", "-b", "replay-test")
        self.git("config", "user.name", "Replay Test")
        self.git("config", "user.email", "replay-test@example.invalid")
        self.git("add", "planner.cpp", "planner.hpp")
        self.git("commit", "-m", "fixture baseline")

    def git(self, *arguments):
        return subprocess.check_output(["git", "-C", str(self.root), *arguments],
                                       text=True, stderr=subprocess.STDOUT).strip()

    def write(self, revision=None):
        command = [self.cmake, f"-DPLANNER_SOURCE={self.source}",
                   f"-DPLANNER_HEADER={self.header}", f"-DPACKAGE_ROOT={self.root}",
                   f"-DOUTPUT={self.output}"]
        if revision is not None:
            command.append(f"-DREPLAY_REVISION={revision}")
        command += ["-P", str(pathlib.Path(__file__).with_name("write_provenance.cmake"))]
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
        return self.output.read_text()

    def test_exact_source_and_header_hashes(self):
        result = self.write()
        for path in (self.source, self.header):
            self.assertIn(hashlib.sha256(path.read_bytes()).hexdigest(), result)
        self.assertIn(self.git("rev-parse", "HEAD"), result)

    def test_incremental_source_and_header_changes(self):
        result = self.write()
        for path in (self.source, self.header):
            path.write_text(path.read_text() + "incremental change\n")
            changed = self.write()
            self.assertNotEqual(result, changed)
            self.assertIn(hashlib.sha256(path.read_bytes()).hexdigest(), changed)
            result = changed

    def test_unchanged_refresh_preserves_timestamp(self):
        before = self.write()
        timestamp = self.output.stat().st_mtime_ns
        self.assertEqual(before, self.write())
        self.assertEqual(timestamp, self.output.stat().st_mtime_ns)

    def test_packed_branch_commit_refreshes_revision(self):
        self.git("pack-refs", "--all", "--prune")
        loose = self.root / ".git" / "refs" / "heads" / "replay-test"
        self.assertFalse(loose.exists())
        before = self.write()
        symbolic_head = (self.root / ".git" / "HEAD").read_bytes()
        (self.root / "report.txt").write_text("only a report changed\n")
        self.git("add", "report.txt")
        self.git("commit", "-m", "report only")
        self.assertTrue(loose.exists())
        self.assertEqual(symbolic_head, (self.root / ".git" / "HEAD").read_bytes())
        self.assertNotEqual(before, self.write())
        self.assertIn(self.git("rev-parse", "HEAD"), self.output.read_text())
        timestamp = self.output.stat().st_mtime_ns
        self.write()
        self.assertEqual(timestamp, self.output.stat().st_mtime_ns)

    def test_archive_explicit_revision(self):
        # A directory outside this temporary repo has no discoverable .git.
        with tempfile.TemporaryDirectory() as archive:
            old_root = self.root
            self.root = pathlib.Path(archive)
            try:
                self.assertIn('REPLAY_REVISION "012345abcdef"', self.write("012345abcdef"))
            finally:
                self.root = old_root


if __name__ == "__main__":
    unittest.main()
