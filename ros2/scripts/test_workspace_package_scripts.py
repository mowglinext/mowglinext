"""Focused regression tests for workspace package setup scripts."""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = REPO_ROOT / "ros2" / "scripts"


def run(*args, cwd=None, env=None):
    return subprocess.run(args, cwd=cwd, env=env, text=True, capture_output=True)


class WorkspacePackageScriptsTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)

    def test_build_and_test_stop_on_sync_failure_even_with_partial_output(self):
        script_dir = self.root / "scripts"
        script_dir.mkdir()
        workspace = self.root / "workspace"
        (workspace / "install").mkdir(parents=True)
        (workspace / "install" / "setup.bash").write_text("")
        sync_script = script_dir / "sync_workspace_packages.sh"

        bin_dir = self.root / "bin"
        bin_dir.mkdir()
        marker = self.root / "colcon_called"
        (bin_dir / "colcon").write_text("#!/bin/bash\ntouch \"$COLCON_MARKER\"\n")
        (bin_dir / "colcon").chmod(0o755)
        env = os.environ.copy()
        env.update(
            WORKSPACE_ROOT=str(workspace),
            COLCON_MARKER=str(marker),
            PATH=f"{bin_dir}:{env['PATH']}",
        )

        for sync_body, expected_error in (
            ("printf '%s\\n' /partial/package\nexit 42", "Failed to sync ROS2 package roots"),
            ("exit 0", "No ROS2 package roots were linked"),
        ):
            sync_script.write_text(f"#!/bin/bash\n{sync_body}\n")
            sync_script.chmod(0o755)
            for name in ("build.sh", "test.sh"):
                with self.subTest(script=name, sync_body=sync_body):
                    shutil.copy2(SCRIPTS / name, script_dir / name)
                    result = run("bash", str(script_dir / name), env=env)
                    self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn(expected_error, result.stderr)
                    self.assertFalse(marker.exists(), "colcon ran after sync failed")

    def test_missing_ug_removes_only_managed_symlinks(self):
        workspace = self.root / "workspace"
        source = self.root / "empty-monorepo"
        source.mkdir()
        src = workspace / "src"
        src.mkdir(parents=True)
        for name in ("universal_gnss_msgs", "universal_gnss_ros2", "unrelated"):
            (src / name).symlink_to(self.root / "missing" / name)

        env = os.environ.copy()
        env.update(MONOREPO_ROOT=str(source), WORKSPACE_ROOT=str(workspace))
        result = run("bash", str(SCRIPTS / "sync_workspace_packages.sh"), env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((src / "universal_gnss_msgs").is_symlink())
        self.assertFalse((src / "universal_gnss_ros2").is_symlink())
        self.assertTrue((src / "unrelated").is_symlink())

        (src / "universal_gnss_msgs").mkdir()
        result = run("bash", str(SCRIPTS / "sync_workspace_packages.sh"), env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((src / "universal_gnss_msgs").is_dir())

    def test_post_create_syncs_existing_clone_url_before_update(self):
        post_create = (REPO_ROOT / ".devcontainer" / "post-create.sh").read_text()
        self.assertLess(
            post_create.index("submodule sync --recursive"),
            post_create.index("submodule update --init --recursive"),
        )

        upstream = self.root / "upstream"
        parent = self.root / "parent"
        run("git", "init", "-q", str(upstream)).check_returncode()
        run("git", "-C", str(upstream), "config", "user.name", "Test").check_returncode()
        run("git", "-C", str(upstream), "config", "user.email", "test@example.com").check_returncode()
        (upstream / "README").write_text("fixture\n")
        run("git", "-C", str(upstream), "add", "README").check_returncode()
        run("git", "-C", str(upstream), "commit", "-qm", "fixture").check_returncode()

        run("git", "init", "-q", str(parent)).check_returncode()
        add = run(
            "git", "-C", str(parent), "-c", "protocol.file.allow=always",
            "submodule", "add", "-q", str(upstream), "ug",
        )
        self.assertEqual(add.returncode, 0, add.stderr)
        old_url = "https://github.com/mowglinext/universal-gnss.git"
        new_url = "https://github.com/Pepeuch/universal-gnss.git"
        (parent / ".gitmodules").write_text(
            f'[submodule "ug"]\n\tpath = ug\n\turl = {new_url}\n'
        )
        run("git", "-C", str(parent), "config", "submodule.ug.url", old_url).check_returncode()
        run("git", "-C", str(parent / "ug"), "remote", "set-url", "origin", old_url).check_returncode()

        sync = run("git", "-C", str(parent), "submodule", "sync", "--recursive")
        self.assertEqual(sync.returncode, 0, sync.stderr)
        configured_url = run("git", "-C", str(parent), "config", "--get", "submodule.ug.url")
        remote_url = run("git", "-C", str(parent / "ug"), "remote", "get-url", "origin")
        self.assertEqual(configured_url.stdout.strip(), new_url)
        self.assertEqual(remote_url.stdout.strip(), new_url)


if __name__ == "__main__":
    unittest.main()
