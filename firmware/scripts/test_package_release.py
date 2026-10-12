#!/usr/bin/env python3
"""Keep release packaging, PlatformIO targets, and release workflows aligned."""

import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PACKAGE_SCRIPT = ROOT / "firmware/scripts/package_release.py"
PLATFORMIO_INI = ROOT / "firmware/stm32/ros_usbnode/platformio.ini"


def workflow_builds(workflow_path, build_step_name, package_step_name):
    """Return the `pio run -e` targets in the build step before packaging."""
    lines = workflow_path.read_text().splitlines()
    step_offsets = {}
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("- name:"):
            step_offsets[stripped.removeprefix("- name:").strip()] = index

    if build_step_name not in step_offsets or package_step_name not in step_offsets:
        raise AssertionError(f"missing release build/package step in {workflow_path}")

    build_index = step_offsets[build_step_name]
    package_index = step_offsets[package_step_name]
    if build_index >= package_index:
        raise AssertionError(f"packaging runs before builds in {workflow_path}")

    build_lines = lines[build_index + 1 : package_index]
    return re.findall(r"\bpio run -e ([A-Za-z0-9_]+)", "\n".join(build_lines))


def main():
    # Preserve existing published identities and publish only the normal RM1000
    # environment. The yaw-180 target is a mounting-specific variant.
    sys.path.insert(0, str(PACKAGE_SCRIPT.parent))
    import package_release

    catalog = package_release.PERMUTATIONS
    by_key = {entry["key"]: entry for entry in catalog}
    expected = {
        "yardforce500": {
            "env": "Yardforce500",
            "board": "BOARD_YARDFORCE500",
            "panel": "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        },
        "yardforce500b": {
            "env": "Yardforce500B",
            "board": "BOARD_YARDFORCE500B",
            "panel": "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        },
        "biltema-rm1000": {
            "env": "BiltemaRM1000",
            "board": "BOARD_YARDFORCE500B",
            "panel": "PANEL_TYPE_YARDFORCE_900_ECO",
        },
    }
    actual = {
        key: {field: entry[field] for field in ("env", "board", "panel")}
        for key, entry in by_key.items()
    }
    if actual != expected:
        raise AssertionError(f"unexpected prebuilt permutation catalog: {actual}")
    if len(by_key) != len(catalog):
        raise AssertionError("release permutation keys must be unique")

    platformio_text = PLATFORMIO_INI.read_text()
    declared_envs = set(re.findall(r"^\[env:([^]]+)\]", platformio_text, re.MULTILINE))
    missing_envs = sorted({entry["env"] for entry in catalog} - declared_envs)
    if missing_envs:
        raise AssertionError(f"catalog environments absent from platformio.ini: {missing_envs}")

    workflows = (
        (
            ROOT / ".github/workflows/firmware-ci.yml",
            "Build all default permutations",
            "Package binaries + manifest",
        ),
        (
            ROOT / ".github/workflows/deployment-release.yml",
            "Build every prebuilt permutation",
            "Package binaries + manifest for this deployment's release",
        ),
    )
    expected_envs = [entry["env"] for entry in catalog]
    for workflow_path, build_step, package_step in workflows:
        built_envs = workflow_builds(workflow_path, build_step, package_step)
        if built_envs != expected_envs:
            raise AssertionError(
                f"{workflow_path.relative_to(ROOT)} builds {built_envs}; "
                f"expected {expected_envs} before packaging"
            )

    # Exercise the real packager with placeholder artifacts and inspect the
    # emitted manifest, without depending on a PlatformIO build in this check.
    with tempfile.TemporaryDirectory(prefix="mowgli-release-package-test-") as temp:
        temp_root = Path(temp)
        build_root = temp_root / "project"
        out_dir = temp_root / "dist"
        for entry in catalog:
            bin_path = build_root / ".pio" / "build" / entry["env"] / "firmware.bin"
            bin_path.parent.mkdir(parents=True, exist_ok=True)
            bin_path.write_bytes(f"test binary for {entry['env']}\n".encode())

        subprocess.run(
            [
                sys.executable,
                str(PACKAGE_SCRIPT),
                "--build-root",
                str(build_root),
                "--tag",
                "v0.0.0-test",
                "--repo",
                "mowglinext/mowglinext",
                "--out-dir",
                str(out_dir),
            ],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        manifest = json.loads((out_dir / "manifest.json").read_text())
        rm1000 = manifest["permutations"]["biltema-rm1000"]
        for field, value in expected["biltema-rm1000"].items():
            if rm1000[field] != value:
                raise AssertionError(f"RM1000 manifest {field}: {rm1000[field]!r}")
        if "BiltemaRM1000_MPU6050_Yaw180" in {
            item["env"] for item in manifest["permutations"].values()
        }:
            raise AssertionError("mount-specific yaw-180 firmware must not be a default prebuilt")

    print("Release catalog, PlatformIO targets, workflows, and manifest are aligned.")


if __name__ == "__main__":
    main()
