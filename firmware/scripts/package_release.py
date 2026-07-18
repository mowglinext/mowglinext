#!/usr/bin/env python3
"""Package prebuilt firmware binaries + a manifest for a GitHub Release.

Phase 1 of the precompiled-firmware plan: after CI has run `pio run` for each
default permutation, this collects the resulting `firmware.bin` / `firmware.elf`,
stamps each with a stable, self-describing name, checksums it, and emits a
`manifest.json` mapping every permutation to {file, url, sha256, protocol_version,
fw_version}. Onboarding (Phase 2, not yet wired) reads the manifest to pick and
flash the right binary WITHOUT compiling.

The binaries themselves are the exact artifacts CI already builds today (the
committed board.h, unchanged) — this script does not render, re-tune, or alter
any firmware value. It is pure packaging.

Versioning (must stay consistent with the rest of the toolchain):
  * protocol_version — parsed from mowgli_protocol.h MOWGLI_PROTOCOL_VERSION.
    This is the host<->firmware compatibility key (see protocol_version_guard.py).
  * fw_version — the human build identity. Computed here with the SAME encoding
    as git_build_id.py (the PlatformIO pre-build hook that bakes the version into
    the binary), so the manifest matches what the flashed firmware reports over
    pkt_config_rsp_t. If you change the encoding in git_build_id.py, change it
    here too (kept in lockstep by hand — the hook runs inside SCons and cannot be
    imported from here without pulling in the PlatformIO runtime).

Usage:
    package_release.py --build-root <pio_project> --tag <vX.Y.Z> \
        --repo <owner/name> --out-dir <dist>
"""

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

# --- fw_version encoding: keep in lockstep with git_build_id.py ---------------
BASE_MAJOR = 1
DIRTY_FLAG = 0x80

# The default permutation catalog. Each entry is a PlatformIO env plus the
# board/panel identity that ends up in the artifact name and the manifest key.
# Extend this list (and firmware-ci.yml's matrix) when a new prebuilt permutation
# is published — NOT by hand-editing rendered binaries. Panel/board strings are
# lowercased, stripped tokens taken from board.h.template's PANEL_TYPE_* /
# BOARD_* names so the selection map (Phase 2) can resolve them deterministically.
PERMUTATIONS = [
    {
        "key": "yardforce500",
        "env": "Yardforce500",
        "board": "BOARD_YARDFORCE500",
        "panel": "PANEL_TYPE_YARDFORCE_500_CLASSIC",
    },
    {
        "key": "yardforce500b",
        "env": "Yardforce500B",
        "board": "BOARD_YARDFORCE500B",
        "panel": "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
    },
]


def _git(args, cwd):
    return subprocess.check_output(
        ["git"] + args, cwd=cwd, stderr=subprocess.DEVNULL, text=True
    ).strip()


def fw_version(repo_root):
    """Mirror git_build_id.py: 'major.minor.patch' from the git commit count."""
    count = int(_git(["rev-list", "--count", "HEAD"], repo_root))
    dirty = bool(_git(["status", "--porcelain"], repo_root))
    major = BASE_MAJOR | (DIRTY_FLAG if dirty else 0)
    minor = (count >> 8) & 0xFF
    patch = count & 0xFF
    return f"{major}.{minor}.{patch}"


def git_short(repo_root):
    return _git(["rev-parse", "--short=8", "HEAD"], repo_root)


def read_protocol_version(repo_root):
    header = (
        repo_root
        / "firmware/stm32/ros_usbnode/include/mowgli_protocol.h"
    )
    m = re.search(
        r"#define\s+MOWGLI_PROTOCOL_VERSION\s+(\d+)", header.read_text()
    )
    if not m:
        sys.exit(f"ERROR: MOWGLI_PROTOCOL_VERSION not found in {header}")
    return int(m.group(1))


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def panel_token(panel):
    """PANEL_TYPE_YARDFORCE_500B_CLASSIC -> 500b_classic (compact artifact tag)."""
    return panel.replace("PANEL_TYPE_YARDFORCE_", "").lower()


def board_token(board):
    """BOARD_YARDFORCE500B -> yardforce500b."""
    return board.replace("BOARD_", "").lower()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-root", required=True, help="PlatformIO project dir")
    ap.add_argument("--tag", required=True, help="Release tag, e.g. v1.2.3")
    ap.add_argument("--repo", required=True, help="owner/name for asset URLs")
    ap.add_argument("--out-dir", required=True, help="dir to write bins + manifest")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    build_root = Path(args.build_root)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    protocol = read_protocol_version(repo_root)
    version = fw_version(repo_root)
    short = git_short(repo_root)

    permutations = {}
    for perm in PERMUTATIONS:
        env = perm["env"]
        bin_src = build_root / ".pio" / "build" / env / "firmware.bin"
        elf_src = build_root / ".pio" / "build" / env / "firmware.elf"
        if not bin_src.exists():
            sys.exit(f"ERROR: {bin_src} missing — did `pio run -e {env}` run?")

        stem = (
            f"mowgli-fw_{board_token(perm['board'])}_{panel_token(perm['panel'])}"
            f"_p{protocol}_v{version}_{short}"
        )
        bin_name = f"{stem}.bin"
        (out_dir / bin_name).write_bytes(bin_src.read_bytes())
        if elf_src.exists():
            (out_dir / f"{stem}.elf").write_bytes(elf_src.read_bytes())
        digest = sha256_file(out_dir / bin_name)

        permutations[perm["key"]] = {
            "env": env,
            "board": perm["board"],
            "panel": perm["panel"],
            "file": bin_name,
            "url": (
                f"https://github.com/{args.repo}/releases/download/"
                f"{args.tag}/{bin_name}"
            ),
            "sha256": digest,
            "protocol_version": protocol,
            "fw_version": version,
        }

    manifest = {
        "tag": args.tag,
        "protocol_version": protocol,
        "fw_version": version,
        "git_short": short,
        "permutations": permutations,
    }
    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

    print(f"Packaged {len(permutations)} permutation(s) into {out_dir}:")
    for name, entry in permutations.items():
        print(f"  {name}: {entry['file']}  sha256={entry['sha256'][:12]}…")
    print(f"Wrote {manifest_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
