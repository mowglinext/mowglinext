"""PlatformIO pre-build hook: derive the firmware version from git.

Injects MOWGLI_FW_VERSION_{MAJOR,MINOR,PATCH} as -D build flags so every commit
yields a unique reported firmware_version with ZERO manual steps. The 3 bytes
travel in pkt_config_rsp_t (unchanged wire) and the host renders them as the
"%u.%u.%u" firmware_version string it already shows in the GUI / PreFlightCheck.

Encoding (fits the existing 3-byte field, no protocol change):
  * minor.patch = low 16 bits of `git rev-list --count HEAD` (monotonic build id)
  * major       = base 1, with bit 0x80 set when the working tree is DIRTY
                  (uncommitted changes) -> shows as 129.x.y so a hand-built /
                  un-committed flash is visible at a glance.

COMPATIBILITY is NOT gated on this value — that is MOWGLI_PROTOCOL_VERSION
(see protocol_version_guard.py). This is the free-moving human build identity.

If git is unavailable (e.g. a source tarball build) the flags are not injected
and the #ifndef fallbacks in mowgli_protocol.h (1.0.0) apply.
"""

import subprocess

Import("env")  # noqa: F821  (provided by the PlatformIO SCons runtime)

BASE_MAJOR = 1
DIRTY_FLAG = 0x80


def _git(args):
    return subprocess.check_output(
        ["git"] + args, stderr=subprocess.DEVNULL, text=True
    ).strip()


try:
    commit_count = int(_git(["rev-list", "--count", "HEAD"]))
    dirty = bool(_git(["status", "--porcelain"]))
    short = _git(["rev-parse", "--short=8", "HEAD"])

    major = BASE_MAJOR | (DIRTY_FLAG if dirty else 0)
    minor = (commit_count >> 8) & 0xFF
    patch = commit_count & 0xFF

    env.Append(  # noqa: F821
        CPPDEFINES=[
            ("MOWGLI_FW_VERSION_MAJOR", "%uu" % major),
            ("MOWGLI_FW_VERSION_MINOR", "%uu" % minor),
            ("MOWGLI_FW_VERSION_PATCH", "%uu" % patch),
        ]
    )
    print(
        "git_build_id: firmware_version %u.%u.%u (git %s%s, count=%u)"
        % (major, minor, patch, short, "-dirty" if dirty else "", commit_count)
    )
except Exception as exc:  # pragma: no cover - build-host dependent
    print("git_build_id: git unavailable (%s); using header defaults" % exc)
