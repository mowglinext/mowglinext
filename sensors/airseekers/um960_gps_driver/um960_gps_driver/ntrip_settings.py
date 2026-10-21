"""Owner NTRIP settings file (``/userdata/ros2/ntrip.yaml``).

Flat YAML, written by the driver whenever the GUI (parameter binding) or
``ros2 param set`` changes a caster field, and read at startup / when its mtime
changes::

    enabled: true
    host: caster.example.net
    port: 2101
    mountpoint: MOUNT
    user: me
    password: secret

The file holds a password, so it is written atomically with mode 0600 and its
contents are never logged. ROS-free; PyYAML is used when available.
"""

import os
import tempfile
from typing import Dict, Optional

DEFAULT_SETTINGS_FILE = "/userdata/ros2/ntrip.yaml"
KEYS = ("enabled", "host", "port", "mountpoint", "user", "password")


def _parse_flat(text: str) -> Dict[str, object]:
    raw: Dict[str, object] = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, _, value = line.partition(":")
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        raw[key.strip()] = value
    return raw


def _as_bool(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "on")
    return bool(value)


def load_settings(path: str) -> Optional[Dict[str, object]]:
    """The normalised settings, or None when the file is absent/unreadable/invalid."""
    if not path:
        return None
    try:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return None
    raw: Dict[str, object]
    try:
        import yaml  # type: ignore

        loaded = yaml.safe_load(text)
        if loaded is None:
            loaded = {}
        if not isinstance(loaded, dict):
            return None
        raw = loaded
    except ImportError:
        raw = _parse_flat(text)
    except Exception:  # noqa: BLE001 - malformed YAML: behave as if absent
        return None
    try:
        port = int(raw.get("port") or 2101)
    except (TypeError, ValueError):
        port = 2101

    def _str(key: str) -> str:
        value = raw.get(key)
        return "" if value is None else str(value).strip()

    return {
        "enabled": _as_bool(raw.get("enabled", False)),
        "host": _str("host"),
        "port": port,
        "mountpoint": _str("mountpoint").lstrip("/"),
        "user": _str("user"),
        "password": "" if raw.get("password") is None else str(raw.get("password")),
    }


def _quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def render_settings(settings: Dict[str, object]) -> str:
    lines = [
        "# NTRIP caster for um960_gps_driver. Written by the driver (GUI NTRIP card /",
        "# ros2 param set); edit by hand only while the driver is stopped or expect it",
        "# to be re-read within a few seconds. Contains a password: keep mode 0600.",
        "enabled: %s" % ("true" if settings.get("enabled") else "false"),
        "host: %s" % _quote(str(settings.get("host") or "")),
        "port: %d" % int(settings.get("port") or 2101),
        "mountpoint: %s" % _quote(str(settings.get("mountpoint") or "")),
        "user: %s" % _quote(str(settings.get("user") or "")),
        "password: %s" % _quote(str(settings.get("password") or "")),
    ]
    return "\n".join(lines) + "\n"


def save_settings(path: str, settings: Dict[str, object]) -> None:
    """Atomically write ``settings`` to ``path`` (mode 0600). Raises OSError."""
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".ntrip.", suffix=".tmp", dir=directory)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(render_settings(settings))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def file_mtime(path: str) -> Optional[float]:
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None
