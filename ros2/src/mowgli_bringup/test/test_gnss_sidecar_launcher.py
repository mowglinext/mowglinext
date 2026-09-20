# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""The GNSS sidecar reads mowgli_robot.yaml — and nothing else.

install/compose/docker-compose.gps.yml runs a third-party image, so the code
that turns mowgli_robot.yaml into the receiver's ROS parameters is embedded in
the compose `command`. The installed file is SPARSE (root CLAUDE.md, Invariant
15) and that image does not carry the mowgli_bringup template, so the launcher
holds its own defaults: this test pins them to the template, which is the one
place a default is allowed to live.
"""

from pathlib import Path
import sys
import types
from concurrent.futures import Future
from unittest.mock import MagicMock

import pytest
import yaml

_PKG_DIR = Path(__file__).resolve().parent.parent
_TEMPLATE = _PKG_DIR / "config" / "mowgli_robot.yaml"
_FRAGMENT = _PKG_DIR.parents[2] / "install" / "compose" / "docker-compose.gps.yml"


def _launcher():
    if not _FRAGMENT.exists():
        pytest.skip("install/compose/docker-compose.gps.yml not in this tree")
    service = yaml.safe_load(_FRAGMENT.read_text())["services"]["gps"]
    interpreter, flag, source = service["command"]
    assert (interpreter, flag) == ("python3", "-c")
    module = types.ModuleType("gnss_sidecar_launcher")
    exec(compile(source, str(_FRAGMENT), "exec"), module.__dict__)  # noqa: S102
    return module, service


def _template() -> dict:
    return yaml.safe_load(_TEMPLATE.read_text())["mowgli"]["ros__parameters"]


@pytest.mark.parametrize("ntrip,failed", [
    (False, None), (True, None),
    (False, "/universal_gnss_receiver/get_health"),
    (True, "/universal_gnss_ntrip/get_health"),
])
def test_healthcheck_checks_only_enabled_services(monkeypatch, ntrip, failed):
    """Execute the shipped probe, including ROS environment and typed result contract."""
    _, service = _launcher()
    kind, entrypoint, interpreter, flag, source = service["healthcheck"]["test"]
    assert (kind, entrypoint, interpreter, flag) == (
        "CMD", "/usr/local/bin/universal-gnss-entrypoint", "python3", "-c")
    assert service["healthcheck"]["timeout"] == "12s"
    ros = MagicMock()
    node = ros.create_node.return_value
    called = []

    def client_for(_, name):
        called.append(name)
        client = MagicMock()
        client.wait_for_service.return_value = True
        future = Future()
        future.set_result(types.SimpleNamespace(success=name != failed))
        client.call_async.return_value = future
        return client

    node.create_client.side_effect = client_for
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    srv = types.ModuleType("std_srvs.srv")
    srv.Trigger = MagicMock()
    monkeypatch.setitem(sys.modules, "std_srvs", types.ModuleType("std_srvs"))
    monkeypatch.setitem(sys.modules, "std_srvs.srv", srv)
    module = types.ModuleType("healthcheck")
    exec(compile(source, str(_FRAGMENT), "exec"), module.__dict__)  # noqa: S102
    monkeypatch.setattr(module.pathlib.Path, "exists", lambda _: ntrip)
    assert module.main() == (1 if failed else 0)
    assert called == ["/universal_gnss_receiver/get_health"] + (
        ["/universal_gnss_ntrip/get_health"] if ntrip else [])
    assert node.destroy_client.call_count == len(called)
    node.destroy_node.assert_called_once()
    ros.shutdown.assert_called_once()


@pytest.mark.parametrize("failure", ["unavailable", "timeout", "exception", "empty"])
def test_healthcheck_rejects_incomplete_service_responses(monkeypatch, failure):
    _, service = _launcher()
    source = service["healthcheck"]["test"][-1]
    ros = MagicMock()
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    srv = types.ModuleType("std_srvs.srv")
    srv.Trigger = MagicMock()
    monkeypatch.setitem(sys.modules, "std_srvs", types.ModuleType("std_srvs"))
    monkeypatch.setitem(sys.modules, "std_srvs.srv", srv)
    module = types.ModuleType("healthcheck")
    exec(compile(source, str(_FRAGMENT), "exec"), module.__dict__)  # noqa: S102
    node = MagicMock()
    client = node.create_client.return_value
    client.wait_for_service.return_value = failure != "unavailable"
    future = Future()
    if failure == "exception":
        future.set_exception(RuntimeError("receiver disconnected"))
    elif failure != "timeout":
        future.set_result(None)
    client.call_async.return_value = future
    assert not module.responsive(node, "/universal_gnss_receiver/get_health")
    node.destroy_client.assert_called_once_with(client)


def test_launcher_defaults_mirror_the_template():
    launcher, _ = _launcher()
    template = _template()
    shared = sorted(set(launcher.DEFAULTS) & set(template))
    # Every key the operator can see in the template must be covered.
    assert {"gnss_receiver_family", "gnss_serial_device", "gnss_serial_baud", "gnss_frame_id",
            "ntrip_enabled", "ntrip_host", "ntrip_port", "ntrip_user", "ntrip_password",
            "ntrip_mountpoint"} <= set(shared)
    for key in shared:
        assert launcher.DEFAULTS[key] == template[key], key


def test_sparse_config_overrides_only_what_it_names():
    launcher, _ = _launcher()
    params, ntrip = launcher.build({
        "gnss_receiver_family": "ublox",
        "gnss_serial_device": "/dev/serial/by-id/usb-u-blox-if00",
        "gnss_serial_baud": 460800,
        "datum_lat": 48.0,  # unrelated keys are ignored
    })
    receiver = params["/universal_gnss_receiver"]["ros__parameters"]
    assert receiver["receiver_family"] == "ublox"
    assert receiver["serial_device"] == "/dev/serial/by-id/usb-u-blox-if00"
    assert receiver["serial_baud"] == 460800
    assert receiver["frame_id"] == "gps_link"  # template default
    assert isinstance(receiver["publish_rate_hz"], float)
    assert ntrip is False  # template default: NTRIP off


def test_ntrip_needs_a_host_and_a_mountpoint():
    launcher, _ = _launcher()
    base = {"ntrip_enabled": True, "ntrip_host": "caster.example", "ntrip_mountpoint": "MP"}
    assert launcher.build(base)[1] is True
    assert launcher.build({**base, "ntrip_host": " "})[1] is False
    assert launcher.build({**base, "ntrip_mountpoint": ""})[1] is False
    assert launcher.build({**base, "ntrip_enabled": "false"})[1] is False


def test_credentials_survive_the_generated_parameter_file():
    launcher, _ = _launcher()
    secret = "p@ss: #word' \"quoted\" $x"
    params, _ = launcher.build({"ntrip_password": secret, "ntrip_user": "centipede"})
    round_trip = yaml.safe_load(yaml.safe_dump(params, default_flow_style=False))
    ntrip = round_trip["/universal_gnss_ntrip"]["ros__parameters"]
    assert ntrip["password"] == secret
    assert ntrip["username"] == "centipede"


def test_fragment_needs_no_host_side_gnss_configuration():
    _, service = _launcher()
    assert "./docker/config/mowgli:/config:ro" in service["volumes"]
    assert not any("parameters.yaml" in str(volume) for volume in service["volumes"])
    assert service.get("privileged") is not True
    # A robot updated through the GUI never ran the installer: every compose
    # variable must therefore carry a default.
    import re
    text = _FRAGMENT.read_text()
    assert re.findall(r"\$\{[A-Z_0-9]+\}", text) == []
