"""Unit tests for the bounded cmd_vel WebSocket relay heartbeat."""
import asyncio
import importlib.util
from pathlib import Path

import websockets.exceptions


RELAY_PATH = Path(__file__).parents[1] / "scripts" / "cmd_vel_ws_relay.py"
SPEC = importlib.util.spec_from_file_location("cmd_vel_ws_relay", RELAY_PATH)
relay = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(relay)


def test_twist_from_json_clamps_and_defaults() -> None:
    msg = relay.twist_from_json(
        '{"twist":{"linear":{"x":9.0},"angular":{"z":-9.0}}}'
    )
    assert msg.twist.linear.x == relay._MAX_LINEAR_MPS
    assert msg.twist.angular.z == -relay._MAX_ANGULAR_RAD_S
    assert msg.twist.linear.y == 0.0


def test_replay_timing_stays_inside_firmware_watchdog() -> None:
    assert 0 < relay._REPLAY_INTERVAL_SEC < relay._COMMAND_LEASE_SEC
    assert relay._REPLAY_INTERVAL_SEC < 0.2
    assert relay._COMMAND_LEASE_SEC < 0.2


class _Logger:
    def info(self, _message: str) -> None:
        pass

    def warn(self, _message: str) -> None:
        pass


class _Node:
    def __init__(self) -> None:
        self.published = []
        self.zero_count = 0

    def get_logger(self) -> _Logger:
        return _Logger()

    def publish_json(self, raw: str):
        msg = relay.twist_from_json(raw)
        self.published.append(msg)
        return msg

    def publish(self, msg) -> None:
        self.published.append(msg)

    def publish_zero(self) -> None:
        self.zero_count += 1


class _Socket:
    remote_address = ("127.0.0.1", 12345)

    def __init__(self) -> None:
        self.calls = 0

    async def recv(self) -> str:
        self.calls += 1
        if self.calls == 1:
            return '{"twist":{"linear":{"x":0.2}}}'
        if self.calls == 6:
            raise websockets.exceptions.ConnectionClosedOK(None, None)
        await asyncio.sleep(relay._REPLAY_INTERVAL_SEC * 2)
        return ""


def test_replays_fresh_command_then_stops_on_lease_expiry(monkeypatch) -> None:
    node = _Node()
    monkeypatch.setattr(relay, "_node", node, raising=False)
    asyncio.run(relay._ws_handler(_Socket()))
    assert len(node.published) >= 2
    # One zero at lease expiry and another for the final disconnect cleanup.
    assert node.zero_count == 2
