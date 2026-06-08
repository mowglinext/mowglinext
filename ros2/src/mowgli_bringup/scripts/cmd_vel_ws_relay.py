#!/usr/bin/env python3
"""
cmd_vel_ws_relay — minimal WebSocket relay for /cmd_vel_teleop.

Accepts TwistStamped JSON on port 8766 and publishes directly to
/cmd_vel_teleop via rclpy, bypassing foxglove_bridge's JSON→CDR
conversion overhead and shared-connection head-of-line blocking with
subscription data.

Wire format (client → relay, newline-delimited JSON or bare JSON frames):
  {"twist": {"linear": {"x": 0.2, "y": 0, "z": 0},
             "angular": {"x": 0, "y": 0, "z": 0.5}}}

The header field is accepted but ignored; stamp defaults to zeros, which
is fine because twist_mux only checks the arrival time of the message, not
the header stamp.
"""
import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
import websockets
import websockets.exceptions

_RELAY_PORT = 8766
# Velocity clamps applied before publishing — defense-in-depth so a
# malformed or hostile frame can't command extreme motor speeds.
_MAX_LINEAR_MPS = 2.0
_MAX_ANGULAR_RAD_S = 5.0


class CmdVelRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_ws_relay")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", qos)
        self.get_logger().info("cmd_vel_ws_relay: publisher ready on /cmd_vel_teleop")

    def publish_json(self, raw: str) -> None:
        d = json.loads(raw)
        t = d.get("twist", {})
        lin = t.get("linear", {})
        ang = t.get("angular", {})

        def clamp(v: float, limit: float) -> float:
            return max(-limit, min(limit, v))

        msg = TwistStamped()
        msg.twist.linear.x = clamp(float(lin.get("x", 0.0)), _MAX_LINEAR_MPS)
        msg.twist.linear.y = clamp(float(lin.get("y", 0.0)), _MAX_LINEAR_MPS)
        msg.twist.linear.z = clamp(float(lin.get("z", 0.0)), _MAX_LINEAR_MPS)
        msg.twist.angular.x = clamp(float(ang.get("x", 0.0)), _MAX_ANGULAR_RAD_S)
        msg.twist.angular.y = clamp(float(ang.get("y", 0.0)), _MAX_ANGULAR_RAD_S)
        msg.twist.angular.z = clamp(float(ang.get("z", 0.0)), _MAX_ANGULAR_RAD_S)
        self._pub.publish(msg)


# Module-level node shared between the asyncio loop (main thread) and the
# rclpy spin thread.
_node: CmdVelRelayNode


async def _ws_handler(websocket) -> None:
    addr = websocket.remote_address
    _node.get_logger().info(f"cmd_vel_ws_relay: client connected {addr}")
    try:
        async for raw in websocket:
            try:
                _node.publish_json(raw)
            except (ValueError, KeyError) as exc:
                _node.get_logger().warn(f"cmd_vel_ws_relay: bad message: {exc}")
    except websockets.exceptions.ConnectionClosedError:
        pass
    finally:
        _node.get_logger().info(f"cmd_vel_ws_relay: client {addr} disconnected")


async def _serve() -> None:
    # ping_interval=None disables the WebSocket keep-alive ping so the
    # relay does not send periodic frames that could delay publish writes.
    async with websockets.serve(
        _ws_handler,
        "127.0.0.1",
        _RELAY_PORT,
        ping_interval=None,
        max_size=65536,
    ):
        _node.get_logger().info(f"cmd_vel_ws_relay: listening on :{_RELAY_PORT}")
        await asyncio.Future()  # run until cancelled


def main() -> None:
    global _node
    rclpy.init()
    _node = CmdVelRelayNode()

    # rclpy.spin in a daemon thread. For a pure-publisher node the spin loop
    # has no callbacks to run — it just keeps the DDS participant alive and
    # allows graceful shutdown. Daemon=True means it exits when asyncio ends.
    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(_serve())
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
