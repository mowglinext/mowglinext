# --- ROS test isolation (keep identical in every package's test/conftest.py) ---------------
# Tests that create real rclpy nodes/publishers must never reach a live robot: on 2026-10-07 a
# dev-host `colcon test` (docker --network host, domain 0) injected /odometry/filtered_map
# x=27,y=0 (test_goal_inputs_real_rclpy feeder) into the mowing robot's graph over the LAN and
# latched BOUNDARY_EMERGENCY_STOP. Force localhost-only discovery on a private domain before
# anything imports rclpy. MOWER_TEST_ROS_DOMAIN_ID overrides the domain.
import os as _iso_os  # noqa: E402
_iso_os.environ['ROS_LOCALHOST_ONLY'] = '1'  # deprecated on Jazzy, still honoured
_iso_os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
_iso_os.environ['ROS_DOMAIN_ID'] = _iso_os.environ.get('MOWER_TEST_ROS_DOMAIN_ID', '77')
# -------------------------------------------------------------------------------------------

"""Test configuration for um960_gps_driver.

The parser and serial tests run anywhere. The node tests need ``rclpy``; when it is
not importable (a plain Python environment, no ROS install) a small shim is
installed so the node's parsing and message-building logic is still exercised. On a
real ROS 2 Jazzy machine the genuine message classes are used instead, so the same
tests cover the real types.
"""

import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _install_rclpy_shim() -> None:
    """Register minimal stand-ins for the rclpy APIs the node uses."""

    class _Stamp:
        def __init__(self, value=0):
            self.sec = value
            self.nanosec = 0

        def to_msg(self):
            return _Stamp(self.sec)

    class Header:
        def __init__(self):
            self.stamp = _Stamp()
            self.frame_id = ""

    class NavSatStatus:
        STATUS_NO_FIX = -1
        STATUS_FIX = 0
        STATUS_SBAS_FIX = 1
        STATUS_GBAS_FIX = 2
        SERVICE_GPS = 1
        SERVICE_GLONASS = 2
        SERVICE_COMPASS = 4
        SERVICE_GALILEO = 8

        def __init__(self):
            self.status = 0
            self.service = 0

    class NavSatFix:
        COVARIANCE_TYPE_UNKNOWN = 0
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 3

        def __init__(self):
            self.header = Header()
            self.status = NavSatStatus()
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0
            self.position_covariance = [0.0] * 9
            self.position_covariance_type = 0

    class TwistStamped:
        def __init__(self):
            self.header = Header()
            linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.twist = types.SimpleNamespace(linear=linear, angular=angular)

    class String:
        def __init__(self, data=""):
            self.data = data

    class Float32:
        def __init__(self, data=0.0):
            self.data = data

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class Publisher:
        def __init__(self):
            self.msgs = []

        def publish(self, msg):
            self.msgs.append(msg)

    class Node:
        """Records parameters and published messages; callbacks are run inline."""

        def __init__(self, name, **kwargs):
            self._params = {}
            self.pubs = {}
            for parameter in kwargs.get("parameter_overrides") or []:
                self._params[parameter.name] = parameter.value

        def declare_parameter(self, name, value):
            # Like real rclpy, a declared parameter keeps any override supplied at
            # construction time and only takes the default otherwise.
            self._params.setdefault(name, value)

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._params[name])

        def set_parameters(self, parameters):
            for parameter in parameters:
                self._params[parameter.name] = parameter.value
            return []

        def create_publisher(self, _msg_type, topic, _qos=None):
            publisher = Publisher()
            self.pubs[topic] = publisher
            return publisher

        def create_timer(self, _period, _callback):
            return lambda: None

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: _Stamp(42))

        def get_logger(self):
            logger = types.SimpleNamespace()
            for level in ("debug", "info", "warn", "warning", "error", "fatal"):
                setattr(logger, level, lambda *_args, **_kwargs: None)
            return logger

        def destroy_node(self):
            return True

    def _ok():
        return True

    rclpy = types.ModuleType("rclpy")
    rclpy.ok = _ok
    rclpy.init = lambda **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy.spin_once = lambda *_args, **_kwargs: None
    rclpy.spin = lambda *_args, **_kwargs: None
    rclpy.parameter = types.SimpleNamespace(
        Parameter=lambda name, value=None: types.SimpleNamespace(name=name, value=value)
    )

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.NavSatFix = NavSatFix
    sensor_msgs_msg.NavSatStatus = NavSatStatus
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs_msg.Float32 = Float32
    std_msgs_msg.Bool = Bool
    std_msgs.msg = std_msgs_msg

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = TwistStamped
    geometry_msgs.msg = geometry_msgs_msg

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **kwargs: kwargs
    qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": node_mod,
            "rclpy.qos": qos_mod,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
        }
    )
    rclpy.executors = types.ModuleType("rclpy.executors")


try:  # pragma: no cover - depends on the environment
    import rclpy as _real_rclpy  # noqa: F401

    HAS_RCLPY = True
except ImportError:  # pragma: no cover
    _install_rclpy_shim()
    HAS_RCLPY = False


import pytest  # noqa: E402


@pytest.fixture(scope="session", autouse=True)
def _rclpy_context():
    """Initialise the real rclpy context once per session (no-op for the shim)."""
    if HAS_RCLPY:
        import rclpy

        if not rclpy.ok():
            rclpy.init()
        yield
        if rclpy.ok():
            rclpy.shutdown()
    else:
        yield
