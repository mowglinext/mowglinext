# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Airseekers Tron hardware bridge (ROS node name ``hardware_bridge``).

Publishes the /hardware_bridge contract (docs/claude/ros-interfaces.md) from
the Tron's vendor MCU (``/dev/serial_mower``), its WIT JY61P IMU
(``/dev/serial_imu``) and its top-panel keys (``/dev/keyboard``). The GNSS
receiver is a second node in this container (um960_gps_driver).

Threads: one serial I/O thread for the MCU (RX + heartbeat + control tick +
odometry, all on one select loop), one for the IMU, one for the keys. ROS
callbacks run on the executor. Everything that touches bridge state holds
``self._lock``.

Safety model: see safety.py and README.md "Safety model".
"""

from __future__ import annotations

import json
import math
import os
import select
import threading
import time
import traceback
from typing import Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import Emergency, HighLevelStatus, Power, Status, WheelTick
from mowgli_interfaces.srv import EmergencyStop, HighLevelControl, MowerControl
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_system_default
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Int16, String
from std_srvs.srv import SetBool, Trigger

from . import keys as keys_logic
from . import protocol as P
from .contract import (blade_rpm, firmware_compatible, firmware_version_string,
                       project_power)
from .drive_shaper import DriveShaper
from .imu_bias import ImuBiasEstimator, RestCalibrator
from .safety import (BladeInputs, DockGuard, EmergencyInputs, EmergencyLatch, HL_AUTONOMOUS,
                     HL_MANUAL_MOWING, HL_NULL, SpeedTx, TickIntegrator, blade_may_run, clamp,
                     wheels_may_run)
from .serial_port import SerialPort
from .tilt import TiltBands, quat_to_roll_pitch
from .wit import TYPE_ANGLE, WitParser, euler_to_quaternion


def update_maintenance_active() -> bool:
    """mowgli_interfaces/update_maintenance.hpp, in Python."""
    path = os.environ.get('MOWGLI_UPDATE_MAINTENANCE', '')
    if not path:
        return False
    parent = os.path.dirname(path) or '.'
    if not os.path.isdir(parent):
        return True
    return os.path.exists(path)


def _diag3(a, b, c):
    return [float(a), 0.0, 0.0, 0.0, float(b), 0.0, 0.0, 0.0, float(c)]


def _quat_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class AirseekersBridge(Node):

    def __init__(self) -> None:
        super().__init__('hardware_bridge')
        self._lock = threading.RLock()
        self._declare()
        self._init_state()
        self._create_interfaces()
        self._stop = threading.Event()
        self._wake_r, self._wake_w = os.pipe()
        os.set_blocking(self._wake_r, False)
        os.set_blocking(self._wake_w, False)
        self._threads = [threading.Thread(target=self._mcu_loop, name='mcu_io', daemon=True)]
        if self.p('imu_enabled'):
            self._threads.append(threading.Thread(target=self._imu_loop, name='imu_io', daemon=True))
        if self.p('keys_enabled'):
            self._threads.append(threading.Thread(target=self._keys_loop, name='keys_io',
                                                  daemon=True))
        self.get_logger().info(
            'Airseekers bridge up: MCU %s @ %d, IMU %s, keys %s; wheel_track %.3f m, clamp '
            '%.2f m/s %.2f rad/s, cmd_vel timeout %.2f s, link timeout %.2f s'
            % (self.p('mcu_port'), self.p('mcu_baud'),
               self.p('imu_port') if self.p('imu_enabled') else 'disabled',
               self.p('keys_device') or 'auto' if self.p('keys_enabled') else 'disabled',
               self.p('wheel_track'), self.p('max_mps'), self.p('max_angular_radps'),
               self.p('cmd_vel_timeout_s'), self.p('mcu_status_timeout_s')))

    # ------------------------------------------------------------------ setup
    def p(self, name):
        return self._params[name]

    def _declare(self) -> None:
        defaults = {
            # serial links
            'mcu_port': '/dev/serial_mower', 'mcu_baud': 115200,
            'imu_enabled': True, 'imu_port': '/dev/serial_imu', 'imu_baud': 115200,
            'keys_enabled': True, 'keys_device': '',
            # timing
            'heartbeat_period_s': 0.1, 'control_rate': 20.0, 'odom_rate': 50.0,
            'status_rate': 10.0, 'mcu_status_timeout_s': 0.5, 'mcu_reopen_timeout_s': 2.0,
            'imu_timeout_s': 0.5, 'cmd_vel_timeout_s': 0.5, 'high_level_status_timeout_s': 5.0,
            # command shaping
            'wheel_track': 0.48, 'ticks_per_meter': 1000.0, 'max_mps': 0.3,
            'max_angular_radps': 0.3, 'min_linear_vel': 0.0,
            'cmd_vel_linear_accel_limit': 0.30, 'cmd_vel_linear_decel_limit': 0.60,
            'cmd_vel_angular_accel_limit': 1.0, 'cmd_vel_angular_decel_limit': 2.0,
            'stop_frames': 3, 'stop_frame_spacing_s': 0.1, 'docked_allow_reverse': False,
            'angular_trim_radps': 0.0, 'angular_deadband_radps': 0.0, 'heading_hold': False,
            'heading_hold_kp': 1.0, 'heading_hold_kd': 0.1, 'heading_hold_max_radps': 0.15,
            'odom_angular_source': 'imu',
            # blade
            'mowing_enabled': True, 'blade_speed_percent': 100, 'cutter_height_mm': 90,
            'blade_off_frames': 3,
            # charging
            'charging_current_min_a': 0.2, 'charge_on_dock': True, 'charge_refresh_s': 30.0,
            # IMU
            'imu_frame_id': 'imu_link', 'forward_imu': True, 'forward_imu_rate': 45.0,
            'forward_imu_roll_offset_deg': 0.0, 'forward_imu_pitch_offset_deg': 0.0,
            'orientation_rp_covariance': 0.02, 'orientation_yaw_covariance': 1.0,
            'angular_velocity_covariance': 0.005, 'linear_acceleration_covariance': 0.05,
            'imu_cal_samples': 200, 'imu_cal_auto_rest_sec': 15.0,
            # emergency sources
            'tilt_emergency_enabled': True, 'tilt_critical_roll_deg': 30.0,
            'tilt_critical_pitch_deg': 32.0, 'tilt_emergency_ms': 300,
            'bumper_is_emergency': False, 'latch_emergency_on_boot': True,
            # panel keys
            'power_short_resets_emergency': True,
        }
        self._params = {}
        for name, value in defaults.items():
            self._params[name] = self.declare_parameter(name, value).value
        # Live-settable: deck height and the straight-driving knobs.
        self.add_on_set_parameters_callback(self._on_set_params)
        for name in ('max_mps', 'max_angular_radps', 'wheel_track'):
            v = float(self._params[name])
            if not math.isfinite(v) or v <= 0.0:
                raise ValueError('%s must be > 0' % name)
        if self._params['odom_angular_source'] not in ('imu', 'mcu'):
            raise ValueError('odom_angular_source must be imu or mcu')

    _LIVE = ('cutter_height_mm', 'angular_trim_radps', 'angular_deadband_radps', 'heading_hold',
             'heading_hold_kp', 'heading_hold_kd', 'heading_hold_max_radps',
             'blade_speed_percent')

    def _on_set_params(self, params):
        for prm in params:
            if prm.name not in self._LIVE:
                return SetParametersResult(successful=False,
                                           reason='%s is read at start-up only' % prm.name)
            if prm.name == 'cutter_height_mm':
                v = int(prm.value)
                if not P.CUTTER_HEIGHT_MIN_MM <= v <= P.CUTTER_HEIGHT_MAX_MM:
                    return SetParametersResult(successful=False, reason='cutter_height_mm must be '
                                               '%d..%d' % (P.CUTTER_HEIGHT_MIN_MM,
                                                           P.CUTTER_HEIGHT_MAX_MM))
        with self._lock:
            for prm in params:
                self._params[prm.name] = prm.value
            self._shaper = self._make_shaper()
            self._height_dirty = True
        return SetParametersResult(successful=True)

    def _make_shaper(self) -> DriveShaper:
        return DriveShaper(self.p('angular_trim_radps'), self.p('angular_deadband_radps'),
                           self.p('heading_hold'), self.p('heading_hold_kp'),
                           self.p('heading_hold_kd'), self.p('heading_hold_max_radps'))

    def _init_state(self) -> None:
        self._ser: Optional[SerialPort] = None
        self._parser = P.FrameParser()
        self._last_open_try = -1e9
        self._mcu_rx_t = 0.0             # any byte decoded
        self._sensor: Optional[P.SensorInfo] = None
        self._sensor_t = 0.0
        self._battery: Optional[P.BatteryInfo] = None
        self._motors: Dict[str, P.MotorInfo] = {}
        self._motors_t = 0.0
        self._versions: Optional[P.VersionInfo] = None
        self._meas = (0.0, 0.0)
        self._meas_t = 0.0
        self._cmd = (0.0, 0.0)
        self._cmd_t = -1e9
        self._hl_mode = HL_NULL
        self._hl_t = -1e9
        self._mow_requested = False
        self._mow_direction = 0
        self._latch = EmergencyLatch(latch_on_boot=bool(self.p('latch_emergency_on_boot')))
        self._em_state = self._latch.state()
        self._dock = DockGuard(allow_reverse=bool(self.p('docked_allow_reverse')))
        self._tx = SpeedTx(self.p('cmd_vel_linear_accel_limit'),
                           self.p('cmd_vel_linear_decel_limit'),
                           self.p('cmd_vel_angular_accel_limit'),
                           self.p('cmd_vel_angular_decel_limit'),
                           max(1, int(self.p('stop_frames'))), self.p('stop_frame_spacing_s'))
        self._shaper = self._make_shaper()
        self._ticks = TickIntegrator(self.p('ticks_per_meter'), self.p('wheel_track'))
        self._blade_sent: Optional[bool] = None
        self._blade_off_left = 0
        self._height_dirty = True
        self._height_published = None
        self._blade_dir_text = 'unknown'
        self._dock_note = ''
        self._applied_published = None
        self._was_docked = None
        self._charge_sent_t = -1e9
        self._last_odom_t = time.monotonic()
        # IMU
        self._imu_t = 0.0
        self._imu_rpy = None
        self._imu_gyro_z = 0.0
        self._imu_fwd_t = 0.0
        self._imu_fwd_count = 0
        self._bias = ImuBiasEstimator(int(self.p('imu_cal_samples')))
        self._cal = RestCalibrator(self._bias, 1.0, float(self.p('imu_cal_auto_rest_sec')))
        crit_s = max(0.01, float(self.p('tilt_emergency_ms')) / 1000.0)
        r_c = float(self.p('tilt_critical_roll_deg'))
        p_c = float(self.p('tilt_critical_pitch_deg'))
        self._tilt = TiltBands(
            roll_thresholds_deg=(min(15.0, r_c * 0.5), min(22.0, r_c * 0.75), r_c),
            pitch_thresholds_deg=(min(18.0, p_c * 0.5), min(25.0, p_c * 0.75), p_c),
            debounce_s=(1.0, 0.6, crit_s),
            mount_roll_offset_deg=float(self.p('forward_imu_roll_offset_deg')),
            mount_pitch_offset_deg=float(self.p('forward_imu_pitch_offset_deg')))
        self._tilt_critical = False
        # keys
        self._keys_state = 'not started'

    def _create_interfaces(self) -> None:
        q10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status = self.create_publisher(Status, '~/status', q10)
        self._pub_emergency = self.create_publisher(Emergency, '~/emergency', q10)
        self._pub_power = self.create_publisher(Power, '~/power', q10)
        self._pub_battery = self.create_publisher(BatteryState, '/battery_state', q10)
        # RELIABLE on purpose (fusion_graph subscribes reliable), as mowgli_hardware.
        self._pub_imu = self.create_publisher(Imu, '~/imu/data_raw', q10)
        self._pub_odom = self.create_publisher(Odometry, '~/wheel_odom', q10)
        self._pub_ticks = self.create_publisher(WheelTick, '~/wheel_ticks', q10)
        self._pub_applied = self.create_publisher(TwistStamped, '~/cmd_vel_applied', q10)
        self._pub_dig = self.create_publisher(Bool, '~/dig_escalated', latched)
        self._pub_dig.publish(Bool(data=False))
        # Backend extension (no contract equivalent): commanded deck height.
        self._pub_height = self.create_publisher(Int16, '~/airseekers/cutter_height_mm', latched)
        self._pub_diag = self.create_publisher(String, '~/airseekers/state', q10)

        self.create_subscription(TwistStamped, '~/cmd_vel', self._on_cmd_vel,
                                 qos_profile_system_default)
        self.create_subscription(HighLevelStatus, '/behavior_tree_node/high_level_status',
                                 self._on_hl_status, q10)
        self.create_service(MowerControl, '~/mower_control', self._on_mower_control)
        self.create_service(EmergencyStop, '~/emergency_stop', self._on_emergency_stop)

        def refuse(text):
            def cb(_req, res):
                res.success = False
                res.message = text
                return res
            return cb
        self.create_service(Trigger, '~/reboot_board',
                            refuse('reboot_board is not supported by the Airseekers MCU'))
        self.create_service(SetBool, '~/set_firmware_debug',
                            refuse('firmware debug flags do not exist on the Airseekers MCU'))
        self.create_service(Trigger, '~/clear_dig_escalation',
                            refuse('no dig escalation to clear: the wheel-slip dig detector does '
                                   'not run on the Airseekers backend yet'))
        self._hlc = self.create_client(HighLevelControl, '/behavior_tree_node/high_level_control')

    def start(self) -> None:
        for t in self._threads:
            t.start()

    def shutdown(self) -> None:
        self._stop.set()
        self._wake()
        for t in self._threads:
            t.join(timeout=2.0)
        with self._lock:
            if self._ser is not None:
                # Polite exit: stop the wheels and the blade before letting go.
                self._write(P.speed_frame(0.0, 0.0))
                self._write(P.cutter_frame(False, 0, self.p('cutter_height_mm')))
                self._ser.close()
                self._ser = None

    # -------------------------------------------------------------- ROS input
    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        vx, wz = float(msg.twist.linear.x), float(msg.twist.angular.z)
        if not (math.isfinite(vx) and math.isfinite(wz)):
            self.get_logger().warning('Rejecting non-finite cmd_vel', throttle_duration_sec=5.0)
            return
        with self._lock:
            # Same fallback as the STM32/OpenMower bridges: motion before the BT
            # published a mode means somebody is driving -> treat as AUTONOMOUS.
            if self._hl_mode == HL_NULL and not self._latch.is_emergency and (vx or wz):
                self._hl_mode = HL_AUTONOMOUS
                self.get_logger().warning('cmd_vel before any high-level state; assuming AUTONOMOUS')
            if abs(vx) < float(self.p('min_linear_vel')):
                vx = 0.0
            self._cmd = (vx, wz)
            self._cmd_t = time.monotonic()
        self._wake()

    def _on_hl_status(self, msg: HighLevelStatus) -> None:
        with self._lock:
            self._hl_mode = int(msg.state)
            self._hl_t = time.monotonic()

    def _on_mower_control(self, req, res):
        with self._lock:
            requested = req.mow_enabled != 0
            self._mow_requested = requested and bool(self.p('mowing_enabled'))
            self._mow_direction = int(req.mow_direction)
            if requested and not self._mow_requested:
                self.get_logger().warning('MowerControl: blade SUPPRESSED (mowing_enabled=false)')
            self.get_logger().info('MowerControl: mow_enabled=%s direction=%d'
                                   % (self._mow_requested, self._mow_direction))
        self._wake()
        res.success = True
        return res

    def _on_emergency_stop(self, req, res):
        with self._lock:
            if req.emergency != 0:
                self.get_logger().warning('Emergency stop requested via service')
                self._latch.request_emergency()
                self._em_state = self._latch.state()
                # Stop NOW, not on the next tick -- but only if moving: any
                # SpeedData (zeros too) releases the drives' position hold,
                # which on the dock lets the wheels creep off the contacts.
                if self._tx.moving:
                    self._write(P.speed_frame(0.0, 0.0))
                    self._tx.applied = (0.0, 0.0)   # the next tick runs the stop sequence
                self._write(P.cutter_frame(False, 0, self.p('cutter_height_mm')))
                self._blade_sent = False
                res.success = True
            else:
                ok, why = self._latch.request_release()
                self._em_state = self._latch.state()
                if ok:
                    # The MCU's own lift/stop latch is released by an all-zero
                    # SensorInfoControl; it refuses while a trigger is held.
                    self._write(P.clear_latch_frame())
                self.get_logger().info('Emergency release: %s' % why)
                res.success = ok
        self._publish_emergency()
        self._wake()
        return res

    # ---------------------------------------------------------- MCU I/O loop
    def _wake(self) -> None:
        try:
            os.write(self._wake_w, b'x')
        except (BlockingIOError, OSError):
            pass

    def _write(self, data: bytes) -> None:
        if self._ser is None:
            return
        try:
            self._ser.write(data)
        except OSError as exc:
            self._close_mcu('write failed: %s' % exc)

    def _close_mcu(self, why: str) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None
            self.get_logger().error('MCU port closed: %s' % why)

    def _open_mcu(self) -> None:
        now = time.monotonic()
        if self._ser is not None or now - self._last_open_try < 2.0:
            return
        self._last_open_try = now
        try:
            self._ser = SerialPort(self.p('mcu_port'), self.p('mcu_baud'))
            self._parser = P.FrameParser()
            self._mcu_rx_t = now
            self.get_logger().info('opened MCU port %s' % self.p('mcu_port'))
        except OSError as exc:
            self.get_logger().warning('cannot open MCU port %s: %s (retrying)'
                                   % (self.p('mcu_port'), exc), throttle_duration_sec=10.0)

    def _mcu_loop(self) -> None:
        periodic = [
            [float(self.p('heartbeat_period_s')), self._heartbeat],
            [1.0 / float(self.p('control_rate')), self._control_tick],
            [1.0 / float(self.p('odom_rate')), self._publish_odom],
            [1.0 / float(self.p('status_rate')), self._publish_status_bundle],
        ]
        now = time.monotonic()
        due = [now + per for per, _ in periodic]
        logged = set()
        while not self._stop.is_set():
            try:
                timeout = max(0.0, min(due) - time.monotonic())
                with self._lock:
                    if self._ser is None:
                        self._open_mcu()
                    ser = self._ser
                fds = [self._wake_r] + ([ser.fileno()] if ser is not None else [])
                ready = select.select(fds, [], [], min(timeout, 0.05))[0]
                if self._wake_r in ready:
                    try:
                        os.read(self._wake_r, 4096)
                    except (BlockingIOError, OSError):
                        pass
                    with self._lock:
                        # Start from rest / react to a stop or an e-stop at
                        # once; while moving the fixed-rate tick streams.
                        if not self._tx.moving:
                            self._control_tick()
                if ser is not None and ser.fileno() in ready:
                    with self._lock:
                        if self._ser is ser:
                            self._rx(ser)
                with self._lock:
                    if (self._ser is not None and time.monotonic() - self._mcu_rx_t
                            > float(self.p('mcu_reopen_timeout_s'))):
                        self._close_mcu('no data for %.1f s' % self.p('mcu_reopen_timeout_s'))
                now = time.monotonic()
                for i, (per, fn) in enumerate(periodic):
                    if now >= due[i]:
                        due[i] += per
                        if due[i] <= now:
                            due[i] = now + per
                        with self._lock:
                            fn()
            except Exception:  # noqa: BLE001 - keep the link alive, log once per type
                key = traceback.format_exc().splitlines()[-1]
                if key not in logged:
                    logged.add(key)
                    self.get_logger().error('MCU loop error:\n%s' % traceback.format_exc())
                time.sleep(0.01)

    def _rx(self, ser: SerialPort) -> None:
        try:
            chunk = ser.read(4096)
        except OSError as exc:
            self._close_mcu('read failed: %s' % exc)
            return
        if not chunk:
            return
        for type_id, mod_id, payload in self._parser.feed(chunk):
            self._mcu_rx_t = time.monotonic()
            if type_id != P.TYPE_MCU_TO_HOST:
                continue
            self._dispatch(mod_id, payload)

    def _dispatch(self, mod_id: int, payload: bytes) -> None:
        now = time.monotonic()
        if mod_id == P.MOD_SPEED:
            s = P.decode_speed(payload)
            if s is not None:
                dt = now - self._meas_t if self._meas_t else 0.0
                self._meas = (s.linear, s.angular)
                self._meas_t = now
                if 0.0 < dt < 0.5:
                    ang = self._odom_angular()
                    self._ticks.feed(s.linear if not self._docked() else 0.0,
                                     ang if not self._docked() else 0.0, dt)
        elif mod_id == P.MOD_SENSOR:
            s = P.decode_sensor(payload)
            if s is not None:
                if s != self._sensor:
                    self._sensor = s
                    self._evaluate_emergency()
                self._sensor_t = now
        elif mod_id == P.MOD_BATTERY:
            b = P.decode_battery(payload)
            if b is not None:
                self._battery = b
                self._on_battery()
        elif mod_id == P.MOD_MOTORS:
            m = P.decode_motors(payload)
            if m is not None:
                self._motors = m
                self._motors_t = now
        elif mod_id == P.MOD_VERSION:
            v = P.decode_version(payload)
            if v is not None and v != self._versions:
                self._versions = v
                self.get_logger().info('MCU firmware: %s' % firmware_version_string(v))
        elif mod_id == P.MOD_LOG:
            self.get_logger().info('MCU: %s' % payload.decode('utf-8', 'replace').strip())

    # --------------------------------------------------------------- state
    def _link_alive(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return self._sensor_t > 0.0 and now - self._sensor_t <= float(self.p('mcu_status_timeout_s'))

    def _imu_alive(self, now: Optional[float] = None) -> bool:
        if not self.p('imu_enabled'):
            return True
        now = time.monotonic() if now is None else now
        return self._imu_t > 0.0 and now - self._imu_t <= float(self.p('imu_timeout_s'))

    def _docked(self) -> bool:
        return bool(self._battery is not None and self._battery.dock_ok)

    def _hl_fresh(self, now: float) -> bool:
        return now - self._hl_t <= float(self.p('high_level_status_timeout_s'))

    def _odom_angular(self) -> float:
        if self.p('odom_angular_source') == 'imu' and self._imu_alive():
            return self._imu_gyro_z
        return self._meas[1]

    def _evaluate_emergency(self) -> None:
        s = self._sensor
        trig = EmergencyInputs(
            stop=bool(s and s.stop), lift=bool(s and s.lift),
            tilt=self._tilt_critical,
            bumper=bool(s and s.any_bumper and self.p('bumper_is_emergency')))
        before = self._em_state
        self._em_state = self._latch.update(trig)
        if (before.active, before.latched, before.reason) != \
                (self._em_state.active, self._em_state.latched, self._em_state.reason):
            if self._em_state.latched:
                self.get_logger().warning('Emergency: %s' % self._em_state.reason)
            self._publish_emergency()

    def _on_battery(self) -> None:
        docked = self._docked()
        if docked != self._was_docked:
            self.get_logger().info('dock contacts %s' % ('LIVE (docked)' if docked else 'open'))
            self._was_docked = docked
            self._charge_sent_t = -1e9
        if docked and self.p('charge_on_dock') and \
                time.monotonic() - self._charge_sent_t >= float(self.p('charge_refresh_s')):
            self._write(P.charge_frame(True))
            self._charge_sent_t = time.monotonic()
        f = project_power(self._battery, float(self.p('charging_current_min_a')))
        stamp = self.get_clock().now().to_msg()
        pw = Power()
        pw.stamp = stamp
        pw.v_charge = f.v_charge
        pw.v_battery = f.v_battery
        pw.charge_current = f.charge_current
        pw.charger_enabled = f.charger_enabled
        pw.charger_status = f.charger_status
        self._pub_power.publish(pw)
        bs = BatteryState()
        bs.header.stamp = stamp
        bs.header.frame_id = 'base_link'
        bs.voltage = f.v_battery
        bs.current = f.battery_current
        bs.percentage = f.battery_percentage
        bs.temperature = float(self._battery.temperature_c)
        bs.power_supply_status = f.power_supply_status
        bs.present = True
        self._pub_battery.publish(bs)

    # ------------------------------------------------------------ control
    def _heartbeat(self) -> None:
        self._write(P.heartbeat_frame())

    def _control_tick(self) -> None:
        now = time.monotonic()
        link = self._link_alive(now)
        self._evaluate_emergency()
        emergency = self._latch.is_emergency
        cmd_fresh = now - self._cmd_t <= float(self.p('cmd_vel_timeout_s'))
        hl_fresh = self._hl_fresh(now)
        lin, ang = 0.0, 0.0
        note = ''
        if wheels_may_run(emergency, link, self._hl_mode, cmd_fresh):
            lin, ang = self._cmd
            yaw = self._imu_rpy[2] if (self._imu_rpy and self._imu_alive(now)) else None
            ang = self._shaper.shape(lin, ang, yaw, self._imu_gyro_z)
            lin = clamp(lin, float(self.p('max_mps')))
            ang = clamp(ang, float(self.p('max_angular_radps')))
            lin, ang, note = self._dock.filter(self._docked(), self._hl_mode, hl_fresh, lin, ang)
        else:
            self._shaper.reset()
        if note != self._dock_note:
            if note:
                self.get_logger().info(note)
            self._dock_note = note
        for f_lin, f_ang in self._tx.tick(now, lin, ang):
            self._write(P.speed_frame(f_lin, f_ang))
        self._publish_applied()
        self._drive_blade(now, emergency, link, hl_fresh)

    def _drive_blade(self, now: float, emergency: bool, link: bool, hl_fresh: bool) -> None:
        allowed = blade_may_run(BladeInputs(
            self._mow_requested, bool(self.p('mowing_enabled')), emergency, link,
            self._hl_mode, hl_fresh, update_maintenance_active()))
        height = int(self.p('cutter_height_mm'))
        reverse = False  # the Tron blade is only ever driven forward (vendor behaviour)
        if allowed != self._blade_sent or (allowed and self._height_dirty):
            self.get_logger().info('Blade %s (height %d mm)' % ('ON' if allowed else 'OFF', height))
            self._write(P.cutter_frame(allowed, int(self.p('blade_speed_percent')), height,
                                       reverse))
            self._blade_sent = allowed
            self._height_dirty = False
            # A single OFF frame can be lost and nothing acknowledges it:
            # repeat OFF on the next ticks.
            self._blade_off_left = 0 if allowed else max(0, int(self.p('blade_off_frames')) - 1)
        elif self._blade_off_left > 0:
            self._write(P.cutter_frame(False, 0, height))
            self._blade_off_left -= 1
        elif self._height_dirty:
            self._write(P.cutter_frame(False, 0, height))
            self._height_dirty = False
        if self._ser is None:
            self._blade_dir_text = 'unknown'
        elif not allowed:
            self._blade_dir_text = 'off'
        else:
            self._blade_dir_text = 'forward'
        if height != self._height_published:
            self._height_published = height
            self._pub_height.publish(Int16(data=height))

    def _publish_applied(self) -> None:
        a = self._tx.applied
        if a == self._applied_published:
            return
        self._applied_published = a
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(a[0]), float(a[1])
        self._pub_applied.publish(msg)

    # ---------------------------------------------------------- publishers
    def _publish_odom(self) -> None:
        now = time.monotonic()
        if not self._link_alive(now) or now - self._meas_t > float(self.p('mcu_status_timeout_s')):
            return
        docked = self._docked()
        vx = 0.0 if docked else float(self._meas[0])
        wz = 0.0 if docked else float(self._odom_angular())
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz
        cov = [0.0] * 36
        cov[0] = 1e-6 if docked else 0.01
        cov[7] = 1e-4
        cov[14] = cov[21] = cov[28] = 1e6
        cov[35] = 1e-6 if docked else 9e-4
        msg.twist.covariance = cov
        self._pub_odom.publish(msg)
        t = WheelTick()
        t.stamp = stamp
        t.wheel_tick_factor = float(self._ticks.ticks_per_meter)
        t.valid_wheels = WheelTick.WHEEL_VALID_RL | WheelTick.WHEEL_VALID_RR
        t.wheel_ticks_rl, t.wheel_ticks_rr = self._ticks.ticks()
        t.wheel_direction_rl, t.wheel_direction_rr = self._ticks.direction
        self._pub_ticks.publish(t)

    def _publish_emergency(self) -> None:
        e = self._em_state
        msg = Emergency()
        msg.stamp = self.get_clock().now().to_msg()
        msg.active_emergency = bool(e.active)
        msg.latched_emergency = bool(e.latched)
        msg.lift_warning = False
        msg.lift_duration_sec = 0.0
        msg.reason = e.reason
        self._pub_emergency.publish(msg)

    def _publish_status_bundle(self) -> None:
        now = time.monotonic()
        link = self._link_alive(now)
        msg = Status()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mower_status = Status.MOWER_STATUS_OK if link else Status.MOWER_STATUS_INITIALIZING
        msg.reset_cause = Status.RESET_CAUSE_UNKNOWN
        msg.reset_cause_name = 'unknown'
        msg.raspberry_pi_power = False
        msg.is_charging = self._docked()
        msg.rain_detected = bool(self._sensor and self._sensor.rain)
        msg.esc_power = bool(self._mow_requested or self._blade_sent)
        msg.mow_enabled = bool(self._mow_requested)
        msg.firmware_debug_enabled = False
        msg.dig_escalated = False
        msg.mower_esc_status = 1 if self._blade_sent else 0
        cut = self._motors.get('cutter')
        if cut is not None:
            msg.mower_esc_current = float(cut.current_a)
            msg.mower_motor_temperature = float(cut.temperature_c)
            msg.mower_esc_temperature = float(cut.temperature_c)
            msg.mower_motor_rpm = blade_rpm(self._motors)
            age = now - self._motors_t
            msg.blade_status_stamp = (self.get_clock().now()
                                      - Duration(seconds=age)).to_msg()
        msg.blade_requested_direction = self._blade_dir_text
        msg.firmware_version = firmware_version_string(self._versions)
        msg.firmware_protocol_version = 0
        msg.firmware_compatible = firmware_compatible(link, self._versions, self._imu_alive(now))
        self._pub_status.publish(msg)
        self._publish_emergency()
        self._pub_diag.publish(String(data=json.dumps({
            'link_alive': link, 'imu_alive': self._imu_alive(now), 'docked': self._docked(),
            'hl_mode': self._hl_mode, 'emergency': self._em_state.reason,
            'tilt': self._tilt.band(now), 'dock_guard': self._dock_note,
            'applied': list(self._tx.applied), 'frames_ok': self._parser.frames_ok,
            'frames_bad': self._parser.frames_bad, 'imu_bias_ready': self._bias.ready,
            'keys': self._keys_state, 'imu_forwarded': self._imu_fwd_count,
        })))

    # ------------------------------------------------------------ IMU loop
    def _imu_loop(self) -> None:
        parser = WitParser()
        port = None
        rp = float(self.p('orientation_rp_covariance'))
        o_cov = _diag3(rp, rp, self.p('orientation_yaw_covariance'))
        av = float(self.p('angular_velocity_covariance'))
        a_cov = _diag3(av, av, av)
        la = float(self.p('linear_acceleration_covariance'))
        l_cov = _diag3(la, la, la)
        last_try = -1e9
        while not self._stop.is_set():
            if port is None:
                if time.monotonic() - last_try < 2.0:
                    self._stop.wait(0.2)
                    continue
                last_try = time.monotonic()
                try:
                    port = SerialPort(self.p('imu_port'), self.p('imu_baud'))
                    self.get_logger().info('opened IMU port %s' % self.p('imu_port'))
                except OSError as exc:
                    self.get_logger().warning('cannot open IMU port %s: %s' % (self.p('imu_port'), exc),
                                           throttle_duration_sec=10.0)
                    continue
            try:
                if not select.select([port.fileno()], [], [], 0.5)[0]:
                    continue
                data = port.read(4096)
            except OSError as exc:
                self.get_logger().error('IMU read failed: %s; reopening' % exc)
                port.close()
                port = None
                continue
            for ftype in parser.feed(data):
                if ftype == TYPE_ANGLE and parser.sample.complete:
                    try:
                        self._on_imu_sample(parser.sample, o_cov, a_cov, l_cov)
                    except Exception:  # noqa: BLE001
                        self.get_logger().error('IMU sample failed:\n%s' % traceback.format_exc(),
                                                throttle_duration_sec=10.0)
        if port is not None:
            port.close()

    def _on_imu_sample(self, s, o_cov, a_cov, l_cov) -> None:
        now = time.monotonic()
        quat = s.quat if s.quat is not None else euler_to_quaternion(*s.rpy)
        roll, pitch, yaw = s.rpy
        with self._lock:
            moving = (self._tx.moving or abs(self._meas[0]) > 0.01 or abs(self._meas[1]) > 0.02
                      or self._tx.stopping)
            event = self._cal.step(now, self._docked(), moving)
            if event:
                self.get_logger().info('IMU calibration %s' % event)
            if self._bias.collecting and self._bias.feed(s.acc, s.gyro):
                b = self._bias.bias
                self.get_logger().info('IMU bias: gyro (%.4f, %.4f, %.4f) rad/s'
                                       % (b.gx, b.gy, b.gz))
            acc, gyro = self._bias.apply(s.acc, s.gyro)
            self._imu_t = now
            self._imu_rpy = (roll, pitch, yaw)
            self._imu_gyro_z = gyro[2]
            # Tilt monitor: critical band = emergency source.
            if self.p('tilt_emergency_enabled'):
                r, p = quat_to_roll_pitch(*quat)
                band = self._tilt.update(now, r, p)
                crit = band == 'critical'
                if crit != self._tilt_critical:
                    self._tilt_critical = crit
                    self._evaluate_emergency()
            # Forward to the MCU (it computes `lift` from this attitude).
            period = 1.0 / max(1.0, float(self.p('forward_imu_rate')))
            if self.p('forward_imu') and now >= self._imu_fwd_t:
                # Scheduled (not "at least period since the last"), so a 100 Hz
                # sensor yields ~45 Hz instead of every third sample (33 Hz).
                self._imu_fwd_t = max(self._imu_fwd_t + period, now - period)
                fr = math.remainder(roll + math.radians(self.p('forward_imu_roll_offset_deg')),
                                    2.0 * math.pi)
                fp = math.remainder(pitch + math.radians(self.p('forward_imu_pitch_offset_deg')),
                                    2.0 * math.pi)
                self._write(P.imu_frame(fr, fp, yaw, s.acc, s.gyro))
                self._imu_fwd_count += 1
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p('imu_frame_id')
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat
        msg.orientation_covariance = o_cov
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.angular_velocity_covariance = a_cov
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.linear_acceleration_covariance = l_cov
        self._pub_imu.publish(msg)

    # ----------------------------------------------------------- keys loop
    def _keys_loop(self) -> None:
        code_map = {
            keys_logic.VENDOR_KEY_POWER: keys_logic.KEY_POWER_SHORT,
            keys_logic.VENDOR_KEY_GO_DOCKING: keys_logic.KEY_GO_DOCKING,
            keys_logic.VENDOR_KEY_POWER_LONG: keys_logic.KEY_POWER_LONG,
            keys_logic.VENDOR_KEY_WORK_PAUSE: keys_logic.KEY_WORKING_OR_PAUSE,
            keys_logic.VENDOR_KEY_DOCK_AND_PAUSE: keys_logic.KEY_DOCK_AND_PAUSE,
        }
        sm = keys_logic.KeyStateMachine(code_map)
        cfg = keys_logic.ActionConfig(
            power_short_reset_emergency=bool(self.p('power_short_resets_emergency')))
        fd = None
        buf = b''
        last_try = -1e9
        while not self._stop.is_set():
            if fd is None:
                if time.monotonic() - last_try < 5.0:
                    self._stop.wait(0.5)
                    continue
                last_try = time.monotonic()
                path = self.p('keys_device')
                if not path:
                    try:
                        with open('/proc/bus/input/devices', encoding='utf-8') as fh:
                            path, _why = keys_logic.find_device(fh.read())
                    except OSError:
                        path = None
                    path = path or keys_logic.VENDOR_DEVICE_PATH
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    self._keys_state = 'reading %s' % path
                    self.get_logger().info('panel keys: %s' % path)
                except OSError as exc:
                    if self._keys_state != 'unavailable':
                        self.get_logger().warning('panel keys unavailable (%s): %s' % (path, exc))
                    self._keys_state = 'unavailable'
                    continue
            try:
                if not select.select([fd], [], [], 0.5)[0]:
                    continue
                data = os.read(fd, 24 * 16)
                if not data:
                    raise OSError('EOF')
            except BlockingIOError:
                continue
            except OSError as exc:
                self.get_logger().warning('panel keys read failed: %s' % exc)
                os.close(fd)
                fd = None
                sm.reset()
                continue
            events, buf = keys_logic.decode_events(buf + data)
            for t, etype, code, value in events:
                for bit in sm.feed(etype, code, value, t):
                    with self._lock:
                        hl = self._hl_mode if self._hl_t > 0 else None
                    for act in keys_logic.map_key(bit, hl, cfg):
                        self._key_action(act)
        if fd is not None:
            os.close(fd)

    def _key_action(self, act) -> None:
        self.get_logger().info('panel key: %s' % (act.note or act.kind))
        if act.kind == 'clear_estop':
            with self._lock:
                ok, why = self._latch.request_release()
                self._em_state = self._latch.state()
                if ok:
                    self._write(P.clear_latch_frame())
            self.get_logger().info('panel key emergency release: %s' % why)
            self._publish_emergency()
        elif act.kind == 'hlc':
            if not self._hlc.service_is_ready():
                self.get_logger().warning('high_level_control unavailable; key dropped')
                return
            req = HighLevelControl.Request()
            req.command = int(act.arg)
            self._hlc.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirseekersBridge()
    node.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
