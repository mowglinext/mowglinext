"""ROS 2 node wrapping the Unicore UM960 GNSS/RTK receiver.

Publishes
    /fix          sensor_msgs/msg/NavSatFix   (status from NMEA GGA quality + GSA/BESTNAV mode)
    /vel          geometry_msgs/msg/TwistStamped
    /heading      std_msgs/msg/Float32        (degrees, true north, from GPHPR/VTG/RMC)
    /heading_valid std_msgs/msg/Bool          (true only while publish_rtk_heading is set AND
                                               the latest GPHPR is < heading_timeout_s old AND
                                               reports a fixed RTK baseline; false by default:
                                               the UM960 is single-antenna)
    /fix_status   std_msgs/msg/String         (human-readable quality/solution summary)
    /nmea         std_msgs/msg/String         (raw ASCII sentences, optional)
    /gps/corrections std_msgs/msg/String      (1 Hz JSON correction-source diagnostics)
    /ntrip/status std_msgs/msg/String         (1 Hz JSON NTRIP client stats, latched)

Serves
    /mower_gps_node/set_lora  mower_interfaces/srv/SetLoRa (vendor LoRa base pairing)

RTK corrections (``rtcm_source``; only one source ever feeds RTCM to the UM960):
    auto   (default) NTRIP while the caster delivers valid RTCM, otherwise the LoRa
           base. NTRIP is used only when enabled in /userdata/ros2/ntrip.yaml
           (``ntrip_settings_file``), the ``ntrip_enabled`` parameter or the vendor
           ntrip.yaml.
    lora   the vendor base station sends RTCM over LoRa to the rtk_rover board, which
           feeds the UM960 itself; the host only pairs it (set_lora).
    ntrip  a host NTRIP client streams RTCM from a caster into the same serial port
           and uploads the rover GGA; the LoRa base is locked out even while the
           caster is unreachable.
    none   no corrections; nothing is ever written except ``config_commands``.
The legacy ``correction_source`` (none|lora|ntrip, "" = auto) is used when
``rtcm_source`` is empty; the GUI NTRIP switch still drives it.

The correction parameters (``correction_source``, ``ntrip_*``, ...) can be changed at
runtime (``ros2 param set``, or the GUI settings binding): a change restarts the
corrections - the NTRIP client is stopped and, for ``ntrip``, reconnected with the
new caster - without touching the serial port. Changes arriving together (one
SetParameters request) are applied once, ``RECONFIGURE_DEBOUNCE_S`` after the last.

Hardware: Unicore UM960 on /dev/serial_rtk (ttyS4, UART4 @ 0xfeb70000), 115200 8N1.
"""

import json
import math
import threading
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

try:  # absent from the plain-Python test shim
    from rclpy.qos import DurabilityPolicy
except ImportError:  # pragma: no cover - depends on the environment
    DurabilityPolicy = None
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String

from .parsers import (
    GSA_FIX_TYPE,
    SOL_COMPUTED,
    NmeaParser,
    Um960StreamSplitter,
    bestnav_position_type_name,
    bestnav_solution_status_name,
    bestnav_velocity_type_name,
    hdop_to_covariance,
    nmea_checksum_ok,
    sigma_to_covariance,
)
from . import lora
from .ntrip_client import (
    NETMODE_DEVICES,
    STATE_OFF,
    VENDOR_NTRIP_FILE,
    NtripClient,
    NtripConfig,
    load_vendor_ntrip_file,
    make_gga,
)
from .ntrip_client import STATE_CONNECTED, STATE_STREAMING
from .ntrip_settings import DEFAULT_SETTINGS_FILE, file_mtime, load_settings, save_settings
from .serial_port import SerialError, SerialPort

try:  # rcl_interfaces ships with rclpy; the plain-Python test shim lacks it.
    from rcl_interfaces.msg import SetParametersResult
except ImportError:  # pragma: no cover - depends on the environment
    class SetParametersResult:  # type: ignore[no-redef]
        def __init__(self, successful: bool = True, reason: str = "") -> None:
            self.successful = successful
            self.reason = reason

try:  # MowgliNext: the public /gps/status contract (optional so the plain tests run).
    from mowgli_interfaces.msg import GnssStatus
except ImportError:  # pragma: no cover - depends on the workspace
    GnssStatus = None
from .gnss_status import gnss_status_fields

try:  # mower_interfaces is optional: without it the set_lora service is not offered.
    from mower_interfaces.srv import SetLoRa
except ImportError:  # pragma: no cover - depends on the workspace
    SetLoRa = None

CORRECTION_SOURCES = ("none", "lora", "ntrip")
RTCM_SOURCES = ("auto", "ntrip", "lora", "none")
# Live changes of these are also written to ntrip_settings_file (GUI persistence).
PERSISTED_PARAMS = {
    "ntrip_host": "host", "ntrip_port": "port", "ntrip_mountpoint": "mountpoint",
    "ntrip_user": "user", "ntrip_password": "password", "ntrip_enabled": "enabled",
    "correction_source": "enabled", "rtcm_source": "enabled",
}
# Parameters read by _init_correction_params(): changing one at runtime restarts the
# corrections.
CORRECTION_PARAMS = frozenset({
    "correction_source", "ntrip_host", "ntrip_port", "ntrip_mountpoint", "ntrip_user",
    "ntrip_password", "ntrip_gga_interval_s", "ntrip_version", "ntrip_config_file",
    "ntrip_no_data_timeout_s", "ntrip_reconnect_max_s", "ntrip_fallback_lat",
    "ntrip_fallback_lon", "ntrip_keepalive_s", "forward_rtcm_without_board_ack",
    "ntrip_netmode", "corrections_stale_s", "lora_pairing_enabled",
    "rtcm_source", "ntrip_enabled", "ntrip_settings_file", "ntrip_fallback_grace_s",
    "ntrip_validate_rtcm",
})
RECONFIGURE_DEBOUNCE_S = 0.3
# A raw receiver GGA older than this is not uploaded; a synthesized one is used.
GGA_MAX_AGE_S = 5.0

# NMEA sentence suffixes that carry a position. feed() returns the full talker id
# ("GNGGA", "GPGGA", ...), so match on the suffix.
FIX_BEARING_SUFFIXES = ("GGA", "RMC")
# Unicore ASCII position logs use the whole id as the talker.
FIX_BEARING_IDS = frozenset({"BESTNAV", "BESTNAVA"})

# Unicore BESTNAV position types that are an integer/fixed solution. Mapped to NMEA
# GGA quality 4; every other position type is reported as quality 1.
FIXED_POS_TYPES = frozenset({1, 10, 11, 12, 17, 19})

# NMEA GGA quality -> sensor_msgs/NavSatStatus.STATUS_*.
# Jazzy only knows NO_FIX(-1) / FIX(0) / SBAS_FIX(1) / GBAS_FIX(2). Follow the
# nmea_navsat_driver convention: differential -> SBAS_FIX, RTK fixed *and* float ->
# GBAS_FIX (the gate distinguishes them by position covariance), dead reckoning -> FIX.
STATUS_BY_QUALITY = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_FIX,
    2: NavSatStatus.STATUS_SBAS_FIX,
    4: NavSatStatus.STATUS_GBAS_FIX,
    5: NavSatStatus.STATUS_GBAS_FIX,
    6: NavSatStatus.STATUS_FIX,
}


class Um960Node(Node):
    """Read the UM960 serial stream and publish ROS navigation messages.

    ``**kwargs`` is forwarded to ``rclpy.node.Node``, which lets callers pass
    ``parameter_overrides`` (used by the tests to point the node at a pty).
    """

    _ntrip_active = False
    _ntrip_switches = 0
    _rtcm_idle_bytes = 0
    _persist_error = ""

    def __init__(self, **kwargs) -> None:
        super().__init__("um960_node", **kwargs)

        self.declare_parameter("port", "/dev/serial_rtk")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("vel_topic", "/vel")
        self.declare_parameter("heading_topic", "/heading")
        self.declare_parameter("heading_valid_topic", "/heading_valid")
        # /heading_valid may only go true on a confirmed dual-antenna receiver (the UM960 is
        # single-antenna: any GPHPR heading it gives is motion-derived or invalid).
        self.declare_parameter("publish_rtk_heading", False)
        # A GPHPR heading older than this is dropped (/heading falls back to the track).
        self.declare_parameter("heading_timeout_s", 1.0)
        self.declare_parameter("fix_status_topic", "/fix_status")
        # MowgliNext /gps/status (mowgli_interfaces/GnssStatus); "" = off.
        self.declare_parameter("status_topic", "")
        self.declare_parameter("nmea_topic", "/nmea")
        self.declare_parameter("publish_nmea", False)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("serial_timeout_s", 0.2)
        self.declare_parameter("fix_timeout_s", 2.0)
        self.declare_parameter("sane_lat_limit", 90.0)
        self.declare_parameter("sane_lon_limit", 180.0)
        # Receiver configuration written on (re)connect; empty by default so the
        # driver never fights with a receiver that was already configured.
        self.declare_parameter("config_commands", [])
        # --- RTK corrections ---
        # "" = automatic: ntrip if the vendor ntrip.yaml says ntrip_enable: true,
        # otherwise lora (the vendor default).
        self.declare_parameter("correction_source", "")
        # auto|ntrip|lora|none; "" = derive from correction_source ("" there = auto).
        self.declare_parameter("rtcm_source", "")
        self.declare_parameter("ntrip_enabled", False)
        self.declare_parameter("ntrip_settings_file", DEFAULT_SETTINGS_FILE)
        self.declare_parameter("ntrip_status_topic", "/ntrip/status")
        # auto: NTRIP keeps the receiver this long after its last valid RTCM frame
        # before the LoRa base gets it back.
        self.declare_parameter("ntrip_fallback_grace_s", 5.0)
        self.declare_parameter("ntrip_validate_rtcm", True)
        self.declare_parameter("ntrip_host", "")
        self.declare_parameter("ntrip_port", 2101)
        self.declare_parameter("ntrip_mountpoint", "")
        self.declare_parameter("ntrip_user", "")
        self.declare_parameter("ntrip_password", "")
        self.declare_parameter("ntrip_gga_interval_s", 10.0)
        self.declare_parameter("ntrip_version", 2)
        self.declare_parameter("ntrip_config_file", VENDOR_NTRIP_FILE)
        self.declare_parameter("ntrip_no_data_timeout_s", 20.0)
        self.declare_parameter("ntrip_reconnect_max_s", 30.0)
        # Position for the caster before the receiver has one (0/0 = unset).
        self.declare_parameter("ntrip_fallback_lat", 0.0)
        self.declare_parameter("ntrip_fallback_lon", 0.0)
        # Network-RTK handshake with the rtk_rover board (ntrip mode only): RTKRESET +
        # $GNRTK,ON*69 on the off->on transition, $GNRTK,ON*69 every keepalive period
        # (the board falls back to LoRa after ~6 s without it).
        self.declare_parameter("ntrip_keepalive_s", 4.0)
        # RTCM is only forwarded while the board reports $GNRTK,ON. true = forward
        # anyway (bench tests without the board).
        self.declare_parameter("forward_rtcm_without_board_ack", False)
        # "" / "auto" = any interface, "4g" = usb0, "wifi" = wlan0 (vendor netmode).
        self.declare_parameter("ntrip_netmode", "")
        self.declare_parameter("corrections_topic", "/gps/corrections")
        self.declare_parameter("corrections_stale_s", 10.0)
        self.declare_parameter("set_lora_service", "/mower_gps_node/set_lora")
        # set_lora is a logged stub unless this is true (see docs/um960.md).
        self.declare_parameter("lora_pairing_enabled", False)

        self.port_name = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout_s").value)
        self.fix_timeout = float(self.get_parameter("fix_timeout_s").value)
        self.sane_lat = float(self.get_parameter("sane_lat_limit").value)
        self.sane_lon = float(self.get_parameter("sane_lon_limit").value)
        self.config_commands = [str(c) for c in self.get_parameter("config_commands").value]
        publish_nmea = bool(self.get_parameter("publish_nmea").value)
        self.publish_rtk_heading = bool(self.get_parameter("publish_rtk_heading").value)
        self.heading_timeout = float(self.get_parameter("heading_timeout_s").value)
        self._hpr_logged = False
        if not self.publish_rtk_heading:
            self.get_logger().info(
                "publish_rtk_heading: false -> /heading_valid stays false: the UM960 is a "
                "single-antenna receiver, so a GPHPR heading is motion-derived or invalid, "
                "not an absolute heading. Set it true only on a confirmed dual-antenna "
                "receiver (docs/um960.md)."
            )
        self._init_correction_params()
        self.nmea_topic = str(self.get_parameter("nmea_topic").value)

        fix_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.fix_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter("fix_topic").value), fix_qos
        )
        self.vel_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("vel_topic").value), fix_qos
        )
        self.heading_pub = self.create_publisher(
            Float32, str(self.get_parameter("heading_topic").value), fix_qos
        )
        self.heading_valid_pub = self.create_publisher(
            Bool, str(self.get_parameter("heading_valid_topic").value), fix_qos
        )
        self.fix_status_pub = self.create_publisher(
            String, str(self.get_parameter("fix_status_topic").value), fix_qos
        )
        self.nmea_pub = (
            self.create_publisher(String, self.nmea_topic, 10) if publish_nmea else None
        )
        self.corrections_pub = self.create_publisher(
            String, str(self.get_parameter("corrections_topic").value), 10
        )
        status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE)
        if DurabilityPolicy is not None:
            status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched
        self.ntrip_status_pub = self.create_publisher(
            String, str(self.get_parameter("ntrip_status_topic").value), status_qos
        )

        status_topic = str(self.get_parameter("status_topic").value or "")
        self.gnss_status_pub = None
        if status_topic and GnssStatus is not None:
            self.gnss_status_pub = self.create_publisher(GnssStatus, status_topic, 10)
        elif status_topic:
            self.get_logger().warning("status_topic set but mowgli_interfaces is missing")
        self._observation_sequence = 0

        self.nmea = NmeaParser()
        self.splitter = Um960StreamSplitter()
        self._lock = threading.Lock()
        # Set by destroy_node() so the reader thread stops reconnecting instead of
        # holding the device open after the node is gone.
        self._stopping = threading.Event()
        self._port: Optional[SerialPort] = None
        self._connected = False
        self._last_fix_time = 0.0
        self._last_publish = 0.0
        self._unhealthy_streak = 0
        # Cumulative bytes handed to the parsers; handy for diagnostics and tests.
        self._bytes_read = 0
        # Serialises every write to the port (config, RTCM, LoRa pairing).
        self._write_lock = threading.Lock()
        self._serial_bytes_written = 0
        self._last_gga: Optional[str] = None
        self._last_gga_time = 0.0
        self._rover_nrtk: Optional[str] = None      # "ON"/"OFF" from $GNRTK
        self._nrtk_on_sent = 0.0                    # monotonic time of the last ON
        self._nrtk_lock = threading.Lock()
        self._rtcm_held_bytes = 0
        self._rtcm_held_warned = 0.0
        self._nrtk_thread: Optional[threading.Thread] = None
        self._nrtk_stop = threading.Event()         # stops the current keepalive thread
        self._ntrip_active = False                  # host currently owns the RTCM input
        self._ntrip_switches = 0                    # auto: NTRIP <-> LoRa hand-overs
        self._rtcm_idle_bytes = 0                   # valid RTCM not written (not active)
        self._pending_persist: Dict[str, object] = {}
        self._persist_error = ""
        self._lora_pairing: Optional[str] = None    # "addr,channel" from $GNCON
        self.ntrip: Optional[NtripClient] = None
        # Runtime reconfiguration of the corrections (see _on_set_parameters).
        self._reconfigure_lock = threading.Lock()
        self._reconfigure_timer: Optional[threading.Timer] = None
        self.corrections_reconfigured = 0           # count, for diagnostics/tests
        self.set_lora_srv = None
        if SetLoRa is not None and hasattr(self, "create_service"):
            self.set_lora_srv = self.create_service(
                SetLoRa, str(self.get_parameter("set_lora_service").value), self._on_set_lora
            )

        self.get_logger().info(
            "UM960 driver starting on %s @ %d baud (frame_id=%s)"
            % (self.port_name, self.baud, self.frame_id)
        )
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="um960_serial", daemon=True
        )
        self._reader_thread.start()
        # Publishing happens on the ROS executor thread so QoS/timers behave normally.
        self._timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish_tick)
        self._corrections_timer = self.create_timer(1.0, self.publish_corrections)
        self._ntrip_status_timer = self.create_timer(1.0, self.publish_ntrip_status)
        self._start_corrections()
        if hasattr(self, "add_on_set_parameters_callback"):
            self.add_on_set_parameters_callback(self._on_set_parameters)

    # -- corrections: lifecycle ----------------------------------------------------
    def _start_corrections(self, reason: str = "", runtime: bool = False) -> None:
        """Start the NTRIP client + rover keepalive for ``correction_source=ntrip``.

        ``runtime``: a reconfiguration while the port may already be open, so the
        off->on handshake _open_port() does on connect is done here instead.
        """
        self._ntrip_active = False
        if not self._ntrip_wanted:
            self.get_logger().info("corrections: %s%s" % (self.correction_source, reason))
            return
        self.ntrip = NtripClient(
            self.ntrip_config, self._write_rtcm, self._gga_for_caster, log=self._ntrip_log,
        )
        self.get_logger().info(
            "corrections: rtcm_source=%s, NTRIP %s (GGA every %.0fs, config from %s)%s"
            % (self.rtcm_source, self.ntrip_config.describe(),
               self.ntrip_config.gga_interval_s, self.ntrip_config_source, reason)
        )
        port = self._port
        if self.rtcm_source == "ntrip":
            self._ntrip_active = True
        if self.rtcm_source == "ntrip" and runtime and port is not None and port.is_open:
            # Switched at runtime with the port already open: _open_port() will not
            # run, so do its off->on handshake here.
            self._rover_nrtk = None
            self._nrtk_enable("correction source changed")
        self.ntrip.start()
        self._nrtk_stop = threading.Event()
        self._nrtk_thread = threading.Thread(
            target=self._nrtk_keepalive_loop, args=(self._nrtk_stop,), name="um960_nrtk",
            daemon=True,
        )
        self._nrtk_thread.start()

    def _stop_corrections(self) -> None:
        """Stop the NTRIP client and the keepalive thread (port stays open)."""
        client, self.ntrip = self.ntrip, None
        if client is not None:
            client.stop()
        self._nrtk_stop.set()
        thread, self._nrtk_thread = self._nrtk_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Validate correction parameter changes and schedule a reconfiguration.

        rclpy (Jazzy) calls this before the values are stored, so the restart runs
        a little later on a timer thread and reads them back with get_parameter().
        """
        touched = False
        for param in params:
            if param.name == "correction_source":
                value = str(param.value or "").strip().lower()
                if value not in CORRECTION_SOURCES + ("", "auto"):
                    return SetParametersResult(
                        successful=False,
                        reason="correction_source must be one of none|lora|ntrip (or empty)",
                    )
            if param.name == "rtcm_source":
                value = str(param.value or "").strip().lower()
                if value not in RTCM_SOURCES + ("",):
                    return SetParametersResult(
                        successful=False,
                        reason="rtcm_source must be one of auto|ntrip|lora|none (or empty)",
                    )
            if param.name in CORRECTION_PARAMS:
                touched = True
        for param in params:
            if param.name in PERSISTED_PARAMS:
                self._pending_persist[param.name] = param.value
        if touched and not self._stopping.is_set():
            with self._reconfigure_lock:
                if self._reconfigure_timer is not None:
                    self._reconfigure_timer.cancel()
                timer = threading.Timer(RECONFIGURE_DEBOUNCE_S, self._reconfigure_corrections)
                timer.daemon = True
                self._reconfigure_timer = timer
                timer.start()
        return SetParametersResult(successful=True)

    def _reconfigure_corrections(self) -> None:
        """Re-read the correction parameters and restart the corrections."""
        with self._reconfigure_lock:
            self._reconfigure_timer = None
            if self._stopping.is_set():
                return
            previous = self.correction_source
            self._stop_corrections()
            self._persist_settings()
            self._init_correction_params()
            self._start_corrections(" (reconfigured; was %s)" % previous, runtime=True)
            self.corrections_reconfigured += 1

    # -- corrections: configuration ------------------------------------------------
    def _init_correction_params(self) -> None:
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        legacy = str(get("correction_source") or "").strip().lower()
        mode = str(get("rtcm_source") or "").strip().lower()
        if not mode:
            mode = legacy or "auto"
        host = str(get("ntrip_host") or "").strip()
        port = int(get("ntrip_port") or 2101)
        mount = str(get("ntrip_mountpoint") or "").strip()
        user = str(get("ntrip_user") or "")
        password = str(get("ntrip_password") or "")
        enabled = bool(get("ntrip_enabled"))
        self.ntrip_config_source = "parameters"
        vendor_file = str(get("ntrip_config_file") or "")
        vendor = None
        netmode = str(get("ntrip_netmode") or "").strip().lower()
        if vendor_file and (not host or mode == "auto"):
            vendor = load_vendor_ntrip_file(vendor_file)
        if vendor is not None:
            if not host and vendor["host"]:
                host = str(vendor["host"])
                port = int(vendor["port"]) or port
                mount = mount or str(vendor["mountpoint"])
                user = user or str(vendor["user"])
                password = password or str(vendor["password"])
                netmode = netmode or str(vendor.get("netmode") or "").lower()
                self.ntrip_config_source = vendor_file
            enabled = enabled or bool(vendor["enable"])
        # The owner's settings file (written from the GUI) wins over both.
        self.ntrip_settings_file = str(get("ntrip_settings_file") or "")
        self._settings_mtime = file_mtime(self.ntrip_settings_file) \
            if self.ntrip_settings_file else None
        settings = load_settings(self.ntrip_settings_file)
        if settings is not None:
            host, port = str(settings["host"]), int(settings["port"])
            mount, user = str(settings["mountpoint"]), str(settings["user"])
            password = str(settings["password"])
            enabled = bool(settings["enabled"])
            self.ntrip_config_source = self.ntrip_settings_file
        if mode not in RTCM_SOURCES:
            self.get_logger().error(
                "rtcm_source %r unknown (expected auto|ntrip|lora|none); using none" % mode
            )
            mode = "none"
        if mode == "ntrip" and (not host or not mount):
            self.get_logger().error(
                "rtcm_source=ntrip but the NTRIP host/mountpoint are empty "
                "(parameters, %s, %s); corrections disabled"
                % (self.ntrip_settings_file or "no settings file", vendor_file or "no vendor file")
            )
            mode = "none"
        if mode == "auto" and enabled and (not host or not mount):
            self.get_logger().warning(
                "NTRIP enabled but host/mountpoint are empty; using the LoRa base only"
            )
        self.rtcm_source = mode
        self.ntrip_enabled = enabled
        # Whether an NTRIP client runs at all.
        self._ntrip_wanted = mode == "ntrip" or (mode == "auto" and enabled
                                                 and bool(host) and bool(mount))
        if netmode not in NETMODE_DEVICES:
            self.get_logger().warning(
                "ntrip_netmode %r unknown (4g|wifi|auto); socket left unbound" % netmode
            )
            netmode = ""
        self.ntrip_netmode = netmode
        self.ntrip_config = NtripConfig(
            host, port, mount, user, password,
            version=int(get("ntrip_version") or 2),
            gga_interval_s=float(get("ntrip_gga_interval_s")),
            no_data_timeout_s=float(get("ntrip_no_data_timeout_s")),
            reconnect_max_s=float(get("ntrip_reconnect_max_s")),
            bind_device=NETMODE_DEVICES[netmode],
            validate_rtcm=bool(get("ntrip_validate_rtcm")),
        )
        self.fallback_lat = float(get("ntrip_fallback_lat") or 0.0)
        self.fallback_lon = float(get("ntrip_fallback_lon") or 0.0)
        self.nrtk_keepalive_s = max(0.5, float(get("ntrip_keepalive_s")))
        self.forward_without_ack = bool(get("forward_rtcm_without_board_ack"))
        self.corrections_stale_s = float(get("corrections_stale_s"))
        self.lora_pairing_enabled = bool(get("lora_pairing_enabled"))
        self.ntrip_fallback_grace_s = max(0.0, float(get("ntrip_fallback_grace_s")))

    @property
    def correction_source(self) -> str:
        """The source currently feeding the receiver: ``ntrip``, ``lora`` or ``none``."""
        mode = getattr(self, "rtcm_source", "none")
        if mode == "auto":
            return "ntrip" if self._ntrip_active else "lora"
        return mode

    def _persist_settings(self) -> None:
        """Write live changes of the caster fields to ntrip_settings_file (mode 0600)."""
        pending, self._pending_persist = self._pending_persist, {}
        if not pending or not self.ntrip_settings_file:
            return
        current = load_settings(self.ntrip_settings_file) or {
            "enabled": self.ntrip_enabled, "host": self.ntrip_config.host,
            "port": self.ntrip_config.port, "mountpoint": self.ntrip_config.mountpoint,
            "user": self.ntrip_config.user, "password": self.ntrip_config.password,
        }
        for name, value in pending.items():
            key = PERSISTED_PARAMS[name]
            if name in ("correction_source", "rtcm_source"):
                value = str(value or "").strip().lower()
                if value in ("", "auto"):
                    continue
                current["enabled"] = value == "ntrip"
            elif key == "enabled":
                current["enabled"] = bool(value)
            elif key == "port":
                current["port"] = int(value or 2101)
            else:
                current[key] = str(value or "").strip() if key != "password" else str(value or "")
        try:
            save_settings(self.ntrip_settings_file, current)
            self._persist_error = ""
            self.get_logger().info(
                "saved NTRIP settings to %s (enabled=%s, %s@%s:%d/%s, password %s)"
                % (self.ntrip_settings_file, current["enabled"], current["user"] or "-",
                   current["host"], int(current["port"]), current["mountpoint"],
                   "set" if current["password"] else "empty")
            )
        except OSError as exc:
            self._persist_error = str(exc)
            self.get_logger().error("could not save %s: %s" % (self.ntrip_settings_file, exc))

    def _check_settings_file(self) -> None:
        """Re-read ntrip_settings_file when it changed on disk (hand edit / other tool)."""
        if not self.ntrip_settings_file or self._stopping.is_set():
            return
        mtime = file_mtime(self.ntrip_settings_file)
        if mtime == self._settings_mtime:
            return
        self._settings_mtime = mtime
        self.get_logger().info("%s changed; reloading NTRIP settings" % self.ntrip_settings_file)
        with self._reconfigure_lock:
            if self._reconfigure_timer is not None:
                return
            timer = threading.Timer(RECONFIGURE_DEBOUNCE_S, self._reconfigure_corrections)
            timer.daemon = True
            self._reconfigure_timer = timer
            timer.start()

    def _ntrip_log(self, level: str, message: str) -> None:
        # One call site per severity: rclpy refuses to change the severity of a
        # given call site between calls ("Logger severity cannot be changed"),
        # which killed the NTRIP thread on its first error after a warning.
        log = self.get_logger()
        if level == "error":
            log.error(message)
        elif level in ("warn", "warning"):
            log.warning(message)
        elif level == "debug":
            log.debug(message)
        else:
            log.info(message)

    # -- corrections: serial writes ------------------------------------------------
    def _serial_write(self, data: bytes, purpose: str) -> bool:
        """Thread-safe write of raw bytes to the receiver port.

        Corrections and rover commands are only written in ntrip mode; LoRa pairing
        only when ``lora_pairing_enabled``. Everything else is refused, so in
        ``lora``/``none`` mode the port stays read-only (bar ``config_commands``).
        """
        allowed = (
            self.correction_source == "ntrip" if purpose in ("rtcm", "rover")
            else purpose == "lora_pairing" and self.lora_pairing_enabled
        )
        if not allowed:
            return False
        port = self._port
        if port is None or not port.is_open:
            return False
        with self._write_lock:
            try:
                port.write_all(data)
            except SerialError as exc:
                self.get_logger().warning("serial write (%s) failed: %s" % (purpose, exc))
                return False
        self._serial_bytes_written += len(data)
        return True

    def _rtcm_gate_open(self) -> bool:
        return self.forward_without_ack or self._rover_nrtk == "ON"

    def _write_rtcm(self, data: bytes) -> None:
        """NTRIP callback (client thread): RTCM into the UM960 port via the board.

        In LoRa mode the rtk_rover board does not pass host bytes to the UM960, so
        RTCM is held (dropped) until the board reports ``$GNRTK,ON`` - as the vendor
        ``rtcmCallback`` does - unless ``forward_rtcm_without_board_ack``.
        """
        if self.correction_source != "ntrip":
            # auto mode before the hand-over: the LoRa base still owns the receiver.
            self._rtcm_idle_bytes += len(data)
            return
        if not self._rtcm_gate_open():
            self._rtcm_held_bytes += len(data)
            now = time.monotonic()
            if now - self._rtcm_held_warned > 10.0:
                self._rtcm_held_warned = now
                self.get_logger().warning(
                    "holding RTCM (%d bytes so far): rover board reports network RTK %s, "
                    "not ON" % (self._rtcm_held_bytes, self._rover_nrtk or "unknown")
                )
            return
        if not self._serial_write(data, "rtcm"):
            raise SerialError("receiver port not open")

    def _nrtk_enable(self, reason: str) -> None:
        """Off->on transition: ``RTKRESET`` once, then ``$GNRTK,ON*69``."""
        with self._nrtk_lock:
            ok = self._serial_write(b"RTKRESET\r\n", "rover")
            ok = self._serial_write((lora.NRTK_ON + "\r\n").encode("ascii"), "rover") and ok
            self._nrtk_on_sent = time.monotonic()
        self.get_logger().info(
            "network RTK on (%s): sent RTKRESET + %s%s" % (reason, lora.NRTK_ON,
                                                           "" if ok else " (write FAILED)")
        )

    def _ntrip_should_own(self) -> bool:
        """auto: NTRIP owns the receiver while valid RTCM arrived within the grace."""
        client = self.ntrip
        if client is None:
            return False
        if self.rtcm_source == "ntrip":
            return True
        if client.state not in (STATE_CONNECTED, STATE_STREAMING):
            return False
        last = client.last_frame_time
        if last is None:
            return False
        grace = max(self.ntrip_fallback_grace_s, 0.5)
        return client._clock() - last <= grace

    def _nrtk_keepalive_loop(self, stop: Optional[threading.Event] = None) -> None:
        """Hand-over (auto) + ``$GNRTK,ON*69`` every ``ntrip_keepalive_s``."""
        stop = stop or threading.Event()
        while not self._stopping.wait(0.05) and not stop.is_set():
            port = self._port
            if port is None or not port.is_open:
                continue
            if self.rtcm_source == "auto":
                want = self._ntrip_should_own()
                if want and not self._ntrip_active:
                    self._ntrip_active = True
                    self._ntrip_switches += 1
                    self._rover_nrtk = None
                    self.get_logger().info("rtcm_source auto: NTRIP delivering, taking over "
                                           "from the LoRa base")
                    self._nrtk_enable("NTRIP connected")
                    continue
                if not want and self._ntrip_active:
                    # No OFF command (the vendor never sends one): stopping the
                    # keep-alive lets the board return to LoRa within ~6 s.
                    self._ntrip_active = False
                    self._ntrip_switches += 1
                    self.get_logger().warning("rtcm_source auto: NTRIP lost (%s); handing the "
                                           "receiver back to the LoRa base"
                                           % (self.ntrip.state if self.ntrip else "stopped"))
                    continue
            if not self._ntrip_active:
                continue
            with self._nrtk_lock:
                if time.monotonic() - self._nrtk_on_sent < self.nrtk_keepalive_s:
                    continue
                self._serial_write((lora.NRTK_ON + "\r\n").encode("ascii"), "rover")
                self._nrtk_on_sent = time.monotonic()

    def _gga_for_caster(self) -> Optional[str]:
        """Latest receiver GGA, else one synthesized from the fix or the fallback."""
        with self._lock:
            now = time.monotonic()
            if self._last_gga and now - self._last_gga_time < GGA_MAX_AGE_S:
                return self._last_gga
            fix = self.nmea.fix
            lat, lon = fix.get("lat"), fix.get("lon")
            if lat is not None and lon is not None and self._last_fix_time \
                    and now - self._last_fix_time < GGA_MAX_AGE_S:
                return make_gga(float(lat), float(lon), float(fix.get("altitude") or 0.0),
                                num_sats=int(fix.get("num_sats") or 10))
        if self.fallback_lat or self.fallback_lon:
            return make_gga(self.fallback_lat, self.fallback_lon, quality=1)
        return None

    # -- corrections: status -------------------------------------------------------
    def correction_summary(self) -> Dict[str, object]:
        """Correction health, shared by /fix_status tokens and /gps/corrections."""
        fix = self.nmea.fix
        diff_age = fix.get("fix_age")
        diff_age = None if diff_age is None else float(diff_age)
        out: Dict[str, object] = {"source": self.correction_source,
                                  "rtcm_source": self.rtcm_source,
                                  "receiver_diff_age_s": diff_age}
        if self.ntrip is not None:
            out["ntrip_state"] = self.ntrip.state
        if self.correction_source == "ntrip" and self.ntrip is not None:
            stats = self.ntrip.stats()
            out.update(stats)
            out["host"] = self.ntrip_config.host
            out["port"] = self.ntrip_config.port
            out["mountpoint"] = self.ntrip_config.mountpoint
            out["config_from"] = self.ntrip_config_source
            out["rover_nrtk"] = self._rover_nrtk
            out["rtcm_held_bytes"] = self._rtcm_held_bytes
            out["rtcm_gate"] = ("forced" if self.forward_without_ack
                                else "open" if self._rover_nrtk == "ON" else "held")
            out["netmode"] = self.ntrip_netmode or "unbound"
            out["serial_bytes_written"] = self._serial_bytes_written
            age = stats["frame_age_s"]
            if age is None:
                flow = "waiting"
            elif age <= self.corrections_stale_s:
                flow = "active"
            else:
                flow = "stale"
            out["corr_age_s"] = age if age is not None else stats["age_s"]
            if out["rtcm_gate"] == "held" and stats["bytes_rx"]:
                flow = "held"  # arriving from the caster, not reaching the receiver
        elif self.correction_source == "lora":
            out["state"] = "lora"
            out["lora_pairing"] = self._lora_pairing
            out["lora_pairing_enabled"] = self.lora_pairing_enabled
            out["reference_station_status"] = fix.get("reference_station_status")
            if diff_age is None:
                flow = "waiting"
            elif diff_age <= self.corrections_stale_s:
                flow = "active"
            else:
                flow = "stale"
            out["corr_age_s"] = diff_age
        else:
            out["state"] = STATE_OFF
            flow = "idle"
            out["corr_age_s"] = None
        out["flow"] = flow
        return out

    def _correction_tokens(self) -> List[str]:
        summary = self.correction_summary()
        parts = ["corr_src=%s" % summary["source"], "corr=%s" % summary["state"],
                 "corr_flow=%s" % summary["flow"]]
        if summary.get("corr_age_s") is not None:
            parts.append("corr_age=%.1fs" % float(summary["corr_age_s"]))
        if summary.get("rate_bps") is not None:
            parts.append("corr_rate=%.0fB/s" % float(summary["rate_bps"]))
        return parts

    def publish_corrections(self) -> None:
        with self._lock:
            summary = self.correction_summary()
        clean = {k: (None if isinstance(v, float) and not math.isfinite(v) else v)
                 for k, v in summary.items()}
        self.corrections_pub.publish(String(data=json.dumps(clean, sort_keys=True)))

    def ntrip_status(self) -> Dict[str, object]:
        """The /ntrip/status payload (never contains the password)."""
        cfg = self.ntrip_config
        fix = self.nmea.fix
        out: Dict[str, object] = {
            "rtcm_source": self.rtcm_source,
            "active_source": self.correction_source,
            "enabled": self.ntrip_enabled,
            "configured": bool(cfg.host and cfg.mountpoint),
            "host": cfg.host,
            "port": cfg.port,
            "mountpoint": cfg.mountpoint,
            "user_set": bool(cfg.user),
            "password_set": bool(cfg.password),
            "version": cfg.version,
            "config_from": self.ntrip_config_source,
            "settings_file": self.ntrip_settings_file,
            "persist_error": self._persist_error,
            "switches": self._ntrip_switches,
            "rover_nrtk": self._rover_nrtk,
            "rtcm_held_bytes": self._rtcm_held_bytes,
            "rtcm_idle_bytes": self._rtcm_idle_bytes,
            "receiver_quality": fix.get("quality_label"),
            "receiver_solution": fix.get("position_type"),
            "receiver_diff_age_s": fix.get("fix_age"),
        }
        if self.ntrip is not None:
            stats = self.ntrip.stats()
            out.update(stats)
            out["rtcm_age_s"] = stats["frame_age_s"]
        else:
            out.update({"state": STATE_OFF, "connected": False, "rate_bps": 0.0,
                        "bytes_rx": 0, "rtcm_age_s": None, "retries": 0})
        return out

    def publish_ntrip_status(self) -> None:
        self._check_settings_file()
        with self._lock:
            status = self.ntrip_status()
        clean = {k: (None if isinstance(v, float) and not math.isfinite(v) else v)
                 for k, v in status.items()}
        self.ntrip_status_pub.publish(String(data=json.dumps(clean, sort_keys=True)))

    # -- vendor LoRa pairing -------------------------------------------------------
    def _on_set_lora(self, request, response):
        sn = str(getattr(request, "sn", "") or "")
        addr = int(getattr(request, "addr", 0))
        channel = int(getattr(request, "channel", 0))
        area = str(getattr(request, "area", "") or "")
        response.result = False
        ok, commands, note = lora.pairing_commands(sn, addr, channel, area)
        self.get_logger().info(
            "set_lora sn=%s addr=%d channel=%d area=%r -> %s" % (sn, addr, channel, area, note)
        )
        if self.correction_source != "lora":
            self.get_logger().warning(
                "set_lora ignored: correction_source is %r, not 'lora'" % self.correction_source
            )
            return response
        if not ok:
            self.get_logger().warning("set_lora refused: %s" % note)
            return response
        if not self.lora_pairing_enabled:
            self.get_logger().warning(
                "set_lora is a stub (lora_pairing_enabled=false): would write %s to %s via "
                "the rtk_rover board; unverified on hardware, see docs/um960.md"
                % (" then ".join(commands), self.port_name)
            )
            return response
        written = True
        for index, command in enumerate(commands):
            if index:
                self._stopping.wait(0.5)  # vendor: 0.5 s between band and pairing
            written = self._serial_write((command + "\r\n").encode("ascii"), "lora_pairing") \
                and written
            self.get_logger().info("set_lora wrote %s (%s)" % (command, "ok" if written else
                                                                 "FAILED"))
        response.result = bool(written)
        return response

    # -- serial --------------------------------------------------------------------
    def _open_port(self) -> bool:
        if self._stopping.is_set():
            return False
        port = SerialPort(self.port_name, self.baud, self.serial_timeout)
        try:
            port.open()
        except SerialError as exc:
            self.get_logger().error(
                "Failed to connect to Unicore GPS on port %s with baudrate %d: %s"
                % (self.port_name, self.baud, exc)
            )
            return False
        self._port = port
        self._connected = True
        self.splitter.reset()
        self.get_logger().info(
            "Connected to Unicore GPS on port %s with baudrate %d."
            % (self.port_name, self.baud)
        )
        for command in self.config_commands:
            try:
                port.write_line(command)
                self.get_logger().info("sent receiver config: %s" % command)
            except SerialError as exc:
                self.get_logger().warning("failed to send config %r: %s" % (command, exc))
        if self.rtcm_source == "auto" and self._ntrip_active:
            # Re-do the hand-over from the keepalive thread once NTRIP is flowing.
            self._ntrip_active = False
        if self.correction_source == "ntrip":
            # A (re)opened port is an off->on transition: the board may have
            # rebooted or fallen back to LoRa while we were away.
            self._rover_nrtk = None
            self._nrtk_enable("port connected")
        elif self.correction_source == "lora" and self.lora_pairing_enabled:
            # Vendor checkRun(): ask the rover board for its current LoRa pairing.
            self._serial_write((lora.QUERY_PAIRING + "\r\n").encode("ascii"), "lora_pairing")
        return True

    def _close_port(self) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None
        self._connected = False

    def _sleep_before_retry(self) -> None:
        """Wait out the reconnect delay, but wake immediately when shutting down."""
        self._stopping.wait(self.reconnect_period)

    def _reader_loop(self) -> None:
        while rclpy.ok() and not self._stopping.is_set():
            port = self._port
            if port is None or not port.is_open:
                if not self._open_port():
                    self._sleep_before_retry()
                    continue
                port = self._port
            try:
                # Read through the local reference: _close_port() can null out
                # self._port from destroy_node() while this thread is blocked.
                data = port.read()
            except SerialError as exc:  # pragma: no cover - unplugged cable
                if self._port is port:
                    self.get_logger().error("serial read failed: %s" % exc)
                    self._close_port()
                self._sleep_before_retry()
                continue
            if not data:
                continue
            self._consume(data)

    def _consume(self, data: bytes) -> None:
        """Feed raw serial bytes to the mixed ASCII/binary stream splitter."""
        for kind, value in self.splitter.feed(data):
            if kind == "line":
                self._consume_line(value)
            else:
                self._apply_frame(value)
        # Bumped last, once the bytes have actually been parsed, so an observer can
        # treat it as "the reader has caught up with N bytes" rather than "has seen
        # N bytes".
        self._bytes_read += len(data)

    def _apply_frame(self, frame: Dict[str, object]) -> None:
        if frame.get("kind") == "bestnav":
            self._apply_bestnav(frame)
        elif frame.get("kind") == "version":
            with self._lock:
                self.nmea.fix["version"] = frame["version"]

    def _consume_line(self, line: str) -> None:
        if self.nmea_pub is not None:
            self.nmea_pub.publish(String(data=line))
        if not line.startswith("$"):
            return
        if not nmea_checksum_ok(line):
            return
        if line.startswith(("$GNRTK,", "$GNCON,")):
            self._consume_rover_line(line)
            return
        with self._lock:
            message_id = self.nmea.feed(line)
            if message_id is None:
                return
            if message_id.endswith("GGA") and self.nmea.has_fix:
                self._last_gga = line
                self._last_gga_time = time.monotonic()
            position_bearing = message_id in FIX_BEARING_IDS or message_id.endswith(
                FIX_BEARING_SUFFIXES
            )
            # A position-bearing ASCII log keeps /fix alive when no binary BESTNAV
            # is being streamed. Sentences that report "no fix" (GGA quality 0)
            # deliberately do not, so /fix stops instead of repeating a dead one.
            fresh = position_bearing and self.nmea.has_fix
        if fresh:
            self._mark_fix_time()

    def _consume_rover_line(self, line: str) -> None:
        """Replies of the rtk_rover board: ``$GNRTK,ON|OFF`` and ``$GNCON,addr,ch``."""
        fields = line[1:].split("*")[0].split(",")
        if fields[0] == "GNRTK" and len(fields) > 1:
            state = fields[1].strip().upper()
            previous = self._rover_nrtk
            self._rover_nrtk = state
            if state != previous:
                self.get_logger().info("rover board reports network RTK %s" % state)
            # ON -> OFF: the board fell back to LoRa. Vendor: RTKReset(), then the
            # keep-alive switches it on again; do both right away.
            if self.correction_source == "ntrip" and previous == "ON" and state != "ON":
                self._nrtk_enable("board dropped to %s" % state)
        elif fields[0] == "GNCON" and len(fields) > 2:
            self._lora_pairing = "%s,%s" % (fields[1], fields[2])

    # -- fix construction ----------------------------------------------------------
    def _mark_fix_time(self) -> None:
        self._last_fix_time = time.monotonic()
        self._observation_sequence = getattr(self, "_observation_sequence", 0) + 1

    def _apply_bestnav(self, frame: Dict[str, object]) -> None:
        """Fold a Unicore binary/ASCII BESTNAV solution into the NMEA fix state."""
        if int(frame.get("solution_status", 1)) != SOL_COMPUTED:
            return
        with self._lock:
            fix = self.nmea.fix
            fix["lat"] = frame["latitude"]
            fix["lon"] = frame["longitude"]
            fix["altitude"] = frame["height"]
            fix["lat_sigma"] = frame["lat_sigma"]
            fix["lon_sigma"] = frame["lon_sigma"]
            fix["alt_sigma"] = frame["height_sigma"]
            fix["fix_age"] = frame["diff_age"]
            fix["solution_age"] = frame["solution_age"]
            fix["undulation"] = frame.get("undulation")
            fix["datum_id"] = frame.get("datum_id")
            fix["num_sats"] = frame["num_sats"]
            fix["num_sats_used"] = frame["num_sats_used"]
            fix["station_id"] = frame["station_id"] or fix.get("station_id", "")
            pos_type = int(frame["position_type"])
            fix["position_type"] = bestnav_position_type_name(pos_type)
            fix["solution_status"] = bestnav_solution_status_name(int(frame["solution_status"]))
            fix["velocity_type"] = bestnav_velocity_type_name(int(frame["velocity_type"]))
            fix["horizontal_speed"] = frame["horizontal_speed"]
            fix["vertical_speed"] = frame["vertical_speed"]
            fix["track_ground"] = frame["track_ground"]
            fix["speed_mps"] = frame["horizontal_speed"]
            fix["course_deg"] = frame["track_ground"]
            fix["gga_sentence"] = "BESTNAV"
            # The position type implies the fix quality flag used for NavSatStatus:
            # integer solutions map to GGA quality 4, everything else to 1.
            fix["quality"] = 4 if pos_type in FIXED_POS_TYPES else 1
            fix["quality_label"] = fix["position_type"]
            fix["status"] = "STATUS_FIX"
        self._mark_fix_time()

    def _build_fix(self) -> Optional[NavSatFix]:
        fix = self.nmea.fix
        lat = fix.get("lat")
        lon = fix.get("lon")
        if lat is None or lon is None:
            return None
        if abs(lat) > self.sane_lat or abs(lon) > self.sane_lon:
            self.get_logger().warning("coords not sane (lat=%s lon=%s), skip publish." % (lat, lon))
            return None

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(fix.get("altitude") or 0.0)

        quality = int(fix.get("quality") or 0)
        msg.status.status = STATUS_BY_QUALITY.get(quality, NavSatStatus.STATUS_FIX)
        msg.status.service = (
            NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        )

        lat_sigma = fix.get("lat_sigma")
        lon_sigma = fix.get("lon_sigma")
        alt_sigma = fix.get("alt_sigma")
        if lat_sigma and lon_sigma and alt_sigma:
            cov = sigma_to_covariance(float(lat_sigma), float(lon_sigma), float(alt_sigma))
        else:
            cov = hdop_to_covariance(float(fix.get("hdop") or 0.0), int(fix.get("num_sats") or 0),
                                     int(fix.get("quality") or 1))
        if not any(cov):
            # No usable accuracy information: advertise "no covariance" rather than
            # a fictitious perfect fix.
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            msg.position_covariance = [0.0] * 9
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            msg.position_covariance = cov
        return msg

    def _build_twist(self) -> Optional[TwistStamped]:
        fix = self.nmea.fix
        speed = fix.get("horizontal_speed")
        if speed is None:
            speed = fix.get("speed_mps")
        if speed is None:
            return None
        course = fix.get("track_ground")
        if course is None:
            course = fix.get("course_deg")
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        speed = float(speed)
        # Ground track is published in the GPS frame; sign it so that travelling
        # southward yields a negative body-x velocity (mower convention).
        if course is not None and not (0.0 <= float(course) < 180.0):
            speed = -speed
        msg.twist.linear.x = speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.0
        return msg

    def _build_heading(self) -> Optional[Float32]:
        fix = self.nmea.fix
        # GPHPR is the receiver's own heading estimate and wins when present.
        heading = fix.get("heading_deg")
        if heading is None:
            heading = fix.get("track_ground")
            if heading is None:
                heading = fix.get("course_deg")
        if heading is None:
            return None
        # Normalise into [0, 360); both GPHPR and the track angle can arrive negative.
        return Float32(data=float(heading) % 360.0)

    def _build_heading_valid(self) -> Bool:
        """True only for a fresh, fixed-baseline GPHPR heading on a dual-antenna receiver.

        Requires ``publish_rtk_heading`` (off by default: the UM960 is single-antenna), a
        GPHPR younger than ``heading_timeout_s`` and its quality flag = 4 (RTK fixed). The
        track/course fall-back is course over ground and never valid.
        """
        fixed = self.nmea.heading_is_fixed(self.heading_timeout)
        if fixed and not self.publish_rtk_heading and not self._hpr_logged:
            self._hpr_logged = True
            self.get_logger().info(
                "receiver sends a fixed GPHPR heading but publish_rtk_heading is false: "
                "/heading_valid stays false until a dual-antenna install is confirmed"
            )
        return Bool(data=bool(fixed and self.publish_rtk_heading))

    def _build_status(self) -> str:
        fix = self.nmea.fix
        connected = "connected" if self._connected else "disconnected"
        age = time.monotonic() - self._last_fix_time if self._last_fix_time else float("inf")
        parts: List[str] = ["um960=%s" % connected]
        position_type = fix.get("position_type")
        label = fix.get("quality_label")
        if position_type:
            parts.append("solution=%s" % position_type)
        elif label:
            parts.append("quality=%s" % label)
        if fix.get("solution_status"):
            parts.append("sol_status=%s" % fix["solution_status"])
        if fix.get("velocity_type"):
            parts.append("vel_type=%s" % fix["velocity_type"])
        if fix.get("num_sats"):
            used = fix.get("num_sats_used") or fix["num_sats"]
            parts.append("sats=%s/%s" % (used, fix["num_sats"]))
        if fix.get("hdop"):
            parts.append("hdop=%.2f" % float(fix["hdop"]))
        if fix.get("fix_age") is not None:
            parts.append("diff_age=%.1fs" % float(fix["fix_age"]))
        if fix.get("station_id"):
            parts.append("station=%s" % fix["station_id"])
        ref = fix.get("reference_station_status")
        if ref:
            parts.append("ref=%s" % ref)
        if fix.get("fix_type"):
            parts.append("gsa=%s" % GSA_FIX_TYPE.get(int(fix["fix_type"]), "?"))
        parts.extend(self._correction_tokens())
        if age == float("inf"):
            parts.append("no fix yet")
        else:
            parts.append("age=%.2fs" % age)
        return " ".join(parts)

    def _publish_tick(self) -> None:
        now = time.monotonic()
        if now - self._last_publish < (1.0 / max(self.publish_rate, 1.0)) * 0.9:
            return
        self._last_publish = now
        with self._lock:
            # A GPHPR heading is only good while the receiver keeps sending it: never
            # re-publish one sentence forever (it defeated the consumer's stale check).
            self.nmea.expire_heading(self.heading_timeout)
            fix_msg = self._build_fix()
            twist_msg = None if fix_msg is None else self._build_twist()
            heading_msg = None if fix_msg is None else self._build_heading()
            heading_valid_msg = self._build_heading_valid()
            status = self._build_status()
            gnss_fields = None
            if self.gnss_status_pub is not None:
                stale = bool(self._last_fix_time) and (now - self._last_fix_time) > self.fix_timeout
                gnss_fields = gnss_status_fields(
                    self.nmea.fix, fix_msg is not None and not stale,
                    None if fix_msg is None else list(fix_msg.position_covariance),
                    self.correction_summary(), self._observation_sequence)
        if gnss_fields is not None:
            self._publish_gnss_status(gnss_fields, fix_msg)

        if self._last_fix_time and (now - self._last_fix_time) > self.fix_timeout:
            # Stop publishing the fix, but keep /fix_status alive so an operator can
            # see "the receiver went quiet" rather than an empty graph.
            self._unhealthy_streak += 1
            if self._unhealthy_streak == 1:
                self.get_logger().warning(
                    "no fresh UM960 fix for %.1fs, stopping /fix" % self.fix_timeout
                )
            self.fix_status_pub.publish(String(data="STALE " + status))
            return
        self._unhealthy_streak = 0

        if fix_msg is not None:
            self.fix_pub.publish(fix_msg)
        if twist_msg is not None:
            self.vel_pub.publish(twist_msg)
        if heading_msg is not None:
            self.heading_pub.publish(heading_msg)
        self.heading_valid_pub.publish(heading_valid_msg)
        self.fix_status_pub.publish(String(data=status))

    def _publish_gnss_status(self, fields, fix_msg) -> None:
        msg = GnssStatus()
        msg.header.stamp = (fix_msg.header.stamp if fix_msg is not None
                            else self.get_clock().now().to_msg())
        msg.header.frame_id = self.frame_id
        for key, value in fields.items():
            setattr(msg, key, value)
        self.gnss_status_pub.publish(msg)

    def destroy_node(self) -> bool:
        # Stop the reader thread before closing the port so it cannot reopen the
        # device behind our back, then let it wind down.
        self._stopping.set()
        with self._reconfigure_lock:
            if self._reconfigure_timer is not None:
                self._reconfigure_timer.cancel()
                self._reconfigure_timer = None
            self._stop_corrections()
        self._close_port()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = Um960Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
