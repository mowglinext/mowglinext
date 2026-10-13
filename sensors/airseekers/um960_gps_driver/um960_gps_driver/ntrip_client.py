"""ROS-free NTRIP v1/v2 client for feeding RTCM 3 corrections to the UM960.

The client runs in its own thread:

* connects to the caster, sends an HTTP GET for the mountpoint (NTRIP 2.0 adds
  ``Ntrip-Version: Ntrip/2.0`` and a ``Host`` header; NTRIP 1.0 is a bare
  HTTP/1.0-style request), with HTTP basic authentication;
* accepts ``ICY 200 OK`` (NTRIP 1 casters) and ``HTTP/1.x 200 OK`` (NTRIP 2), and
  de-chunks the body when the caster answers with ``Transfer-Encoding: chunked``;
* hands every received correction byte to ``write_rtcm(bytes)`` unchanged
  (the UM960 takes raw RTCM 3 on the same UART that it streams NMEA on);
* uploads the rover position (a GGA sentence from ``get_gga()``) right after the
  handshake and then every ``gga_interval_s`` on the same socket - VRS / nearest
  base mountpoints need it to pick or synthesize a reference station;
* reconnects with exponential backoff after a connect failure, an auth failure,
  a closed socket or ``no_data_timeout_s`` without bytes;
* counts complete, CRC-24Q-valid RTCM 3 frames so "bytes are flowing" can be
  told apart from "valid corrections are flowing".

The logic follows the vendor's ``libntrip::NtripClient``
(``base_rtk/mower_gps_driver/src/ntrip/ntrip_client.cc``: GET + basic auth, accept
``HTTP/1.1 200 OK`` or ``ICY 200 OK``, send GGA right after the handshake and then on
a fixed interval) and the vendored Microstrain ``ntrip_client.py`` (sourcetable
response => bad mountpoint, ``401`` => bad credentials, reconnect on an RTCM
timeout). It deliberately depends on nothing but the standard library.
"""

import base64
import collections
import socket
import threading
import time
from typing import Callable, Deque, Dict, List, Optional, Tuple

# Connection state strings, also used verbatim as the ``corr=`` token in /fix_status.
STATE_OFF = "off"                    # client not started / stopped
STATE_CONNECTING = "connecting"      # TCP connect + request in progress
STATE_CONNECTED = "connected"        # caster accepted the request, no data yet
STATE_STREAMING = "streaming"        # bytes are arriving
STATE_RECONNECTING = "reconnecting"  # waiting out the backoff after a failure
STATE_AUTH_FAILED = "auth_failed"    # 401: wrong user/password (keeps retrying slowly)
STATE_BAD_MOUNTPOINT = "bad_mountpoint"  # caster sent its sourcetable instead
STATE_ERROR = "error"                # any other failure (also retried)

FAILED_STATES = frozenset({STATE_AUTH_FAILED, STATE_BAD_MOUNTPOINT, STATE_ERROR})

USER_AGENT = "NTRIP um960_gps_driver/0.2"
RTCM3_PREAMBLE = 0xD3

_MAX_HEADER = 16384


class NtripError(Exception):
    """Handshake failure; ``state`` is the STATE_* to report."""

    def __init__(self, state: str, message: str) -> None:
        super().__init__(message)
        self.state = state


# --------------------------------------------------------------------------------------
# helpers (pure functions, unit-tested on their own)
# --------------------------------------------------------------------------------------
def nmea_wrap(body: str) -> str:
    """``"GPGGA,..."`` -> ``"$GPGGA,...*CS"`` (no line terminator)."""
    body = body.lstrip("$").split("*")[0]
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return "$%s*%02X" % (body, checksum)


def make_gga(lat: float, lon: float, alt: float = 0.0, quality: int = 1,
             num_sats: int = 10, hdop: float = 1.0, utc: Optional[float] = None) -> str:
    """Synthesize a GPGGA sentence (with CRLF) for the caster from a lat/lon.

    Mirrors the vendor's ``GGAFrameGenerate``: used when the receiver has a position
    (e.g. from binary BESTNAV) but no GGA log is enabled, or from a configured
    fallback position before the first fix.
    """
    if utc is None:
        utc = time.time()
    tm = time.gmtime(utc)
    hhmmss = "%02d%02d%05.2f" % (tm.tm_hour, tm.tm_min, tm.tm_sec + (utc % 1.0))
    lat_abs, lon_abs = abs(lat), abs(lon)
    lat_dm = "%02d%010.7f" % (int(lat_abs), (lat_abs - int(lat_abs)) * 60.0)
    lon_dm = "%03d%010.7f" % (int(lon_abs), (lon_abs - int(lon_abs)) * 60.0)
    body = "GPGGA,%s,%s,%s,%s,%s,%d,%02d,%.1f,%.3f,M,0.000,M,," % (
        hhmmss, lat_dm, "N" if lat >= 0 else "S", lon_dm, "E" if lon >= 0 else "W",
        quality, num_sats, hdop, alt)
    return nmea_wrap(body) + "\r\n"


def build_request(host: str, port: int, mountpoint: str, user: str = "", password: str = "",
                  version: int = 2, gga: Optional[str] = None) -> bytes:
    """The NTRIP GET request for ``mountpoint``."""
    mountpoint = mountpoint.lstrip("/")
    lines: List[str]
    if int(version) >= 2:
        lines = [
            "GET /%s HTTP/1.1" % mountpoint,
            "Host: %s:%d" % (host, int(port)),
            "Ntrip-Version: Ntrip/2.0",
            "User-Agent: %s" % USER_AGENT,
            "Connection: close",
        ]
        if gga:
            # NTRIP 2 lets the client put its position in the request itself.
            lines.append("Ntrip-GGA: %s" % gga.strip())
    else:
        lines = [
            "GET /%s HTTP/1.0" % mountpoint,
            "User-Agent: %s" % USER_AGENT,
            "Accept: */*",
        ]
    if user or password:
        token = base64.b64encode(("%s:%s" % (user, password)).encode("utf-8")).decode("ascii")
        lines.append("Authorization: Basic %s" % token)
    return ("\r\n".join(lines) + "\r\n\r\n").encode("ascii", errors="ignore")


def parse_response_head(head: bytes) -> Tuple[str, int, Dict[str, str]]:
    """Split a response head into ``(status_line, code, headers)``.

    ``code`` is 200 for ``ICY 200 OK``; ``SOURCETABLE 200 OK`` reports 200 too, the
    caller tells them apart by the status line.
    """
    text = head.decode("latin-1", errors="replace")
    lines = text.split("\r\n")
    status = lines[0].strip()
    code = 0
    parts = status.split()
    if len(parts) >= 2 and parts[1].isdigit():
        code = int(parts[1])
    headers: Dict[str, str] = {}
    for line in lines[1:]:
        if ":" in line:
            key, _, value = line.partition(":")
            headers[key.strip().lower()] = value.strip()
    return status, code, headers


class ChunkedDecoder:
    """Incremental HTTP/1.1 ``Transfer-Encoding: chunked`` body decoder."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._remaining = 0       # payload bytes left in the current chunk
        self._need_crlf = False   # a chunk ended, its trailing CRLF is pending
        self.finished = False

    def feed(self, data: bytes) -> bytes:
        self._buf.extend(data)
        out = bytearray()
        while not self.finished:
            if self._remaining:
                take = min(self._remaining, len(self._buf))
                if not take:
                    break
                out.extend(self._buf[:take])
                del self._buf[:take]
                self._remaining -= take
                if not self._remaining:
                    self._need_crlf = True
                continue
            if self._need_crlf:
                if len(self._buf) < 2:
                    break
                del self._buf[:2]
                self._need_crlf = False
                continue
            end = self._buf.find(b"\r\n")
            if end < 0:
                if len(self._buf) > 1024:  # garbage, not a size line
                    raise ValueError("bad chunk header")
                break
            size_field = bytes(self._buf[:end]).split(b";")[0].strip()
            del self._buf[: end + 2]
            try:
                size = int(size_field, 16)
            except ValueError as exc:
                raise ValueError("bad chunk size %r" % size_field) from exc
            if size == 0:
                self.finished = True
                break
            self._remaining = size
        return bytes(out)


def crc24q(data: bytes) -> int:
    """CRC-24Q as used by RTCM 3 (poly 0x1864CFB, init 0)."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def rtcm3_frame(payload: bytes) -> bytes:
    """Wrap a payload in RTCM 3 framing (preamble, 10-bit length, CRC-24Q)."""
    head = bytes([RTCM3_PREAMBLE, (len(payload) >> 8) & 0x03, len(payload) & 0xFF])
    crc = crc24q(head + payload)
    return head + payload + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


class Rtcm3FrameCounter:
    """Splits an arbitrary byte stream into complete, CRC-24Q-valid RTCM 3 frames.

    ``extract()`` returns the valid frames; everything else (bytes before a
    preamble, false preambles, frames with a bad CRC) is dropped and counted in
    ``dropped_bytes`` / ``crc_errors``. ``feed()`` is the counting-only variant.
    """

    MAX_BUFFER = 4 * 1029  # a few maximum-size frames

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.dropped_bytes = 0
        self.message_types: Dict[int, int] = {}

    def feed(self, data: bytes) -> int:
        """Return the number of valid frames completed by ``data``."""
        return len(self.extract(data))

    def extract(self, data: bytes) -> List[bytes]:
        """Return the valid frames completed by ``data`` (in stream order)."""
        self._buf.extend(data)
        out: List[bytes] = []
        while True:
            start = self._buf.find(bytes([RTCM3_PREAMBLE]))
            if start < 0:
                self.dropped_bytes += len(self._buf)
                self._buf.clear()
                break
            if start:
                self.dropped_bytes += start
                del self._buf[:start]
            if len(self._buf) < 3:
                break
            if self._buf[1] & 0xFC:
                # The 6 reserved bits after the preamble must be zero: not a frame.
                self.dropped_bytes += 1
                del self._buf[:1]
                continue
            length = ((self._buf[1] & 0x03) << 8) | self._buf[2]
            total = 3 + length + 3
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            crc = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
            if crc24q(frame[:-3]) == crc:
                out.append(frame)
                self.frames += 1
                if length >= 2:
                    msg_type = (frame[3] << 4) | (frame[4] >> 4)
                    self.message_types[msg_type] = self.message_types.get(msg_type, 0) + 1
                del self._buf[:total]
            else:
                self.crc_errors += 1
                self.dropped_bytes += 1
                del self._buf[:1]  # false preamble / corrupt frame: resync on the next 0xD3
        return out


# --------------------------------------------------------------------------------------
# client
# --------------------------------------------------------------------------------------
class NtripConfig:
    """Connection settings. ``version`` is 1 or 2."""

    def __init__(self, host: str, port: int = 2101, mountpoint: str = "", user: str = "",
                 password: str = "", version: int = 2, gga_interval_s: float = 10.0,
                 connect_timeout_s: float = 5.0, no_data_timeout_s: float = 20.0,
                 reconnect_min_s: float = 1.0, reconnect_max_s: float = 30.0,
                 bind_device: str = "", validate_rtcm: bool = True) -> None:
        self.host = host
        self.port = int(port)
        self.mountpoint = mountpoint
        self.user = user
        self.password = password
        self.version = int(version)
        self.gga_interval_s = float(gga_interval_s)
        self.connect_timeout_s = float(connect_timeout_s)
        self.no_data_timeout_s = float(no_data_timeout_s)
        self.reconnect_min_s = float(reconnect_min_s)
        self.reconnect_max_s = float(reconnect_max_s)
        # Network interface to bind the socket to (SO_BINDTODEVICE), "" = any.
        self.bind_device = bind_device
        # True: only complete CRC-24Q-valid RTCM 3 frames reach write_rtcm.
        self.validate_rtcm = bool(validate_rtcm)

    def describe(self) -> str:
        """Log-safe description (never includes the password)."""
        return "ntrip://%s@%s:%d/%s (v%d)" % (
            self.user or "-", self.host, self.port, self.mountpoint, self.version)


class NtripClient:
    """Threaded NTRIP client. ``write_rtcm`` is called from the client thread."""

    RATE_WINDOW_S = 5.0

    def __init__(self, config: NtripConfig, write_rtcm: Callable[[bytes], object],
                 get_gga: Optional[Callable[[], Optional[str]]] = None,
                 log: Optional[Callable[[str, str], None]] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.config = config
        self._write_rtcm = write_rtcm
        self._get_gga = get_gga or (lambda: None)
        self._log = log or (lambda _level, _msg: None)
        self._clock = clock
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._rate: Deque[Tuple[float, int]] = collections.deque()
        self._frames = Rtcm3FrameCounter()
        self.state = STATE_OFF
        self.last_error = ""
        self.bytes_rx = 0
        self.bytes_written = 0
        self.write_errors = 0
        self.connects = 0
        self.attempts = 0
        self.gga_sent = 0
        self.last_rx_time: Optional[float] = None
        self.last_frame_time: Optional[float] = None
        self.last_gga_time: Optional[float] = None
        self.connected_since: Optional[float] = None
        self.frames_written = 0
        self.consecutive_failures = 0
        self.backoff_s = 0.0

    # -- lifecycle -------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ntrip_client", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        self._close_socket()
        if self._thread is not None:
            self._thread.join(timeout)
        self._set_state(STATE_OFF)

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # -- stats -----------------------------------------------------------------------
    def stats(self) -> Dict[str, object]:
        now = self._clock()
        with self._lock:
            self._trim_rate(now)
            window_bytes = sum(n for _t, n in self._rate)
            return {
                "state": self.state,
                "bytes_rx": self.bytes_rx,
                "bytes_written": self.bytes_written,
                "write_errors": self.write_errors,
                "rate_bps": round(window_bytes / self.RATE_WINDOW_S, 1),
                "age_s": None if self.last_rx_time is None else round(now - self.last_rx_time, 2),
                "frame_age_s": (None if self.last_frame_time is None
                                else round(now - self.last_frame_time, 2)),
                "connected": self.state in (STATE_CONNECTED, STATE_STREAMING),
                "frames": self._frames.frames,
                "frames_written": self.frames_written,
                "crc_errors": self._frames.crc_errors,
                "dropped_bytes": self._frames.dropped_bytes,
                "validate_rtcm": self.config.validate_rtcm,
                "message_types": dict(sorted(self._frames.message_types.items())),
                "gga_sent": self.gga_sent,
                "gga_age_s": (None if self.last_gga_time is None
                              else round(now - self.last_gga_time, 2)),
                "connects": self.connects,
                "attempts": self.attempts,
                "retries": self.consecutive_failures,
                "backoff_s": self.backoff_s,
                "uptime_s": (round(now - self.connected_since, 1)
                             if self.connected_since is not None
                             and self.state in (STATE_CONNECTED, STATE_STREAMING) else None),
                "last_error": self.last_error,
            }

    def _trim_rate(self, now: float) -> None:
        while self._rate and now - self._rate[0][0] > self.RATE_WINDOW_S:
            self._rate.popleft()

    def _set_state(self, state: str, error: str = "") -> None:
        with self._lock:
            self.state = state
            if error:
                self.last_error = error

    # -- thread ----------------------------------------------------------------------
    def _run(self) -> None:
        backoff = self.config.reconnect_min_s
        while not self._stop.is_set():
            self.attempts += 1
            try:
                body = self._connect()
                backoff = self.config.reconnect_min_s  # handshake OK: reset the backoff
                self.consecutive_failures = 0
                self.backoff_s = 0.0
                self._stream(body)
                if self._stop.is_set():
                    break
                self._set_state(STATE_RECONNECTING, "caster closed the connection")
            except NtripError as exc:
                self._log("error" if exc.state in FAILED_STATES else "warn",
                          "NTRIP %s: %s" % (self.config.describe(), exc))
                self._set_state(exc.state, str(exc))
                if exc.state in (STATE_AUTH_FAILED, STATE_BAD_MOUNTPOINT):
                    # Retrying a bad login fast just gets the account banned.
                    backoff = self.config.reconnect_max_s
            except (OSError, ValueError) as exc:
                if self._stop.is_set():
                    break
                self._log("warn", "NTRIP %s: %s" % (self.config.describe(), exc))
                self._set_state(STATE_RECONNECTING, str(exc))
            except Exception as exc:  # noqa: BLE001 - the client thread must never die
                if self._stop.is_set():
                    break
                try:
                    self._log("warn", "NTRIP %s: unexpected %s: %s"
                              % (self.config.describe(), type(exc).__name__, exc))
                except Exception:  # noqa: BLE001 - logging itself may be the problem
                    pass
                self._set_state(STATE_RECONNECTING, str(exc))
            finally:
                self._close_socket()
            if self._stop.is_set():
                break
            if self.state not in FAILED_STATES:
                self._set_state(STATE_RECONNECTING)
            self.consecutive_failures += 1
            self.backoff_s = backoff
            self._stop.wait(backoff)
            backoff = min(max(backoff * 2.0, self.config.reconnect_min_s),
                          self.config.reconnect_max_s)
        self._set_state(STATE_OFF)

    def _close_socket(self) -> None:
        sock, self._sock = self._sock, None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _connect(self) -> bytes:
        """Connect + handshake. Returns body bytes that arrived with the header."""
        cfg = self.config
        self._set_state(STATE_CONNECTING)
        if cfg.bind_device:
            sock = self._bound_connection(cfg)
        else:
            sock = socket.create_connection((cfg.host, cfg.port), timeout=cfg.connect_timeout_s)
        self._sock = sock
        gga = self._current_gga()
        sock.sendall(build_request(cfg.host, cfg.port, cfg.mountpoint, cfg.user, cfg.password,
                                   cfg.version, gga if cfg.version >= 2 else None))
        head, rest = self._read_head(sock)
        status, code, headers = parse_response_head(head)
        upper = status.upper()
        if upper.startswith("SOURCETABLE") or "sourcetable" in headers.get("content-type", ""):
            raise NtripError(STATE_BAD_MOUNTPOINT,
                             "caster returned its sourcetable: mountpoint %r not found"
                             % cfg.mountpoint)
        if code == 401 or code == 403:
            raise NtripError(STATE_AUTH_FAILED, "caster refused the credentials (%s)" % status)
        if code != 200:
            raise NtripError(STATE_ERROR, "unexpected caster response %r" % status[:80])
        self._chunked = "chunked" in headers.get("transfer-encoding", "").lower()
        self._decoder = ChunkedDecoder() if self._chunked else None
        self.connects += 1
        self.connected_since = self._clock()
        self._set_state(STATE_CONNECTED)
        self._log("info", "NTRIP connected: %s -> %s%s" % (
            cfg.describe(), status, " (chunked)" if self._chunked else ""))
        # Vendor behaviour: send the position immediately so a VRS caster starts
        # streaming without waiting a full GGA interval.
        self._last_gga_attempt = None
        self._maybe_send_gga(force=True)
        return rest

    @staticmethod
    def _bound_connection(cfg: "NtripConfig") -> socket.socket:
        """Connect through one interface, like the vendor's ``set_bind_device``."""
        family, kind, proto, _name, addr = socket.getaddrinfo(
            cfg.host, cfg.port, socket.AF_INET, socket.SOCK_STREAM)[0]
        sock = socket.socket(family, kind, proto)
        try:
            sock.setsockopt(socket.SOL_SOCKET, getattr(socket, "SO_BINDTODEVICE", 25),
                            cfg.bind_device.encode() + b"\0")
            sock.settimeout(cfg.connect_timeout_s)
            sock.connect(addr)
        except OSError:
            sock.close()
            raise
        return sock

    def _read_head(self, sock: socket.socket) -> Tuple[bytes, bytes]:
        buf = bytearray()
        deadline = self._clock() + self.config.connect_timeout_s
        while True:
            if b"\r\n\r\n" in buf:
                head, _, rest = bytes(buf).partition(b"\r\n\r\n")
                return head, rest
            # NTRIP 1 casters answer "ICY 200 OK\r\n" and may start the RTCM stream
            # immediately, without an empty line after the status.
            if buf.startswith(b"ICY 200") and b"\r\n" in buf:
                head, _, rest = bytes(buf).partition(b"\r\n")
                if rest.startswith(b"\r\n"):
                    rest = rest[2:]
                return head, rest
            if len(buf) > _MAX_HEADER:
                raise NtripError(STATE_ERROR, "response header too long")
            if self._clock() > deadline:
                raise NtripError(STATE_ERROR, "timed out waiting for the caster response")
            sock.settimeout(0.2)
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                if buf:
                    status = bytes(buf).split(b"\r\n")[0].decode("latin-1", "replace")
                    _s, code, _h = parse_response_head(bytes(buf))
                    if code in (401, 403):
                        raise NtripError(STATE_AUTH_FAILED,
                                         "caster refused the credentials (%s)" % status)
                    if status.upper().startswith("SOURCETABLE"):
                        raise NtripError(STATE_BAD_MOUNTPOINT,
                                         "caster returned its sourcetable")
                raise NtripError(STATE_ERROR, "caster closed the connection during handshake")
            buf.extend(chunk)

    def _stream(self, initial: bytes) -> None:
        sock = self._sock
        if initial:
            self._deliver(initial)
        sock.settimeout(0.25)
        last_data = self._clock()
        while not self._stop.is_set():
            self._maybe_send_gga()
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                if self._clock() - last_data > self.config.no_data_timeout_s:
                    raise OSError("no data from caster for %.0fs" % self.config.no_data_timeout_s)
                continue
            if not chunk:
                return
            last_data = self._clock()
            self._deliver(chunk)
            if self._decoder is not None and self._decoder.finished:
                return

    def _deliver(self, raw: bytes) -> None:
        data = self._decoder.feed(raw) if self._decoder is not None else raw
        if not data:
            return
        now = self._clock()
        with self._lock:
            self.bytes_rx += len(data)
            self.last_rx_time = now
            self._rate.append((now, len(data)))
            self._trim_rate(now)
            frames = self._frames.extract(data)
            if frames:
                self.last_frame_time = now
            if self.state != STATE_STREAMING:
                self.state = STATE_STREAMING
        if self.config.validate_rtcm:
            # Only whole, CRC-checked frames reach the receiver; partial frames
            # wait in the framer for the rest of their bytes.
            if not frames:
                return
            out = b"".join(frames)
        else:
            out = data
        try:
            self._write_rtcm(out)
            self.bytes_written += len(out)
            self.frames_written += len(frames)
        except Exception as exc:  # noqa: BLE001 - a serial hiccup must not kill the client
            self.write_errors += 1
            if self.write_errors in (1, 10, 100) or self.write_errors % 1000 == 0:
                self._log("warn", "RTCM write failed (%d so far): %s" % (self.write_errors, exc))

    # -- GGA upload ------------------------------------------------------------------
    def _current_gga(self) -> Optional[str]:
        try:
            gga = self._get_gga()
        except Exception:  # noqa: BLE001
            return None
        if not gga:
            return None
        gga = gga.strip()
        return gga + "\r\n" if gga.startswith("$") else None

    def _maybe_send_gga(self, force: bool = False) -> None:
        interval = self.config.gga_interval_s
        if interval <= 0 or self._sock is None:
            return
        now = self._clock()
        last = self._last_gga_attempt
        if not force and last is not None and now - last < interval:
            return
        gga = self._current_gga()
        if gga is None:
            return  # nothing to send yet; retry on the next loop pass
        self._last_gga_attempt = now
        self._sock.sendall(gga.encode("ascii", errors="ignore"))
        with self._lock:
            self.gga_sent += 1
            self.last_gga_time = now

    _last_gga_attempt: Optional[float] = None
    _decoder: Optional[ChunkedDecoder] = None
    _chunked = False


__all__ = [
    "NtripClient",
    "NtripConfig",
    "NtripError",
    "ChunkedDecoder",
    "Rtcm3FrameCounter",
    "build_request",
    "parse_response_head",
    "make_gga",
    "nmea_wrap",
    "crc24q",
    "rtcm3_frame",
    "STATE_OFF",
    "STATE_CONNECTING",
    "STATE_CONNECTED",
    "STATE_STREAMING",
    "STATE_RECONNECTING",
    "STATE_AUTH_FAILED",
    "STATE_BAD_MOUNTPOINT",
    "STATE_ERROR",
    "VENDOR_NTRIP_FILE",
    "load_vendor_ntrip_file",
    "fetch_sourcetable",
    "parse_sourcetable",
]


# --------------------------------------------------------------------------------------
# vendor credentials file
# --------------------------------------------------------------------------------------
VENDOR_NTRIP_FILE = "/userdata/mower/ntrip.yaml"

# Vendor ntripConfig(): ntrip_netmode -> interface for SO_BINDTODEVICE.
# "auto" (and our default "") leave the socket unbound.
NETMODE_DEVICES = {"4g": "usb0", "wifi": "wlan0", "auto": "", "": ""}


def load_vendor_ntrip_file(path: str) -> Optional[Dict[str, object]]:
    """Read the vendor's ``/userdata/mower/ntrip.yaml``.

    Keys (all flat): ``ntrip_enable`` (bool), ``ntrip_ip``, ``ntrip_port``,
    ``ntrip_user``, ``ntrip_passwd``, ``ntrip_mountpoint``. Returns a dict with
    ``enable/host/port/user/password/mountpoint`` or None when the file is absent
    or unreadable. PyYAML is used when installed; otherwise a flat ``key: value``
    reader (the vendor file is flat) is used, so no new dependency is needed.
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return None
    raw: Dict[str, object] = {}
    try:
        import yaml  # type: ignore

        loaded = yaml.safe_load(text)
        if isinstance(loaded, dict):
            raw = loaded
    except ImportError:
        for line in text.splitlines():
            line = line.split("#", 1)[0].strip()
            if ":" not in line:
                continue
            key, _, value = line.partition(":")
            value = value.strip()
            if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
                value = value[1:-1]
            raw[key.strip()] = value
    except Exception:  # noqa: BLE001 - malformed YAML: behave as if absent
        return None

    def _str(key: str) -> str:
        value = raw.get(key)
        return "" if value is None else str(value).strip()

    try:
        port = int(raw.get("ntrip_port") or 0)
    except (TypeError, ValueError):
        port = 0
    enable = raw.get("ntrip_enable")
    if isinstance(enable, str):
        enable = enable.strip().lower() in ("true", "1", "yes", "on")
    return {
        "enable": bool(enable),
        "host": _str("ntrip_ip"),
        "port": port,
        "user": _str("ntrip_user"),
        "password": _str("ntrip_passwd"),
        "mountpoint": _str("ntrip_mountpoint"),
        "netmode": _str("ntrip_netmode"),
    }


# --------------------------------------------------------------------------------------
# sourcetable (mountpoint picker)
# --------------------------------------------------------------------------------------
# STR record fields (NTRIP 2.0, RTCM 10410.1 table 7): index -> name.
_STR_FIELDS = ("type", "mountpoint", "identifier", "format", "format_details", "carrier",
               "nav_system", "network", "country", "lat", "lon", "nmea", "solution",
               "generator", "compression", "authentication", "fee", "bitrate", "misc")


def parse_sourcetable(text: str) -> Dict[str, List[Dict[str, object]]]:
    """Parse a caster sourcetable into ``{"streams": [...], "casters": [...], "networks": [...]}``.

    Each stream is a dict keyed by the STR field names; ``lat``/``lon`` are floats
    (0.0 when missing), ``nmea`` (the stream wants a GGA upload) is a bool and
    ``bitrate`` an int. Malformed lines are skipped; parsing stops at
    ``ENDSOURCETABLE``.
    """
    streams: List[Dict[str, object]] = []
    casters: List[Dict[str, object]] = []
    networks: List[Dict[str, object]] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.upper().startswith("ENDSOURCETABLE"):
            break
        fields = line.split(";")
        kind = fields[0].upper()
        if kind == "STR" and len(fields) >= 2 and fields[1]:
            rec: Dict[str, object] = {}
            for i, name in enumerate(_STR_FIELDS):
                rec[name] = fields[i].strip() if i < len(fields) else ""
            rec["type"] = "STR"
            for key in ("lat", "lon"):
                try:
                    rec[key] = float(rec[key] or 0.0)
                except ValueError:
                    rec[key] = 0.0
            rec["nmea"] = str(rec["nmea"]) == "1"
            try:
                rec["bitrate"] = int(rec["bitrate"] or 0)
            except ValueError:
                rec["bitrate"] = 0
            streams.append(rec)
        elif kind == "CAS" and len(fields) >= 3:
            casters.append({"host": fields[1], "port": fields[2],
                            "identifier": fields[3] if len(fields) > 3 else ""})
        elif kind == "NET" and len(fields) >= 2:
            networks.append({"identifier": fields[1],
                             "operator": fields[2] if len(fields) > 2 else ""})
    return {"streams": streams, "casters": casters, "networks": networks}


def fetch_sourcetable(host: str, port: int = 2101, user: str = "", password: str = "",
                      version: int = 2, timeout_s: float = 8.0,
                      max_bytes: int = 4 * 1024 * 1024) -> Dict[str, List[Dict[str, object]]]:
    """Download and parse the caster sourcetable (``GET /``).

    Raises ``NtripError`` (state ``auth_failed`` / ``error``) or ``OSError``.
    """
    sock = socket.create_connection((host, int(port)), timeout=timeout_s)
    try:
        sock.settimeout(timeout_s)
        sock.sendall(build_request(host, int(port), "", user, password, version, None))
        buf = bytearray()
        deadline = time.monotonic() + timeout_s
        while len(buf) < max_bytes and time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            buf.extend(chunk)
            if b"ENDSOURCETABLE" in buf:
                break
    finally:
        sock.close()
    head, sep, body = bytes(buf).partition(b"\r\n\r\n")
    if not sep:
        raise NtripError(STATE_ERROR, "no response from caster")
    status, code, headers = parse_response_head(head)
    if code in (401, 403):
        raise NtripError(STATE_AUTH_FAILED, "caster refused the credentials (%s)" % status)
    if code != 200 and not status.upper().startswith("SOURCETABLE"):
        raise NtripError(STATE_ERROR, "unexpected caster response %r" % status[:80])
    if "chunked" in headers.get("transfer-encoding", "").lower():
        body = ChunkedDecoder().feed(body)
    return parse_sourcetable(body.decode("latin-1", "replace"))
