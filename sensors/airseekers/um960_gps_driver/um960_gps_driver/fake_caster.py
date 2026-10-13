"""A tiny fake NTRIP caster for tests and bench checks (no real corrections).

Serves one mountpoint, checks basic auth, answers like an NTRIP 1 (``ICY 200 OK``)
or NTRIP 2 (``HTTP/1.1 200 OK``, optionally chunked) caster, streams synthetic but
CRC-valid RTCM 3 frames, and records every line the client uploads (GGA).

    python3 -m um960_gps_driver.fake_caster --port 2101 --mount TEST --user u --password p
"""

import argparse
import base64
import socket
import socketserver
import sys
import threading
import time
from typing import List, Optional

from .ntrip_client import rtcm3_frame


def sample_frame(msg_type: int = 1005, seq: int = 0, size: int = 19) -> bytes:
    """A CRC-valid RTCM 3 frame whose first 12 bits carry ``msg_type``."""
    payload = bytearray(max(size, 2))
    payload[0] = (msg_type >> 4) & 0xFF
    payload[1] = ((msg_type & 0x0F) << 4) | (seq & 0x0F)
    for i in range(2, len(payload)):
        payload[i] = (seq + i) & 0xFF
    return rtcm3_frame(bytes(payload))


class FakeCaster:
    """Threaded fake caster bound to ``127.0.0.1:<port>`` (0 = pick a free port).

    ``mode``: ``"v2"`` (HTTP/1.1 200, plain body), ``"v2_chunked"``, ``"icy"``
    (ICY 200 OK, NTRIP 1), ``"sourcetable"`` (pretend the mountpoint is unknown).
    ``close_after`` closes each connection after that many frames (reconnect tests).
    """

    def __init__(self, port: int = 0, mountpoint: str = "TEST", user: str = "user",
                 password: str = "pass", mode: str = "v2", frame_period_s: float = 0.05,
                 close_after: Optional[int] = None, host: str = "127.0.0.1",
                 corrupt_every: int = 0, sourcetable: Optional[str] = None) -> None:
        self.mountpoint = mountpoint
        self.user = user
        self.password = password
        self.mode = mode
        self.frame_period_s = frame_period_s
        self.close_after = close_after
        # Every Nth frame goes out with a broken CRC, preceded by junk bytes.
        self.corrupt_every = int(corrupt_every)
        self.corrupt_sent = 0
        self.sourcetable = sourcetable if sourcetable is not None else (
            "STR;%s;Test;RTCM 3.2;1005(10),1077(1);2;GPS+GLO;TESTNET;DEU;52.52;13.40;1;0;"
            "fake;none;B;N;9600;\r\nENDSOURCETABLE\r\n" % mountpoint)
        self.requests: List[str] = []       # raw request heads
        self.uploads: List[str] = []        # lines the client sent after the handshake
        self.upload_times: List[float] = []
        self.connections = 0
        self.frames_sent = 0
        self._stop = threading.Event()
        caster = self

        class Handler(socketserver.BaseRequestHandler):
            def handle(self) -> None:  # noqa: D401 - socketserver API
                caster._handle(self.request)

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._server = Server((host, port), Handler)
        self.host, self.port = self._server.server_address[:2]
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        kwargs={"poll_interval": 0.05}, daemon=True)

    # -- lifecycle -------------------------------------------------------------------
    def start(self) -> "FakeCaster":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._server.shutdown()
        self._server.server_close()

    def __enter__(self) -> "FakeCaster":
        return self.start()

    def __exit__(self, *_exc) -> None:
        self.stop()

    # -- protocol --------------------------------------------------------------------
    def _handle(self, conn) -> None:
        conn.settimeout(2.0)
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = conn.recv(4096)
            if not chunk:
                return
            head += chunk
        head, _, extra = head.partition(b"\r\n\r\n")
        text = head.decode("latin-1")
        self.requests.append(text)
        self.connections += 1
        lines = text.split("\r\n")
        path = lines[0].split()[1] if len(lines[0].split()) > 1 else "/"
        headers = {}
        for line in lines[1:]:
            key, _, value = line.partition(":")
            headers[key.strip().lower()] = value.strip()

        if self.mode == "sourcetable" or path.lstrip("/") != self.mountpoint:
            body = self.sourcetable
            conn.sendall(("SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n"
                          "Content-Length: %d\r\n\r\n%s" % (len(body), body)).encode())
            return
        expected = "Basic " + base64.b64encode(
            ("%s:%s" % (self.user, self.password)).encode()).decode()
        if headers.get("authorization") != expected:
            if self.mode == "icy":
                conn.sendall(b"HTTP/1.0 401 Unauthorized\r\n\r\n")
            else:
                conn.sendall(b"HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic "
                             b"realm=\"/" + self.mountpoint.encode() + b"\"\r\n"
                             b"Content-Length: 0\r\n\r\n")
            return
        if "ntrip-gga" in headers:
            self._record(headers["ntrip-gga"])

        chunked = self.mode == "v2_chunked"
        if self.mode == "icy":
            conn.sendall(b"ICY 200 OK\r\n")
        else:
            conn.sendall(b"HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n"
                         b"Content-Type: gnss/data\r\nCache-Control: no-store\r\n"
                         + (b"Transfer-Encoding: chunked\r\n" if chunked else b"")
                         + b"Connection: close\r\n\r\n")

        reader = threading.Thread(target=self._read_uploads, args=(conn, extra), daemon=True)
        reader.start()
        seq = 0
        try:
            while not self._stop.is_set():
                frame = sample_frame(1005 if seq % 2 == 0 else 1077, seq)
                if self.corrupt_every and seq % self.corrupt_every == self.corrupt_every - 1:
                    frame = b"junk" + frame[:-1] + bytes([frame[-1] ^ 0xFF])
                    self.corrupt_sent += 1
                if chunked:
                    # Split each frame over two chunks so chunk and frame boundaries
                    # do not line up.
                    half = len(frame) // 2
                    for part in (frame[:half], frame[half:]):
                        conn.sendall(b"%X\r\n" % len(part) + part + b"\r\n")
                else:
                    conn.sendall(frame)
                self.frames_sent += 1
                seq += 1
                if self.close_after is not None and seq >= self.close_after:
                    if chunked:
                        conn.sendall(b"0\r\n\r\n")
                    return
                time.sleep(self.frame_period_s)
        except OSError:
            return

    def _read_uploads(self, conn, pending: bytes) -> None:
        buf = pending
        while not self._stop.is_set():
            while b"\n" in buf:
                line, _, buf = buf.partition(b"\n")
                if line.strip():
                    self._record(line.decode("latin-1").strip())
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue  # the client uploads only every gga interval
            except OSError:
                return
            if not chunk:
                return
            buf += chunk

    def _record(self, line: str) -> None:
        self.uploads.append(line)
        self.upload_times.append(time.monotonic())


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=2101)
    parser.add_argument("--mount", default="TEST")
    parser.add_argument("--user", default="user")
    parser.add_argument("--password", default="pass")
    parser.add_argument("--mode", default="v2", choices=["v2", "v2_chunked", "icy", "sourcetable"])
    parser.add_argument("--period", type=float, default=0.2, help="seconds between frames")
    parser.add_argument("--duration", type=float, default=0.0, help="exit after N s (0 = never)")
    args = parser.parse_args(argv)
    caster = FakeCaster(port=args.port, mountpoint=args.mount, user=args.user,
                        password=args.password, mode=args.mode,
                        frame_period_s=args.period).start()
    print("fake caster on %s:%d mount=%s mode=%s" % (caster.host, caster.port, args.mount,
                                                     args.mode), flush=True)
    seen = 0
    start = time.monotonic()
    try:
        while not args.duration or time.monotonic() - start < args.duration:
            time.sleep(0.1)
            while seen < len(caster.uploads):
                print("caster <- %s" % caster.uploads[seen], flush=True)
                seen += 1
    except KeyboardInterrupt:
        pass
    finally:
        print("connections=%d frames_sent=%d uploads=%d" % (
            caster.connections, caster.frames_sent, len(caster.uploads)), flush=True)
        caster.stop()


if __name__ == "__main__":
    main(sys.argv[1:])
