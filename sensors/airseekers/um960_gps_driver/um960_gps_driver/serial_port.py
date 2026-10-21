"""Minimal termios-based serial reader for the UM960 (no pyserial dependency)."""

import errno
import os
import select
import termios
import time
from typing import List, Optional

# termios speed constant lookup; B115200 etc. exist on Linux.
BAUD_RATES = {
    9600: getattr(termios, "B9600", None),
    19200: getattr(termios, "B19200", None),
    38400: getattr(termios, "B38400", None),
    57600: getattr(termios, "B57600", None),
    115200: getattr(termios, "B115200", None),
    230400: getattr(termios, "B230400", None),
    460800: getattr(termios, "B460800", None),
    500000: getattr(termios, "B500000", None),
    576000: getattr(termios, "B576000", None),
    921600: getattr(termios, "B921600", None),
    1000000: getattr(termios, "B1000000", None),
}


class SerialError(IOError):
    """Raised when the serial port cannot be opened or configured."""


class SerialPort:
    """Blocking-ish read-only serial port.

    Opens the device raw (no canonical processing, no flow control, no parity),
    8N1, and exposes a ``read`` with a timeout implemented via ``select``.
    """

    def __init__(self, device: str, baudrate: int = 115200, timeout: float = 0.2) -> None:
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self._fd: Optional[int] = None

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    def open(self) -> None:
        if self._fd is not None:
            return
        speed = BAUD_RATES.get(self.baudrate)
        if speed is None:
            raise SerialError("unsupported baudrate %d" % self.baudrate)
        try:
            fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise SerialError("cannot open %s: %s" % (self.device, exc)) from exc

        attrs = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag = 0  # no parity / CR-NL translation / flow control
        oflag = 0
        cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
        lflag = 0  # no ICANON, no ECHO, no ISIG
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        try:
            termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
            termios.tcflush(fd, termios.TCIOFLUSH)
        except termios.error as exc:
            os.close(fd)
            raise SerialError("cannot configure %s: %s" % (self.device, exc)) from exc
        self._fd = fd

    def close(self) -> None:
        # Detach the descriptor first so a concurrent close() (reader thread plus
        # destroy_node) cannot double-close the same fd.
        fd = self._fd
        self._fd = None
        if fd is None:
            return
        try:
            termios.tcflush(fd, termios.TCIOFLUSH)
        except (termios.error, OSError):
            pass
        try:
            os.close(fd)
        except OSError:
            pass

    def read(self, max_bytes: int = 4096) -> bytes:
        """Read up to ``max_bytes``, waiting at most ``timeout`` seconds."""
        fd = self._fd
        if fd is None:
            raise SerialError("port %s is not open" % self.device)
        # Work from the local fd: close() may run concurrently (destroy_node, a
        # reconnect) and must not turn into a TypeError on a vanished descriptor.
        try:
            ready, _, _ = select.select([fd], [], [], self.timeout)
        except (OSError, ValueError):
            return b""
        if not ready:
            return b""
        try:
            return os.read(fd, max_bytes)
        except BlockingIOError:
            return b""
        except OSError as exc:
            # EBADF / EIO: the device went away underneath us (cable pulled, port
            # closed by someone else). Surface it so the caller can reconnect
            # instead of spinning on an empty read forever.
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                return b""
            raise SerialError("read from %s failed: %s" % (self.device, exc)) from exc

    def write(self, data: bytes) -> int:
        """Write raw bytes (used for receiver configuration commands)."""
        if self._fd is None:
            raise SerialError("port %s is not open" % self.device)
        try:
            return os.write(self._fd, data)
        except OSError as exc:
            raise SerialError("write to %s failed: %s" % (self.device, exc)) from exc

    def write_all(self, data: bytes, timeout: float = 1.0) -> int:
        """Write every byte (the fd is non-blocking), waiting up to ``timeout`` s.

        Used for RTCM injection: a partial write would cut an RTCM frame in half.
        """
        fd = self._fd
        if fd is None:
            raise SerialError("port %s is not open" % self.device)
        view = memoryview(data)
        written = 0
        deadline = time.monotonic() + timeout
        while written < len(view):
            try:
                written += os.write(fd, view[written:])
                continue
            except (BlockingIOError, InterruptedError):
                pass
            except OSError as exc:
                raise SerialError("write to %s failed: %s" % (self.device, exc)) from exc
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SerialError("write to %s timed out after %d/%d bytes"
                                  % (self.device, written, len(view)))
            try:
                select.select([], [fd], [], remaining)
            except (OSError, ValueError) as exc:
                raise SerialError("write to %s failed: %s" % (self.device, exc)) from exc
        return written

    def write_line(self, line: str) -> int:
        return self.write((line.rstrip("\r\n") + "\r\n").encode("ascii", errors="ignore"))

    def __enter__(self) -> "SerialPort":
        self.open()
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()


def available_ports() -> List[str]:
    """List serial devices present on the machine (best effort)."""
    import glob

    return sorted(
        glob.glob("/dev/serial_*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    )
