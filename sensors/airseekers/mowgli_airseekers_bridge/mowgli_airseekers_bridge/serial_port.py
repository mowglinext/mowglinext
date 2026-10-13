# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Raw 8N1 serial port on termios (no pyserial dependency)."""

from __future__ import annotations

import os
import select
import termios

BAUDS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
         57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
         460800: getattr(termios, 'B460800', termios.B230400),
         921600: getattr(termios, 'B921600', termios.B230400)}


class SerialPort:
    def __init__(self, path: str, baud: int) -> None:
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            speed = BAUDS.get(int(baud), termios.B115200)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except termios.error:
            pass  # not a tty (tests): leave it alone

    def fileno(self) -> int:
        return self.fd

    def read(self, size: int = 4096) -> bytes:
        try:
            data = os.read(self.fd, size)
        except BlockingIOError:
            return b''
        if data == b'':
            # A tty that hung up reads EOF while select() keeps reporting it
            # readable: surface it so the caller reopens.
            raise OSError('EOF on %s' % self.path)
        return data

    def write(self, data: bytes, timeout_s: float = 0.2) -> int:
        view = memoryview(data)
        written = 0
        while view:
            _, ready, _ = select.select([], [self.fd], [], timeout_s)
            if not ready:
                raise OSError('write timeout on %s' % self.path)
            try:
                n = os.write(self.fd, view)
            except BlockingIOError:
                continue
            view = view[n:]
            written += n
        return written

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass
