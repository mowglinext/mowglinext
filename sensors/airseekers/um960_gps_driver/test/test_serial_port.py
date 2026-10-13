"""Serial port tests, exercised against a real pty pair (no hardware needed)."""

import os
import pty
import termios

import pytest

from um960_gps_driver.serial_port import BAUD_RATES, SerialError, SerialPort, available_ports


@pytest.fixture
def pty_device():
    master, slave = pty.openpty()
    try:
        yield os.ttyname(slave), master
    finally:
        os.close(master)
        os.close(slave)


def test_open_configures_raw_8n1(pty_device):
    device, _ = pty_device
    port = SerialPort(device, 115200, 0.2)
    port.open()
    try:
        assert port.is_open
        iflag, oflag, cflag, lflag, ispeed, ospeed, _ = termios.tcgetattr(port._fd)
        assert iflag == 0 and oflag == 0 and lflag == 0
        assert ispeed == termios.B115200 and ospeed == termios.B115200
        assert cflag & termios.CS8
        assert not cflag & termios.PARENB
        assert not cflag & termios.CSTOPB
        assert cflag & termios.CREAD
    finally:
        port.close()
    assert not port.is_open


def test_read_and_write(pty_device):
    device, master = pty_device
    with SerialPort(device, 115200, 0.2) as port:
        os.write(master, b"$GNVER,UM960\r\n")
        assert b"$GNVER" in port.read(64)
        assert port.write_line("$GNRTK,ON") > 0


def test_read_timeout_returns_empty(pty_device):
    device, _ = pty_device
    with SerialPort(device, 115200, 0.05) as port:
        assert port.read(64) == b""


def test_missing_device_raises():
    with pytest.raises(SerialError):
        SerialPort("/dev/definitely_missing_xyz", 115200).open()


def test_unsupported_baud_raises(pty_device):
    device, _ = pty_device
    with pytest.raises(SerialError):
        SerialPort(device, 12345).open()


def test_read_on_closed_port_raises(pty_device):
    device, _ = pty_device
    port = SerialPort(device, 115200, 0.2)
    with pytest.raises(SerialError):
        port.read(16)


def test_baud_table_and_helper():
    assert BAUD_RATES[115200] == termios.B115200
    assert BAUD_RATES[921600] == termios.B921600
    assert isinstance(available_ports(), list)
