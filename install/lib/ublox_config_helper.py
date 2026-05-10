#!/usr/bin/env python3
"""Minimal u-blox UBX helper for install-time serial reconfiguration."""

from __future__ import annotations

import os
import select
import struct
import sys
import termios
import time

SYNC = b"\xb5\x62"
ACK_ACK = (0x05, 0x01)
ACK_NAK = (0x05, 0x00)
CFG_PRT = (0x06, 0x00)
CFG_CFG = (0x06, 0x09)
MON_VER = (0x0A, 0x04)


def checksum(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    data = bytes((msg_class, msg_id)) + struct.pack("<H", len(payload)) + payload
    for byte in data:
      ck_a = (ck_a + byte) & 0xFF
      ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


def build_frame(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    return SYNC + bytes((msg_class, msg_id)) + struct.pack("<H", len(payload)) + payload + checksum(msg_class, msg_id, payload)


def configure_raw_port(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_available(fd: int, timeout_s: float) -> bytes:
    end_time = time.monotonic() + timeout_s
    chunks: list[bytes] = []

    while time.monotonic() < end_time:
        wait_s = max(0.0, end_time - time.monotonic())
        readable, _, _ = select.select([fd], [], [], min(0.1, wait_s))
        if not readable:
            continue
        chunk = os.read(fd, 4096)
        if chunk:
            chunks.append(chunk)
    return b"".join(chunks)


def iter_frames(buffer: bytes):
    index = 0
    limit = len(buffer)
    while index + 8 <= limit:
        sync_index = buffer.find(SYNC, index)
        if sync_index < 0:
            return
        if sync_index + 8 > limit:
            return
        msg_class = buffer[sync_index + 2]
        msg_id = buffer[sync_index + 3]
        payload_len = struct.unpack_from("<H", buffer, sync_index + 4)[0]
        frame_end = sync_index + 6 + payload_len + 2
        if frame_end > limit:
            return
        payload = buffer[sync_index + 6 : sync_index + 6 + payload_len]
        expected = checksum(msg_class, msg_id, payload)
        if buffer[frame_end - 2 : frame_end] == expected:
            yield msg_class, msg_id, payload
        index = frame_end


def read_frame(fd: int, wanted: tuple[int, int], timeout_s: float) -> bytes:
    end_time = time.monotonic() + timeout_s
    buffer = b""
    while time.monotonic() < end_time:
        wait_s = max(0.0, end_time - time.monotonic())
        readable, _, _ = select.select([fd], [], [], min(0.1, wait_s))
        if not readable:
            continue
        chunk = os.read(fd, 4096)
        if not chunk:
            continue
        buffer += chunk
        for msg_class, msg_id, payload in iter_frames(buffer):
            if (msg_class, msg_id) == wanted:
                return payload
            if (msg_class, msg_id) == ACK_NAK and wanted[0] == 0x06:
                ack_class, ack_id = payload[0], payload[1]
                if (ack_class, ack_id) == wanted:
                    raise RuntimeError("device returned UBX-ACK-NAK")
    raise TimeoutError(f"timeout waiting for UBX {wanted[0]:02x}/{wanted[1]:02x}")


def write_frame(fd: int, msg_class: int, msg_id: int, payload: bytes = b"") -> None:
    os.write(fd, build_frame(msg_class, msg_id, payload))


def flush_input(fd: int) -> None:
    read_available(fd, 0.15)


def parse_mon_ver(payload: bytes) -> tuple[str, str]:
    sw_version = payload[:30].split(b"\x00", 1)[0].decode("ascii", "ignore").strip()
    hw_version = payload[30:40].split(b"\x00", 1)[0].decode("ascii", "ignore").strip()
    extensions = payload[40:]
    ext_values = []
    for offset in range(0, len(extensions), 30):
        field = extensions[offset : offset + 30].split(b"\x00", 1)[0].decode("ascii", "ignore").strip()
        if field:
            ext_values.append(field)
    return f"{sw_version} {hw_version}".strip(), " | ".join(ext_values)


def identify(fd: int) -> None:
    flush_input(fd)
    write_frame(fd, MON_VER[0], MON_VER[1], b"")
    payload = read_frame(fd, MON_VER, 1.5)
    summary, extra = parse_mon_ver(payload)
    text = " ".join(part for part in (summary, extra) if part).strip()
    if "u-blox" not in text and not summary:
        raise RuntimeError("MON-VER did not look like a u-blox response")
    print(text or "u-blox")


def read_cfg_prt(fd: int, port_id: int) -> bytes:
    flush_input(fd)
    write_frame(fd, CFG_PRT[0], CFG_PRT[1], bytes((port_id,)))
    payload = read_frame(fd, CFG_PRT, 1.5)
    if len(payload) != 20 or payload[0] != port_id:
        raise RuntimeError(f"unexpected CFG-PRT payload for port {port_id}")
    return payload


def set_baud(fd: int, port_id: int, baudrate: int) -> None:
    current = read_cfg_prt(fd, port_id)
    updated = bytearray(current)
    struct.pack_into("<I", updated, 8, baudrate)
    flush_input(fd)
    write_frame(fd, CFG_PRT[0], CFG_PRT[1], bytes(updated))
    ack = read_frame(fd, ACK_ACK, 1.5)
    if ack[:2] != bytes(CFG_PRT):
        raise RuntimeError("unexpected ACK payload for CFG-PRT")


def save_config(fd: int) -> None:
    # clearMask=0 saveMask=0x0000FFFF loadMask=0 devMask=0x17 (BBR+Flash+EEPROM+SPI Flash)
    payload = struct.pack("<IIIb", 0, 0x0000FFFF, 0, 0x17) + b"\x00\x00\x00"
    flush_input(fd)
    write_frame(fd, CFG_CFG[0], CFG_CFG[1], payload)
    ack = read_frame(fd, ACK_ACK, 1.5)
    if ack[:2] != bytes(CFG_CFG):
        raise RuntimeError("unexpected ACK payload for CFG-CFG")


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: ublox_config_helper.py <port> <identify|set-baud|save> [...]", file=sys.stderr)
        return 2

    port = sys.argv[1]
    command = sys.argv[2]

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_raw_port(fd)
        if command == "identify":
            identify(fd)
        elif command == "set-baud":
            if len(sys.argv) != 5:
                raise RuntimeError("set-baud requires <port-id> <baud>")
            set_baud(fd, int(sys.argv[3]), int(sys.argv[4]))
        elif command == "save":
            save_config(fd)
        else:
            raise RuntimeError(f"unknown command: {command}")
        return 0
    except Exception as exc:
        print(f"ublox helper error: {exc}", file=sys.stderr)
        return 1
    finally:
        os.close(fd)
