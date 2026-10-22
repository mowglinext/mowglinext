# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Simulation-only native observations; Python receipt time is never provenance."""

import json
import math
import os
from pathlib import Path


class NativeTrace:
    def __init__(self, path):
        self.path = Path(path)
        self.identity = None
        self.offset = 0
        self.pending = b""
        self.rows = []

    def read(self):
        try:
            with self.path.open("rb") as stream:
                stat = os.fstat(stream.fileno())
                identity = (stat.st_dev, stat.st_ino)
                if self.identity is not None and self.identity != identity:
                    raise ValueError("native trace file replaced")
                if stat.st_size < self.offset:
                    raise ValueError("native trace truncated")
                self.identity = identity
                stream.seek(self.offset)
                data = stream.read()
        except FileNotFoundError:
            if self.identity is not None:
                raise ValueError("native trace disappeared") from None
            return self.rows
        self.offset += len(data)
        lines = (self.pending + data).split(b"\n")
        self.pending = lines.pop()  # A write may currently be in progress.
        for line in lines:
            row = json.loads(line)
            if row["seq"] != len(self.rows) or not isinstance(row["mono_ns"], int):
                raise ValueError("invalid or missing native trace observation")
            if self.rows and row["mono_ns"] < self.rows[-1]["mono_ns"]:
                raise ValueError("native monotonic clock regressed")
            if not self.rows and (
                row["event"] != "identity" or row["schema"] != 1 or row["pid"] <= 0
            ):
                raise ValueError("unknown native trace producer/schema")
            self.rows.append(row)
        return self.rows


def heartbeat_cadence(rows, end_ns, start_ns=None):
    """Strict 150 ms bound on completed native sends, including the silent tail.

    A stopped sender cannot pass merely because all of its old intervals were
    regular. No Python decode/service stamp participates in this computation.
    """
    if not rows:
        return (
            False,
            "no native evidence; build this bench with -DOPENMOWER_SIMULATION_TRACE=ON",
        )
    writes = [
        r for r in rows if r["event"] == "heartbeat_write" and r["end_ns"] <= end_ns
    ]
    successes = [r for r in writes if r["written"] == 1]
    if len(successes) < 2:
        return False, "fewer than two native heartbeat emissions"
    start_ns = successes[0]["end_ns"] if start_ns is None else start_ns
    before = [r for r in successes if r["end_ns"] <= start_ns]
    if not before:
        return False, "no native heartbeat covering the start of the window"
    stamps = [before[-1]["end_ns"]] + [
        r["end_ns"] for r in successes if r["end_ns"] > start_ns
    ]
    if any(r["written"] != 1 for r in writes if r["end_ns"] >= start_ns):
        return False, "native heartbeat write failed"
    if any(r["end_ns"] < r["begin_ns"] for r in writes):
        return False, "invalid native write timestamps"
    gaps = [b - a for a, b in zip(stamps, stamps[1:])] + [end_ns - stamps[-1]]
    maximum = max(gaps)
    return (
        maximum < 150_000_000,
        f"native max gap/tail {maximum/1e6:.3f} ms; {len(stamps)} emissions",
    )


def matching_blade_write(rows, gate):
    return next(
        (
            r
            for r in rows
            if r["event"] == "blade_write" and r["cycle"] == gate["cycle"]
        ),
        None,
    )


def commanded_zero(rows, gate):
    write = matching_blade_write(rows, gate)
    return (
        write is not None
        and write["written"] == 1
        and math.isfinite(write["duty"])
        and abs(write["duty"]) < 1e-9
    )


def stale_blade_window(rows, expired_gate, end_ns):
    """Every observed stale control cycle must actually write zero to the ESC.

    Require complete coverage, not an empty set, a lone effective watchdog zero,
    or old records from before a stopped bridge. Any newer HLS invalidates this
    observation window rather than silently shortening it.
    """
    gates = [
        r
        for r in rows
        if r["event"] == "gate" and expired_gate["mono_ns"] <= r["mono_ns"] <= end_ns
    ]
    if len(gates) < 2:
        return False, "insufficient native control coverage"
    if any(r["hl_stamp_ns"] != expired_gate["hl_stamp_ns"] for r in gates):
        return False, "new HLS received during the intended silence"
    stamps = [g["mono_ns"] for g in gates]
    if (
        max([b - a for a, b in zip(stamps, stamps[1:])] + [end_ns - stamps[-1]])
        >= 150_000_000
    ):
        return False, "native control observation gap"
    for gate in gates:
        if gate["hl_fresh"] or gate["allowed"] or not commanded_zero(rows, gate):
            return False, "stale HLS did not continuously command zero"
    return True, f"{len(gates)} stale control cycles wrote zero; no HLS renewal/rearm"
