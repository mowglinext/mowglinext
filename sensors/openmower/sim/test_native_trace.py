# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Deterministic attribution checks plus a real native emitter/GIL regression."""

import ctypes
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import time
import unittest

from native_trace import (
    NativeTrace,
    commanded_zero,
    heartbeat_cadence,
    stale_blade_window,
)
from openmower_v1_emulator import LowLevelBoard, Plant, cobs_packet, LL_HEARTBEAT


def heartbeat(stamp, written=1):
    return dict(event="heartbeat_write", begin_ns=stamp, end_ns=stamp, written=written)


def gate(stamp, cycle, fresh=0, allowed=0, receipt=1):
    return dict(
        event="gate",
        mono_ns=stamp,
        cycle=cycle,
        hl_fresh=fresh,
        allowed=allowed,
        hl_stamp_ns=receipt,
    )


def duty(cycle, value=0, written=1):
    return dict(event="blade_write", cycle=cycle, duty=value, written=written)


class MeasurementTest(unittest.TestCase):
    def test_true_native_emission_gap_stays_red_at_original_150_ms(self):
        rows = [heartbeat(t * 1_000_000) for t in [0, 40, 80, 280]]
        self.assertFalse(heartbeat_cadence(rows, 280_000_000)[0])

    def test_regular_old_intervals_do_not_hide_a_stopped_sender(self):
        rows = [heartbeat(t * 1_000_000) for t in [0, 40, 80]]
        self.assertFalse(heartbeat_cadence(rows, 240_000_000)[0])
        self.assertFalse(
            heartbeat_cadence(rows, 230_000_000)[0]
        )  # exactly 150 is not <150

    def test_receiver_delay_never_relabels_native_emission_time(self):
        rows = [heartbeat(t * 1_000_000) for t in range(0, 601, 40)]
        python_service = [0, 40, 484, 524, 564, 604]
        self.assertGreater(
            max(b - a for a, b in zip(python_service, python_service[1:])), 150
        )
        self.assertTrue(heartbeat_cadence(rows, 604_000_000)[0])

    def test_failed_native_write_or_missing_provenance_cannot_pass(self):
        self.assertFalse(heartbeat_cadence([], 100)[0])
        rows = [heartbeat(0), heartbeat(40_000_000, written=0), heartbeat(80_000_000)]
        self.assertFalse(heartbeat_cadence(rows, 90_000_000)[0])

    def test_transient_ll_zero_is_not_a_stale_hls_window(self):
        transient = gate(1_174_000_000, 1, fresh=1)
        resumed = gate(1_194_000_000, 2, fresh=1, allowed=1)
        rows = [transient, duty(1), resumed, duty(2, 1)]
        self.assertFalse(stale_blade_window(rows, transient, 1_200_000_000)[0])

    def test_stale_hls_requires_every_cycle_to_write_zero(self):
        rows = [gate(t * 1_000_000, i) for i, t in enumerate(range(5000, 6021, 20))]
        first = rows[0]
        rows += [duty(g["cycle"]) for g in rows]
        self.assertTrue(stale_blade_window(rows, first, 6_020_000_000)[0])
        for bad in [duty(25, 0.1), duty(25, written=0)]:
            with self.subTest(bad=bad):
                changed = [
                    r
                    for r in rows
                    if not (r["event"] == "blade_write" and r["cycle"] == 25)
                ] + [bad]
                self.assertFalse(stale_blade_window(changed, first, 6_020_000_000)[0])

    def test_esc_watchdog_zero_cannot_replace_a_bridge_write(self):
        expired = gate(5_000_000_001, 1)
        self.assertFalse(commanded_zero([expired], expired))
        self.assertFalse(stale_blade_window([expired], expired, 6_000_000_000)[0])

    def test_native_observation_gap_or_new_hls_invalidates_stale_proof(self):
        first = gate(5_000_000_001, 1)
        rows = [first, duty(1), gate(5_300_000_001, 2), duty(2)]
        self.assertFalse(stale_blade_window(rows, first, 5_300_000_001)[0])
        rows[2] = gate(5_020_000_001, 2, receipt=2)
        self.assertFalse(stale_blade_window(rows, first, 5_020_000_001)[0])

    def test_trace_rejects_missing_events_replaced_files_and_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "native.jsonl"
            reader = NativeTrace(path)
            self.assertEqual(reader.read(), [])
            identity = dict(seq=0, event="identity", mono_ns=1, schema=1, pid=42)
            path.write_text(json.dumps(identity) + "\n")
            self.assertEqual(len(reader.read()), 1)
            with path.open("a") as stream:
                stream.write(json.dumps(dict(seq=2, event="bad", mono_ns=2)) + "\n")
            with self.assertRaises(ValueError):
                reader.read()
            path.write_text("")
            with self.assertRaises(ValueError):
                reader.read()
            path.unlink()
            path.write_text(json.dumps(identity) + "\n")
            with self.assertRaises(ValueError):
                reader.read()


class BoardWatchdogTest(unittest.TestCase):
    def test_original_500_ms_timeout_latches_on_true_absence_and_stays_latched(self):
        class Port:
            data = b""

            def read(self):
                data, self.data = self.data, b""
                return data

            def write(self, data):
                pass

        port = Port()
        board = LowLevelBoard(port, Plant())
        port.data = cobs_packet(bytes([LL_HEARTBEAT, 0, 1]))
        board.service(1000)
        self.assertFalse(board.latch)
        board.service(1500)
        self.assertFalse(board.latch)
        board.service(1500.001)
        self.assertTrue(board.latch)
        self.assertFalse(board.ros_running)
        port.data = cobs_packet(bytes([LL_HEARTBEAT, 0, 0]))
        board.service(1540)
        self.assertTrue(board.latch, "a regular heartbeat is not an operator release")


class NativeEmitterTest(unittest.TestCase):
    def test_instrumentation_is_compiled_out_of_the_default_build(self):
        header = Path(__file__).resolve().parents[1] / "mowgli_openmower_bridge/include"
        code = """
#include "mowgli_openmower_bridge/simulation_trace.hpp"
int main() { int calls=0; OM_SIM_TRACE("unused","%d",++calls); return calls; }
"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / "off.cpp").write_text(code)
            subprocess.run(
                [
                    "c++",
                    "-std=c++20",
                    "-I",
                    str(header),
                    str(path / "off.cpp"),
                    "-o",
                    str(path / "off"),
                ],
                check=True,
                capture_output=True,
            )
            subprocess.run(
                [str(path / "off")],
                check=True,
                env=dict(
                    os.environ, OPENMOWER_SIMULATION_TRACE_PATH=str(path / "trace")
                ),
            )
            self.assertFalse((path / "trace").exists())

    def test_native_emissions_remain_observable_during_real_python_gil_block(self):
        # The child uses the exact production-seam writer. A pipe handshake
        # starts observation; native absolute 40 ms deadlines are the heartbeat
        # cadence being tested, not arbitrary sleeps used for synchronization.
        header = Path(__file__).resolve().parents[1] / "mowgli_openmower_bridge/include"
        code = r"""
#include <chrono>
#include <thread>
#include <unistd.h>
#include "mowgli_openmower_bridge/simulation_trace.hpp"
int main(int argc,char** argv) {
  const int ready=std::atoi(argv[1]);
  const int stop=std::atoi(argv[2]);
  auto next=std::chrono::steady_clock::now();
  const auto end=next+std::chrono::seconds(3);
  while(std::chrono::steady_clock::now()<end) {
    const auto stamp=mowgli_openmower_bridge::SimulationMonotonicNs();
    OM_SIM_TRACE("heartbeat_write","\"begin_ns\":%lld,\"end_ns\":%lld,\"written\":1",stamp,stamp);
    const char one='1'; ::write(ready,&one,1);
    char command; if(::read(stop,&command,1)==1) return 0;
    next+=std::chrono::milliseconds(40); std::this_thread::sleep_until(next);
  }
  return 2;
}"""
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / "emitter.cpp").write_text(code)
            subprocess.run(
                [
                    "c++",
                    "-std=c++20",
                    "-DOPENMOWER_SIMULATION_TRACE",
                    "-I",
                    str(header),
                    str(path / "emitter.cpp"),
                    "-o",
                    str(path / "emitter"),
                ],
                check=True,
                capture_output=True,
            )
            ready_r, ready_w = os.pipe()
            stop_r, stop_w = os.pipe()
            os.set_blocking(stop_r, False)
            env = dict(os.environ, OPENMOWER_SIMULATION_TRACE_PATH=str(path / "trace"))
            child = None
            try:
                child = subprocess.Popen(
                    [str(path / "emitter"), str(ready_w), str(stop_r)],
                    env=env,
                    pass_fds=(ready_w, stop_r),
                )
                self.assertTrue(select.select([ready_r], [], [], 5)[0])
                self.assertEqual(os.read(ready_r, 1), b"1")
                start = time.monotonic_ns()
                block = ctypes.PyDLL(None).usleep
                block.argtypes = [ctypes.c_uint]
                block(444000)
                end = time.monotonic_ns()
                reader = NativeTrace(path / "trace")
                ok, detail = heartbeat_cadence(reader.read(), end)
                self.assertGreaterEqual(end - start, 444_000_000)
                self.assertTrue(ok, detail)
                os.write(stop_w, b"1")
                child.wait(timeout=5)
                self.assertEqual(child.returncode, 0)
            finally:
                if child is not None and child.poll() is None:
                    child.terminate()
                    child.wait(timeout=5)
                for fd in [ready_r, ready_w, stop_r, stop_w]:
                    os.close(fd)


if __name__ == "__main__":
    unittest.main()
