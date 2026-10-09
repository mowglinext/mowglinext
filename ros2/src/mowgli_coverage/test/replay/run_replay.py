#!/usr/bin/env python3
"""Repeat real-planner fixtures; retain exact inputs and first-divergence evidence."""

import argparse
import copy
import gzip
import hashlib
import json
import os
import pathlib
import signal
import statistics
import subprocess
import tempfile


PARAMETERS = {
    "operation_width": 0.16,
    "headland_width": 0.18,
    "num_headland_passes": 5,
    "chassis_safety_inset": 0.0,
    "mow_angle_rad": -1.0,
    "min_swath_length": 0.15,
    "ring_direction": 0,
    "min_turn_radius": 0.20,
    "turn_radius": 0.20,
    "step": 0.03,
    "obstacle_margin": 0.0,
    "pivot_sweep_radius": 0.0,
    "boundary_margin": 0.0,
    "perpendicular": False,
    "connector_max_headland_passes": 0,
    "start_hint": None,
}


def digest(value):
    data = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


def first_difference(a, b, path=""):
    if type(a) is not type(b):
        return path or "/"
    if isinstance(a, dict):
        if a.keys() != b.keys():
            return path + "/keys"
        for key in a:
            found = first_difference(a[key], b[key], path + "/" + str(key))
            if found:
                return found
    elif isinstance(a, list):
        if len(a) != len(b):
            return path + "/length"
        for index, (left, right) in enumerate(zip(a, b)):
            found = first_difference(left, right, path + "/" + str(index))
            if found:
                return found
    elif a != b:
        return path or "/"
    return None


def stable(record):
    """Timings/build flags are provenance, not geometry. Never hash diagnostic clocks."""
    return {key: record[key] for key in (
        "transported_polygons", "normalized_polygons", "parameters", "stages",
        "fields", "metrics", "followstrip_fingerprint"
    )}


def input_identity(record):
    # Diagnostic identity only; production does not read this or authorize a cursor.
    return digest({key: record[key] for key in (
        "transported_polygons", "normalized_polygons", "parameters", "build"
    )})


def rectangle(w, h, x=0.0, y=0.0):
    return [[x, y], [x + w, y], [x + w, y + h], [x, y + h]]


def fixture(polygons, **parameters):
    return {"schema": 1, "precision": "float32", "polygons": polygons,
            "parameters": {**PARAMETERS, **parameters}}


def run_command(command, timeout, errors=None):
    """Own the POSIX process group, including /usr/bin/time's planner child."""
    with subprocess.Popen(command, stdout=subprocess.PIPE,
                          stderr=errors if errors is not None else subprocess.PIPE,
                          text=True, start_new_session=(os.name == "posix")) as process:
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            if os.name == "posix":
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            else:
                process.kill()  # Windows does not use the time wrapper.
            error.output, error.stderr = process.communicate()
            raise
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)


def cases(binary, timeout=300):
    recorded = [[0.69736, 0.542974], [0.333294, 0.901446], [0.0692125, 0.84469],
                [-1.83674, 3.2762], [-1.82738, 3.67492], [-0.580451, 4.72558],
                [-0.34252, 5.71537], [-2.52072, 8.45872], [-1.85113, 9.08351],
                [-0.752371, 8.3454], [3.55742, 2.91514], [4.20313, 1.36811],
                [2.98118, 0.11797], [1.29082, 0.260783], [0.664285, 0.365982]]
    # Thin neck erodes into TWO mainland cells; this is a synthetic multi-cell case.
    split = [[0, 0], [4, 0], [4, 2.7], [7, 2.7], [7, 0], [11, 0],
             [11, 6], [7, 6], [7, 3.3], [4, 3.3], [4, 6], [0, 6]]
    small = fixture([[recorded]])
    raw = copy.deepcopy(small)
    raw["precision"] = "float64"
    holes = [rectangle(10, 8), rectangle(1, 1, 2, 2), rectangle(1, 2, 6, 4)]
    result = {
        "square_auto": fixture([[rectangle(6, 6)]]),
        "square_fixed": fixture([[rectangle(6, 6)]], mow_angle_rad=0.3),
        "square_changed_angle": fixture([[rectangle(6, 6)]], mow_angle_rad=0.7),
        "square_changed_width": fixture([[rectangle(6, 6)]], operation_width=0.19),
        "square_changed_geometry": fixture([[rectangle(6.3, 6)]]),
        "recorded_float32": small,
        "recorded_float64": raw,
        "recorded_fixed": fixture([[recorded]], mow_angle_rad=0.3),
        "recorded_reverse": fixture([[list(reversed(recorded))]]),
        "recorded_rotated_start": fixture([[recorded[7:] + recorded[:7]]]),
        "recorded_no_rings_pivot": fixture([[recorded]], num_headland_passes=-1,
                                           pivot_sweep_radius=0.597, boundary_margin=0.597),
        "rings_off_start_hint": fixture([[recorded]], num_headland_passes=-1,
                                        start_hint=[-2.0, 7.0]),
        "recorded_auto_rings": fixture([[recorded]], num_headland_passes=0),
        "split_mainland": fixture([[split]]),
        "split_crosshatch": fixture([[split]], perpendicular=True),
        "holes_auto": fixture([holes], obstacle_margin=0.2,
                               pivot_sweep_radius=0.597, boundary_margin=0.597),
        "holes_fixed": fixture([holes], mow_angle_rad=0.3, obstacle_margin=0.2),
        "multi_polygon": fixture([[rectangle(6, 4)], [rectangle(5, 4, 8, 0)]]),
        "near_degenerate": fixture([[[[0, 0], [0.001, 0.001], [8, 0], [8, 6],
                                       [7.999, 6.001], [0, 6], [0.001, 0.001]]]]),
        "large_auto": fixture([[rectangle(30, 20)]], operation_width=0.32),
        "large_reverse": fixture([[list(reversed(rectangle(30, 20)))]], operation_width=0.32),
        "ring_clockwise": fixture([[recorded]], ring_direction=1),
        "ring_counterclockwise": fixture([[recorded]], ring_direction=2),
        "limited_turns": fixture([[recorded]], connector_max_headland_passes=2),
        "start_hint": fixture([[recorded]], start_hint=[-2.0, 7.0]),
    }
    discovery = run_command([str(binary), "--isabey"], timeout)
    if discovery.returncode:
        raise RuntimeError(f"Isabey discovery failed: {discovery.stderr}")
    isabey = json.loads(discovery.stdout)
    result["isabey"] = fixture([isabey], obstacle_margin=0.389,
                               pivot_sweep_radius=0.597, boundary_margin=0.597)
    return result


def check_controls(results):
    failures = []
    for left, right in (("square_fixed", "square_changed_angle"),
                        ("square_auto", "square_changed_width"),
                        ("square_auto", "square_changed_geometry")):
        if left not in results or right not in results:
            continue
        a, b = results[left], results[right]
        if a["input_identity"] == b["input_identity"]:
            failures.append(f"{left}/{right}: different effective inputs share diagnostic identity")
        if a["followstrip_fingerprint"] == b["followstrip_fingerprint"]:
            failures.append(f"{left}/{right}: changed execution geometry shares resume fingerprint")
    return failures


def run_case(binary, name, data, output, repetitions, processes, no_trace, timeout):
    fixture_file = output / (name + ".json")
    fixture_file.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    first = None
    hashes, timings, rss, walls, failures = [], [], [], [], []
    for process in range(processes):
        command = [str(binary), str(fixture_file), str(repetitions)]
        if no_trace:
            command.append("--no-trace")
        perf = output / f"{name}.{process}.perf.json"
        if pathlib.Path("/usr/bin/time").exists():
            command = ["/usr/bin/time", "-f", '{"rss_kib":%M,"seconds":%e}', "-o", str(perf), *command]
        with tempfile.TemporaryFile(mode="w+") as errors:
            completed = run_command(command, timeout, errors)
            if completed.returncode:
                errors.seek(0)
                raise RuntimeError(f"{name}: replay exited {completed.returncode}: {errors.read()[-4000:]}")
        lines = completed.stdout.splitlines()
        if len(lines) != repetitions:
            raise RuntimeError(f"{name}: expected {repetitions} records, got {len(lines)}")
        for iteration, line in enumerate(lines):
            record = json.loads(line)
            snapshot = stable(record)
            hashes.append(digest(snapshot))
            timings.append(record["timing"])
            if first is None:
                first = record
                with gzip.open(output / (name + ".snapshot.json.gz"), "wt") as file:
                    json.dump(first, file, sort_keys=True, allow_nan=False)
            elif snapshot != stable(first):
                failures.append({"process": process, "iteration": iteration,
                                 "first_difference": first_difference(stable(first), snapshot)})
                with gzip.open(output / f"{name}.diverged.{process}.{iteration}.json.gz", "wt") as file:
                    json.dump(record, file, sort_keys=True, allow_nan=False)
        if perf.exists():
            measurement = json.loads(perf.read_text())
            rss.append(measurement["rss_kib"])
            walls.append(measurement["seconds"])
    return {"runs": len(hashes), "unique_exact_outputs": len(set(hashes)),
            "output_sha256": hashes[0], "fixture_sha256": digest(data),
            "input_identity": input_identity(first),
            "followstrip_fingerprint": first["followstrip_fingerprint"],
            "stage_hashes": [{"name": stage["name"], "sha256": digest(stage)} for stage in first["stages"]],
            "metrics": first["metrics"], "fields": first["fields"], "build": first["build"],
            "median_planning_ms": statistics.median(t["planning_ms"] for t in timings),
            "median_connectors_ms": statistics.median(t["connectors_ms"] for t in timings),
            "peak_process_rss_kib": max(rss) if rss else None,
            "process_wall_seconds": walls, "failures": failures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=pathlib.Path, required=True)
    parser.add_argument("--fixture", type=pathlib.Path, action="append", default=[])
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--same-process", type=int, default=5)
    parser.add_argument("--processes", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--no-trace", action="store_true")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.same_process < 1 or args.processes < 1 or args.timeout <= 0:
        parser.error("repetitions, processes and timeout must be positive")
    args.binary = args.binary.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    selected = {p.stem: json.loads(p.read_text()) for p in args.fixture}
    if not selected:
        available = cases(args.binary, args.timeout)
        selected = {name: available[name] for name in args.case} if args.case else available
    results = {}
    for name, data in selected.items():
        # Keep report filenames inside output even for externally named fixtures.
        if pathlib.Path(name).name != name or name in (".", ".."):
            parser.error("invalid fixture name")
        result = run_case(args.binary, name, data, args.output, args.same_process,
                          args.processes, args.no_trace, args.timeout)
        results[name] = result
        print(f"{name}: {result['runs']} runs, {result['unique_exact_outputs']} outputs, "
              f"{result['metrics']['poses']} poses", flush=True)
        (args.output / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
    failures = check_controls(results)
    (args.output / "controls.json").write_text(json.dumps({"failures": failures}, indent=2) + "\n")
    if failures or any(result["failures"] for result in results.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
