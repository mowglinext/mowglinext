#!/usr/bin/env python3
"""Compare exact planner replay output and process performance summaries."""

import argparse
import gzip
import hashlib
import json
import math
import pathlib
import re
import statistics
import struct
import subprocess
import sys
import tempfile

from run_replay import digest, input_identity, stable


METRIC_KEYS = (
    'poses',
    'swaths',
    'rings',
    'subpaths',
    'blade_off_transits',
    'path_length_m',
    'endpoint_transit_length_m',
    'attempted',
    'arc',
    'straight_kept',
    'pivot',
    'split',
)
BUILD_KEYS = ('compiler', 'configuration', 'gdal', 'geos')


def require_hex(value, length):
    """Reject missing or malformed exact hash/coordinate values."""
    if not isinstance(value, str) or re.fullmatch(f'[0-9a-f]{{{length}}}', value) is None:
        raise ValueError(f'expected a {length}-digit hexadecimal value')


def require_number(value, integer=False, positive=False):
    """Reject null, boolean, non-finite and negative measurements."""
    valid_type = type(value) is int if integer else type(value) in (int, float)
    if not valid_type or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError('expected a finite nonnegative numeric measurement')


def require_paths(paths):
    """Validate nonempty exact coordinate or quaternion paths."""
    if not isinstance(paths, list) or not paths:
        raise ValueError('expected a nonempty path array')
    for path in paths:
        if not isinstance(path, list) or not path:
            raise ValueError('expected a nonempty point array')
        for point in path:
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError('expected a coordinate pair')
            for coordinate in point:
                require_hex(coordinate, 16)


def validate_evidence(directory, name, case):
    """Bind required summary values to retained fixture and snapshot evidence."""
    if not isinstance(case, dict):
        raise ValueError(f'{name}: expected a case object')
    for key in ('fixture_sha256', 'output_sha256', 'input_identity'):
        require_hex(case.get(key), 64)
    require_hex(case.get('followstrip_fingerprint'), 16)
    for key in ('runs', 'unique_exact_outputs'):
        require_number(case.get(key), integer=True, positive=True)
    if not isinstance(case.get('failures'), list):
        raise ValueError(f'{name}: missing failure list')
    for key in ('median_planning_ms', 'median_connectors_ms'):
        require_number(case.get(key))
    fixture = load_json(pathlib.Path(directory) / f'{name}.json')
    if not isinstance(fixture, dict) or fixture.get('schema') != 1:
        raise ValueError(f'{name}: invalid retained fixture')
    if digest(fixture) != case['fixture_sha256']:
        raise ValueError(f'{name}: retained fixture hash differs from summary')
    snapshot = load_snapshot(directory, name)
    trace_geometry(snapshot)
    fields = snapshot.get('fields')
    if not isinstance(fields, list) or not fields:
        raise ValueError(f'{name}: missing field records')
    for field in fields:
        if not isinstance(field, dict):
            raise ValueError(f'{name}: invalid field record')
        heading = field.get('first_swath_heading_rad')
        if type(heading) not in (int, float) or not math.isfinite(heading):
            raise ValueError(f'{name}: invalid field heading')
        require_hex(field.get('fingerprint'), 16)
        for key in ('swaths', 'rings', 'subpaths'):
            require_number(field.get(key), integer=True)
    metrics = snapshot.get('metrics')
    build = snapshot.get('build')
    if not isinstance(metrics, dict) or not isinstance(build, dict):
        raise ValueError(f'{name}: missing metrics or build metadata')
    for key in METRIC_KEYS:
        require_number(metrics.get(key), integer=not key.endswith('_m'))
    require_number(metrics['poses'], integer=True, positive=True)
    for key in BUILD_KEYS + ('revision',):
        if not isinstance(build.get(key), str) or not build[key]:
            raise ValueError(f'{name}: missing build field {key}')
    for key in ('planner_source_sha256', 'planner_header_sha256'):
        require_hex(build.get(key), 64)
    for key in ('metrics', 'fields', 'build', 'followstrip_fingerprint'):
        if case.get(key) != snapshot.get(key):
            raise ValueError(f'{name}: summary {key} differs from retained snapshot')
    if digest(stable(snapshot)) != case['output_sha256']:
        raise ValueError(f'{name}: retained output hash differs from summary')
    if input_identity(snapshot) != case['input_identity']:
        raise ValueError(f'{name}: retained input identity differs from summary')
    stages = [{'name': stage['name'], 'sha256': digest(stage)} for stage in snapshot['stages']]
    if case.get('stage_hashes') != stages:
        raise ValueError(f'{name}: retained stage hashes differ from summary')
    return snapshot


def load_json(path):
    """Load JSON and surface a concise path-aware error."""
    try:
        return json.loads(pathlib.Path(path).read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'{path}: {error}') from error


def load_summary(directory):
    """Load one run_replay summary and validate its top-level shape."""
    path = pathlib.Path(directory) / 'summary.json'
    summary = load_json(path)
    if not isinstance(summary, dict) or not summary:
        raise ValueError(f'{path}: expected a non-empty case map')
    return summary


def load_snapshot(directory, name):
    """Load the first exact stable record retained by run_replay."""
    path = pathlib.Path(directory) / f'{name}.snapshot.json.gz'
    try:
        with gzip.open(path, 'rt', encoding='utf-8') as snapshot_file:
            snapshot = json.load(snapshot_file)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f'{path}: {error}') from error
    if not isinstance(snapshot, dict):
        raise ValueError(f'{path}: expected a stable record object')
    return snapshot


def trace_geometry(snapshot):
    """Select exact planner geometry fields shared by traced and plain runs."""
    required_keys = ('transported_polygons', 'normalized_polygons', 'parameters')
    missing_keys = [key for key in required_keys if key not in snapshot]
    if missing_keys:
        raise ValueError(f'snapshot is missing required geometry fields: {missing_keys}')
    for key in ('transported_polygons', 'normalized_polygons'):
        polygons = snapshot[key]
        if not isinstance(polygons, list) or not polygons:
            raise ValueError(f'snapshot has invalid {key}')
        for polygon in polygons:
            require_paths(polygon)
    if not isinstance(snapshot['parameters'], dict) or not snapshot['parameters']:
        raise ValueError('snapshot has invalid parameters')
    stages = snapshot.get('stages')
    if not isinstance(stages, list):
        raise ValueError('snapshot is missing the stages array')
    selected = []
    names = set()
    for stage in stages:
        if not isinstance(stage, dict) or not isinstance(stage.get('name'), str):
            raise ValueError('snapshot contains a malformed stage')
        stage_name = stage['name']
        if stage_name.endswith('.execution') or stage_name.endswith('.quaternions'):
            if 'geometry' not in stage or stage_name in names:
                raise ValueError(f'snapshot contains an invalid stage: {stage_name}')
            require_paths(stage['geometry'])
            selected.append({'name': stage_name, 'geometry': stage['geometry']})
            names.add(stage_name)
    if not any(name.endswith('.execution') for name in names):
        raise ValueError('snapshot has no execution geometry stage')
    if not any(name.endswith('.quaternions') for name in names):
        raise ValueError('snapshot has no quaternion geometry stage')
    geometry = {key: snapshot[key] for key in required_keys}
    geometry['stages'] = selected
    return geometry


def stable_run(case, expected_runs):
    """Check that the case completed the requested exact repetitions."""
    return (
        case.get('runs') == expected_runs
        and case.get('unique_exact_outputs') == 1
        and not case.get('failures')
    )


def performance_samples(directory, name):
    """Read process-level wall-time and RSS samples written by run_replay."""
    values = []
    for path in sorted(pathlib.Path(directory).glob(f'{name}.*.perf.json')):
        sample = load_json(path)
        require_number(sample.get('seconds'))
        require_number(sample.get('rss_kib'), integer=True, positive=True)
        values.append(sample)
    if not values:
        raise ValueError(f'{directory}: no process performance samples for {name}')
    return values


def median_value(values, key):
    return statistics.median(sample[key] for sample in values)


def relative_percent(baseline, candidate):
    if not isinstance(baseline, (int, float)) or not isinstance(candidate, (int, float)):
        return None
    if baseline == 0:
        return None
    return (candidate / baseline - 1.0) * 100.0


def metric_comparison(baseline, candidate):
    left = baseline.get('metrics', {})
    right = candidate.get('metrics', {})
    result = {}
    for key in METRIC_KEYS:
        if key not in left or key not in right:
            result[key] = {'equal': False, 'baseline': left.get(key), 'candidate': right.get(key)}
            continue
        result[key] = {
            'equal': left[key] == right[key],
            'baseline': left[key],
            'candidate': right[key],
        }
        if isinstance(left[key], (int, float)) and isinstance(right[key], (int, float)):
            result[key]['delta'] = right[key] - left[key]
    return result


def compatible_build_metadata(baseline, candidate):
    """Compare build fields that should be shared across source variants."""
    left = baseline.get('build', {})
    right = candidate.get('build', {})
    return {
        key: {
            'equal': left.get(key) == right.get(key),
            'baseline': left.get(key),
            'candidate': right.get(key),
        }
        for key in BUILD_KEYS
    }


def planner_identity(baseline, candidate):
    """The complete existing strict identity predicate, including structure/build."""
    return (
        baseline.get('output_sha256') == candidate.get('output_sha256')
        and baseline.get('followstrip_fingerprint') == candidate.get('followstrip_fingerprint')
        and baseline.get('fields') == candidate.get('fields')
        and baseline.get('stage_hashes') == candidate.get('stage_hashes')
        and all(value['equal'] for value in metric_comparison(baseline, candidate).values())
        and all(value['equal'] for value in compatible_build_metadata(baseline, candidate).values())
    )


def validate_bevel_observer(snapshot, stages):
    """Validate optional observer evidence even when strict output identity suffices."""
    def path(points):
        if not isinstance(points, list):
            raise ValueError('invalid bevel observer path')
        if points:
            require_paths([points])
        for point in points:
            for number in point:
                if not math.isfinite(struct.unpack('>d', bytes.fromhex(number))[0]):
                    raise ValueError('nonfinite bevel observer coordinate')

    expected = set()
    for field, record in enumerate(snapshot['fields']):
        rings = stages[f'{field}.plan_rings']['geometry']
        if not isinstance(rings, list) or len(rings) != record['rings']:
            raise ValueError('bevel observer ring count differs')
        for index, ring in enumerate(rings):
            require_paths([ring])
            path(ring)
            values = []
            for kind in ('source', 'intermediate', 'rounded', 'replacements'):
                name = f'{field}.bevel_{kind}.{index}'
                expected.add(name)
                item = stages[name]
                if (set(item) != {'name', 'geometry', 'angles'}
                        or not isinstance(item['geometry'], list)
                        or not isinstance(item['angles'], list)):
                    raise ValueError('invalid bevel observer schema')
                for points in item['geometry']:
                    path(points)
                if kind != 'replacements':
                    if len(item['geometry']) != 1 or item['angles']:
                        raise ValueError('invalid bevel observer singleton')
                    values.append(item['geometry'][0])
                else:
                    if len(item['angles']) != 2 * len(item['geometry']):
                        raise ValueError('invalid bevel observer indices')
                    for encoded in item['angles']:
                        require_hex(encoded, 16)
                        value = struct.unpack('>d', bytes.fromhex(encoded))[0]
                        if not math.isfinite(value) or value < 1 or not value.is_integer():
                            raise ValueError('invalid bevel observer index')
                    if not values[0] and (any(values) or item['geometry'] or item['angles']):
                        raise ValueError('invalid bypassed bevel observer')
    if {name for name in stages if '.bevel_' in name} != expected:
        raise ValueError('missing or unrecognized bevel observer stage')
    return expected


def reference_oracle_hash():
    root = pathlib.Path(__file__).resolve().parent
    files = ('ring_contract_reference.hpp', 'ring_contract.hpp', 'ring_contract_adapter.cpp.in',
             'semantic_diagnostics.hpp')
    return hashlib.sha256(b''.join((root / name).read_bytes() for name in files)).hexdigest()


def certify_fillets(name, baseline, candidate, traced, baseline_dir, candidate_dir,
                    trace_dir, binary):
    """Require strict identity or a narrowly certified change, with common controls."""
    original = validate_evidence(baseline_dir, name, baseline)
    plain = validate_evidence(candidate_dir, name, candidate)
    observed = validate_evidence(trace_dir, name, traced)
    if not (baseline['fixture_sha256'] == candidate['fixture_sha256'] == traced['fixture_sha256']):
        raise ValueError(f'{name}: certification fixture identities differ')
    for summary in (baseline, candidate, traced):
        if summary['unique_exact_outputs'] != 1 or summary['failures']:
            raise ValueError(f'{name}: unstable replay evidence')
    shared = {}
    for stage in observed['stages']:
        if stage['name'] in shared:
            raise ValueError(f'{name}: duplicate traced stage')
        shared[stage['name']] = stage
    seen = set()
    for stage in plain['stages']:
        if stage['name'] in seen or shared.get(stage['name']) != stage:
            raise ValueError(f'{name}: traced public planner stage differs from plain candidate')
        seen.add(stage['name'])
    bevel_stages = validate_bevel_observer(observed, shared)
    if (trace_geometry(plain) != trace_geometry(observed)
            or candidate['followstrip_fingerprint'] != traced['followstrip_fingerprint']
            or candidate['fields'] != traced['fields']
            or not all(value['equal'] for value in metric_comparison(candidate, traced).values())):
        raise ValueError(f'{name}: candidate trace is not passive')
    for key in ('compiler', 'configuration', 'gdal', 'geos', 'requested_cxx_flags_sha256',
                'compiler_fp_features'):
        if key not in baseline['build'] or baseline['build'][key] != candidate['build'].get(key):
            raise ValueError(f'{name}: incompatible baseline/candidate {key}')
    for key in ('revision', 'planner_source_sha256', 'planner_header_sha256', 'compiler',
                'configuration', 'gdal', 'geos', 'requested_cxx_flags_sha256',
                'compiler_fp_features'):
        if candidate['build'].get(key) != traced['build'].get(key):
            raise ValueError(f'{name}: traced candidate source differs')
    oracle_hash = reference_oracle_hash()

    def checked_build(snapshot):
        build = snapshot.get('build', {})
        for key in ('revision', 'planner_source_sha256', 'planner_header_sha256', 'compiler',
                    'configuration', 'gdal', 'geos', 'requested_cxx_flags_sha256',
                    'compiler_fp_features'):
            if key not in baseline['build'] or build.get(key) != baseline['build'][key]:
                raise ValueError(f'{name}: oracle uses a different pristine baseline {key}')
        if build.get('reference_oracle_sha256') != oracle_hash:
            raise ValueError(f'{name}: oracle helper source differs from reviewed tooling')
        return build

    with tempfile.TemporaryDirectory(prefix='coverage-certified-fillets-') as temporary:
        certificate = pathlib.Path(temporary) / 'trace.json'
        certificate.write_text(json.dumps(observed, allow_nan=False), encoding='utf-8')
        fixture = str(pathlib.Path(baseline_dir) / f'{name}.json')
        control = subprocess.run([str(binary), fixture, '1', '--no-trace'],
                                 capture_output=True, text=True, timeout=300, check=False)
        if control.returncode:
            raise ValueError(f'{name}: baseline reference control failed: {control.stderr.strip()}')
        try:
            control_snapshot = json.loads(control.stdout)
            control_geometry = stable(control_snapshot)
        except (json.JSONDecodeError, KeyError, TypeError) as error:
            raise ValueError(f'{name}: malformed baseline reference control') from error
        if control_geometry != stable(original):
            raise ValueError(f'{name}: reference adapter changes pristine baseline output')
        build = checked_build(control_snapshot)
        audit = {
            'accepted': True,
            'baseline_revision': build['revision'],
            'baseline_source_sha256': build['planner_source_sha256'],
            'reference_oracle_sha256': oracle_hash,
            'pristine_baseline_control_sha256': digest(control_geometry),
            'candidate_output_sha256': candidate['output_sha256'],
        }
        identical = planner_identity(baseline, candidate)
        if identical:
            # A baseline that already contains the repair must retain strict
            # identity eligibility; its ring need not equal pre-repair oldRound(S).
            observed_control = subprocess.run([str(binary), fixture, '1'], capture_output=True,
                                              text=True, timeout=300, check=False)
            if observed_control.returncode:
                raise ValueError(f'{name}: traced baseline reference control failed')
            observed_reference = json.loads(observed_control.stdout)
            if checked_build(observed_reference) != build:
                raise ValueError(f'{name}: traced reference build differs')
            reference_names = set()
            for stage in observed_reference['stages']:
                if stage['name'] in reference_names or shared.get(stage['name']) != stage:
                    raise ValueError(f'{name}: identical planner observer stage differs')
                reference_names.add(stage['name'])
            if ([stage['name'] for stage in observed_reference['stages']]
                    != [stage['name'] for stage in observed['stages']
                        if stage['name'] in reference_names]):
                raise ValueError(f'{name}: identical planner observer stage order differs')
            if set(shared) - reference_names - bevel_stages:
                raise ValueError(f'{name}: unrecognized identical planner observer stage')
            if (trace_geometry(observed_reference) != trace_geometry(original)
                    or observed_reference['fields'] != original['fields']
                    or observed_reference['metrics'] != original['metrics']
                    or observed_reference['followstrip_fingerprint'] != original['followstrip_fingerprint']):
                raise ValueError(f'{name}: baseline reference observer is not passive')
            if not set(shared) - reference_names:
                return {**audit, 'method': 'identical', 'replacement_count': 0,
                        'observer_validation': 'exact traced pristine reference',
                        'scope': 'strict output identity; no functional geometry change accepted'}
            # Older baselines lack bevel observer records. Bind those extra records
            # with the existing oracle; unchanged rings reconstruct the old output.
            # Future baselines bind all records directly above and need no replay
            # of a pre-repair ring that their planner no longer emits.
        command = [str(binary), '--certify-fillets', fixture, str(certificate)]
        process = subprocess.run(command, capture_output=True, text=True, timeout=300, check=False)
    if process.returncode:
        raise ValueError(f'{name}: baseline ring oracle rejected: {process.stderr.strip()}')
    try:
        reconstructed = json.loads(process.stdout)
    except json.JSONDecodeError as error:
        raise ValueError(f'{name}: invalid baseline oracle output') from error
    if reconstructed.get('certified_fillets') is not True:
        raise ValueError(f'{name}: baseline oracle did not certify the rings')
    if checked_build(reconstructed) != build:
        raise ValueError(f'{name}: oracle helper source differs from reviewed tooling')
    if stable(reconstructed) != stable(plain):
        raise ValueError(f'{name}: candidate differs from exact baseline downstream reconstruction')
    return {
        **audit,
        'method': 'identical' if identical else 'certified_fillets',
        'observer_validation': ('old-baseline oracle for additional bevel records' if identical
                                else 'independent fillet and downstream reconstruction'),
        'reconstructed_output_sha256': digest(stable(reconstructed)),
        'replacement_count': sum(
            len(stage['geometry']) for stage in observed['stages']
            if '.bevel_replacements.' in stage['name']
        ),
        'scope': ('strict output identity; no functional geometry change accepted' if identical
                  else 'certifies emitted ring polylines; no physical chassis-sweep claim'),
    }


def case_comparison(name, baseline, candidate, trace, baseline_dir, candidate_dir,
                    trace_dir, expected_runs, expected_trace_runs, expected_processes,
                    expected_trace_processes):
    validate_evidence(baseline_dir, name, baseline)
    candidate_snapshot = validate_evidence(candidate_dir, name, candidate)
    trace_snapshot = validate_evidence(trace_dir, name, trace) if trace is not None else None
    fixture_equal = baseline.get('fixture_sha256') == candidate.get('fixture_sha256')
    inputs = {
        'baseline_fixture_sha256': baseline.get('fixture_sha256'),
        'candidate_fixture_sha256': candidate.get('fixture_sha256'),
        'equal': fixture_equal,
    }
    result = {
        'inputs': inputs,
        'baseline_stable': stable_run(baseline, expected_runs),
        'candidate_stable': stable_run(candidate, expected_runs),
        'candidate_trace_stable': stable_run(trace, expected_trace_runs) if trace else None,
        'comparison_valid': fixture_equal,
    }
    if not fixture_equal:
        result['planner_comparison'] = 'skipped: fixture hashes differ'
        return result

    metric_results = metric_comparison(baseline, candidate)
    fingerprint_equal = (
        baseline.get('followstrip_fingerprint') == candidate.get('followstrip_fingerprint')
    )
    fields_equal = baseline.get('fields') == candidate.get('fields')
    output_equal = baseline.get('output_sha256') == candidate.get('output_sha256')
    stage_hashes_equal = baseline.get('stage_hashes') == candidate.get('stage_hashes')
    result.update(
        {
            'output_sha256_equal': output_equal,
            'followstrip_fingerprint_equal': fingerprint_equal,
            'fields_equal': fields_equal,
            'stage_hashes_equal': stage_hashes_equal,
            'metrics': metric_results,
            'build_compatibility': compatible_build_metadata(baseline, candidate),
        }
    )
    result['planner_identical'] = planner_identity(baseline, candidate)

    if trace:
        trace_fixture_equal = candidate.get('fixture_sha256') == trace.get('fixture_sha256')
        trace_metrics = metric_comparison(candidate, trace)
        candidate_geometry = trace_geometry(candidate_snapshot)
        traced_geometry = trace_geometry(trace_snapshot)
        snapshot_geometry_equal = candidate_geometry == traced_geometry
        fingerprint_equal = (
            candidate.get('followstrip_fingerprint') == trace.get('followstrip_fingerprint')
        )
        fields_equal = candidate.get('fields') == trace.get('fields')
        metrics_equal = all(metric['equal'] for metric in trace_metrics.values())
        trace_equal = (
            trace_fixture_equal
            and snapshot_geometry_equal
            and fingerprint_equal
            and fields_equal
            and metrics_equal
        )
        trace_samples = performance_samples(trace_dir, name)
        result['trace_passivity'] = {
            'fixture_sha256_equal': trace_fixture_equal,
            'snapshot_geometry_equal': snapshot_geometry_equal,
            'compared_stage_names': [stage['name'] for stage in candidate_geometry['stages']],
            'followstrip_fingerprint_equal': fingerprint_equal,
            'fields_equal': fields_equal,
            'metrics': trace_metrics,
            'passive': trace_equal,
            'processes': len(trace_samples),
            'process_count_matches': len(trace_samples) == expected_trace_processes,
            'trace_output_sha256': trace.get('output_sha256'),
            'note': (
                'compares exact transported, normalized, parameter, execution, and quaternion '
                'geometry; output SHA is not compared because trace adds diagnostic stages'
            ),
        }

    baseline_samples = performance_samples(baseline_dir, name)
    candidate_samples = performance_samples(candidate_dir, name)
    baseline_wall = median_value(baseline_samples, 'seconds')
    candidate_wall = median_value(candidate_samples, 'seconds')
    baseline_rss = median_value(baseline_samples, 'rss_kib')
    candidate_rss = median_value(candidate_samples, 'rss_kib')
    baseline_planning = baseline.get('median_planning_ms')
    candidate_planning = candidate.get('median_planning_ms')
    baseline_connectors = baseline.get('median_connectors_ms')
    candidate_connectors = candidate.get('median_connectors_ms')
    result['performance'] = {
        'baseline_processes': len(baseline_samples),
        'candidate_processes': len(candidate_samples),
        'process_count_matches': (
            len(baseline_samples) == expected_processes
            and len(candidate_samples) == expected_processes
        ),
        'baseline_median_process_seconds': baseline_wall,
        'candidate_median_process_seconds': candidate_wall,
        'process_seconds_change_percent': relative_percent(baseline_wall, candidate_wall),
        'baseline_median_rss_kib': baseline_rss,
        'candidate_median_rss_kib': candidate_rss,
        'rss_change_percent': relative_percent(baseline_rss, candidate_rss),
        'baseline_median_planning_ms': baseline_planning,
        'candidate_median_planning_ms': candidate_planning,
        'planning_ms_change_percent': relative_percent(baseline_planning, candidate_planning),
        'baseline_median_connectors_ms': baseline_connectors,
        'candidate_median_connectors_ms': candidate_connectors,
        'connectors_ms_change_percent': relative_percent(
            baseline_connectors, candidate_connectors
        ),
    }
    return result


def report_markdown(report):
    """Render a concise artifact report with explicit comparison limits."""
    case_count = report['case_count']
    expected_runs = report['expected_runs']
    expected_trace_runs = report['expected_trace_runs']
    expected_trace_processes = report['expected_trace_processes']
    exact_output_matches = report['exact_output_matches']
    performance_case_count = report['performance_case_count']
    planning_change = report['median_case_planning_change_percent']
    connectors_change = report['median_case_connectors_change_percent']
    wall_change = report['median_case_wall_change_percent']
    rss_change = report['median_case_rss_change_percent']
    trace_passivity_matches = report['trace_passivity_matches']
    build_type = (report.get('environment') or {}).get('build_type')
    architecture = (report.get('environment') or {}).get('architecture')
    compiler = (report.get('environment') or {}).get('compiler')
    f2c_version = (report.get('environment') or {}).get('fields2cover_version')
    f2c_revision = (report.get('environment') or {}).get('fields2cover_revision')
    lines = [
        '# Coverage planner replay comparison',
        '',
        f'- Cases: {case_count}',
        f'- Baseline/candidate runs per case: {expected_runs}',
        f'- Candidate traced runs per case: {expected_trace_runs}',
        f'- Candidate trace fresh processes per case: {expected_trace_processes}',
        f'- Exact output matches: {exact_output_matches}/{case_count}',
        f'- Trace passivity matches: {trace_passivity_matches}/{case_count}',
        f'- Performance valid cases: {performance_case_count}/{case_count}',
        f'- Median per-case planner time change: {planning_change}%',
        f'- Median per-case connector time change: {connectors_change}%',
        f'- Median per-case process time change: {wall_change}%',
        f'- Median per-case RSS change: {rss_change}%',
    ]
    if report.get('require_certified_fillets'):
        certified = report['certified_fillet_matches']
        identical = report['exact_identity_matches']
        lines.extend([
            f'- Accepted strict identity: {identical}/{case_count}',
            f'- Certified local-fillet/downstream reconstructions: {certified}/{case_count}',
            '- Raw baseline/candidate differences above remain part of this report.',
            '- Fillet certification checks emitted polylines, not physical chassis sweeps.',
        ])
        for name, case in report['cases'].items():
            error = case.get('planner_contract', {}).get('error')
            if error:
                lines.append(f'- Rejected {name}: {error}')
    environment = report.get('environment') or {}
    if environment:
        libraries = ', '.join(environment.get('fields2cover_library_sha256', []))
        lines.extend(
            [
                f'- Build: {build_type} on {architecture}',
                f'- Compiler: {compiler}',
                f'- Fields2Cover: {f2c_version} at {f2c_revision}',
                f'- Fields2Cover library SHA-256: {libraries}',
            ]
        )
    lines.extend(
        [
            '',
            '| Fixture | Poses | Planner SHA | Fingerprint | Plan % | Connector % | '
            'Process % | RSS % | Contract |',
            '| --- | ---: | :---: | :---: | ---: | ---: | ---: | ---: | --- |',
        ]
    )
    for name, case in report['cases'].items():
        metrics = case.get('metrics', {})
        poses = metrics.get('poses', {}).get('baseline', 'n/a')
        performance = case.get('performance', {})
        plan = performance.get('planning_ms_change_percent')
        connectors = performance.get('connectors_ms_change_percent')
        wall = performance.get('process_seconds_change_percent')
        rss = performance.get('rss_change_percent')
        plan_text = 'n/a' if plan is None else f'{plan:+.1f}%'
        connectors_text = 'n/a' if connectors is None else f'{connectors:+.1f}%'
        wall_text = 'n/a' if wall is None else f'{wall:+.1f}%'
        rss_text = 'n/a' if rss is None else f'{rss:+.1f}%'
        output_match = 'yes' if case.get('output_sha256_equal') else 'no'
        fingerprint_match = 'yes' if case.get('followstrip_fingerprint_equal') else 'no'
        lines.append(
            f'| {name} | {poses} | '
            f'{output_match} | {fingerprint_match} '
            f'| {plan_text} | {connectors_text} | {wall_text} | {rss_text} '
            f'| {case.get("planner_contract", {}).get("method", "n/a")} |'
        )
    lines.extend(
        [
            '',
            'Positive performance deltas mean candidate used more time or memory. '
            'Planner/connector times are medians across planner repetitions. '
            'Process runtime/RSS values are medians across fresh-process samples; '
            'the table reports relative changes against the baseline.',
            'Source revisions and source/header hashes are provenance and are expected '
            'to differ. Planner comparisons are valid only when the exact fixture hashes '
            'match. Trace passivity compares exact transported/normalized geometry, '
            'parameters, execution geometry, quaternion geometry, fingerprints, fields, '
            'and metrics with candidate no-trace output.',
            '',
        ]
    )
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-dir', type=pathlib.Path, required=True)
    parser.add_argument('--candidate-dir', type=pathlib.Path, required=True)
    parser.add_argument('--trace-dir', type=pathlib.Path)
    parser.add_argument('--environment', type=pathlib.Path, required=True)
    parser.add_argument('--output-json', type=pathlib.Path, required=True)
    parser.add_argument('--output-markdown', type=pathlib.Path, required=True)
    parser.add_argument('--expected-cases', type=int, default=26)
    parser.add_argument('--expected-runs', type=int, default=15)
    parser.add_argument('--expected-trace-runs', type=int, default=100)
    parser.add_argument('--expected-processes', type=int, default=3)
    parser.add_argument('--expected-trace-processes', type=int, default=5)
    contract = parser.add_mutually_exclusive_group()
    contract.add_argument('--require-identical', action='store_true')
    contract.add_argument('--require-certified-fillets', action='store_true')
    parser.add_argument('--baseline-binary', type=pathlib.Path)
    args = parser.parse_args()
    if args.require_certified_fillets and (not args.trace_dir or not args.baseline_binary):
        parser.error('certified fillets require --trace-dir and --baseline-binary')
    if min(
        args.expected_cases,
        args.expected_runs,
        args.expected_trace_runs,
        args.expected_processes,
        args.expected_trace_processes,
    ) < 1:
        parser.error('expected counts must be positive')

    try:
        baseline = load_summary(args.baseline_dir)
        candidate = load_summary(args.candidate_dir)
        trace = load_summary(args.trace_dir) if args.trace_dir else None
        environment = load_json(args.environment)
        if not isinstance(environment, dict):
            raise ValueError(f'{args.environment}: expected an environment object')
        for key in (
            'architecture',
            'compiler',
            'build_type',
            'fields2cover_revision',
            'fields2cover_library_sha256',
        ):
            if not environment.get(key):
                raise ValueError(f'{args.environment}: missing environment field {key}')
        names = sorted(baseline)
        names_match = set(baseline) == set(candidate)
        trace_names_match = trace is None or set(baseline) == set(trace)
        report = {
            'schema': 1,
            'case_count': len(names),
            'expected_cases': args.expected_cases,
            'expected_runs': args.expected_runs,
            'expected_trace_runs': args.expected_trace_runs,
            'expected_trace_processes': args.expected_trace_processes,
            'case_names_match': names_match,
            'trace_case_names_match': trace_names_match,
            'require_identical': args.require_identical,
            'require_certified_fillets': args.require_certified_fillets,
            'environment': environment,
            'cases': {},
        }
        if not names_match or not trace_names_match:
            raise ValueError('baseline, candidate and trace summaries must have matching cases')
        for name in names:
            report['cases'][name] = case_comparison(
                name,
                baseline[name],
                candidate[name],
                trace[name] if trace else None,
                args.baseline_dir,
                args.candidate_dir,
                args.trace_dir,
                args.expected_runs,
                args.expected_trace_runs,
                args.expected_processes,
                args.expected_trace_processes,
            )
            if args.require_certified_fillets:
                try:
                    certification = certify_fillets(
                        name, baseline[name], candidate[name], trace[name],
                        args.baseline_dir, args.candidate_dir, args.trace_dir, args.baseline_binary,
                    )
                except (KeyError, TypeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
                    certification = {'accepted': False, 'method': 'rejected', 'error': str(error)}
                report['cases'][name]['planner_contract'] = certification
        valid_cases = [case for case in report['cases'].values() if case['comparison_valid']]
        report['exact_output_matches'] = sum(
            case.get('output_sha256_equal', False) for case in valid_cases
        )
        report['trace_passivity_matches'] = sum(
            case.get('trace_passivity', {}).get('passive', False)
            and case.get('trace_passivity', {}).get('process_count_matches', False)
            for case in valid_cases
        ) if trace else 0
        report['stable_runs'] = all(
            case['baseline_stable']
            and case['candidate_stable']
            and (case['candidate_trace_stable'] is not False)
            for case in report['cases'].values()
        )
        report['performance_case_count'] = sum(
            'performance' in case and case['performance']['process_count_matches']
            for case in report['cases'].values()
        )
        wall_changes = [
            case['performance']['process_seconds_change_percent']
            for case in report['cases'].values()
            if 'performance' in case
            and case['performance']['process_seconds_change_percent'] is not None
        ]
        rss_changes = [
            case['performance']['rss_change_percent']
            for case in report['cases'].values()
            if 'performance' in case and case['performance']['rss_change_percent'] is not None
        ]
        report['median_case_wall_change_percent'] = (
            statistics.median(wall_changes) if wall_changes else None
        )
        report['median_case_rss_change_percent'] = (
            statistics.median(rss_changes) if rss_changes else None
        )
        plan_changes = [
            case['performance']['planning_ms_change_percent']
            for case in report['cases'].values()
            if 'performance' in case
            and case['performance']['planning_ms_change_percent'] is not None
        ]
        connector_changes = [
            case['performance']['connectors_ms_change_percent']
            for case in report['cases'].values()
            if 'performance' in case
            and case['performance']['connectors_ms_change_percent'] is not None
        ]
        report['median_case_planning_change_percent'] = (
            statistics.median(plan_changes) if plan_changes else None
        )
        report['median_case_connectors_change_percent'] = (
            statistics.median(connector_changes) if connector_changes else None
        )
        report['all_fixture_hashes_match'] = all(
            case['inputs']['equal'] for case in report['cases'].values()
        )
        report['all_planner_outputs_match'] = (
            report['exact_output_matches'] == args.expected_cases
            and all(case.get('planner_identical') for case in valid_cases)
        )
        report['all_trace_passivity_matches'] = (
            trace is None or report['trace_passivity_matches'] == args.expected_cases
        )
        report['all_planner_contracts_match'] = args.require_certified_fillets and all(
            case.get('planner_contract', {}).get('accepted') for case in report['cases'].values()
        )
        for method, key in (('identical', 'exact_identity_matches'),
                            ('certified_fillets', 'certified_fillet_matches')):
            report[key] = sum(case.get('planner_contract', {}).get('method') == method
                              for case in report['cases'].values())
        report['passed'] = (
            report['case_count'] == args.expected_cases
            and report['stable_runs']
            and report['all_fixture_hashes_match']
            and report['performance_case_count'] == args.expected_cases
            and (not args.require_certified_fillets or (
                report['all_planner_contracts_match'] and trace is not None
                and report['all_trace_passivity_matches']
            ))
            and (
                not args.require_identical
                or (
                    report['all_planner_outputs_match']
                    and report['all_trace_passivity_matches']
                )
            )
        )
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8'
        )
        args.output_markdown.write_text(report_markdown(report), encoding='utf-8')
        case_count = report['case_count']
        exact_output_matches = report['exact_output_matches']
        trace_passivity_matches = report['trace_passivity_matches']
        wall_change = report['median_case_wall_change_percent']
        rss_change = report['median_case_rss_change_percent']
        print(
            f'Compared {case_count} fixtures: '
            f'{exact_output_matches} exact baseline/candidate outputs, '
            f'{trace_passivity_matches} trace-passive cases; '
            f'median case deltas wall={wall_change}%, RSS={rss_change}%'
        )
        return 0 if report['passed'] else 1
    except (KeyError, TypeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
        print(f'comparison failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
