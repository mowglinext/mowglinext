#!/usr/bin/env python3
"""Reject incomplete, inconsistent and reordered replay comparison evidence."""

import copy
import gzip
import json
import pathlib
import tempfile
import unittest

from compare_replay import case_comparison, METRIC_KEYS, trace_geometry, validate_evidence
from run_replay import digest, input_identity, stable


class EvidenceTests(unittest.TestCase):

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = pathlib.Path(self.temporary.name)
        self.fixture = {'schema': 1, 'precision': 'float32', 'polygons': [[[0, 0]]]}
        paths = [[['0000000000000000', '3ff0000000000000']]]
        self.snapshot = {
            'transported_polygons': [paths],
            'normalized_polygons': [paths],
            'parameters': {'operation_width': 0.16},
            'stages': [
                {'name': '0.execution', 'geometry': paths},
                {'name': '0.quaternions', 'geometry': paths},
            ],
            'fields': [{
                'first_swath_heading_rad': -0.5, 'swaths': 1, 'rings': 0,
                'subpaths': 1, 'fingerprint': 'a' * 16,
            }],
            'metrics': dict.fromkeys(METRIC_KEYS, 1),
            'followstrip_fingerprint': 'a' * 16,
            'build': {
                'compiler': 'GNU-15', 'configuration': 'Release', 'gdal': '3', 'geos': '3',
                'revision': 'a' * 40, 'planner_source_sha256': 'b' * 64,
                'planner_header_sha256': 'c' * 64,
            },
        }

    def write_evidence(self, label, snapshot=None):
        directory = self.root / label
        directory.mkdir(exist_ok=True)
        snapshot = copy.deepcopy(self.snapshot if snapshot is None else snapshot)
        (directory / 'case.json').write_text(json.dumps(self.fixture), encoding='utf-8')
        with gzip.open(directory / 'case.snapshot.json.gz', 'wt') as file:
            json.dump(snapshot, file)
        (directory / 'case.0.perf.json').write_text(
            json.dumps({'seconds': 0.1, 'rss_kib': 100}), encoding='utf-8'
        )
        summary = {
            'fixture_sha256': digest(self.fixture), 'output_sha256': digest(stable(snapshot)),
            'input_identity': input_identity(snapshot), 'runs': 1, 'unique_exact_outputs': 1,
            'failures': [], 'median_planning_ms': 1, 'median_connectors_ms': 1,
            'stage_hashes': [
                {'name': stage['name'], 'sha256': digest(stage)} for stage in snapshot['stages']
            ],
        }
        for key in ('metrics', 'fields', 'build', 'followstrip_fingerprint'):
            summary[key] = snapshot[key]
        return directory, summary

    def test_valid_retained_evidence_passes(self):
        directory, summary = self.write_evidence('valid')
        self.assertEqual(validate_evidence(directory, 'case', summary), self.snapshot)

    def test_missing_summary_values_are_rejected(self):
        directory, summary = self.write_evidence('missing')
        for key in ('fixture_sha256', 'output_sha256', 'input_identity',
                    'followstrip_fingerprint', 'fields', 'build', 'stage_hashes', 'metrics'):
            with self.subTest(key=key):
                malformed = copy.deepcopy(summary)
                del malformed[key]
                with self.assertRaises(ValueError):
                    validate_evidence(directory, 'case', malformed)

    def test_snapshot_and_fixture_hash_mismatches_are_rejected(self):
        directory, summary = self.write_evidence('hashes')
        for key in ('fixture_sha256', 'output_sha256', 'input_identity'):
            with self.subTest(key=key):
                malformed = copy.deepcopy(summary)
                malformed[key] = 'f' * 64
                with self.assertRaises(ValueError):
                    validate_evidence(directory, 'case', malformed)

    def test_missing_baseline_snapshot_is_rejected(self):
        directory, summary = self.write_evidence('absent')
        (directory / 'case.snapshot.json.gz').unlink()
        with self.assertRaises(ValueError):
            validate_evidence(directory, 'case', summary)

    def test_null_geometry_and_metadata_are_rejected(self):
        for key in ('transported_polygons', 'normalized_polygons', 'parameters',
                    'fields', 'metrics', 'build'):
            with self.subTest(key=key):
                malformed = copy.deepcopy(self.snapshot)
                malformed[key] = None
                directory, summary = self.write_evidence(key, malformed)
                with self.assertRaises(ValueError):
                    validate_evidence(directory, 'case', summary)
        malformed = copy.deepcopy(self.snapshot)
        malformed['stages'][0]['geometry'] = None
        with self.assertRaises(ValueError):
            trace_geometry(malformed)

    def test_nonfinite_boolean_and_missing_metrics_are_rejected(self):
        for value in (None, True, float('nan'), float('inf'), -1):
            with self.subTest(value=value):
                malformed = copy.deepcopy(self.snapshot)
                malformed['metrics']['poses'] = value
                directory, summary = self.write_evidence('numbers')
                with gzip.open(directory / 'case.snapshot.json.gz', 'wt') as file:
                    json.dump(malformed, file)
                with self.assertRaises(ValueError):
                    validate_evidence(directory, 'case', summary)

    def test_stage_reordering_is_detected(self):
        before = copy.deepcopy(self.snapshot)
        before['stages'] += [
            {'name': '1.execution', 'geometry': before['stages'][0]['geometry']},
            {'name': '1.quaternions', 'geometry': before['stages'][1]['geometry']},
        ]
        after = copy.deepcopy(before)
        after['stages'] = after['stages'][2:] + after['stages'][:2]
        self.assertNotEqual(trace_geometry(before), trace_geometry(after))

    def test_duplicate_stages_are_rejected(self):
        malformed = copy.deepcopy(self.snapshot)
        malformed['stages'].append(malformed['stages'][0])
        with self.assertRaises(ValueError):
            trace_geometry(malformed)

    def test_exact_trace_drift_is_rejected_by_comparison(self):
        baseline_dir, baseline = self.write_evidence('baseline')
        candidate_dir, candidate = self.write_evidence('candidate')
        traced = copy.deepcopy(self.snapshot)
        traced['stages'][0]['geometry'][0][0][0] = '0000000000000001'
        trace_dir, trace = self.write_evidence('trace', traced)
        result = case_comparison(
            'case', baseline, candidate, trace, baseline_dir, candidate_dir,
            trace_dir, 1, 1, 1, 1
        )
        self.assertTrue(result['planner_identical'])
        self.assertFalse(result['trace_passivity']['passive'])


if __name__ == '__main__':
    unittest.main()
