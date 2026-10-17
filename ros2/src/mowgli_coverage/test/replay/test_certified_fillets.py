#!/usr/bin/env python3
"""Real-planner RED controls for the narrow certified-fillets replay contract."""

import argparse
import copy
import gzip
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

from compare_replay import case_comparison, certify_fillets
from run_replay import digest, fixture, input_identity, rectangle, run_case, stable


class CertifiedReplayTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='coverage-fillet-controls-')
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = pathlib.Path(cls.temporary.name)
        cls.data = fixture([[rectangle(6, 6)]])
        cls.directories, cls.summaries, cls.snapshots = {}, {}, {}
        for label, binary, plain in (
                ('baseline', OPTIONS.baseline, True),
                ('candidate', OPTIONS.candidate, True),
                ('traced', OPTIONS.candidate, False)):
            directory = cls.root / label
            directory.mkdir()
            summary = run_case(binary, 'control', cls.data, directory, 1, 1, plain, 60)
            with gzip.open(directory / 'control.snapshot.json.gz', 'rt') as file:
                snapshot = json.load(file)
            cls.directories[label], cls.summaries[label], cls.snapshots[label] = (
                directory, summary, snapshot)

    def setUp(self):
        self.summaries = copy.deepcopy(type(self).summaries)
        self.snapshots = copy.deepcopy(type(self).snapshots)
        for label in self.summaries:
            self.write(label)

    def write(self, label):
        directory, snapshot, summary = (
            self.directories[label], self.snapshots[label], self.summaries[label])
        (directory / 'control.json').write_text(json.dumps(self.data), encoding='utf-8')
        with gzip.open(directory / 'control.snapshot.json.gz', 'wt') as file:
            json.dump(snapshot, file)
        for key in ('metrics', 'fields', 'build', 'followstrip_fingerprint'):
            summary[key] = snapshot[key]
        summary.update(output_sha256=digest(stable(snapshot)),
                       input_identity=input_identity(snapshot),
                       stage_hashes=[{'name': stage['name'], 'sha256': digest(stage)}
                                     for stage in snapshot['stages']])
        (directory / 'summary.json').write_text(json.dumps({'control': summary}), encoding='utf-8')

    def arguments(self):
        return ('control', self.summaries['baseline'], self.summaries['candidate'],
                self.summaries['traced'], self.directories['baseline'],
                self.directories['candidate'], self.directories['traced'], OPTIONS.reference)

    def stage(self, suffix):
        return next(stage for stage in self.snapshots['traced']['stages']
                    if stage['name'].endswith(suffix))

    def ring(self):
        return next(stage['name'].split('.')[-1] for stage in self.snapshots['traced']['stages']
                    if '.bevel_replacements.' in stage['name'] and stage['geometry'])

    def compare_cli(self, strict=False):
        environment = self.root / 'test-environment.json'
        # Synthetic metadata identifies comparator controls, not field/build evidence.
        environment.write_text(
            json.dumps({'architecture': 'comparator-test', 'compiler': 'comparator-test',
                        'build_type': 'comparator-test',
                        'fields2cover_revision': 'comparator-test',
                        'fields2cover_library_sha256': ['comparator-test']}), encoding='utf-8')
        command = [sys.executable, str(pathlib.Path(__file__).with_name('compare_replay.py')),
                   '--baseline-dir', str(self.directories['baseline']),
                   '--candidate-dir', str(self.directories['candidate']),
                   '--trace-dir', str(self.directories['traced']),
                   '--baseline-binary', str(OPTIONS.reference),
                   '--environment', str(environment),
                   '--output-json', str(self.root / 'comparison.json'),
                   '--output-markdown', str(self.root / 'comparison.md'),
                   '--expected-cases', '1', '--expected-runs', '1',
                   '--expected-trace-runs', '1', '--expected-processes', '1',
                   '--expected-trace-processes', '1',
                   '--require-identical' if strict else '--require-certified-fillets']
        completed = subprocess.run(command, capture_output=True, text=True, timeout=60)
        self.assertIn(completed.returncode, (0, 1), completed.stderr)
        return completed.returncode, json.loads((self.root / 'comparison.json').read_text())

    def test_real_planner_contract_and_strict_diagnostic(self):
        result = case_comparison(
            'control', self.summaries['baseline'], self.summaries['candidate'],
            self.summaries['traced'], self.directories['baseline'],
            self.directories['candidate'], self.directories['traced'], 1, 1, 1, 1)
        method = 'identical' if result['planner_identical'] else 'certified_fillets'
        if OPTIONS.expect_method:
            self.assertEqual(method, OPTIONS.expect_method)
        self.assertTrue(result['trace_passivity']['passive'])
        self.assertEqual(certify_fillets(*self.arguments())['method'], method)
        status, report = self.compare_cli()
        self.assertEqual(status, 0)
        self.assertTrue(report['all_planner_contracts_match'])
        self.assertEqual(report['cases']['control']['planner_contract']['method'], method)
        strict_status, strict_report = self.compare_cli(strict=True)
        self.assertEqual(strict_status, 0 if method == 'identical' else 1)
        self.assertEqual(strict_report['all_planner_outputs_match'], method == 'identical')

    def test_unrelated_downstream_bit_change_is_rejected(self):
        stage = next(stage for stage in self.snapshots['candidate']['stages']
                     if stage['name'].endswith('.execution'))
        stage['geometry'][0][0][0] = '0000000000000001'
        self.stage('.execution')['geometry'][0][0][0] = '0000000000000001'
        self.write('candidate')
        self.write('traced')
        with self.assertRaises(ValueError):
            certify_fillets(*self.arguments())
        status, report = self.compare_cli()
        self.assertEqual(status, 1)
        self.assertFalse(report['all_planner_contracts_match'])

    def test_unaccounted_inserted_point_is_rejected(self):
        path = self.stage('bevel_intermediate.' + self.ring())['geometry'][0]
        path.insert(1, copy.deepcopy(path[1]))
        self.write('traced')
        with self.assertRaises(ValueError):
            certify_fillets(*self.arguments())

    def test_erased_pivot_and_forged_source_are_rejected(self):
        path = self.stage('bevel_source.' + self.ring())['geometry'][0]
        path.insert(1, copy.deepcopy(path[1]))
        self.write('traced')
        with self.assertRaises(ValueError):
            certify_fillets(*self.arguments())

    def test_wrong_radius_samples_are_rejected(self):
        local = self.stage('bevel_replacements.' + self.ring())['geometry'][0]
        local[2][0] = '0000000000000001'
        self.write('traced')
        with self.assertRaises(ValueError):
            certify_fillets(*self.arguments())

    def test_forged_rounded_trace_is_rejected(self):
        self.stage('bevel_rounded.' + self.ring())['geometry'][0][1][0] = '0000000000000001'
        self.write('traced')
        with self.assertRaises(ValueError):
            certify_fillets(*self.arguments())

    def test_changed_boundary_is_rejected(self):
        state = self.stage('.plan_state')['state']
        state['boundaries'][0][0][0] = '0000000000000001'
        next(stage for stage in self.snapshots['candidate']['stages']
             if stage['name'].endswith('.plan_state'))['state'] = copy.deepcopy(state)
        self.write('candidate')
        self.write('traced')
        with self.assertRaisesRegex(ValueError, 'non-ring plan state'):
            certify_fillets(*self.arguments())

    def test_malformed_observer_schema_is_rejected(self):
        self.stage('bevel_replacements.' + self.ring())['geometry'] = None
        self.write('traced')
        status, report = self.compare_cli()
        self.assertEqual(status, 1)
        self.assertFalse(report['all_planner_contracts_match'])

    def test_traced_build_identity_drift_is_rejected(self):
        self.snapshots['traced']['build']['requested_cxx_flags_sha256'] = 'f' * 64
        self.write('traced')
        status, report = self.compare_cli()
        self.assertEqual(status, 1)
        self.assertFalse(report['all_planner_contracts_match'])

    def test_changed_fixture_is_rejected(self):
        changed = copy.deepcopy(self.data)
        changed['parameters']['operation_width'] = .19
        (self.directories['candidate'] / 'control.json').write_text(
            json.dumps(changed), encoding='utf-8')
        self.summaries['candidate']['fixture_sha256'] = digest(changed)
        (self.directories['candidate'] / 'summary.json').write_text(
            json.dumps({'control': self.summaries['candidate']}), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'fixture identities differ'):
            certify_fillets(*self.arguments())
        status, report = self.compare_cli()
        self.assertEqual(status, 1)
        self.assertFalse(report['all_planner_contracts_match'])
        self.assertFalse(report['all_fixture_hashes_match'])

    def test_new_mode_still_rejects_trace_drift_and_unstable_runs(self):
        self.snapshots['traced']['followstrip_fingerprint'] = 'f' * 16
        self.write('traced')
        status, report = self.compare_cli()
        self.assertEqual(status, 1)
        self.assertFalse(report['all_planner_contracts_match'])
        self.assertFalse(report['all_trace_passivity_matches'])
        self.snapshots['traced'] = copy.deepcopy(type(self).snapshots['traced'])
        self.write('traced')
        self.summaries['candidate']['unique_exact_outputs'] = 2
        (self.directories['candidate'] / 'summary.json').write_text(
            json.dumps({'control': self.summaries['candidate']}))
        status, report = self.compare_cli()
        self.assertEqual(status, 1)
        self.assertFalse(report['stable_runs'])
        self.assertFalse(report['all_planner_contracts_match'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=pathlib.Path, required=True)
    parser.add_argument('--reference', type=pathlib.Path, required=True)
    parser.add_argument('--candidate', type=pathlib.Path, required=True)
    parser.add_argument('--expect-method', choices=('identical', 'certified_fillets'))
    OPTIONS, remaining = parser.parse_known_args()
    unittest.main(argv=[sys.argv[0], *remaining])
