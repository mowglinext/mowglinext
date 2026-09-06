import importlib.util
from pathlib import Path
import sys


def _load_e2e_module():
    path = Path(__file__).resolve().parents[2] / 'e2e_test.py'
    spec = importlib.util.spec_from_file_location(
        'e2e_test_under_test', path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


e2e = _load_e2e_module()


def test_informational_failure_does_not_fail_e2e_process():
    criteria = [
        ('core cycle', True, True),
        ('historical coverage metric', False, False),
    ]

    assert e2e._required_criteria_pass(criteria)


def test_required_failure_fails_e2e_process():
    criteria = [
        ('core cycle', False, True),
        ('historical coverage metric', True, False),
    ]

    assert not e2e._required_criteria_pass(criteria)


def test_coverage_target_is_a_required_e2e_criterion():
    source = (
        Path(__file__).resolve().parents[2] / 'e2e_test.py'
    ).read_text()

    assert '("Area coverage >= 80%", coverage_pass, True)' in source


def test_post_cycle_checks_wait_for_actual_docked_state():
    source = (
        Path(__file__).resolve().parents[2] / 'e2e_test.py'
    ).read_text()

    assert 'if state_name == "IDLE_DOCKED" and self.mowing_started:' in source
    assert 'if self.current_is_charging:' in source
    assert (
        'if state_name in ("MOWING_COMPLETE", "IDLE_DOCKED") '
        'and self.mowing_started:'
    ) not in source


def test_mowing_efficiency_is_one_for_exact_distance():
    assert e2e._mowing_efficiency(100.0, 100.0) == 1.0


def test_mowing_efficiency_penalizes_incomplete_traversal():
    assert e2e._mowing_efficiency(300.0, 30.0) == 0.1


def test_mowing_efficiency_penalizes_excess_distance():
    assert e2e._mowing_efficiency(300.0, 400.0) == 0.75


def test_mowing_efficiency_rejects_missing_distance():
    assert e2e._mowing_efficiency(0.0, 100.0) == 0.0
    assert e2e._mowing_efficiency(100.0, 0.0) == 0.0


def test_obstacle_criterion_requires_complete_pass():
    assert e2e._obstacle_test_passed('PASS')
    assert not e2e._obstacle_test_passed('PARTIAL')
    assert not e2e._obstacle_test_passed('FAIL')
    assert not e2e._obstacle_test_passed(None)


def test_mowing_timeout_is_configurable_without_changing_default():
    source = (
        Path(__file__).resolve().parents[2] / 'e2e_test.py'
    ).read_text()

    assert 'os.getenv("E2E_MOWING_TIMEOUT_S", "3000")' in source


def test_e2e_tracks_the_published_full_coverage_plan():
    source_root = Path(__file__).resolve().parents[2]
    e2e_source = (source_root / 'e2e_test.py').read_text()
    behavior_source = (
        source_root / 'mowgli_behavior' / 'src' / 'coverage_nodes.cpp'
    ).read_text()

    assert '"/coverage/full_plan"' in behavior_source
    assert '"/coverage/full_plan"' in e2e_source
    assert '"/coverage_planner_node/coverage_path"' not in e2e_source


def test_simulation_reset_preserves_persisted_area_geometry():
    repo_root = Path(__file__).resolve().parents[4]
    reset_source = (
        repo_root / 'docker' / 'reset-simulation-run-state.sh'
    ).read_text()

    for generated_file in (
        'fusion_graph.graph',
        'fusion_graph.meta',
        'fusion_graph.scans',
        'coverage_resume.txt',
    ):
        assert f'/ros2_ws/maps/{generated_file}' in reset_source

    # The keyframe map (.keyframes) was removed 2026-09-07 with the keyframe
    # anchor; the reset must not reference a file the stack no longer writes.
    assert 'fusion_graph.keyframes' not in reset_source
    assert '/ros2_ws/maps/areas.dat' not in reset_source
    assert '/ros2_ws/maps/*' not in reset_source


def test_self_contained_e2e_targets_reset_generated_run_state():
    repo_root = Path(__file__).resolve().parents[4]
    makefile = (repo_root / 'ros2' / 'Makefile').read_text()

    assert makefile.count('../docker/reset-simulation-run-state.sh;') == 2
