"""Sample evidence and deadlines cannot be bypassed by partial execution."""
import hashlib
import json
import os
from pathlib import Path
import runpy
import sys
import threading
import time
import pytest


@pytest.fixture
def runner():
    return runpy.run_path(str(Path(__file__).resolve().parents[3]/'scripts/run_workbenchmark_sample.py'))


@pytest.mark.parametrize('fault', ['none', 'changed', 'duplicate', 'path'])
def test_selection_is_frozen_before_processes_start(tmp_path, runner, fault):
    source = tmp_path/'two_brick_stack_001.yaml'
    source.write_text('blocks: []\n')
    row = {'product': source.stem, 'tier': 1, 'input_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    rows = [row]
    if fault == 'changed': source.write_text('changed\n')
    if fault == 'duplicate': rows = [row, row]
    if fault == 'path': row['product'] = '../two_brick_stack_001'
    (tmp_path/'selection.json').write_text(json.dumps({'selected': len(rows), 'tasks': rows}))
    if fault == 'none':
        assert runner['load_selection'](tmp_path)['selected'] == 1
    else:
        with pytest.raises(ValueError): runner['load_selection'](tmp_path)


def test_unrun_cases_are_reported_separately_from_failures(runner):
    expected = [{'tier': 1}, {'tier': 1}, {'tier': 1}, {'tier': 2}]
    rows = [{'tier': 1, 'success': True},
            {'tier': 1, 'success': False, 'failed_phase': 'grasp_approach'},
            {'tier': 1, 'success': False, 'not_run': True},
            {'tier': 2, 'success': False, 'not_run': True}]
    result = runner['summarize'](rows, expected)
    assert result['finished'] and not result['fully_executed']
    assert result['tiers']['1']['attempted'] == 2
    assert result['tiers']['1']['failures'] == 1
    assert result['tiers']['1']['not_run'] == 1
    assert result['tiers']['2']['success_fraction_of_attempted'] is None


def test_expired_work_deadline_does_not_launch_a_child(tmp_path, runner, monkeypatch):
    monkeypatch.setattr(runner['subprocess'], 'Popen', lambda *a, **kw: pytest.fail('must not start'))
    with (tmp_path/'log').open('w') as log:
        code, reason, elapsed = runner['run_child']([], os.environ.copy(), log, 60.,
                                                    time.time()-1, threading.Event())
    assert code == 124 and reason and elapsed == 0


@pytest.mark.skipif(os.name != 'posix', reason='the simulation worker uses POSIX process groups')
def test_process_timeout_requests_stop_and_preserves_shutdown_evidence(tmp_path, runner):
    marker = tmp_path/'stopped.txt'
    script = tmp_path/'child.py'
    script.write_text('''import signal,time,sys
from pathlib import Path
def stop(*args):
    Path(sys.argv[1]).write_text('shutdown checkpoint')
    raise SystemExit(0)
signal.signal(signal.SIGTERM, stop)
while True: time.sleep(.01)
''')
    with (tmp_path/'log').open('w') as log:
        code, reason, elapsed = runner['run_child']([sys.executable, str(script), str(marker)],
            os.environ.copy(), log, .4, time.time()+10, threading.Event())
    assert code == 124 and reason == 'episode process timeout'
    assert marker.read_text() == 'shutdown checkpoint'
    assert elapsed < 3


def test_stop_between_selection_and_child_launch_counts_as_unrun(tmp_path, runner, monkeypatch):
    products = tmp_path/'products'
    products.mkdir()
    source = products/'two_brick_stack_001.yaml'
    source.write_text('blocks: []\n')
    row = dict(product=source.stem, tier=1, input_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (products/'selection.json').write_text(json.dumps({'selected': 1, 'tasks': [row]}))
    globals_ = runner['main'].__globals__
    monkeypatch.setitem(globals_, 'run_child', lambda *args: (124, 'work stopped before process launch', 0.))
    monkeypatch.setenv('MJ_BRIDGE_BENCHMARK_CONFIG', '/missing/general/environment-config.yaml')
    output = tmp_path/'execution'
    assert runner['main'](['--products', str(products), '--output-dir', str(output)]) == 1
    summary = json.loads((output/'summary.json').read_text())
    assert summary['finished'] and not summary['fully_executed']
    assert summary['executed_episodes'] == 0 and summary['tiers']['1']['not_run'] == 1
    assert summary['results'][0]['not_run'] is True
