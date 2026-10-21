"""Reports keep missing cases explicit and refuse contradictory evidence."""
import hashlib
import json
from pathlib import Path
import runpy
import pytest


ROOT = Path(__file__).resolve().parents[3]


def fixture(tmp_path):
    report = runpy.run_path(str(ROOT/'scripts/report_workbenchmark_sample.py'))
    runner = runpy.run_path(str(ROOT/'scripts/run_workbenchmark_sample.py'))
    products, suite = tmp_path/'products', tmp_path/'suite'
    products.mkdir(); suite.mkdir()
    product = products/'two_brick_stack_001.yaml'
    product.write_text('blocks: []\n')
    task = dict(product=product.stem, tier=1, input_sha256=hashlib.sha256(product.read_bytes()).hexdigest())
    selection = {'selected': 1, 'tasks': [task]}
    schedule = [dict(task, seed=42), dict(task, seed=43)]
    rows = [dict(task, seed=42, success=False, completion=0., error='missing execution evidence')]
    summary = dict(runner['summarize'](rows, schedule), source_sha256={})
    for name, value in [('selection.json', selection), ('execution_schedule.json', schedule),
                        ('summary.json', summary)]:
        (suite/name).write_text(json.dumps(value))
    return report, products, suite, summary


def test_partial_report_does_not_claim_the_second_seed_executed(tmp_path):
    report, products, suite, _ = fixture(tmp_path)
    evidence = report['read_suite'](suite, products)
    summary = evidence['summary']
    assert not summary['finished'] and summary['executed_episodes'] == 1
    assert summary['tiers']['1']['not_run'] == 1
    assert '尚未结束' in report['markdown']({'current': evidence})


@pytest.mark.parametrize('fault', ['counts', 'finished', 'wrong_seed', 'success', 'changed_input'])
def test_report_rejects_invented_coverage_or_success(tmp_path, fault):
    report, products, suite, summary = fixture(tmp_path)
    if fault == 'counts': summary['tiers']['1']['successes'] = 1
    if fault == 'finished': summary['finished'] = True
    if fault == 'wrong_seed': summary['results'][0]['seed'] = 44
    if fault == 'success': summary['results'][0]['success'] = True
    if fault == 'changed_input': (products/'two_brick_stack_001.yaml').write_text('changed')
    (suite/'summary.json').write_text(json.dumps(summary))
    with pytest.raises((ValueError, KeyError)):
        report['read_suite'](suite, products)


def test_paired_results_ignore_unrun_or_missing_cases(tmp_path):
    report, *_ = fixture(tmp_path)
    current = [dict(product='a', seed=42, success=True),
               dict(product='b', seed=42, success=False, not_run=True),
               dict(product='c', seed=42, success=True)]
    baseline = [dict(product='a', seed=42, success=False),
                dict(product='b', seed=42, success=True)]
    changes = report['paired_changes'](current, baseline)
    assert len(changes) == 1 and changes[0]['change'] == 'improved'


def test_timeout_pair_is_not_attributed_to_controller_improvement(tmp_path):
    report, *_ = fixture(tmp_path)
    current = [dict(product='a', seed=42, success=True)]
    baseline = [dict(product='a', seed=42, success=False, stopped='episode process timeout')]
    assert report['paired_changes'](current, baseline)[0]['change'] == 'limited'
