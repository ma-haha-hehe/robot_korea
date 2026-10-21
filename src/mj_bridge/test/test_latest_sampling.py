"""Version 2 sample coverage, frozen input restoration and tier isolation."""
import hashlib
import json
from pathlib import Path
import runpy

import pytest
import yaml


@pytest.fixture
def sampler():
    path = Path(__file__).resolve().parents[3] / 'scripts/select_workbenchmark_sample.py'
    return runpy.run_path(str(path))


def product(directory, name, count):
    path = directory / (name + '.yaml')
    path.write_text(yaml.safe_dump({'product': {'name': name}, 'blocks': [
        {'id': f'b{i}', 'type': 'brick_2x2', 'color': 'red',
         'target': {'position': [0., 0., .0192*i], 'yaw_deg': 0.}}
        for i in range(count)]}))
    return path


def test_sampling_current_names_and_requested_tiers_only(tmp_path, sampler):
    for tier, prefix in enumerate(sampler['TASK_CLASSES'], 1):
        for i in range(2):
            product(tmp_path, f'{prefix}_{i+1:03d}', 2 if tier <= 2 else 3)
    inventory, chosen, coverage = sampler['select'](tmp_path, 2, 42, [], tiers=(1, 2, 3))
    assert len(chosen) == 6
    assert set(coverage) == {1, 2, 3}
    assert all(inventory[name]['tier'] != 4 for name in chosen)
    assert {row['population'] for row in coverage.values()} == {2}


def test_count_coverage_includes_infrequent_high_count_cases(tmp_path, sampler):
    for i, count in enumerate((3, 3, 4, 5, 6, 8, 9, 12), 1):
        product(tmp_path, f'simple_shape_assembly_{i:03d}', count)
    inventory, chosen, coverage = sampler['select'](tmp_path, 3, 42, [], tiers=(3,))
    groups = {sampler['count_group'](inventory[name]['stratum'][0], 3) for name in chosen}
    assert groups == {'3-5', '6-8', '9-12'}
    assert all(row['selected'] == 1 for row in coverage[3]['count_groups'].values())


def test_selection_is_reproducible_and_keeps_mandatory_failure(tmp_path, sampler):
    for i in range(1, 9):
        product(tmp_path, f'simple_multi_brick_stack_{i:03d}', 2 + i % 4)
    args = (tmp_path, 5, 42, ['simple_multi_brick_stack_002'])
    first = sampler['select'](*args, tiers=(2,))
    second = sampler['select'](*args, tiers=(2,))
    assert first == second
    assert 'simple_multi_brick_stack_002' in first[1]


def test_mandatory_case_cannot_silently_add_an_excluded_tier(tmp_path, sampler):
    product(tmp_path, 'two_brick_stack_001', 2)
    product(tmp_path, 'balanced_order_sensitive_001', 3)
    with pytest.raises(ValueError, match='not found'):
        sampler['select'](tmp_path, 1, 42, ['balanced_order_sensitive_001'], tiers=(1,))


def test_current_selection_restores_original_bytes_without_replanning(tmp_path, sampler):
    products = tmp_path / 'products'
    products.mkdir()
    path = product(products, 'simple_shape_assembly_100', 12)
    data = {'population': 100, 'selected': 1, 'seed': 42, 'tasks': [
        {'product': path.stem, 'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}]}
    manifest = tmp_path / 'selection.json'
    manifest.write_text(json.dumps(data))
    output = tmp_path / 'restored'
    sampler['restore_selection'](products, manifest, output)
    assert (output / path.name).read_bytes() == path.read_bytes()
    assert (output / 'selection.json').read_bytes() == manifest.read_bytes()


@pytest.mark.parametrize('name', ['../two_brick_stack_001', '/two_brick_stack_001',
                                'two_brick_stack_001/other', 'simple_shape_assembly_x'])
def test_tier_parser_rejects_paths_and_ambiguous_ids(sampler, name):
    assert sampler['task_tier'](name) is None
