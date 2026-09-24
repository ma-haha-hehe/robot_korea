import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


@pytest.fixture
def sampler():
    path = Path(__file__).resolve().parents[3] / 'scripts/select_workbenchmark_sample.py'
    spec = importlib.util.spec_from_file_location('sample_restore_test_module', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def selection(tmp_path):
    products = tmp_path / 'products'
    products.mkdir()
    source = products / 'tier1_task_001.yaml'
    source.write_bytes(b'# Keep recorded formatting.\nblocks: []\n')
    manifest = tmp_path / 'selection.json'
    data = dict(seed=42, population=2, selected=1, tasks=[dict(
        product='tier1_task_001', input_sha256=hashlib.sha256(source.read_bytes()).hexdigest())])
    manifest.write_text(json.dumps(data, indent=2) + '\n')
    return products, source, manifest, tmp_path / 'restored'


def test_restore_cli_preserves_exact_inputs_without_replanning(sampler, selection, monkeypatch):
    products, source, manifest, output = selection
    def unexpected_plan(*args, **kwargs):
        raise AssertionError('restoring a frozen sample must not replan')
    monkeypatch.setattr(sampler, 'plan_assembly', unexpected_plan)
    monkeypatch.setattr(sys, 'argv', ['select_workbenchmark_sample.py',
        '--products', str(products), '--selection-manifest', str(manifest),
        '--output-dir', str(output)])
    sampler.main()
    assert (output / source.name).read_bytes() == source.read_bytes()
    assert (output / 'selection.json').read_bytes() == manifest.read_bytes()


@pytest.mark.parametrize('fault', ['changed', 'missing'])
def test_restore_rejects_changed_or_missing_input_before_writing(sampler, selection, fault):
    products, source, manifest, output = selection
    if fault == 'changed':
        source.write_text('different input\n')
    else:
        source.unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        sampler.restore_selection(products, manifest, output)
    assert not output.exists()


@pytest.mark.parametrize('fault', ['duplicate', 'path_escape'])
def test_restore_rejects_ambiguous_task_names(sampler, selection, fault):
    products, _, manifest, output = selection
    data = json.loads(manifest.read_text())
    if fault == 'duplicate':
        data['tasks'] *= 2
        data['selected'] = 2
    else:
        data['tasks'][0]['product'] = '../tier1_task_001'
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        sampler.restore_selection(products, manifest, output)
    assert not output.exists()


@pytest.mark.parametrize('option', [['--seed', '42'], ['--per-tier', '60'],
                                    ['--include', 'tier1_task_001']])
def test_restore_cli_rejects_new_sampling_options(sampler, selection, monkeypatch, option):
    products, _, manifest, output = selection
    monkeypatch.setattr(sys, 'argv', ['select_workbenchmark_sample.py',
        '--products', str(products), '--selection-manifest', str(manifest),
        '--output-dir', str(output), *option])
    with pytest.raises(SystemExit) as exc:
        sampler.main()
    assert exc.value.code == 2
    assert not output.exists()
