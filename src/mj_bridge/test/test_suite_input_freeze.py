"""Changed inputs cannot silently join a frozen physical regression suite."""
import hashlib
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('mutation', ['none', 'config', 'product', 'deleted_config'])
def test_suite_rejects_inputs_changed_during_child_execution(tmp_path, monkeypatch, mutation):
    script = Path(__file__).resolve().parents[3] / 'scripts/test_product_suite.py'
    namespace = runpy.run_path(str(script))
    products = tmp_path / 'products'
    products.mkdir()
    product = products / 'part.yaml'
    product.write_text('name: original\n')
    config = tmp_path / 'external-config.yaml'
    config.write_text('max_episode_time_s: 60\n')
    config_hash = hashlib.sha256(config.read_bytes()).hexdigest()
    product_hash = hashlib.sha256(product.read_bytes()).hexdigest()
    output = tmp_path / 'results'
    monkeypatch.setenv('MJ_BRIDGE_BENCHMARK_CONFIG', str(config))
    monkeypatch.setattr(sys, 'argv', [str(script), '--products', str(products),
        '--output-dir', str(output), '--seeds', '42', '--jobs', '1'])

    def child(command, **kwargs):
        run = Path(command[command.index('--output-dir') + 1])
        (run / 'result.json').write_text(json.dumps({'success': True, 'completion': 1}))
        if mutation == 'config':
            config.write_text('max_episode_time_s: 600\n')
        elif mutation == 'product':
            product.write_text('name: changed\n')
        elif mutation == 'deleted_config':
            config.unlink()
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(namespace['subprocess'], 'run', child)
    code = namespace['main']()
    result = json.loads((output / 'summary.json').read_text())
    frozen = result['execution_inputs_sha256']
    assert frozen['configuration'] == {str(config): config_hash}
    assert frozen['products'] == {str(product): product_hash}
    assert result['results'][0]['returncode'] == 0
    assert result['finished'] is True
    assert code == (0 if mutation == 'none' else 1)
    assert result['successes'] == (1 if mutation == 'none' else 0)
    if mutation != 'none':
        assert 'result cannot be accepted' in result['results'][0]['error']


def test_missing_external_configuration_never_starts_an_episode(tmp_path, monkeypatch):
    script = Path(__file__).resolve().parents[3] / 'scripts/test_product_suite.py'
    namespace = runpy.run_path(str(script))
    products = tmp_path / 'products'
    products.mkdir()
    (products / 'part.yaml').write_text('name: part\n')
    monkeypatch.setenv('MJ_BRIDGE_BENCHMARK_CONFIG', str(tmp_path / 'missing.yaml'))
    monkeypatch.setattr(sys, 'argv', [str(script), '--products', str(products),
        '--output-dir', str(tmp_path / 'out')])
    monkeypatch.setattr(namespace['subprocess'], 'run',
                        lambda *a, **k: pytest.fail('episode must not start'))
    with pytest.raises(SystemExit) as error:
        namespace['main']()
    assert error.value.code == 2


@pytest.mark.parametrize('skip_invalid', [False, True])
def test_queue_freezes_all_selected_inputs_before_filtering(tmp_path, monkeypatch, skip_invalid):
    script = Path(__file__).resolve().parents[3] / 'scripts/test_product_suite.py'
    namespace = runpy.run_path(str(script))
    products = tmp_path / 'products'
    products.mkdir()
    for name in ('a', 'b'):
        (products / f'{name}.yaml').write_text(f'name: {name}\n')
    config = tmp_path / 'config.yaml'
    config.write_text('max_episode_time_s: 60\n')
    output = tmp_path / 'out'
    argv = [str(script), '--products', str(products), '--output-dir', str(output),
            '--jobs', '1', '--seeds', '42']
    if skip_invalid:
        argv.append('--skip-invalid-products')
    monkeypatch.setattr(sys, 'argv', argv)
    monkeypatch.setenv('MJ_BRIDGE_BENCHMARK_CONFIG', str(config))
    globals_ = namespace['main'].__globals__
    monkeypatch.setitem(globals_, 'normalized_from_path', lambda p: Path(p).stem)
    monkeypatch.setitem(globals_, 'audit_product',
                        lambda name: {'valid_for_snap': name == 'a', 'issues': ['invalid']})
    started = []
    def child(command, **kwargs):
        started.append(command)
        run = Path(command[command.index('--output-dir') + 1])
        (run / 'result.json').write_text(json.dumps({'success': True, 'completion': 1}))
        if not skip_invalid:
            config.write_text('max_episode_time_s: 600\n')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(namespace['subprocess'], 'run', child)
    assert namespace['main']() == (0 if skip_invalid else 1)
    result = json.loads((output / 'summary.json').read_text())
    assert len(started) == 1
    assert len(result['execution_inputs_sha256']['products']) == 2
    if skip_invalid:
        assert len(result['invalid_products']) == 1
        assert result['successes'] == 1
    else:
        assert result['executed_episodes'] == 1
        cancelled = next(row for row in result['results'] if row['product'] == 'b')
        assert cancelled['not_run'] is True
        assert 'execution cancelled' in cancelled['error']
