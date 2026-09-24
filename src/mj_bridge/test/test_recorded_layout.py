import copy
import math
import subprocess
import runpy
from pathlib import Path

import pytest
import yaml
from mj_bridge.benchmark_core import ProductError, normalize_product, generate_episode


def product():
    return normalize_product({
        'blocks': [{'name': '2x2_a', 'pos': [0, 0, 0]},
                   {'name': '4x2_b', 'pos': [0, 0, .0191]}],
        'initial_layout': {'frame': 'world_tabletop', 'blocks': [
            {'id': '2x2_a', 'position_xy_m': [.25, -.2], 'yaw_deg': 90},
            {'id': '4x2_b', 'position_xy_m': [.45, -.2], 'yaw_deg': 0}]}})


def test_recorded_layout_preserved_and_seed_independent():
    p = product()
    normalized = normalize_product(p)
    assert normalized == p
    normalized['initial_layout']['blocks'][0]['position_xy_m'][0] = .3
    assert p['initial_layout']['blocks'][0]['position_xy_m'][0] == .25
    a = generate_episode(p, seed=0, connection_mode='physics')
    b = generate_episode(p, seed=42, connection_mode='physics')
    assert a['spawned_blocks'] == b['spawned_blocks']
    assert a['initial_layout_mode'] == 'recorded'
    assert a['spawned_blocks'][0]['position'] == pytest.approx([.25, -.2, .0586])
    assert a['spawned_blocks'][0]['yaw_rad'] == pytest.approx(math.pi / 2)


@pytest.mark.parametrize('problem', ['missing', 'duplicate', 'overlap', 'edge', 'nan', 'frame'])
def test_invalid_recorded_layout_fails_before_execution(problem):
    p = product()
    layout = p['initial_layout']
    rows = layout['blocks']
    if problem == 'missing': rows.pop()
    if problem == 'duplicate': rows[1]['id'] = rows[0]['id']
    if problem == 'overlap': rows[1]['position_xy_m'] = rows[0]['position_xy_m'][:]
    if problem == 'edge': rows[0]['position_xy_m'][0] = .1
    if problem == 'nan': rows[0]['yaw_deg'] = float('nan')
    if problem == 'frame': layout['frame'] = 'unknown'
    with pytest.raises(ProductError):
        generate_episode(p, seed=42, connection_mode='physics')


def test_import_preserves_relative_goals_and_recorded_xy(tmp_path):
    repo = tmp_path / 'repo'
    tasks = repo / 'benchmark_tasks'
    tasks.mkdir(parents=True)
    raw = {'blocks': [
        {'name': '2x2_a', 'type': 'brick_2x2', 'pos': [.1, .2, 0], 'rotation': [0, 0, 0]},
        {'name': '4x2_b', 'type': 'brick_4x2', 'pos': [.116, .2, .0191], 'rotation': [0, 0, 90]}]}
    raw['initial_blocks'] = copy.deepcopy(raw['blocks'])
    for b, x in zip(raw['initial_blocks'], [.25, .45]):
        b['pos'] = [x, -.2, .0495]
    source = tasks / 'sample.yaml'
    source.write_text(yaml.safe_dump(raw))
    original = source.read_bytes()
    subprocess.run(['git', 'init', '-q', str(repo)], check=True)
    subprocess.run(['git', '-C', str(repo), 'add', '.'], check=True)
    subprocess.run(['git', '-C', str(repo), '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                    'commit', '-qm', 'fixture'], check=True)
    importer = runpy.run_path(str(Path(__file__).resolve().parents[3] / 'scripts/import_workbenchmark.py'))['import_tasks']
    output = tmp_path / 'output'
    manifest = importer(repo, output, 'recorded')
    p = normalize_product(yaml.safe_load((output / 'sample.yaml').read_text()))
    a, b = (r['target']['position'] for r in p['blocks'])
    assert [v-u for u, v in zip(a, b)] == pytest.approx([.016, 0, .0191])
    assert p['blocks'][1]['target']['yaw_deg'] == 90
    assert p['initial_layout']['blocks'][0]['position_xy_m'] == [.25, -.2]
    assert source.read_bytes() == original
    assert len(manifest['tasks']) == 1
    importer(repo, output, 'recorded')  # Identical import is safe.
    (output / 'sample.yaml').write_text('different')
    with pytest.raises(ValueError, match='refusing to overwrite'):
        importer(repo, output, 'recorded')


def test_generate_cli_accepts_physics_model(monkeypatch, tmp_path):
    from mj_bridge import benchmark_cli
    seen = {}
    def generate(path, seed, output, connection_mode, robot_base_x=0., contact_profile="loose"):
        seen['mode'] = connection_mode
        return {'episode_id': 'test'}, tmp_path / 'scene.xml'
    monkeypatch.setattr(benchmark_cli, 'generate', generate)
    benchmark_cli.main(['generate', '--product', 'recorded.yaml', '--connection-mode', 'physics'])
    assert seen['mode'] == 'physics'


def test_robot_base_offset_preserves_brick_world_coordinates(tmp_path):
    import mujoco
    from mj_bridge.benchmark_cli import generate
    path=tmp_path/'product.yaml'
    path.write_text(yaml.safe_dump(product()))
    zero,_=generate(str(path),42,str(tmp_path/'zero'),connection_mode='physics')
    shifted,scene=generate(str(path),42,str(tmp_path/'shifted'),connection_mode='physics',robot_base_x=-.05)
    assert zero['spawned_blocks']==shifted['spawned_blocks']
    assert zero['target_blocks']==shifted['target_blocks']
    assert shifted['robot_base_position_m']==[-.05,0.,0.]
    model=mujoco.MjModel.from_xml_path(str(scene))
    assert model.body('link0').pos==pytest.approx([-.05,0.,0.])


def test_shifted_base_rejects_unconfigured_external_frames():
    from mj_bridge.benchmark_cli import main
    with pytest.raises(SystemExit) as error:
        main(['run','--product','unused.yaml','--robot-base-x=-.05'])
    assert error.value.code==2
