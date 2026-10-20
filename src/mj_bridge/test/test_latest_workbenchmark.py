import copy
import hashlib
import json
import math
from pathlib import Path
import runpy

import pytest
import yaml

from mj_bridge.benchmark_core import generate_episode, normalize_product


IMPORT = runpy.run_path(str(Path(__file__).resolve().parents[3] /
                           'scripts/import_workbenchmark_latest.py'))['import_latest']
VERIFY = runpy.run_path(str(Path(__file__).resolve().parents[3] /
                           'scripts/import_workbenchmark_latest.py'))['verify_imported_task']


def task_fixture(tmp_path):
    root = tmp_path / 'tasks'
    task = root / 'balanced_order_sensitive' / 'balanced_order_sensitive_001'
    task.mkdir(parents=True)
    rows = [dict(name='base', type='brick_2x2', color='red', pos=[.1, .2, 0.], rotation=[0., 0., 0.]),
            dict(name='bridge', type='brick_4x2', color='blue', pos=[.1, .2, .02], rotation=[0., 0., math.pi/2]),
            dict(name='top', type='brick_2x2', color='green', pos=[.1, .216, .04], rotation=[0., 0., 0.])]
    initial = copy.deepcopy(rows)
    for block, x in zip(initial, [-.12, 0., .12]):
        block['pos'] = [x, -.16, 0.]
    initial[0]['rotation'][2] = math.pi
    documents = {'final_product.yaml': {'blocks': rows},
                 'scene_init.yaml': {'geometry': {'length_unit': 'm', 'rotation_unit': 'rad',
                     'pose_origin': 'xy_center_body_bottom', 'stud_pitch_m': .016,
                     'engaged_layer_height_m': .02},
                     'parts_plate': {'center': [0., -.16, 0.]}, 'blocks': initial},
                 'metadata.yaml': {'instance_id': task.name, 'task_class': task.parent.name,
                     'generator_version': 2, 'num_blocks': 3}}
    for name, data in documents.items():
        (task/name).write_text(yaml.safe_dump(data))
    return root, task, documents


def test_latest_import_has_explicit_geometry_and_frozen_originals(tmp_path):
    root, source, _ = task_fixture(tmp_path)
    out = tmp_path/'imported'
    manifest = IMPORT(root, out)
    row = manifest['tasks'][0]
    product = normalize_product(yaml.safe_load((out/row['product']).read_text()))
    assert [b['target']['position'] for b in product['blocks']] == [
        [0., 0., 0.], [0., 0., .0192], [0., .016, .0384]]
    assert product['blocks'][1]['target']['yaw_deg'] == 90
    initial = product['initial_layout']['blocks']
    assert initial[0]['position_xy_m'] == [.28, -.2]
    assert initial[0]['yaw_deg'] == 180
    assert initial[2]['position_xy_m'][0]-initial[0]['position_xy_m'][0] == pytest.approx(.24)
    a = generate_episode(product, seed=0, connection_mode='physics', contact_profile='plastic')
    b = generate_episode(product, seed=42, connection_mode='physics', contact_profile='plastic')
    assert a['spawned_blocks'] == b['spawned_blocks']
    for name, expected in row['source_sha256'].items():
        original = (source/name).read_bytes()
        assert (out/'source'/row['source_directory']/name).read_bytes() == original
        assert hashlib.sha256(original).hexdigest() == expected
    with pytest.raises(ValueError, match='output already exists'):
        IMPORT(root, out)


@pytest.mark.parametrize('problem', ['units', 'layer', 'inventory', 'rotation', 'duplicate', 'metadata'])
def test_invalid_latest_contract_rejected_before_output(tmp_path, problem):
    root, task, docs = task_fixture(tmp_path)
    if problem == 'units': docs['scene_init.yaml']['geometry']['rotation_unit'] = 'deg'
    if problem == 'layer': docs['final_product.yaml']['blocks'][1]['pos'][2] = .0192
    if problem == 'inventory': docs['scene_init.yaml']['blocks'][0]['color'] = 'yellow'
    if problem == 'rotation': docs['scene_init.yaml']['blocks'][0]['rotation'][2] = 1.
    if problem == 'duplicate': docs['final_product.yaml']['blocks'][1]['name'] = 'base'
    if problem == 'metadata': docs['metadata.yaml']['num_blocks'] = 4
    for name, data in docs.items(): (task/name).write_text(yaml.safe_dump(data))
    output = tmp_path/'imported'
    with pytest.raises(ValueError): IMPORT(root, output)
    assert not output.exists()


def test_duplicate_yaml_keys_not_silently_accepted(tmp_path):
    root, task, _ = task_fixture(tmp_path)
    p = task/'metadata.yaml'
    p.write_text(p.read_text()+'num_blocks: 3\n')
    with pytest.raises(ValueError, match='duplicate YAML key'): IMPORT(root, tmp_path/'out')


@pytest.mark.parametrize('changed', ['live', 'frozen', 'normalized'])
def test_run_preflight_rejects_changed_data(tmp_path, changed):
    root, source, _ = task_fixture(tmp_path)
    out = tmp_path/'imported'
    manifest = IMPORT(root, out)
    row = manifest['tasks'][0]
    assert VERIFY(out, row['product'])['verified']
    paths = {'live': source/'scene_init.yaml',
             'frozen': out/'source'/row['source_directory']/'scene_init.yaml',
             'normalized': out/row['product']}
    paths[changed].write_text(paths[changed].read_text()+'\n# changed\n')
    with pytest.raises(ValueError, match='changed'):
        VERIFY(out, row['product'])


def test_portable_frozen_import_is_identified_explicitly(tmp_path):
    root, _, _ = task_fixture(tmp_path)
    out = tmp_path/'imported'
    manifest = IMPORT(root, out)
    check = VERIFY(out, manifest['tasks'][0]['product'], tmp_path/'unavailable')
    assert check['source'] == 'frozen source snapshot'


def test_partial_source_manifest_cannot_skip_original_validation(tmp_path):
    root, _, _ = task_fixture(tmp_path)
    out = tmp_path/'imported'
    manifest = IMPORT(root, out)
    row = manifest['tasks'][0]
    del row['source_sha256']['scene_init.yaml']
    (out/'import_manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='all three source files'):
        VERIFY(out, row['product'])
