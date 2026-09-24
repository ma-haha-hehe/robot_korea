"""Support ordering must retain valid reverse grasps and immutable goals."""
import copy
import json
from pathlib import Path

from mj_bridge.assembly_planner import accessible, load_registry, plan_assembly


def test_bridge_precedes_cantilever_even_when_wider_grasp_is_needed():
    targets = json.loads((Path(__file__).parent / 'fixtures/tier4_task_001_targets.json').read_text())
    original = copy.deepcopy(targets)
    plan = plan_assembly(targets)
    ids = [row['block_id'] for row in plan]
    assert ids.index('2x2_brick_10') < ids.index('4x2_brick_11')
    assert next(row for row in plan if row['block_id'] == '4x2_brick_11')['grasp_spin_deg'] == 90
    by_id = {block['id']: block for block in targets}
    prefix = []
    for row in plan:
        block = by_id[row['block_id']]
        prefix.append(block)
        assert accessible(block, prefix, row['grasp_spin_deg'], load_registry())
    assert len(ids) == len(set(ids)) == len(targets)
    assert targets == original


def test_symmetric_support_keeps_existing_tie_order():
    targets = [dict(id='base', type='brick_4x2', position=[0,0,0], yaw_rad=0),
               dict(id='left', type='brick_2x2', position=[-.016,0,.0192], yaw_rad=0),
               dict(id='right', type='brick_2x2', position=[.016,0,.0192], yaw_rad=0)]
    from mj_bridge.assembly_planner import prefer_braced_order
    rows = [dict(block_id='base', grasp_spin_deg=0, grasp_offset_rad=0),
            dict(block_id='right', grasp_spin_deg=0, grasp_offset_rad=0),
            dict(block_id='left', grasp_spin_deg=0, grasp_offset_rad=0)]
    assert prefer_braced_order(targets, rows, load_registry()) == rows


def test_unrelated_support_branches_keep_existing_order():
    from mj_bridge.assembly_planner import prefer_braced_order
    targets = [dict(id='base_a', type='brick_4x2', position=[0,0,0], yaw_rad=0),
               dict(id='base_b', type='brick_4x2', position=[.2,0,0], yaw_rad=0),
               dict(id='overhang', type='brick_2x2', position=[.016,.016,.0192], yaw_rad=0),
               dict(id='supported', type='brick_2x2', position=[.2,0,.0192], yaw_rad=0)]
    rows = [dict(block_id=block['id'], grasp_spin_deg=0, grasp_offset_rad=0)
            for block in targets]
    assert prefer_braced_order(targets, rows, load_registry()) == rows


def test_reuse_requires_identical_old_and_new_bracing_steps(tmp_path):
    import hashlib
    import importlib.util
    import runpy
    import pytest
    from mj_bridge import assembly_planner
    current = Path(assembly_planner.__file__)
    baseline = tmp_path / 'old_planner.py'
    source = current.read_text().split('\ndef prefer_braced_order(')[0]
    source = source.replace(
        'return prefer_braced_order(targets, prefer_narrow_plan(targets, original, registry), registry)',
        'return prefer_narrow_plan(targets, original, registry)')
    baseline.write_text(source)
    spec = importlib.util.spec_from_file_location('mj_bridge.test_old_planner', baseline)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    verify = runpy.run_path(str(Path(__file__).resolve().parents[3] / 'scripts/direct_plan_reuse.py'))['verify_equivalent_plan_reuse']
    recorded = {'assembly_planner.py': hashlib.sha256(baseline.read_bytes()).hexdigest()}
    single = [dict(id='single', type='brick_2x2', position=[0,0,0], yaw_rad=0)]
    unchanged = {'method': 'assembly_by_disassembly', 'steps': old.plan_assembly(single)}
    assert verify(baseline, current, recorded, {'target_blocks': single}, unchanged)
    dense = json.loads((Path(__file__).parent / 'fixtures/tier4_task_001_targets.json').read_text())
    changed = {'method': 'assembly_by_disassembly', 'steps': old.plan_assembly(dense)}
    with pytest.raises(ValueError, match='steps differ'):
        verify(baseline, current, recorded, {'target_blocks': dense}, changed)
