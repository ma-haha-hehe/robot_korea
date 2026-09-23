import math
import pytest


def test_disassembly_plan_reverses_removals_and_keeps_grasp_frame():
    from mj_bridge.assembly_planner import plan_assembly, accessible
    from mj_bridge.benchmark_core import load_registry
    targets = [
        dict(id='left', type='brick_4x2', position=[-.016, 0, 0], yaw_rad=math.pi/2),
        dict(id='right', type='brick_4x2', position=[.016, 0, 0], yaw_rad=math.pi/2),
        dict(id='top', type='brick_2x2', position=[0, 0, .0192], yaw_rad=0.),
    ]
    plan = plan_assembly(targets)
    assert plan[-1]['block_id'] == 'top'
    remaining = list(targets)
    for step in reversed(plan):
        target = next(b for b in remaining if b['id'] == step['block_id'])
        assert accessible(target, remaining, step['grasp_spin_deg'], load_registry())
        assert target['yaw_rad'] + step['grasp_offset_rad'] == pytest.approx(
            math.radians(step['grasp_spin_deg']))
        remaining.remove(target)
    # Opposing neighbouring rectangles require the alternative world-axis grip.
    assert next(s for s in plan if s['block_id'] == 'left')['grasp_spin_deg'] == 0



def test_disassembly_plan_rejects_overlapping_goals():
    from mj_bridge.assembly_planner import plan_assembly, PlanningError
    targets = [dict(id=str(i), type='brick_4x2', position=[0, 0, 0], yaw_rad=0.)
               for i in range(2)]
    with pytest.raises(PlanningError, match='overlapping target'):
        plan_assembly(targets)



def test_disassembly_plan_refuses_when_clearance_check_fails(monkeypatch):
    from mj_bridge import assembly_planner as planner
    monkeypatch.setattr(planner, 'accessible', lambda *args: False)
    with pytest.raises(planner.PlanningError, match='no collision-free'):
        planner.plan_assembly([dict(id='part', type='brick_2x2', position=[0,0,0], yaw_rad=0.)])



def test_grasp_clearance_checks_off_axis_neighbour_footprint():
    from mj_bridge.assembly_planner import accessible
    from mj_bridge.benchmark_core import load_registry
    target = dict(id='target', type='brick_2x2', position=[0,0,0], yaw_rad=0.)
    neighbour = dict(id='diagonal', type='brick_2x2', position=[.023,.035,0], yaw_rad=0.)
    assert not accessible(target, [target, neighbour], 0, load_registry())
    assert accessible(target, [target, neighbour], 90, load_registry())



def test_disassembly_checks_covering_footprint_not_only_centres():
    from mj_bridge.assembly_planner import plan_assembly
    base = dict(id='base', type='brick_4x2', position=[0,0,0], yaw_rad=0.)
    top = dict(id='top', type='brick_2x2', position=[.024,0,.0192], yaw_rad=0.)
    assert [step['block_id'] for step in plan_assembly([base, top])] == ['base','top']



@pytest.mark.parametrize('source,target,spin', [(0,0,0),(30,90,0),(-170,90,90),(45,0,90)])
def test_grasp_angles_preserve_the_product_orientation(source, target, spin):
    from mj_bridge.assembly_planner import resolve_grasp_yaws
    pick, place = resolve_grasp_yaws(math.radians(source), math.radians(target), spin)
    resulting_part_yaw = math.radians(source) + place - pick
    error = (resulting_part_yaw - math.radians(target) + math.pi) % (2*math.pi) - math.pi
    assert error == pytest.approx(0.)
    assert place == pytest.approx(math.radians(spin))



@pytest.mark.parametrize('patch', [
    {'position': [float('nan'), 0, 0]}, {'position': [0, 0]},
    {'yaw_rad': float('inf')}, {'type': 'unknown'}, {'id': ''},
])
def test_planner_rejects_invalid_input(patch):
    from mj_bridge.assembly_planner import plan_assembly, PlanningError
    target = dict(id='brick', type='brick_2x2', position=[0,0,0], yaw_rad=0.)
    target.update(patch)
    with pytest.raises(PlanningError):
        plan_assembly([target])


def test_all_nominally_valid_designs_have_verified_grasp_plans():
    from pathlib import Path
    from mj_bridge.assembly_planner import plan_assembly, accessible
    from mj_bridge.benchmark_cli import normalized_from_path
    from mj_bridge.benchmark_core import load_registry
    from mj_bridge.product_geometry import audit_product
    root = Path(__file__).resolve().parents[3]
    count = 0
    for folder in ('catalog', 'supported_variants'):
        for path in (root/'examples/products'/folder).glob('*.yaml'):
            product = normalized_from_path(str(path))
            if not audit_product(product)['valid_for_snap']:
                continue
            remaining = [dict(id=b['id'],type=b['type'],position=b['target']['position'],
                              yaw_rad=math.radians(b['target']['yaw_deg'])) for b in product['blocks']]
            plan = plan_assembly(remaining)
            assert len(plan) == len(remaining)
            for step in reversed(plan):
                target = next(b for b in remaining if b['id'] == step['block_id'])
                assert step['grasp_spin_deg'] in (0,90)
                assert accessible(target,remaining,step['grasp_spin_deg'],load_registry())
                remaining.remove(target)
            count += 1
    assert count == 36
