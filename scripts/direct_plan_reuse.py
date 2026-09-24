"""Conservative reuse check for changes confined to an unexecuted fallback."""
import ast
import copy
import hashlib
from pathlib import Path


def _direct_module(source):
    tree = ast.parse(source)
    functions = [node for node in tree.body
                 if isinstance(node, ast.FunctionDef) and node.name == 'plan_with_release_above']
    if len(functions) != 1:
        raise ValueError('baseline must define one explicit fallback planner')
    function = functions[0]
    body = function.body
    index = 1 if (isinstance(body[0], ast.Expr)
                  and isinstance(body[0].value, ast.Constant)
                  and isinstance(body[0].value.value, str)) else 0
    expected = ast.parse('try:\n return plan_assembly(targets)\nexcept GraspAccessError:\n pass').body[0]
    if ast.dump(body[index]) != ast.dump(expected):
        raise ValueError('direct return before fallback has changed')
    function.body = body[:index + 1]
    return ast.dump(tree)


def verify_direct_plan_reuse(baseline_file, current_file, recorded, manifest, plan):
    """Prove that this complete direct plan did not run the edited fallback."""
    baseline_file, current_file = Path(baseline_file), Path(current_file)
    baseline, current = baseline_file.read_bytes(), current_file.read_bytes()
    if hashlib.sha256(baseline).hexdigest() != recorded.get('assembly_planner.py'):
        raise ValueError('baseline planner does not match the executed source')
    if _direct_module(baseline.decode()) != _direct_module(current.decode()):
        raise ValueError('changes extend beyond the unexecuted planner fallback')
    if plan.get('method') != 'assembly_by_disassembly':
        raise ValueError('fallback execution cannot reuse a different planner revision')
    # This module is imported from the final candidate by the report command.
    from mj_bridge.assembly_planner import plan_assembly
    if plan.get('steps') != plan_assembly(copy.deepcopy(manifest['target_blocks'])):
        raise ValueError('recorded direct plan differs from the final direct planner')
    return {
        'scope': 'identical direct return and module; only unused fallback changed',
        'executed_planner_sha256': hashlib.sha256(baseline).hexdigest(),
        'final_planner_sha256': hashlib.sha256(current).hexdigest(),
    }


def _non_planning_module(source):
    """Keep geometry, yaw conversion, imports and every non-planning definition."""
    tree = ast.parse(source)
    planning = {'plan_assembly', 'plan_with_release_above', 'prefer_narrow_plan', 'prefer_braced_order'}
    tree.body = [node for node in tree.body
                 if not (isinstance(node, ast.FunctionDef) and node.name in planning)]
    return ast.dump(tree)


def verify_equivalent_plan_reuse(baseline_file, current_file, recorded, manifest, plan):
    """Verify identical issued steps when only the discrete planner changed.

    The caller must separately check every controller, contact, configuration
    and model hash. A different order, angle or placement mode needs a new run.
    """
    import importlib.util
    baseline_file, current_file = Path(baseline_file), Path(current_file)
    baseline, current = baseline_file.read_bytes(), current_file.read_bytes()
    old_hash = hashlib.sha256(baseline).hexdigest()
    if old_hash != recorded.get('assembly_planner.py'):
        raise ValueError('baseline planner does not match the executed source')
    if _non_planning_module(baseline.decode()) != _non_planning_module(current.decode()):
        raise ValueError('changes extend beyond discrete planning functions')
    from mj_bridge.assembly_planner import plan_with_release_above
    spec = importlib.util.spec_from_file_location('mj_bridge._archived_planner', baseline_file)
    previous = importlib.util.module_from_spec(spec)
    # These are the archived, reviewed repository sources, never remote input.
    spec.loader.exec_module(previous)
    steps = plan.get('steps')
    for planner in (previous.plan_with_release_above, plan_with_release_above):
        if planner(copy.deepcopy(manifest['target_blocks'])) != steps:
            raise ValueError('recorded steps differ from a verified planner output')
    mode = ('geometric_reverse_order_with_release_above_fallback'
            if any(step.get('placement_mode') == 'release_above_press' for step in steps)
            else 'assembly_by_disassembly')
    if plan.get('method') != mode:
        raise ValueError('recorded planning method contradicts its steps')
    return {'scope': 'identical recorded and recomputed steps; only discrete planner changed',
            'executed_planner_sha256': old_hash,
            'final_planner_sha256': hashlib.sha256(current).hexdigest()}
