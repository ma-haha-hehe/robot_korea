"""Reuse only successful executions that never entered changed pose feedback."""
import ast
import hashlib
from pathlib import Path


def _executor_tree(source):
    tree = ast.parse(source)
    classes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OracleExecutor']
    if len(classes) != 1:
        raise ValueError('expected one OracleExecutor')
    methods = [n for n in classes[0].body if isinstance(n, ast.FunctionDef)
               and n.name == 'descend_with_pose_feedback']
    if len(methods) != 1:
        raise ValueError('expected one pose feedback method')
    method = methods[0]
    # A successful call must record the marker before its sole return. The
    # remainder of the executor must stay identical, including marker storage.
    returns = [n for n in ast.walk(method) if isinstance(n, ast.Return)]
    marked = 0
    marker = ast.parse("self.last_seating_confirmation['pose_feedback']", mode='eval').body
    marker.ctx = ast.Store()
    expected = ast.dump(marker)
    for node in ast.walk(method):
        body = getattr(node, 'body', [])
        if not isinstance(body, list):
            continue
        for i, statement in enumerate(body):
            if isinstance(statement, ast.Return) and i > 0:
                previous = body[i-1]
                if (isinstance(previous, ast.Assign) and len(previous.targets) == 1
                        and ast.dump(previous.targets[0]) == expected):
                    marked += 1
    if len(returns) != 1 or marked != 1:
        raise ValueError('successful feedback does not have a proven completion marker')
    method.body = [ast.Pass()]
    return ast.dump(tree)


def verify_unused_feedback_reuse(baseline_file, current_file, recorded, result, manifest):
    baseline, current = Path(baseline_file).read_bytes(), Path(current_file).read_bytes()
    old_hash = hashlib.sha256(baseline).hexdigest()
    if old_hash != recorded.get('reference_executor.py'):
        raise ValueError('baseline executor does not match recorded source')
    if _executor_tree(baseline.decode()) != _executor_tree(current.decode()):
        raise ValueError('executor changes extend beyond pose feedback')
    events = result.get('events', [])
    if (result.get('success') is not True or result.get('execution_error')
            or not events or len(events) != len(manifest['target_blocks'])
            or {e.get('block') for e in events} != {b['id'] for b in manifest['target_blocks']}):
        raise ValueError('reuse requires a complete successful execution')
    for event in events:
        seating = event.get('seating_confirmation')
        if not isinstance(seating, dict):
            raise ValueError('seating evidence is missing')
        if ('pose_feedback' in seating
                or seating.get('placement_mode') == 'release_before_seating'):
            raise ValueError('changed pose feedback was executed; rerun required')
    return {'scope': 'changed pose feedback never completed in this successful execution; all other executor code identical',
            'executed_executor_sha256': old_hash,
            'final_executor_sha256': hashlib.sha256(current).hexdigest()}
