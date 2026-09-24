#!/usr/bin/env python3
"""Inventory and audit an external Workbenchmark checkout without modifying it."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.benchmark_core import load_yaml
from mj_bridge.product_geometry import audit_product
from mj_bridge.assembly_planner import plan_assembly, PlanningError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--output', default='runs/workbenchmark-audit.json')
    args = parser.parse_args()
    root = Path(args.repository).resolve()
    rows, parts, signatures = [], Counter(), Counter()
    for path in sorted((root/'benchmark_tasks').glob('*.yaml')):
        product = normalized_from_path(str(path))
        audit = audit_product(product)
        targets = [dict(id=b['id'],type=b['type'],position=b['target']['position'],
                        yaw_rad=math.radians(b['target']['yaw_deg'])) for b in product['blocks']]
        shape = sorted((b['type'], *b['target']['position'], b['target']['yaw_deg'])
                       for b in product['blocks'])
        signature = hashlib.sha256(json.dumps(shape).encode()).hexdigest()
        signatures[signature] += 1
        parts.update(b['type'] for b in product['blocks'])
        raw = load_yaml(path)
        row = dict(path=str(path.relative_to(root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                   shape_sha256=signature,blocks=len(targets),geometry=audit,
                   initial_blocks=len(raw.get('initial_blocks',[])))
        try: row.update(angle_plan=plan_assembly(targets),angle_plan_valid=True)
        except PlanningError as exc: row.update(angle_plan_valid=False,planning_error=str(exc))
        rows.append(row)
    if not rows:
        parser.error('no benchmark_tasks/*.yaml found')
    sha=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    report=dict(source_commit=sha,tasks=len(rows),unique_shapes=len(signatures),parts=dict(parts),
                nominal_geometry_valid=sum(r['geometry']['valid_for_snap'] for r in rows),
                angle_plan_valid=sum(r['angle_plan_valid'] for r in rows),
                scope='static inventory and geometry only; no physical execution',results=rows)
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))


if __name__=='__main__':main()
