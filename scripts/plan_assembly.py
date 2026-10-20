#!/usr/bin/env python3
"""Export the same assembly-by-disassembly plan used by the simulator."""
import argparse
import math
from pathlib import Path
from mj_bridge.assembly_planner import plan_assembly
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.benchmark_core import dump_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('product')
    parser.add_argument('--output', default='runs/assembly_plan.json')
    args = parser.parse_args()
    product = normalized_from_path(args.product)
    targets = [dict(id=b['id'], type=b['type'], position=b['target']['position'],
                    yaw_rad=math.radians(b['target']['yaw_deg'])) for b in product['blocks']]
    plan = plan_assembly(targets)
    dump_json(Path(args.output), dict(method='assembly_by_disassembly',
                                    product=product['product']['name'], steps=plan))
    print(args.output)


if __name__ == '__main__':
    main()
