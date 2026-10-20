"""Measure contact-monitor overhead on restored state; not assembly evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from probe_materials import OfflineNode, restore, ex


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=.2)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 2:
        parser.error('seconds must be finite and in (0, 2]')
    args.output.mkdir(parents=True, exist_ok=False)
    marker = {'scope': 'Restored-state contact-monitor diagnostic; not assembly acceptance'}
    (args.output/'DIAGNOSTIC_ONLY.json').write_text(json.dumps(marker))
    node = OfflineNode(args.scene_dir)
    executor = ex.OracleExecutor(node)
    restore(executor, args.snapshot/'state.npz', args.snapshot/'state.events.json', use_exact=False)
    target = json.loads((args.snapshot/'target.json').read_text())
    part = node.model.body(target['body_name']).id
    supports = {node.model.body(b['body_name']).id for b in node.episode_manifest['target_blocks']
                if b['id'] != target['id']}
    executor._insertion_contact_guard = (part, supports)
    cpu, wall = time.process_time(), time.perf_counter()
    error = None
    try:
        executor.step(args.seconds)
    except ex.ExecutionFailure as caught:
        error = str(caught)
    cpu, wall = time.process_time()-cpu, time.perf_counter()-wall
    fields = {name: getattr(node.data, name).copy() for name in
              ('qpos', 'qvel', 'ctrl', 'xpos', 'xquat', 'qacc_warmstart')}
    fields.update(time=np.array(node.data.time), max_part_penetration_m=np.array(node.max_part_penetration_m))
    np.savez_compressed(args.output/'final_state.npz', **fields)
    report = dict(marker, error=error, requested_simulation_s=args.seconds, cpu_s=cpu, wall_s=wall,
                  executor_sha256=hashlib.sha256(Path(ex.__file__).read_bytes()).hexdigest())
    (args.output/'diagnostic.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
