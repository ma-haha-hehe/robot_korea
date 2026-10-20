"""Check an ideally assembled model under gravity; NEVER robot acceptance."""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml

from mj_bridge.benchmark_core import score_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--catalog-mass', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 30:
        parser.error('seconds must be finite and in (0, 30]')
    args.output.mkdir(parents=True, exist_ok=False)
    marker = {'scope': 'IDEAL-POSE GRAVITY DIAGNOSTIC; object poses initialized directly; not robot assembly'}
    (args.output / 'DIAGNOSTIC_ONLY.json').write_text(json.dumps(marker, indent=2))
    episode = yaml.safe_load((args.scene_dir / 'episode_manifest.yaml').read_text())
    model = mujoco.MjModel.from_xml_path(str(args.scene_dir / 'scene.xml'))
    data = mujoco.MjData(model)
    bodies = {}
    for target in episode['target_blocks']:
        body = model.body(target['body_name']).id
        bodies[target['id']] = body
        if args.catalog_mass:
            mass = {'brick_2x2': .00675, 'brick_4x2': .0122}[target['type']]
            model.body_inertia[body] *= mass / model.body_mass[body]
            model.body_mass[body] = mass
        model.tree_sleep_policy[model.body_treeid[body]] = mujoco.mjtSleepPolicy.mjSLEEP_NEVER
    mujoco.mj_setConst(model, data)
    for target in episode['target_blocks']:
        body = bodies[target['id']]
        address = model.jnt_qposadr[model.body_jntadr[body]]
        data.qpos[address:address+3] = target['position']
        yaw = target['yaw_rad']
        data.qpos[address+3:address+7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
    mujoco.mj_forward(model, data)
    peak = 0.
    samples = []
    for index in range(math.ceil(args.seconds/model.opt.timestep)):
        mujoco.mj_step(model, data)
        for contact in data.contact:
            if any(int(model.geom_bodyid[g]) in bodies.values() for g in (contact.geom1, contact.geom2)):
                peak = max(peak, -float(contact.dist))
        if index % 200 == 0:
            samples.append({'simulation_s': float(data.time), 'positions': {
                name: data.xpos[body].tolist() for name, body in bodies.items()}})
    actual = {name: {'position': data.xpos[body].tolist(),
                     'quaternion_wxyz': data.xquat[body].tolist(),
                     'yaw_rad': math.atan2(data.xmat[body, 3], data.xmat[body, 0])}
              for name, body in bodies.items()}
    report = dict(marker, catalog_mass=args.catalog_mass, simulation_s=float(data.time),
                  max_part_contact_penetration_m=peak,
                  score=score_episode(episode, {'blocks': actual}), actual=actual)
    (args.output / 'diagnostic.json').write_text(json.dumps(report, indent=2)+'\n')
    (args.output / 'motion_trace.json').write_text(json.dumps(samples, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
