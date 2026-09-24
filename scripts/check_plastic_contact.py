#!/usr/bin/env python3
"""Measure the simulated clutch in an aligned axial press/pull fixture."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from mj_bridge import plastic_contact
from mj_bridge.scene_builder import create_episode_brick_body


def measure(part_type, parameters=None):
    parameters = parameters or plastic_contact.PlasticContact()
    root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', angle='radian')
    ET.SubElement(root, 'option', timestep='.0005', integrator='implicitfast',
                  gravity='0 0 0', iterations='100', cone='pyramidal')
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    for name, z in (('base', 0.), ('test', .0272)):
        body = create_episode_brick_body(dict(type=part_type, body_name=name,
            position=[0, 0, z], geometry='hollow_primitives_v1'))
        meshes = plastic_contact.add_clutch_fit(body, part_type, parameters)
        body.remove(body.find('freejoint'))
        if name == 'test':
            ET.SubElement(body, 'joint', type='slide', axis='0 0 1', damping='.2')
        world.append(body)
    asset.extend(meshes)
    xml = ET.tostring(root, encoding='unicode')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    maximum_depth = 0.

    def advance(force, seconds):
        nonlocal maximum_depth
        for _ in range(round(seconds / model.opt.timestep)):
            data.qfrc_applied[0] = force
            mujoco.mj_step(model, data)
            if data.ncon:
                maximum_depth = max(maximum_depth, float(-np.min(data.contact.dist)))
            if maximum_depth > .0005 or not np.isfinite(data.qpos).all():
                raise RuntimeError('invalid contact depth or simulation state')
        return float(.0272 + data.qpos[0] - .0192)

    # The moving brick is force driven. Its lateral/rotational constraints
    # represent the test jig, not the robot assembly scene.
    for force in np.linspace(0, -3., 41):
        advance(float(force), .02)
    seating_error = advance(-3., .5)
    released_error = advance(0., .5)
    pullout = None
    for force in np.linspace(0, 12., 121):
        if advance(float(force), .05) > .006:
            pullout = float(force)
            break
    return dict(part=part_type, press_force_N=3.,
        solver={'cone': mujoco.mjtCone(model.opt.cone).name,
                'algorithm': mujoco.mjtSolver(model.opt.solver).name,
                'timestep_s': float(model.opt.timestep),
                'iterations': int(model.opt.iterations)},
        seating_error_m=seating_error, released_error_m=released_error,
        pullout_force_N=pullout, pullout_ramp_N_per_s=2., pullout_limit_N=12.,
        max_contact_depth_m=maximum_depth,
        generated_model_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        passed=abs(seating_error) < .0001 and abs(released_error) < .0001
               and pullout is not None and pullout > .2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--time-constant', type=float, default=plastic_contact.PlasticContact().contact_time_constant_s)
    args = parser.parse_args()
    parameters = plastic_contact.PlasticContact(contact_time_constant_s=args.time_constant)
    parameters.attributes()
    if args.output.exists():
        parser.error('output already exists; use a new evidence file')
    rows = [measure(kind, parameters) for kind in ('brick_2x2', 'brick_4x2')]
    source = Path(plastic_contact.__file__)
    report = dict(scope='aligned axial fixture; not robot or measured-material validation',
        parameters=vars(parameters),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mujoco_version=mujoco.__version__, results=rows,
        passed=all(row['passed'] for row in rows))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
