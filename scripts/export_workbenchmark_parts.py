#!/usr/bin/env python3
"""Export standalone primitive MJCF parts used by a Workbenchmark inventory."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from mj_bridge.benchmark_core import normalize_product, load_yaml
from mj_bridge import scene_builder, plastic_contact


def export_parts(repository, output, contact_profile="loose"):
    if contact_profile not in {"loose", "plastic"}:
        raise ValueError("unknown contact profile")
    geometry = "hollow_plastic_clutch_v2" if contact_profile == "plastic" else "hollow_primitives_v1"
    repo, out = Path(repository).resolve(), Path(output).resolve()
    paths = sorted((repo / 'benchmark_tasks').glob('*.yaml'))
    if not paths:
        raise ValueError('no benchmark task YAMLs')
    counts = Counter(b['type'] for p in paths for b in normalize_product(load_yaml(p))['blocks'])
    artifacts = {}
    entries = []
    for part, count in sorted(counts.items()):
        root = ET.Element('mujoco', model=part)
        ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
        ET.SubElement(root, 'option', timestep='.0005', integrator='implicitfast',
                      cone='pyramidal', solver='Newton', iterations='100')
        world = ET.SubElement(root, 'worldbody')
        floor = ET.SubElement(world, 'geom', name='floor', type='plane', size='.2 .2 .01',
                             rgba='.44 .48 .53 1', friction='1 .005 .0001')
        if contact_profile == 'plastic':
            floor.attrib.update(priority='3', solref='.0011 1', solimp='.9999 .9999 .0001')
        meshes = {}
        world.append(scene_builder.create_episode_brick_body({
            'type': part, 'body_name': part, 'geometry': geometry,
            'position': [0., 0., .0186], 'color': 'blue'}, meshes))
        if meshes:
            ET.SubElement(root, 'asset').extend(meshes.values())
        ET.indent(root)
        text = ET.tostring(root, encoding='unicode') + '\n'
        model = mujoco.MjModel.from_xml_string(text)
        data = mujoco.MjData(model)
        peak = 0.
        for _ in range(2000):
            mujoco.mj_step(model, data)
            if data.ncon:
                peak = max(peak, float(-np.min(data.contact.dist)))
        body_id = model.body(part).id
        if (not np.isfinite(data.qpos).all() or peak > .0005
                or abs(data.xpos[body_id, 2] - .0186) > .001
                or data.xmat[body_id, 8] < np.cos(np.deg2rad(1))):
            raise ValueError(f'part resting-contact validation failed: {part}')
        filename = part + '.xml'
        artifacts[filename] = text
        entries.append({'type': part, 'instances': count, 'mjcf': filename,
                        'sha256': hashlib.sha256(text.encode()).hexdigest(),
                        'mass_kg': float(model.body_mass[body_id]),
                        'outer_body_dimensions_m': [float(2 * half - .0002) for half in
                                                    scene_builder.BRICK_SPECS[part]['body_half_size'][:2]] + [.0192],
                        'nominal_axis_clearance_m': .0002,
                        'resting_test_seconds': 1., 'max_contact_penetration_m': peak})
    manifest = {'geometry': geometry, 'task_count': len(paths),
                'source_repository': 'https://github.com/ma-haha-hehe/Workbenchmark',
                'source_commit': subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
                'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'scene_builder_sha256': hashlib.sha256(Path(scene_builder.__file__).read_bytes()).hexdigest(),
                'contact_profile': contact_profile,
                'solver': {'cone': 'pyramidal', 'algorithm': 'Newton',
                           'timestep_s': .0005, 'iterations': 100},
                'contact_parameters': vars(plastic_contact.PlasticContact()) if contact_profile == 'plastic' else None,
                'plastic_contact_sha256': hashlib.sha256(Path(plastic_contact.__file__).read_bytes()).hexdigest(),
                'scope': 'self-authored contact approximation; no measured material calibration; not assembly validation',
                'parts': entries}
    artifacts['manifest.json'] = json.dumps(manifest, indent=2) + '\n'
    artifacts['LICENSE'] = (Path(__file__).resolve().parents[1] / 'LICENSE').read_text()
    artifacts['NOTICE'] = ('Primitive brick models generated from robot_korea.\n'
                           'Source: https://github.com/ma-haha-hehe/robot_korea\n'
                           'See manifest.json for generator and geometry source hashes.\n'
                           'No third-party robot meshes or LEGO CAD assets are included.\n')
    artifacts['README.md'] = '''# Primitive parts

These standalone MuJoCo XML scenes cover the two part types in the benchmark:
`brick_2x2` and `brick_4x2`. Each scene includes one free body and a floor.
No external mesh, texture, or robot asset is required. Units are metres and kilograms.
Visual and collision geometry use the same primitives.

Open a scene with `python -m mujoco.viewer --mjcf=parts/brick_2x2.xml` from the repository root.
Copy the part body into another MJCF world to reuse it; give each instance a unique body name.
The body origin is 18.6 mm above its underside. Body height is 19.2 mm, excluding studs.
The body outline leaves 0.2 mm clearance per nominal axis between adjacent bricks;
the 16 mm stud pitch and task coordinates are unchanged. This clearance is not
a manufacturer measurement.

These are simplified, self-authored hollow rigid-body models, not manufacturer CAD.
Wall, tube, stud, mass and friction parameters are approximations. Plastic deformation
and clutch force have not been calibrated. Unsupported cantilevers may fall;
resting-contact validation does not demonstrate successful robotic assembly.
The experimental internal ribs are not included.

`manifest.json` records source provenance, instance counts, file hashes and the
one-second resting-contact check. Regenerate with robot_korea's
`scripts/export_workbenchmark_parts.py --repository ../Workbenchmark --output ../Workbenchmark/parts`.
Original task layouts are unchanged.

The generated models retain robot_korea's Apache-2.0 license; see LICENSE and NOTICE.
This notice applies to this parts directory and does not assign a license to the
original benchmark task YAML files.
'''
    if contact_profile == 'plastic':
        artifacts['README.md'] = artifacts['README.md'].replace(
            'The experimental internal ribs are not included.',
            'This variant includes compliant internal ribs and lead-ins. Its shell uses '
            'equivalent convex hulls to avoid near-parallel analytic box contact artifacts. The fit uses '            'contact forces and friction, without attachment constraints. Copy the asset '            'meshes as well as the body when reusing this model. Parameters are uncalibrated; '            'see manifest.json. Generate this variant with --contact-profile plastic.')
    artifacts['README.md'] = artifacts['README.md'].replace(
        '--output ../Workbenchmark/parts`',
        f'--output ../Workbenchmark/parts --contact-profile {contact_profile}`')
    out.mkdir(parents=True, exist_ok=True)
    for name, text in artifacts.items():
        target = out / name
        if target.exists() and target.read_text() != text:
            raise ValueError(f'refusing to overwrite different artifact: {target}')
    for name, text in artifacts.items():
        (out / name).write_text(text)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--contact-profile', choices=['loose', 'plastic'], default='loose')
    args = parser.parse_args()
    result = export_parts(args.repository, args.output, args.contact_profile)
    print(f"Exported {len(result['parts'])} part types for {result['task_count']} tasks")
