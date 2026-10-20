#!/usr/bin/env python3
"""Gravity-settle nominal targets; this diagnostic is NOT robot execution."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from mj_bridge.benchmark_cli import normalized_from_path
from mj_bridge.benchmark_core import generate_episode, score_episode
from mj_bridge.scene_builder import create_assembly_base_plate, create_episode_brick_body


def check(product, seconds):
    episode=generate_episode(product,seed=42,connection_mode='physics')
    root=ET.Element('mujoco')
    ET.SubElement(root,'compiler',angle='radian',autolimits='true')
    opt=ET.SubElement(root,'option',timestep='.0005',integrator='implicitfast',gravity='0 0 -9.81')
    ET.SubElement(opt,'flag',sleep='enable')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'geom',type='box',size='.30 .50 .02',pos='.40 0 .02',friction='3 .1 .01',solref='.001 1',solimp='.999 .9999 .0001')
    world.append(create_assembly_base_plate())
    for target in episode['target_blocks']:
        world.append(create_episode_brick_body(dict(target,geometry='hollow_primitives_v1')))
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    data=mujoco.MjData(model)
    peak=0.
    for _ in range(math.ceil(seconds/model.opt.timestep)):
        mujoco.mj_step(model,data)
        if data.ncon:peak=max(peak,float(-np.min(data.contact.dist)))
    actual={}
    for target in episode['target_blocks']:
        b=model.body(target['body_name']).id
        r=data.xmat[b].reshape(3,3)
        actual[target['id']]=dict(position=data.xpos[b].tolist(),quaternion_wxyz=data.xquat[b].tolist(),yaw_rad=math.atan2(r[1,0],r[0,0]))
    score=score_episode(episode,{'blocks':actual})
    return dict(nominal_target_stable=score['success'] and peak<=.0005,
                max_contact_penetration_m=peak,score=score,actual=actual,
                simulation_seconds=seconds)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--products',required=True)
    parser.add_argument('--seconds',type=float,default=2.)
    parser.add_argument('--output',default='runs/workbenchmark-stability.json')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:parser.error('seconds must be finite and positive')
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    rows=[];cache={}
    paths=sorted(Path(args.products).glob('*.yaml'))
    if not paths:parser.error('no product YAMLs found')
    for path in paths:
        product=normalized_from_path(str(path))
        signature=hashlib.sha256(json.dumps([(b['id'],b['type'],b['target']) for b in product['blocks']],sort_keys=True).encode()).hexdigest()
        if signature not in cache:cache[signature]=check(product,args.seconds)
        result=cache[signature]
        # Include IDs in the cache key because measurements retain block labels.
        rows.append(dict(product=path.stem,geometry_signature=signature,measurement=result))
        report=dict(kind='nominal_target_gravity_diagnostic_not_robot_execution',expected_tasks=len(paths),
                    finished=len(rows)==len(paths),tasks=len(rows),unique_simulations=len(cache),
                    stable=sum(r['measurement']['nominal_target_stable'] for r in rows),results=rows)
        tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(report,indent=2)+'\n');tmp.replace(out)
        print(f"{len(rows)}/{len(paths)} {'STABLE' if result['nominal_target_stable'] else 'UNSTABLE'} {path.stem}",flush=True)


if __name__=='__main__':main()
