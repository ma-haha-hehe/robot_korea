#!/usr/bin/env python3
"""Command-line interface for the public LEGO manipulation benchmark."""
from __future__ import annotations

import argparse
import os
import shutil
import json
import importlib.util
from pathlib import Path

from .benchmark_core import (
    dump_json, dump_yaml, generate_episode, load_yaml, normalize_product,
    score_episode, validate_product,
)
from .scene_builder import BASE_DIR, build


def normalized_from_path(path: str) -> dict:
    source = Path(path)
    return normalize_product(load_yaml(source), name=source.stem)


def oracle_preflight():
    required = ('mujoco', 'rclpy', 'control_msgs', 'sensor_msgs',
                'std_msgs', 'std_srvs', 'trajectory_msgs')
    errors = [f'missing Python module: {name}' for name in required
              if importlib.util.find_spec(name) is None]
    return {'backend': 'oracle', 'ready': not errors, 'errors': errors}


def generate(product_path: str, seed: int, output_dir: str) -> tuple[dict, Path]:
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    product = normalized_from_path(product_path)
    episode = generate_episode(product, seed=seed)
    product_out = output / "product.normalized.yaml"
    manifest = output / "episode_manifest.yaml"
    scene = output / "scene.xml"
    shutil.copy2(Path(BASE_DIR) / "panda.xml", output / "panda.xml")
    shutil.copy2(Path(BASE_DIR) / "hand.xml", output / "hand.xml")
    shutil.copytree(
        Path(BASE_DIR) / "assets", output / "assets", dirs_exist_ok=True,
        ignore=shutil.ignore_patterns("LEGO_Duplo_brick_*.stl"),
    )
    dump_yaml(product_out, product)
    dump_yaml(manifest, episode)
    build(template_xml=str(Path(BASE_DIR) / "scene_template.xml"), output_xml=str(scene),
          episode_manifest=str(manifest))
    return episode, scene


def main(argv=None):
    parser = argparse.ArgumentParser(prog="lego-bench")
    commands = parser.add_subparsers(dest="command", required=True)

    validate_cmd = commands.add_parser("validate", help="validate and normalize a product YAML")
    validate_cmd.add_argument("product")
    audit_cmd = commands.add_parser("audit", help="check nominal support, solid overlap and snap layer heights")
    audit_cmd.add_argument("product")

    convert_cmd = commands.add_parser("convert", help="convert a legacy YAML to schema v1")
    convert_cmd.add_argument("product")
    convert_cmd.add_argument("output")

    generate_cmd = commands.add_parser("generate", help="generate a deterministic loose-parts scene")
    generate_cmd.add_argument("--product", required=True)
    generate_cmd.add_argument("--seed", type=int, default=0)
    generate_cmd.add_argument("--output-dir", default="runs/latest")

    run_cmd = commands.add_parser("run", help="generate and run one benchmark episode")
    run_cmd.add_argument("--product", required=True)
    run_cmd.add_argument("--seed", type=int, default=0)
    run_cmd.add_argument("--output-dir", default="runs/latest")
    run_cmd.add_argument("--headless", action="store_true")
    run_cmd.add_argument("--observation", choices=["oracle", "rgbd"], default="oracle")
    run_cmd.add_argument("--connection-mode", choices=["snap", "physics"], default="snap")
    run_cmd.add_argument("--executor", choices=["external", "baseline", "oracle-baseline"], default="external",
                         help="external starts the ROS environment; oracle-baseline runs a bounded torque controller")
    run_cmd.add_argument("--backend", choices=["oracle", "groundingdino-sam-foundationpose"], default="oracle")
    run_cmd.add_argument("--allow-invalid-product", action="store_true",
                         help="diagnostic only: execute a target rejected by nominal geometry checks")

    doctor_cmd = commands.add_parser("doctor", help="check optional perception dependencies")
    doctor_cmd.add_argument("--backend", choices=["oracle", "groundingdino-sam-foundationpose"], default="oracle")
    perceive_cmd = commands.add_parser("perceive", help="infer poses from a calibrated RGB-D NPZ without Oracle access")
    perceive_cmd.add_argument("--frame", required=True, help="NPZ: rgb, depth (metres), K, camera_to_world")
    perceive_cmd.add_argument("--product", required=True)
    perceive_cmd.add_argument("--output", required=True)

    score_cmd = commands.add_parser("score", help="score an exported actual_state YAML/JSON")
    score_cmd.add_argument("--manifest", required=True)
    score_cmd.add_argument("--actual", required=True)
    score_cmd.add_argument("--output")

    batch_cmd = commands.add_parser("batch-generate", help="generate a reproducible suite of episodes")
    batch_cmd.add_argument("--product", required=True)
    batch_cmd.add_argument("--first-seed", type=int, default=0)
    batch_cmd.add_argument("--count", type=int, required=True)
    batch_cmd.add_argument("--output-dir", default="runs/batch")

    summary_cmd = commands.add_parser("summarize", help="summarize result.json files under a run directory")
    summary_cmd.add_argument("run_dir")

    args = parser.parse_args(argv)
    if args.command == "doctor":
        from .perception import vision_preflight
        status = (oracle_preflight()
                  if args.backend == "oracle" else vision_preflight())
        print(json.dumps(status, indent=2))
        return 0 if status['ready'] else 2
    elif args.command == "perceive":
        import numpy as np
        from .perception import GroundedPoseBackend
        backend = GroundedPoseBackend()
        with np.load(args.frame, allow_pickle=False) as frame:
            result = backend.observe(frame['rgb'], frame['depth'], frame['K'],
                                     frame['camera_to_world'], normalized_from_path(args.product))
        dump_json(args.output, result)
        return 0 if result['detections'] else 1
    elif args.command == "audit":
        from .product_geometry import audit_product
        report = audit_product(normalized_from_path(args.product))
        print(json.dumps(report, indent=2))
        return 0 if report['valid_for_snap'] else 2
    elif args.command == "validate":
        product = normalized_from_path(args.product)
        validate_product(product)
        print(f"VALID: {args.product} ({len(product['blocks'])} blocks)")
    elif args.command == "convert":
        product = normalized_from_path(args.product)
        dump_yaml(args.output, product)
        print(f"WROTE: {Path(args.output).resolve()}")
    elif args.command == "generate":
        episode, scene = generate(args.product, args.seed, args.output_dir)
        print(f"EPISODE: {episode['episode_id']}")
        print(f"SCENE: {scene}")
    elif args.command == "score":
        result = score_episode(load_yaml(args.manifest), load_yaml(args.actual))
        if args.output:
            dump_json(args.output, result)
        print(result)
    elif args.command == "batch-generate":
        suite = []
        for seed in range(args.first_seed, args.first_seed + args.count):
            directory = Path(args.output_dir) / f"seed-{seed}"
            episode, scene = generate(args.product, seed, str(directory))
            suite.append({"seed": seed, "episode_id": episode["episode_id"], "scene": str(scene)})
        dump_json(Path(args.output_dir) / "suite.json", {"episodes": suite})
        print(f"GENERATED: {len(suite)} episodes in {Path(args.output_dir).resolve()}")
    elif args.command == "summarize":
        results = [json.loads(p.read_text(encoding="utf-8")) for p in Path(args.run_dir).rglob("result.json")]
        summary = {
            "episodes": len(results),
            "successes": sum(bool(r.get("success")) for r in results),
            "success_rate": (sum(bool(r.get("success")) for r in results) / len(results)) if results else 0.0,
            "mean_completion": (sum(float(r.get("completion", 0.0)) for r in results) / len(results)) if results else 0.0,
        }
        print(json.dumps(summary, indent=2))
    elif args.command == "run":
        if args.connection_mode == "snap" and not args.allow_invalid_product:
            from .product_geometry import audit_product
            report = audit_product(normalized_from_path(args.product))
            if not report['valid_for_snap']:
                print(json.dumps(report, indent=2))
                return 2
        if args.backend != "oracle":
            from .perception import vision_preflight
            status = vision_preflight()
            if not status['ready']:
                print(json.dumps(status, indent=2))
                return 2
            if args.executor != "baseline":
                parser.error("the vision backend requires --executor baseline")
            args.observation = "rgbd"
        if args.executor == "oracle-baseline" and args.observation != "oracle":
            parser.error("oracle-baseline requires --observation oracle; it cannot validate visual perception")
        episode, scene = generate(args.product, args.seed, args.output_dir)
        output = Path(args.output_dir).resolve()
        os.environ["MJ_BRIDGE_MODEL"] = str(scene)
        os.environ["LEGO_BENCH_MANIFEST"] = str(output / "episode_manifest.yaml")
        os.environ["LEGO_BENCH_RUN_DIR"] = str(output)
        os.environ["LEGO_BENCH_OBSERVATION"] = args.observation
        os.environ["LEGO_BENCH_CONNECTION_MODE"] = args.connection_mode
        if args.headless:
            os.environ["MJ_BRIDGE_HEADLESS"] = "1"
            if args.observation == "rgbd":
                os.environ.setdefault("MUJOCO_GL", "egl")
        print(f"STARTING: {episode['episode_id']}")
        if args.executor in ("baseline", "oracle-baseline"):
            if args.backend == "oracle" and args.observation != "oracle":
                parser.error("the Oracle baseline requires --observation oracle")
            from .reference_executor import execute
            result = execute(output, backend=args.backend)
            print(json.dumps(result, indent=2))
            return 0 if result["success"] else 1
        from .mj_bridge3 import main as bridge_main
        bridge_main()


if __name__ == "__main__":
    raise SystemExit(main())
