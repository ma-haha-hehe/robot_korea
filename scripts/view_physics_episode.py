#!/usr/bin/env python3
"""Interactive Oracle assembly preview; never commands physical hardware."""
import argparse
import os
import time
from pathlib import Path
import mujoco.viewer
import rclpy
from mj_bridge.benchmark_cli import generate
from mj_bridge.benchmark_core import dump_json
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure


def main():
    parser = argparse.ArgumentParser(description="Watch a torque-controlled Oracle physics episode.")
    parser.add_argument('--product', default='examples/products/catalog/final_product_hammer.yaml')
    parser.add_argument('--contact-profile', choices=['loose', 'plastic'], default='loose')
    parser.add_argument('--robot-base-x', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--speed-scale', type=float, default=1.5)
    parser.add_argument('--output-dir', default='runs/viewer/' + time.strftime('%Y%m%d-%H%M%S'))
    args = parser.parse_args()
    if not .25 <= args.speed_scale <= 2.:
        parser.error('--speed-scale must be between 0.25 and 2')
    out = Path(args.output_dir).resolve()
    if out.exists() and any(out.iterdir()):
        parser.error('output directory is not empty; choose a new preview directory')
    _, scene = generate(args.product, args.seed, str(out), connection_mode='physics',
                        robot_base_x=args.robot_base_x, contact_profile=args.contact_profile)
    os.environ.update(MJ_BRIDGE_MODEL=str(scene), LEGO_BENCH_MANIFEST=str(out / 'episode_manifest.yaml'), LEGO_BENCH_RUN_DIR=str(out), LEGO_BENCH_OBSERVATION='oracle', LEGO_BENCH_CONNECTION_MODE='physics')
    from mj_bridge.mj_bridge3 import MuJoCoActionServer
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MuJoCoActionServer()
    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
            node.model.vis.quality.shadowsize = 256
            viewer.cam.lookat[:] = [.35, .1, .12]
            viewer.cam.distance = 1.45
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -30
            original_step = node.step_pid
            last_sync = [float(node.data.time), time.monotonic()]
            def visible_step():
                original_step()
                if node.data.time - last_sync[0] >= 1/30:
                    if not viewer.is_running():
                        raise KeyboardInterrupt
                    viewer.sync(state_only=True)
                    time.sleep(max(0, 1/30 - (time.monotonic() - last_sync[1])))
                    last_sync[:] = [float(node.data.time), time.monotonic()]
            node.step_pid = visible_step
            viewer.sync(state_only=True)
            print('Viewer opened; assembly starts in 3 seconds.', flush=True)
            display_start = time.monotonic()
            time.sleep(3)
            executor = OracleExecutor(node, speed_scale=args.speed_scale)
            error = None
            try:
                executor.run()
            except (ExecutionFailure, KeyboardInterrupt, mujoco.FatalError) as exc:
                error = str(exc) or 'viewer interrupted'
            result = node.benchmark_result()
            result['execution_error'] = error
            result['success'] = bool(result['success'] and error is None)
            result['events'] = executor.events
            result['speed_scale'] = args.speed_scale
            result['purpose'] = 'interactive preview; use benchmark_cli for regression evidence'
            dump_json(out / 'result.json', result)
            dump_json(out / 'actual_state.json', node.current_block_state())
            print(f'Assembly finished: {result}. Results: {out}. Display elapsed: {time.monotonic()-display_start}', flush=True)
            print('Close the MuJoCo window to exit.', flush=True)
            while viewer.is_running():
                viewer.sync(state_only=True)
                time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if locals().get('result', {}).get('success') else 1


if __name__ == "__main__":
    raise SystemExit(main())
