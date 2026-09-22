# LEGO Bench: Panda + MuJoCo manipulation benchmark

LEGO Bench is a ROS 2 / MuJoCo environment for evaluating third-party LEGO
planning, perception and execution methods. A product YAML describes the goal;
the environment creates the required loose parts at deterministic random poses,
exposes standard control/observation interfaces, and scores the final assembly.

中文完整说明见 [docs/PUBLIC_BENCHMARK_ZH.md](docs/PUBLIC_BENCHMARK_ZH.md).

## Quick start

Requires Ubuntu 22.04 with ROS 2 Humble base already installed and its apt
repository configured. The script below installs the additional simulation packages.
Simulator dependencies are pinned in `requirements-sim.txt` to the versions used
for the recorded validation.

```bash
bash install_sim_system_deps.sh
source enter_sim_env.sh
colcon build --base-paths src/mj_bridge --packages-select mj_bridge --symlink-install
source enter_sim_env.sh

lego-bench validate examples/products/traffic_light.yaml
lego-bench generate --product examples/products/traffic_light.yaml --seed 42
lego-bench run --product examples/products/traffic_light.yaml --seed 42 \
  --headless --executor baseline --output-dir runs/traffic-light-42
```

Full Panda + MoveIt pipeline (recommended for external executors):

```bash
ros2 launch mj_bridge lego_bench.launch.py \
  product:=$PWD/examples/products/traffic_light.yaml seed:=42 headless:=true
```

Headless RGB-D episode:

```bash
lego-bench run --product examples/products/traffic_light.yaml --seed 42 \
  --headless --observation rgbd --output-dir runs/traffic-light-42
```

Docker:

```bash
docker compose -f docker-compose.benchmark.yaml up --build
```

## Automatic simulation validation

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python -m pytest -q
python -m mj_bridge.benchmark_cli run \
  --product examples/products/catalog/final_product_trafficlight.yaml \
  --seed 42 --headless --executor oracle-baseline --output-dir runs/trafficlight
python scripts/test_product_suite.py --seeds 0 17 42 --skip-invalid-products
```

The torque-controlled Oracle baseline reports failed grasps and unreachable poses.
It is not a complete collision-free planner. The 36 legacy product targets are
preserved; some describe unsupported geometry or floating targets. See
[validation scope and optional vision setup](docs/VALIDATION_ZH.md).

The [measured validation report](docs/validation/REPORT_ZH.md) records the tested
products and seeds, with final states and per-part scores in its JSON evidence.
The recorded Oracle/snap run passed 87 episodes across 29 original products and
three seeds, plus seven separately named corrected designs. The seven invalid
original targets remain available for inspection and are not counted as successes.

`lego-bench audit PRODUCT.yaml` diagnoses overlapping solids, inadequate support
and incompatible layer heights. `run` rejects invalid snap targets before starting
the simulator. Rejected products remain in the catalogue and in suite reports;
they are never counted as successful executions.

`lego-bench doctor --backend groundingdino-sam-foundationpose` checks optional GPU
and weight dependencies. `lego-bench perceive` processes calibrated RGB-D NPZ
frames through GroundingDINO, SAM and FoundationPose. The selectable
`run --executor baseline --backend groundingdino-sam-foundationpose` integration
is implemented but has not yet been validated with GPU weights.

## Public interfaces

- Arm: `/mj_panda_arm_controller/follow_joint_trajectory`
- Hand: `/mj_panda_hand_controller/follow_joint_trajectory`
- Robot state: `/joint_states`
- Goal: `/lego_bench/goal` (`std_msgs/String`, JSON)
- Oracle state: `/lego_bench/ground_truth` (`std_msgs/String`, JSON)
- RGB-D: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`
- Segmentation: `/camera/segmentation`
- Reset: `/mj_bridge/reset`
- Result: `/mj_bridge/result`
- Live score: `/mj_bridge/benchmark_state`

See [examples/external_executor.py](examples/external_executor.py) for the policy adapter.
Segmentation uses MuJoCo's two-channel `32SC2` output (`object id`, `object type`).
The optical frame is `realsense`, with a static `world -> realsense` transform in
the full launch.

## Scope

Version 0.1 supports registered `brick_2x2` and `brick_4x2` parts. “Any product
YAML” means any schema-valid product composed of registered parts. Add new part
types through `part_registry.yaml` plus visual/collision assets.

The `snap` connection mode is an explicit simplified stud-engagement model.
Use `--connection-mode physics` to disable automatic connections. Results from
different connection or observation modes must not be compared as one benchmark.

## License and third-party components

The public benchmark software and Panda model carry the Apache-2.0 license in
`LICENSE`. FoundationPose is separately licensed by
NVIDIA and is restricted to non-commercial research/evaluation. It is installed
separately. Hardware SDK binaries, unverified legacy LEGO meshes, model weights,
caches and real-robot data are excluded from the public benchmark; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
