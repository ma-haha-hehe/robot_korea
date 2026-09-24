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

Interactive pure-physics preview (desktop display required):

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
MUJOCO_GL=glfw LP_NUM_THREADS=2 python scripts/view_physics_episode.py \
  --product examples/products/catalog/final_product_hammer.yaml --seed 42
```

Change `--product` to select another registered design. The window stays open
when execution ends; `result.json` records failures as well as successes.
`--speed-scale` accepts 0.25–2 (default 1.5); final insertion remains slower.

Workbenchmark plastic-contact tasks use a compliant interference fit and sliding,
torsional and rolling friction. Run a bundled example with the tested robot-base
position. The dedicated configuration allows up to 3600 seconds of wall-clock
time for dense CPU simulations; reaching the limit is a failed run. It does not
change the physics timestep or acceptance thresholds:

```bash
export MJ_BRIDGE_BENCHMARK_CONFIG="$PWD/examples/config/plastic_benchmark.yaml"
python -m mj_bridge.benchmark_cli run \
  --product examples/products/workbenchmark/tier2_task_001.yaml \
  --seed 42 --headless --executor oracle-baseline \
  --connection-mode physics --contact-profile plastic --robot-base-x=-.05 \
  --output-dir runs/workbenchmark-plastic-example

MUJOCO_GL=glfw LP_NUM_THREADS=2 python scripts/view_physics_episode.py \
  --product examples/products/workbenchmark/tier2_task_001.yaml \
  --seed 42 --contact-profile plastic --robot-base-x=-.05
```

To use the full task collection from a neighbouring Workbenchmark checkout:

```bash
python scripts/import_workbenchmark.py --repository ../Workbenchmark \
  --initial-layout recorded --output runs/workbenchmark-recorded
```

Restore the committed regression sample of 240 tasks (60 per difficulty tier):

```bash
python scripts/select_workbenchmark_sample.py --products runs/workbenchmark-recorded \
  --selection-manifest examples/validation/workbenchmark_selection.json \
  --output-dir runs/workbenchmark-selected
export MJ_BRIDGE_BENCHMARK_CONFIG="$PWD/examples/config/plastic_benchmark.yaml"
python scripts/test_product_suite.py --products runs/workbenchmark-selected \
  --seeds 42 --jobs 4 --timeout 3650 --connection-mode physics \
  --contact-profile plastic --backend oracle --robot-base-x=-.05 \
  --output-dir runs/workbenchmark-regression
```

Restoration checks every input hash and preserves the original selection metadata.
The strata record part count, rectangular-part count, height and planned grasp
angles at selection time. A fresh draw can change after planner updates, even
with the same random seed. A saved selection records inputs, not successful tests.

Then select a generated task YAML with `--product`. The importer preserves the
original relative target geometry and recorded starting layout. Plastic parameters
are simulation settings, not measured ABS properties; see
[contact model and validation limits](docs/PLASTIC_CONTACT_ZH.md).
The current 240-task stratified validation is still in progress. Preview output is
labelled separately from regression evidence.

Full Panda + MoveIt pipeline (for external executors):

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

The executor uses assembly by disassembly, shared with the research
`myplanner.py` adapter: remove accessible parts from the finished structure with
a 0°/90° gripper approach, then reverse that order. It rejects an unavailable
approach instead of forcing a grasp. For adjacent steps on the same layer,
connecting a loaded support to another lower branch takes priority over a
narrow-side hold. Any reordered pair must still have clear 0°/90° removal
grasps; this support heuristic does not certify physical stability.
Preview the plan before execution:

```bash
python scripts/plan_assembly.py examples/products/catalog/final_product_hammer.yaml
```

Every automatic episode saves `assembly_plan.json`. The planner's world-frame
gripper angle is converted to a part-relative offset for picking randomly
oriented source parts; the finished part's orientation is preserved. Geometric
accessibility does not establish dynamic stability or collision-free arm travel.

The torque-controlled Oracle baseline reports failed grasps and unreachable poses.
It is not a complete collision-free planner. The 36 legacy product targets are
preserved; some describe unsupported geometry or floating targets. See
[validation scope and optional vision setup](docs/VALIDATION_ZH.md).

The [measured validation report](docs/validation/REPORT_ZH.md) records the tested
products and seeds, with final states and per-part scores in its JSON evidence.
This historical report predates the gripper-contact fixes and does **not**
validate the current physics controller or prove nonpenetrating grasps.
Current contact criteria and limitations are documented in
[physics validation](docs/PHYSICS_VALIDATION_ZH.md).
The historical Oracle/snap run passed 87 episodes across 29 original products and
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
