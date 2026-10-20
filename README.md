# LEGO Bench: Panda + MuJoCo manipulation benchmark

LEGO Bench is a ROS 2 / MuJoCo environment for evaluating third-party LEGO
planning, perception and execution methods. A product YAML describes the goal;
the environment creates the required loose parts from a recorded layout or deterministic random poses,
exposes standard control/observation interfaces, and scores the final assembly.

中文完整说明见 [docs/PUBLIC_BENCHMARK_ZH.md](docs/PUBLIC_BENCHMARK_ZH.md).

Panda + DUPLO 当前检查结果见
[当前本地最新版审核](docs/PANDA_DUPLO_LATEST_20261015.txt)（[旧数据记录](docs/PANDA_DUPLO_AUDIT_20261014.txt)）。
代码入口与目录职责见 [项目导航](PROJECT_MAP.txt)。
日常控制源码位于 `src/mj_bridge/`。冻结候选位于 `dist/`，逐任务证据位于 `runs/`；
已采用版本与仍未解决的失败见当前审核记录；逐任务成绩见
[本地 v2 完整执行报告](docs/validation/local-v2-current/REPORT_ZH.md)。
验证范围以审核记录为准；历史 snap 成绩不能当作当前塑料卡扣物理成功率。

## Panda + DUPLO 仿真装配

完成下方依赖安装后，在项目根目录运行：

```bash
bash run_panda_duplo.sh
# 指定任务、一个尚不存在的输出目录和 seed：
bash run_panda_duplo.sh examples/products/workbenchmark_latest_20261014/simple_shape_assembly_001.yaml runs/my-duplo-latest 42
# 另一个终端查看阶段和结果：
python3 scripts/show_episode_status.py runs/my-duplo-latest
```

该入口直接运行主源码，不重复构建 ROS 包；使用 Oracle 状态反馈、真实接触求解的
`physics`/`plastic` 模式、`speed_scale=1.5` 和 `robot_base_x=-0.05`。
当前 benchmark 来源是用户指定的本地目录
`/home/ubuntu22/workbenchmark_latest_20261014/versions/latest/tasks`，共 400 个任务。
导入数据及 1200 个原始 YAML 副本位于 `examples/products/workbenchmark_latest_20261014/`；
默认入口运行新版 `two_brick_stack_001`，执行前核对任务、来源副本及可访问的本地原文件哈希。
原始目标采用 20 mm 层号约定，物理模型采用 19.2 mm 装配层距；转换明确记录在 `import_manifest.json`。
目标 XY 相对底层包络中心平移，散件整盘平移到机器人工作区，保留相对 XY 与朝向。
400 项静态校验及规划通过不代表 400 次完整物理成功。最新物理执行范围、失败和控制修复见当前审核记录。
旧 `examples/products/workbenchmark/` 及其 5/8 指定案例结果只作为历史证据，不能当作新版成绩。
材料参数尚未实测标定；上述仿真使用 Oracle 状态反馈，不代表视觉识别或实机验收。

`result.json` 保存成功/失败和接触指标，`phase_trace.json` 保存阶段及耗时。
密集接触的计算时间可能长于仿真时间；3600 秒配置是超时上限，不是固定等待。
并行运行请为各进程设置不同的 `DUPLO_ROS_DOMAIN_ID`（0–232）。
本轮没有采用会导致复杂任务退化的提速和摩擦改动，详见审核记录。

需要可视化时，在桌面终端使用独立预览入口（新输出目录）：

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
MJ_BRIDGE_BENCHMARK_CONFIG="$PWD/examples/config/plastic_benchmark.yaml" MUJOCO_GL=glfw \
  python scripts/view_physics_episode.py --product examples/products/workbenchmark_latest_20261014/two_brick_stack_001.yaml \
  --contact-profile plastic --robot-base-x=-0.05 --seed 42 --output-dir runs/duplo-preview
```

上一轮冻结源码包位于 `dist/panda-duplo-reviewed-20261014/`，对应旧数据及旧审核记录。
本轮净化源码包位于 `dist/panda-duplo-reviewed-20261015/`；最终验证状态以包内 `VALIDATION.json` 为准。
新旧运行结果分别保存，不混算成功率。

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

The container defaults to the imported local v2 `two_brick_stack_001` task in
Oracle/physics/plastic mode, verifies its frozen source snapshot, and writes each
run to a new unique directory under the mounted `runs/` folder.
The Docker CLI is unavailable on the validation host; the YAML and shell entrypoints
were checked, but the image was not built or run in this audit.

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
approach instead of forcing a grasp. Preview the plan before execution:

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
