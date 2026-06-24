# 真实机械臂论文实验 Protocol 中文版

这个 protocol 对应论文里的 real-robot validation 部分。它的目的不是替代仿真大规模 benchmark，而是证明同一套 product YAML、planner、open-vocabulary vision、6D pose estimation、execution bridge 可以在真实 UR5 + Robotiq + RealSense 系统上跑通，并且记录导师要求的 planning / vision / conversion / execution 分段时间。

当前真实机械臂实验只跑 Tier 1 到 Tier 3。暂时不跑 Tier 4。之前“三块白色 2x4 + 一块红色 2x4 + 一块黄色 2x2”的结构现在归为真实机械臂 Tier 3，因为它已经包含重复同类积木、颜色区分、不同尺寸、5 步执行和更高结构。

## 1. 实验 Tier 定义

| Tier | Product YAML | 步数 | 实验目的 |
|---|---|---:|---|
| Tier 1 | `src/panda_pick/src/final_product_tier1_single_2x4.yaml` | 1 | 单个目标的 detection、6D pose、抓取、放置闭环验证 |
| Tier 2 | `src/panda_pick/src/final_product_tier2_stack_3x_2x4.yaml` | 3 | 多步堆叠、放置高度累积误差、连续执行稳定性 |
| Tier 3 | `src/panda_pick/src/final_product_tier3_bridge_plus_red_yellow.yaml` | 5 | 三块白色重复物体选择、红/黄颜色区分、2x4/2x2 混合、异步视觉节省时间 |

实际记录次数：

- Tier 1: 10 次，9 次成功，1 次失败，平均端到端时间 55.775 s。
- Tier 2: 10 次，9 次成功，1 次失败，平均端到端时间 167.977 s。
- Tier 3: 10 次，8 次成功，2 次失败，平均端到端时间 183.330 s。

这里的时间是整条 pipeline 的端到端总时间，包含 planning、vision、6D pose estimation、YAML 转换、节点/文件通信和真实机械臂执行时间。它不是单独的 robot motion time，也不是单独的 vision time。

## 2. 开始前检查

先确认四个 terminal 的职责：

1. Terminal 1: UR ROS2 driver / launch 文件。
2. Terminal 2: MoveIt 启动文件。
3. Terminal 3: Docker vision 环境。
4. Terminal 4: 运行 `run_auto_pick_pipeline.py`。

开始任何真实动作前检查：

- 急停按钮可用。
- 机器人 workspace 内没有手和无关物体。
- 积木都在相机能看到的 picking area。
- assembly area 是空的，除非你在继续一个中断实验。
- `UR5_OBSERVE_JOINTS`、`UR5_PREGRASP_JOINTS`、`UR5_PREPLACE_JOINTS` 已经是你最新示教的值。
- `src/my_robot_vision/config/vision_execution_bridge.yaml` 里的 reference 和 placement offset 是当前标定版本。

## 3. 每次实验保存什么

每次 trial 建一个 artifact 文件夹，例如：

```bash
mkdir -p real_robot_eval/artifacts/RR-T3-01
```

每次 trial 至少保存：

- 第四个 terminal 的完整输出日志。
- `src/panda_pick/src/plan_from_product.yaml`
- `src/panda_pick/src/plan_perception_test.yaml`
- `FoundationPose/root/FoundationPose/vision_output.yaml`
- `src/panda_pick/config/pick_place_task.yaml`
- `src/panda_pick/src/vision_frozen_observation.jpg`
- `src/panda_pick/src/vision_used_regions.jpg`
- `src/panda_pick/src/vision_pose_debug.jpg`
- 实验前 workspace 照片。
- 实验后 final product 照片。

建议每次跑完手动复制：

```bash
TRIAL=RR-T3-01
mkdir -p real_robot_eval/artifacts/$TRIAL
cp src/panda_pick/src/plan_from_product.yaml real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
cp src/panda_pick/src/plan_perception_test.yaml real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
cp FoundationPose/root/FoundationPose/vision_output.yaml real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
cp src/panda_pick/config/pick_place_task.yaml real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
cp src/panda_pick/src/vision_frozen_observation.jpg real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
cp src/panda_pick/src/vision_used_regions.jpg real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
cp src/panda_pick/src/vision_pose_debug.jpg real_robot_eval/artifacts/$TRIAL/ 2>/dev/null || true
```

## 4. 先 build 并 source

改过代码后只需要 build 一次：

```bash
cd /home/i6user/Desktop/robot_lego
colcon build --packages-select my_robot_vision panda_pick
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env
```

之后每开一个新 terminal，都要重新 source：

```bash
cd /home/i6user/Desktop/robot_lego
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/panda_pick/config/ur5_taught_joints.env
```

## 5. 正式运行命令

### Tier 1

```bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_tier1_single_2x4.yaml \
  --async-next-vision \
  --start-index 0 \
  --max-tasks 1 \
  --gripper-mode robotiq_socket \
  --gripper-speed 180 \
  --gripper-force 80 \
  --gripper-wait 4.0 \
  --gripper-release-position 140 \
  --gripper-release-wait 0.5 \
  --vision-timeout 180
```

Tier 1 只抓放一块黄色 2x4。它主要用来确认相机、标定、抓取高度、放置偏移没有明显问题。

### Tier 2

```bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_tier2_stack_3x_2x4.yaml \
  --async-next-vision \
  --start-index 0 \
  --max-tasks 3 \
  --gripper-mode robotiq_socket \
  --gripper-speed 180 \
  --gripper-force 80 \
  --gripper-wait 4.0 \
  --gripper-release-position 140 \
  --gripper-release-wait 0.5 \
  --vision-timeout 180
```

Tier 2 是三块 2x4 竖直堆叠。重点观察第二块、第三块有没有因为放置高度或横向偏移导致没有完全坐下。

### Tier 3

```bash
ros2 run my_robot_vision run_auto_pick_pipeline.py \
  --product-yaml src/panda_pick/src/final_product_tier3_bridge_plus_red_yellow.yaml \
  --async-next-vision \
  --start-index 0 \
  --max-tasks 5 \
  --gripper-mode robotiq_socket \
  --gripper-speed 180 \
  --gripper-force 80 \
  --gripper-wait 4.0 \
  --gripper-release-position 140 \
  --gripper-release-wait 0.5 \
  --vision-timeout 180
```

Tier 3 是当前真实机械臂最复杂实验：三块白色 2x4 构成 bridge，再放红色 2x4 和黄色 2x2。重点观察：

- 三块白色积木是否被依次选择为不同物体。
- `vision_used_regions.jpg` 里已经选过的区域是否被红色覆盖并在后续 detection 中排除。
- 红色 2x4 和黄色 2x2 是否被颜色逻辑正确区分。
- 第二层和第三层是否因为高度、放置速度或夹爪释放策略失败。

## 6. 怎么填 CSV 数字

当前已经填写两个端到端实验表：

- `real_robot_eval/tier_trials.csv`: 每个 trial 一行，记录整次任务是否成功和端到端总时间。
- `real_robot_eval/tier_summary.csv`: 每个 tier 一行，记录成功率、失败次数、平均时间、标准差、最大/最小值。

`real_robot_eval/tier_step_trials.csv` 暂时只保留表头，因为这次你提供的是每次 trial 的总时间，而不是每一步的 planner / vision / conversion / robot execution 分段时间。不要伪造 step-level 时间；论文里已经按 end-to-end total time 写。

第四个 terminal 会输出这些行：

```text
[AUTO][TIMING] planner_time_s=...
[AUTO][TIMING] vision_compute_time_s=...
[AUTO][TIMING] conversion_time_s=...
[AUTO][TIMING] robot_execution_time_s=...
[AUTO][TIMING] next_vision_wait_after_execution_s=...
[AUTO][TIMING] next_vision_ready_before_execution_done=true
```

填写规则：

- `planner_time_s`: 从 `planner_time_s=...` 抄。它是整份 product YAML 的 planning 时间，通常只填到 `step_index=0` 那一行。
- `vision_time_s`: 从 `vision_compute_time_s=...` 抄。这是真正 detection + SAM + FoundationPose 的计算时间。
- `conversion_time_s`: 从 `conversion_time_s=...` 抄。这是 vision YAML 转 execution YAML 的时间。
- `robot_execution_time_s`: 从 `robot_execution_time_s=...` 抄。这是机械臂执行当前 pick/place 的时间。
- `next_vision_ready_before_execution_done`: 从日志里的 true/false 抄。
- `overlap_saved_s`: 用 `max(0, vision_compute_time_s - next_vision_wait_after_execution_s)`。如果 ready_before_execution_done 是 true，可以近似写成这一步 next vision 的 `vision_compute_time_s`。

成功/失败字段填写：

- `vision_success`: 有非空 `vision_output.yaml`，pose 看起来合理，填 `true`；否则 `false`。
- `grasp_success`: 抓起了正确积木，填 `true`；抓错、夹空、滑落，填 `false`。
- `placement_success`: 放到了目标附近并释放成功，填 `true`；明显放偏、撞倒、提前掉落，填 `false`。
- `insertion_quality`: 完全按下写 `fully_seated`；部分卡住写 `partially_seated`；失败写 `failed`。
- `failure_stage`: 可填 `none`、`planner`、`vision_detection`、`vision_pose`、`conversion`、`grasp`、`transport`、`place`、`release`。

## 7. 怎么改 FINALPRODUCT YAML 的数字

FINALPRODUCT YAML 只描述最终成品几何，不用它来修正相机标定误差。

字段意义：

- `pos: [x, y, z]`: 这块积木在 assembly origin 下的目标位置，单位是米。
- `x/y`: 平面方向位置。0.005 m 等于 0.5 cm，0.010 m 等于 1 cm。
- `z`: 高度。一层 Duplo 约为 0.0192 m。第 0 层是 0.0000，第 1 层是 0.0192，第 2 层是 0.0384，第 3 层是 0.0576。
- `rotation: [0, 0, yaw]`: 最终绕竖直方向的角度，单位是度。`0` 是正常方向，`90` 是横着放。
- `vision_label`: 视觉要找什么。比如 `white 2x4 brick.`、`red 2x4 brick.`、`yellow 2x2 brick.`。

如果所有放置都整体偏右、偏前、偏高、偏低，不要改 FINALPRODUCT YAML，应该改：

```text
src/my_robot_vision/config/vision_execution_bridge.yaml
```

其中：

- `placement.place_offset_xyz`: 全局放置偏移。
- `pick.pick_offset_xyz`: 抓取参考偏移。
- `execution_axis.final_pick_offset_xyz`: flip 后的最终抓取平面补偿。
- `pick.descend` / `place.descend`: 抓取或放置下降距离。

## 8. 论文怎么写

论文里推荐这样表述：

1. Simulation benchmark 是主实验，因为可以大规模、多任务、可重复比较。
2. Real-robot validation 是硬件验证，不宣称大样本统计显著性。
3. 真实机械臂只跑 Tier 1 到 Tier 3。
4. Tier 3 是当前硬件最复杂任务，包含三白重复选择、颜色区分、2x2/2x4 混合、五步长时序和更高结构。
5. 异步视觉的贡献不是让 pose 更准，而是减少等待时间：当前动作执行时，视觉同时从 frozen observation 里计算下一步目标。
6. `vision_used_regions.jpg` 和日志里的 `next_vision_ready_before_execution_done` 是证明异步视觉和重复物体排除有效的关键证据。
