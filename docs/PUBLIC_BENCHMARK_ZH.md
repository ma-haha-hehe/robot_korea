# LEGO Bench 使用指南

本仓库是 Workbenchmark 积木任务的机器人仿真与评测环境，提供 Panda/MuJoCo 场景、
ROS 2 控制与观测接口、参考执行器和自动报告。完整任务、版本与图片由独立的
[benchmark_lego 仓库](https://github.com/ma-haha-hehe/benchmark_lego)管理。

## 安装与首次运行

运行环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10 和 MuJoCo 3.11.0。
先安装 ROS 2 Humble 并配置其 apt 源，然后运行：

```bash
git clone https://github.com/ma-haha-hehe/robot_korea.git
cd robot_korea
bash install_sim_system_deps.sh
source enter_sim_env.sh
colcon build --base-paths src/mj_bridge --packages-select mj_bridge --symlink-install
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
lego-bench validate examples/products/traffic_light.yaml
lego-bench audit examples/products/traffic_light.yaml
lego-bench run --product examples/products/traffic_light.yaml --seed 42 \
  --headless --executor oracle-baseline --connection-mode physics \
  --output-dir runs/traffic-light-physics
```

安装脚本使用 sudo 安装系统依赖，并在仓库中建立 `.venv`。
每个新终端都要 `source enter_sim_env.sh`；找不到 `lego-bench` 时可使用
`python -m mj_bridge.benchmark_cli`。Mac 可通过 SSH 编辑 Ubuntu 上的代码，
仿真与 ROS 仍在 Ubuntu 上运行，详见[工作区指南](WORKSPACE_SNAPSHOT_ZH.md)。

## 输入和工作流程

一个产品 YAML 指定目标结构。环境规范化产品、统计零件、生成初态与 MuJoCo 场景，
再由外部方法或参考执行器操作机器人，最后独立导出实际末态并评分。
有记录初态的任务保持该布局，其余产品使用 seed 生成可复现散料。

```yaml
schema_version: 1
product: {name: single_block}
blocks:
  - id: green_base
    type: brick_2x2
    color: green
    target: {position: [0, 0, 0], yaw_deg: 0}
```

`position` 的单位为米，是相对装配参考位置的偏移；角度单位由字段名明确。
当前注册 `brick_2x2`、`brick_4x2`，未知零件类型直接拒绝。
旧 `blocks + pos + rotation` 和部分 `tasks + place` 文件由适配器转换。
最新版 Workbenchmark 的三文件实例应使用专门的[导入器](WORKBENCHMARK_INTEGRATION_ZH.md)。

## 选择运行条件

| 选项 | 含义 |
|---|---|
| `--executor external` | 启动环境并等待外部控制，是默认值 |
| `--executor oracle-baseline` | 使用仿真位姿与接触真值执行参考控制器 |
| `--observation oracle` | 发布目标与真实状态 |
| `--observation rgbd` | 使用图像、深度与相机参数，不发布 Oracle 状态 |
| `--connection-mode snap` | 简化、显式声明的扣合机制 |
| `--connection-mode physics` | 禁止自动吸附，使用动力学接触 |
| `--contact-profile loose` | 默认松配合空心刚体 |
| `--contact-profile plastic` | physics 模式下的柔顺过盈/摩擦接触 |

接触参数未经实物标定，Oracle 不等同于视觉或学习策略。
不同条件下的结果应分开报告。[能力范围](RELEASE_STATUS_ZH.md)列出验证边界。

## 常用命令

```bash
lego-bench validate PRODUCT.yaml
lego-bench audit PRODUCT.yaml
lego-bench convert OLD.yaml NEW.yaml
lego-bench generate --product PRODUCT.yaml --seed 42 --output-dir runs/scene-42
lego-bench batch-generate --product PRODUCT.yaml --first-seed 0 --count 10 \
  --output-dir runs/scenes
lego-bench summarize runs/scenes
```

`generate` 只生成场景，不执行装配。批量场景中没有执行结果时，汇总不能推导成功率。
查看物理过程需要桌面显示：

```bash
MUJOCO_GL=glfw LP_NUM_THREADS=2 python scripts/view_physics_episode.py \
  --product examples/products/catalog/final_product_hammer.yaml --seed 42
```

窗口在结束后保持打开。正式抽样与评测见[状态机指南](EXECUTION_STATE_MACHINE_ZH.md)
和[自动评测指南](VALIDATION_ZH.md)。

## 模块与目录

| 位置 | 职责 |
|---|---|
| `src/mj_bridge/mj_bridge/product_schema.json` | 产品规范 |
| `src/mj_bridge/mj_bridge/part_registry.yaml` | 零件几何、质量和对称性 |
| `src/mj_bridge/mj_bridge/benchmark_core.py` | 格式转换、初态生成和评分 |
| `src/mj_bridge/mj_bridge/scene_builder.py` | MuJoCo 场景构建 |
| `src/mj_bridge/mj_bridge/mj_bridge3.py` | 物理循环、ROS 接口、观测和结果 |
| `src/mj_bridge/mj_bridge/assembly_planner.py` | 拆解逆序、抓取方向与释放模式 |
| `src/mj_bridge/mj_bridge/reference_executor.py` | 力矩控制与接触执行 |
| `src/mj_bridge/mj_bridge/execution_phases.py` | 执行阶段及证据门槛 |
| `src/mj_bridge/mj_bridge/benchmark_cli.py` | 命令行入口 |
| `scripts/` | 任务导入、抽样运行、诊断与报告 |
| `examples/` | 产品、配置和外部执行器示例 |

规划说明见[抓取方向](GRASP_PLANNING_ZH.md)，接触几何与坐实验收见
[物理模型](PHYSICS_VALIDATION_ZH.md)和[塑料接触](PLASTIC_CONTACT_ZH.md)。

## 外部方法接入

```bash
ros2 launch mj_bridge lego_bench.launch.py \
  product:=$PWD/examples/products/traffic_light.yaml seed:=42 headless:=true \
  observation:=oracle connection_mode:=physics contact_profile:=loose
```

外部方法订阅 `/lego_bench/goal` 和自己的观测，通过
`/mj_panda_arm_controller/follow_joint_trajectory` 与
`/mj_panda_hand_controller/follow_joint_trajectory` 控制机器人。
[接口模板](../examples/external_executor.py)默认订阅 Oracle，需替换 `policy()` 才能操作。
RGB-D 方法应改用图像输入，不能读取真值作为预测。
完成后调用 `/mj_bridge/result`（`std_srvs/Trigger`）保存评分。
完整话题表见 [README](../README.md#connect-an-external-method)。

## 本地结果与可复现性

输出通常包含规范化目标、episode manifest、独立 XML、实际末态、逐块误差与总结果。
参考执行器另外保存阶段、释放、支撑和接触证据。抽样报告输出 CSV 与 JSON，
区分成功、失败、超时和未执行；不要仅凭调度完成或单帧目标相似判成功。

`runs/`、构建目录、日志和缓存由 Git 忽略。进行实验比较时完整保存这些本地结果，
同时固定源码、任务哈希、布局、观测、接触、基座和时间预算。
静态验证、模块测试、完整物理执行和实机验证是不同层级，不能相互替代。
