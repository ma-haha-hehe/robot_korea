# 自动评测与第三方方法接入

评测从相同任务和初态开始，记录执行过程并独立检查末态。
本仓库提供单例、产品批量和固定 Workbenchmark 抽样三种入口；
运行输出保存到本地 `runs/`，不属于源码分发内容。

常规检查在本地运行。GitHub Actions 仅保留 `workflow_dispatch` 手动入口，
不会因 push 或 pull request 自动执行，也不上传实验产物。
手动触发的任务仍会产生 GitHub 标准作业日志。

## 运行前检查

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python -m pytest -q
lego-bench validate examples/products/traffic_light.yaml
lego-bench audit examples/products/traffic_light.yaml
python scripts/plan_assembly.py examples/products/traffic_light.yaml
```

`validate` 检查格式；`audit` 检查旋转后的实体轮廓、名义支撑和兼容层高；
`plan_assembly.py` 检查拆解逆序与夹爪方向。它们都不能证明全身可达或动态装配成功。

`examples/products/catalog/` 保留历史目标，其中部分配置包含悬空、实体重叠或不兼容层高。
它们用于审计与适配研究，不应自动计为有效任务。
[独立替代设计](../examples/products/supported_variants/README.md)使用明确的新文件名和变更记录，
应分别统计。`--allow-invalid-product` 仅供诊断无效目标。

## 单例与批量执行

```bash
python -m mj_bridge.benchmark_cli run \
  --product examples/products/catalog/final_product_trafficlight.yaml \
  --seed 42 --headless --executor oracle-baseline --connection-mode physics \
  --output-dir runs/trafficlight-physics
python scripts/test_product_suite.py \
  --seeds 0 17 42 --jobs 2 --skip-invalid-products \
  --connection-mode physics --output-dir runs/product-suite
```

`--skip-invalid-products` 将静态无效输入单独列入摘要，不能计为成功执行。
基线执行异常返回非零退出状态；缺少完整结果、超时或部分完成均不算成功。
`run` 默认选择 `external`，需显式指定 `oracle-baseline` 才会自动执行参考控制器。

塑料任务应显式增加 `--contact-profile plastic --robot-base-x=-.05`，并按需要设置：

```bash
export MJ_BRIDGE_BENCHMARK_CONFIG="$PWD/examples/config/plastic_benchmark.yaml"
```

几何密集的接触场景可能消耗较长墙钟时间。增加预算不改变物理步长、目标或验收门槛。
固定抽样、独立 ROS domain、整体截止时间和 CSV 报告见
[执行状态机与抽样](EXECUTION_STATE_MACHINE_ZH.md)。

## 结果解释

完整运行包含 `episode_manifest.yaml`、`assembly_plan.json`、`phase_trace.json`、
`actual_state.json` 和 `result.json` 等文件；具体字段随执行入口和模式而异。
产品批量摘要区分无效输入与执行结果，固定抽样报告额外重新核对输入/源码哈希、
末态、物理接触、释放和坐实支撑。

比较时至少记录：任务版本和输入哈希、初始布局、seed、观测模式、连接模式、接触参数、
控制器源码、工位基座、墙钟预算，以及成功、失败、超时、未执行数量。
规划器或控制器变化后通常应重新执行；仅目标末态相同不能证明过程约束相同。

固定布局中更换 seed 不会自动产生新布局。少量有针对性的抽样不能作为整个数据集
成功率的无偏估计。Oracle、RGB-D、snap 与不同物理配置需要分别报告。
[能力范围](RELEASE_STATUS_ZH.md)与[物理验收](PHYSICS_VALIDATION_ZH.md)说明了各层验证的区别。

## 接入自己的执行器

使用 [外部执行器模板](../examples/external_executor.py)订阅目标和所需观测，
将 `policy()` 替换为自己的动作生成逻辑。该模板默认订阅 Oracle，仅作为接口示例。
完整 ROS 启动方式和话题表见 [README](../README.md#connect-an-external-method)。
第三方方法可以使用 MoveIt 或直接发送 `FollowJointTrajectory`，执行完后调用
`/mj_bridge/result` 保存独立评分。真实关节状态、末态评分和执行器自报成功应分别处理。

## 可选 RGB-D 感知后端

```bash
export FP_REPO=/absolute/path/to/FoundationPose
export SAM_CHECKPOINT=/absolute/path/to/sam_vit_h_4b8939.pth
python -m mj_bridge.benchmark_cli doctor \
  --backend groundingdino-sam-foundationpose
python -m mj_bridge.benchmark_cli perceive \
  --frame frame.npz --product examples/products/traffic_light.yaml \
  --output observations.json
```

`frame.npz` 必须含 `rgb`（H×W×3、uint8、RGB）、`depth`（H×W、米、缺失为0）、
`K`（3×3 内参）和 `camera_to_world`（4×4 光学相机到世界变换）。
后端使用 GroundingDINO 检测、SAM 分割和 FoundationPose 位姿注册；
模型代码、CUDA 扩展及权重由使用者另行安装。依赖检查失败时不会回退到 Oracle。

视觉闭环接口：

```bash
python -m mj_bridge.benchmark_cli run \
  --product examples/products/traffic_light.yaml --seed 42 --headless \
  --executor baseline --backend groundingdino-sam-foundationpose \
  --output-dir runs/vision
```

该路径通过渲染 RGB-D 选择源零件，评分仍独立读取真实末态。
同色同型多实例的目标身份关联、真实 GPU 闭环和视觉在手跟踪尚需单独验证。
不能将 Oracle 接触反馈的结果当作视觉模型性能。单 GPU 批量运行建议 `--jobs 1`。
