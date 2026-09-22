# 仿真验证说明

本项目的验证分为配置/场景测试和机械臂实际执行测试。场景加载通过不能代表装配成功。

## 可复现命令

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python -m pytest -q
python -m mj_bridge.benchmark_cli run \
  --product examples/products/catalog/final_product_trafficlight.yaml \
  --seed 42 --headless --executor oracle-baseline --output-dir runs/trafficlight
python scripts/test_product_suite.py \
  --seeds 0 17 42 --jobs 4 --skip-invalid-products --output-dir runs/product-suite
```

`oracle-baseline` 使用 MuJoCo 关节雅可比逆运动学、力矩控制和实体夹爪接触。
它不通过移动积木的 qpos 来完成抓取或搬运。`snap` 模式仍使用明确声明的
简化扣合：接触底板后对齐 16 mm 网格，积木接触扣合的层间距为 19.2 mm。
这不是对真实 LEGO 扣合形变的物理建模。`physics` 模式禁用上述吸附。

基线不具备完整的避障规划，也不保证任意随机布局的抓取成功。每轮失败、
超时、部分完成均保留在 `summary.json` 和该轮 `execution.log` 中；进程失败返回非零。
默认 `run` 仍启动供第三方执行器使用的 ROS 环境，必须显式选择基线才会自动执行。

`examples/products/catalog/` 保留了 36 个历史 `final_product*.yaml` 的规范化目标，
没有为得到通过率而重写目标。某些历史文件含无支撑的悬空目标、非标准层高等，
即使格式合法也可能无法按当前零件几何装配。

`lego-bench validate` 检查输入格式；`lego-bench audit PRODUCT.yaml` 检查
实体重叠、支撑面积与 19.2 mm 扣合层高。检查采用旋转后的矩形交面积，
通过检查并不保证机械臂可达或无碰撞。

当前目录的 36 个历史产品中，29 个通过几何检查；以下 7 个被明确拒绝：
`final_product`、`final_product2`、`final_product_burger`、`final_product_door`、
`final_product_flower`、`final_product_simple`、`final_product_t`。
它们仍保留原始目标。运行时会打印具体积木 ID 和问题，不会把无效输入算作装配成功。
仅在诊断原始失败时使用 `run --allow-invalid-product`。

另提供 [7 个明确命名的替代设计](../examples/products/supported_variants/README.md)，
逐块坐标和方向改动记录于该目录的 `changes.json`。它们是独立目标，执行结果必须
单独统计；不能把替代设计通过写成原历史产品通过。

批量命令的 `--skip-invalid-products` 会把这些配置写入 `summary.json` 的
`invalid_products`，执行次数与成功次数只统计真正执行的产品。
不加该参数时，遇到无效产品会记录非零退出状态。

方形积木在相邻同层零件旁放置时，会在等价的 0/90 度方向中选择夹指净空更大的方向。
放置时检查已装零件的邻近关系：存在同层近邻或多个支撑时垂直下降，
其余位置使用平滑关节轨迹。垂直路径采用关节三次插值保持速度连续。
末端额外下压 2 mm，确保接触后再张开夹指，
避免悬空释放时被夹指摩擦带偏。这个动作不修改目标或评分容差。
每块释放后保存 `progress.json`，即使外部超时终止，
也能查看最近一次完成的动作和当时的实际状态；进度文件不代表整轮成功。

## 视觉后端

```bash
export FP_REPO=/absolute/path/to/FoundationPose
export SAM_CHECKPOINT=/absolute/path/to/sam_vit_h_4b8939.pth
python -m mj_bridge.benchmark_cli doctor \
  --backend groundingdino-sam-foundationpose
python -m mj_bridge.benchmark_cli perceive \
  --frame frame.npz --product examples/products/traffic_light.yaml --output observations.json
```

`frame.npz` 必须含 `rgb`（H×W×3 uint8 RGB）、`depth`（H×W，米，缺失为 0）、
`K`（3×3 内参）和 `camera_to_world`（4×4 光学相机到世界的刚体变换）。
后端执行 GroundingDINO 检测、SAM 分割和 FoundationPose 位姿注册，不接收 Oracle 状态。
同类物体返回独立 detection ID，不声称已与目标实例 ID 完成关联。

模型代码、CUDA 扩展、SAM 和 FoundationPose 权重需由使用者另行安装。
后端检查失败时不回退到 Oracle，不生成视觉成功结果。
视觉闭环入口（尚未在 GPU 环境验收）：

```bash
python -m mj_bridge.benchmark_cli run \
  --product examples/products/traffic_light.yaml --seed 42 --headless \
  --executor baseline --backend groundingdino-sam-foundationpose --output-dir runs/vision
```

该模式只通过渲染的 RGB-D 选择源零件，最终评分独立读取真实末态。同色同型多实例的
目标身份关联尚未验证，严格按原实例 ID 评分可能判失败。视觉闭环执行尚未验收，
不能把接口的存在当作测试通过。

同一批量入口可显式选择视觉后端。建议单 GPU 使用 `--jobs 1`，缺少依赖时整批
预检直接失败，不会逐个产品反复加载或回退 Oracle：

```bash
python scripts/test_product_suite.py --backend groundingdino-sam-foundationpose \
  --seeds 42 --jobs 1 --skip-invalid-products --output-dir runs/vision-suite
```
