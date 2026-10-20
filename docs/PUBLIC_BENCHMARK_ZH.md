# LEGO Bench 公共环境说明

## 1. 环境负责什么

输入成品 YAML 后，环境完成：格式验证、零件统计、seed 随机摆放、MuJoCo
场景生成、Panda 控制、Oracle/RGB-D 观测、复位、整件装配评分和结果保存。
环境不绑定任何抓取或装配算法。

## 2. 主要文件

| 文件 | 内容 |
|---|---|
| `product_schema.json` | 公共成品 YAML 的机器可读规范 |
| `part_registry.yaml` | 零件尺寸、质量、mesh、对称性注册表 |
| `benchmark_core.py` | 旧格式转换、验证、随机摆放、评分 |
| `benchmark_cli.py` | `lego-bench` 命令行入口 |
| `scene_builder.py` | episode manifest 到 MuJoCo XML |
| `mj_bridge3.py` | 物理循环、ROS 控制、观测、复位和实时评分 |
| `benchmark.yaml` | 容差与单轮超时配置 |
| `examples/external_executor.py` | 第三方算法接入模板 |

## 3. 产品格式

```yaml
schema_version: 1
product: {name: traffic_light}
blocks:
  - id: green_base
    type: brick_2x2
    color: green
    target: {position: [0, 0, 0], yaw_deg: 0}
```

`position` 是相对装配底板中心的目标偏移，单位为米。环境兼容仓库原有的
`blocks + pos + rotation` 和部分 `tasks + place` 文件，但公开数据集应使用 v1 格式。

## 4. 命令

```bash
source enter_sim_env.sh

lego-bench validate PRODUCT.yaml
lego-bench convert OLD.yaml NEW.yaml
lego-bench generate --product PRODUCT.yaml --seed 42 --output-dir runs/e42
lego-bench run --product PRODUCT.yaml --seed 42 --output-dir runs/e42
```

第三方方案需要 MoveIt 时，用同一个入口启动完整 pipeline：

```bash
ros2 launch mj_bridge lego_bench.launch.py \
  product:=$PWD/PRODUCT.yaml seed:=42 headless:=true \
  observation:=oracle connection_mode:=snap
```

批量生成和汇总：

```bash
lego-bench batch-generate --product PRODUCT.yaml --first-seed 0 --count 100 \
  --output-dir runs/suite
lego-bench summarize runs/suite
```

`runs/e42` 中包含：

- `product.normalized.yaml`：规范化目标；
- `episode_manifest.yaml`：随机初态、目标、seed 和 body 映射；
- `scene.xml`：本轮独立 MuJoCo 场景；
- `actual_state.json`：查询结果时导出的真实末态；
- `result.json`：逐块误差和总成功率。

结果还包含 `elapsed_s`、`timeout`、`collision_count`、
`stability_violations`、观测模式和连接模式。

## 5. 公平评测

- 相同产品、seed、observation mode、connection mode 才能直接比较。
- `oracle` 给出 MuJoCo 真值，只评测规划和控制。
- `rgbd` 关闭 Oracle 状态发布，视觉执行器使用图像、深度和相机参数。
- 分割图为 MuJoCo `32SC2`（对象 ID、对象类型），相机坐标系是
  `realsense`；完整 launch 发布 `world -> realsense` 静态外参。
  分割图是仿真标注，不属于 GroundingDINO/SAM/FoundationPose 执行器的输入。
- `snap` 使用公开的简化连接机制；`physics` 禁止自动吸附。
- 报告必须保存 episode manifest，不能只记录成功率。

## 6. 外部方法接入

外部方法订阅 `/lego_bench/goal` 和观测话题，通过标准
`FollowJointTrajectory` action 控制手臂与夹爪。完成后调用：

```bash
ros2 service call /mj_bridge/result std_srvs/srv/Trigger '{}'
```

环境将结果写入本轮 run 目录。

## 7. 扩展零件

加入新零件需要：视觉 mesh、简化 collision、尺寸、质量、对称角度，并在
`part_registry.yaml` 注册。未注册类型会被验证器拒绝，不会退化成默认方块。
