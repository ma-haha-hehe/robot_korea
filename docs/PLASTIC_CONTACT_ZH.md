# 塑料扣合接触模型

`mj_bridge.plastic_contact` 为自建空心积木增加内筋、管壁过盈和2 mm导入斜面。
扣合阻力来自柔顺接触和摩擦，不创建焊接约束，也不设置积木位姿。
通过 `--connection-mode physics --contact-profile plastic` 显式启用，默认仍为松配合。

## 碰撞表示与参数

`hollow_plastic_clutch_v2` 将每段管壁、内筋与导入斜面建成连续凸碰撞体，
避免分离部件内部端面的额外碰撞。外壳采用与名义盒体等尺寸的八顶点凸网格，
桌面接触保留1 µm碰撞检测边距。表示方式不改变目标位置、名义外形或0.5 mm穿透上限。

| 参数 | 值 |
|---|---:|
| 内筋/凸点名义过盈 | 0.05 mm |
| 接触维数 `condim` | 6，含扭转与滚动 |
| 内部滑动摩擦系数 | 0.3 |
| 扭转摩擦长度 | 0.001 m |
| 滚动摩擦长度 | 0.00005 m |
| 接触时间常数 / 阻尼比 | 0.03 s / 1 |
| 接触阻抗 `solimp` | 0.9 0.95 0.001 |
| 物理步长 | 0.5 ms |
| 求解器 / 摩擦锥 | Newton / pyramidal |

这些是 MuJoCo 接触求解参数，不是杨氏模量，也未经真实 ABS 材料或扣合实验标定。
外壳/桌面硬支撑与内部柔顺接触分别设置，不能通过软化桌面或增加允许穿透来获得通过。
参数含义见 [MuJoCo 文档](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters)。

## 执行示例

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
export MJ_BRIDGE_BENCHMARK_CONFIG="$PWD/examples/config/plastic_benchmark.yaml"
python -m mj_bridge.benchmark_cli run \
  --product examples/products/workbenchmark/tier2_task_001.yaml --seed 42 \
  --output-dir runs/plastic-example --connection-mode physics \
  --contact-profile plastic --executor oracle-baseline --robot-base-x=-.05 --headless
```

该配置允许3600秒单轮墙钟预算，供密集接触场景使用；一般配置默认预算为600秒。
超时按超时记录，不表示结构物理上不可装配。预算不改变物理步长、力或验收阈值。
整体抽样运行还受独立截止时间约束，详见[抽样指南](EXECUTION_STATE_MACHINE_ZH.md)。

## 底板与目标坐标

导入器只对成品和料盘分别执行统一的几何变换，保留各自内部的相对位置与角度。
塑料模式根据首层积木网格调整固定底板安装位置，单轴最多8 mm，
偏移写入 `assembly_fixture.position_offset_xy_m`；成品目标不因此改变。
首层无法对齐同一凸点网格时报告配置错误。回放必须使用相同底板安装位置。

完整导入变换、20 mm到19.2 mm层距映射和记录式布局见
[Workbenchmark 导入说明](WORKBENCHMARK_INTEGRATION_ZH.md)。

## 在手修正、插入与释放

Oracle 执行器根据实测积木姿态修正机械臂动作，保持双指实际承载检查。
空中修正只允许在无外部接触、双指接触有效且高度/倾斜受限时执行。
接近装配面后采用小步下降；需要补偿时逐步降低夹持预紧力，减少与扣合方向的干扰。
同层近邻和支撑边缘使用各自的受限交接分支，具体触发条件见
[状态机与运动约束](EXECUTION_STATE_MACHINE_ZH.md#运动与接触约束)。

顶部压实是明确的独立动作：先确认双指充分张开并脱离，再抬升、调整空夹爪，
从实际指尖碰撞体边界计算接近高度，以0.25 mm步进、最多4 mm行程压合。
空爪压实时保留3 mm总间隙并等待位置/速度稳定；撤离前再次消除持续下压目标并确认全开。
高位释放模式单独标为 `release_above_press`，不能混同直接插入。

两指松爪确认要求每指开度至少39.5 mm、速度低于2 mm/s、与目标间距大于0.1 mm，
连续稳定80 ms。开爪超时或接触不可观测时不撤离，也不继续下一块。

## 支撑与稳定性

坐实检查使用实际承重接触：刚性底面附近的接触位置、法线与法向力共同判断支撑。
若内筋摩擦承担重量而底面法向力不明显，则要求底面间隙小于20 µm，
来自底板、地面或其它积木的向上接触合力达到自重的80%以上，夹爪力不计入。
仅接近名义目标高度不等于已坐实。

待取料区积木可以休眠。抓取前唤醒目标，进入装配区前唤醒已装支撑，
释放与撤离阶段持续求解相关接触。最终唤醒全部积木并积分1秒，再检查逐块姿态。
休眠策略不直接移动积木；空接触列表只有在目标已唤醒且可观测时才有意义。

塑料模式采用XY 1 mm、Z 0.4 mm、倾斜3°和偏航8°门槛，并持续检查0.5 mm穿透上限。
局部部分支撑分支可要求更严格的交接姿态。视觉在手跟踪尚未单独验收，
不能把这条 Oracle 接触反馈路径的结果描述为视觉泛化能力。

## 轴向夹具诊断

```bash
python scripts/check_plastic_contact.py --output runs/plastic-contact-check.json
```

夹具固定下块，只允许上块沿插入轴运动，以3 N压入后撤力，再以2 N/s递增拉力，
最大12 N。该诊断检查坐实、保持与拔出行为，结果依赖尺寸、求解器和接触参数。
它不是自由放置、机器人全程装配或实物扣合力测量。
