# 物理模型与执行验收

`--connection-mode physics` 禁止自动扣合，积木由重力、接触和夹爪作用运动。
参考执行器驱动机器人关节，不通过修改积木位姿完成抓取、搬运或释放。

## 几何与接触配置

松配合 `hollow_primitives_v1` 由顶板、侧壁、底部管和顶部凸点构成；
塑料配置 `hollow_plastic_clutch_v2` 增加柔顺内筋与过盈接触，详见
[塑料接触说明](PLASTIC_CONTACT_ZH.md)。

积木本体高19.2 mm，外侧壁厚1.5 mm、顶板厚2 mm；
2×2、4×2 的名义质量分别为12 g、22 g，惯量采用包围盒近似。
模型不是厂商 CAD，也未经实物尺寸、摩擦和材料标定。

首层高度由底板表面、积木几何与身体原点共同推导，不能直接套用旧实心/snap 模型的参考高度。
场景清单保存连接模式、接触配置、几何与工位信息；这些配置变化后结果应分开记录。

## 控制约束

- 物理步长0.5 ms；每指指令驱动力限制为5 N，夹爪伺服目标变化速度上限0.06 m/s。
- IK 与关节段检查机器人自碰撞和手臂/环境碰撞，转移受阻时只尝试有界替代路径。
- 合爪前检查实测工具位姿，抬升前后检查双指真实接触，丢失抓持后停止。
- Oracle 姿态补偿通过机械臂运动修正手中积木，接触交接和释放保留独立确认。
- 每个物理步监测接触；超过0.5 mm的积木、夹爪或机器人环境/自接触穿透会停止执行。

MuJoCo 接触是数值软约束，0.5 mm 是验收上限，不是数学上的零穿透。
5 N 是驱动指令上限，也不等于瞬时碰撞力峰值上限。离散碰撞检查不是完整的连续路径保证。

## 塑料执行的成功条件

完整任务需满足：全部目标积木按计划执行，双指实际全开并与目标脱离，
放置后具有真实支撑证据，最终全部积木唤醒后继续积分1秒，仍通过逐块验收。
末态门槛为 XY 1 mm、Z 0.4 mm、倾斜3°、按零件对称性计算的偏航8°。

同时要求执行过程中接触约束有效，不能只凭最后一帧位置正确判成功。
休眠物体的空接触列表不能证明已松爪或有支撑，因此执行器会主动唤醒目标与支撑，
并确认相关接触可观测。详见[状态机](EXECUTION_STATE_MACHINE_ZH.md)。

## 运行

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python -m mj_bridge.benchmark_cli run \
  --product examples/products/catalog/final_product_hammer.yaml \
  --seed 42 --headless --executor oracle-baseline --connection-mode physics \
  --output-dir runs/physics-hammer
```

交互查看同一目标：

```bash
MUJOCO_GL=glfw LP_NUM_THREADS=2 python scripts/view_physics_episode.py \
  --product examples/products/catalog/final_product_hammer.yaml --seed 42
```

预览用于观察行为；正式评测使用固定输入、自动运行和独立报告。
静态摆放、局部恢复重放和简化 snap 运行都不能替代完整物理装配。
