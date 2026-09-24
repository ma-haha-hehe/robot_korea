# 纯物理装配验收（开发中）

2026-09-23 的可视化复查发现旧夹爪明显穿透积木。旧报告中的装配成功率
只衡量目标末态，不能证明夹持接触正确。此次修订采用独立的接触验收，
不沿用旧版 94 轮的成功结论。

## 控制与接触

- 每指驱动力上限 5 N，伺服目标变化速度上限 0.06 m/s。
- 物理步长 0.5 ms，指尖采用高阻抗接触；夹持位置上移 6 mm，避开下层凸点。
- 放置时不再额外下压 2 mm；松爪后由重力使积木落座。
- 每个物理步检查积木与其他物体、机械臂与环境的接触深度。纯物理执行超过 0.5 mm 立即停止并判失败，即使末态正确。
- 在手位姿由 Oracle 测量后通过机械臂运动校准，不直接修改积木位姿。从目标上方 40 mm 开始精细放置。真实视觉在手跟踪尚未实现验收，不能以 Oracle 校准冒充视觉。
- `physics` 路径不调用积木吸附或固定。关节由执行器驱动，积木仅由接触、重力运动。

MuJoCo 接触为数值软约束。0.5 mm 是目前明确的验收上限，不是“绝对零穿透”的承诺。
驱动力上限也不是瞬时碰撞力上限；碰撞峰值受惯性和约束影响。

## 纯物理积木

`--connection-mode physics` 使用 `hollow_primitives_v1`，由顶板、四面侧壁、
底部管状结构与顶部凸点构成。积木本体高度 19.2 mm，外侧壁厚 1.5 mm、
顶板厚 2 mm。2×2 与 4×2 的质量分别为 12 g、22 g；惯量使用外包围盒近似。

该模型留有装配间隙，用于刚体重力放置，不包含塑料弹性、过盈配合或真实扣合力。
尺寸、摩擦、惯量未经实物测量标定，因此不能称为实物等效模型。

产品 YAML 中的相对目标坐标不变。空腔可容纳凸点，首层应落在底板主体表面，
其世界坐标参考高度由底板表面 46 mm、本体半高 9.6 mm 和体原点偏移 9 mm
推导为 64.6 mm。旧实心/snap 模型的 68.5 mm 参考高度只适用于旧模型。
场景清单保存所用模式和几何版本，不能混用两种模型的结果。

## 命令与证据

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python -m mj_bridge.benchmark_cli run \
  --product examples/products/catalog/final_product_hammer.yaml \
  --seed 42 --headless --executor baseline --connection-mode physics \
  --output-dir runs/physics-hammer
```

最新统一版本回归目录：`runs/physics-release-catalog`，覆盖几何审计通过的29个原始产品与种子0/17/42。以完整摘要和逐轮结果为准；运行中或部分成功不能作为发布验收。

查看交互演示：

```bash
MUJOCO_GL=glfw LP_NUM_THREADS=2 python scripts/view_physics_episode.py \
  --product examples/products/catalog/final_product_hammer.yaml --seed 42
```

可视化预览与自动回归证据分开保存。发布前必须核实最新批次状态；旧版94轮snap结果不能替代新版纯物理验收。
