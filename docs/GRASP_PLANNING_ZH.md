# 抓取方向与装配顺序规划

规划核心为 `mj_bridge.assembly_planner`，采用 assembly-by-disassembly：
从成品中逐块选出可向上取出的积木，检查0°/90°夹爪方向，再反转步骤得到装配顺序。
旧 `panda_pick/src/myplanner.py` 适配器的 `tasksh` 导出也使用这个核心。

检查使用积木旋转后的轮廓与夹指从闭合到张开的扫掠范围。
上方投影重叠会阻挡取出；两种方向都不可用时返回无解。
重复 ID 与实体重叠的目标被拒绝，不能通过忽略相邻块获得可行抓取。

## 角度约定

- `grasp_spin_deg`：成品坐标系中的夹爪方向，仅0°或90°；当前成品坐标系与世界轴向相同。
- `grasp_offset_rad`：夹爪相对积木方向，等于规划方向减去成品积木方向。
- 散料抓取方向：观测散料方向加上 `grasp_offset_rad`。
- 放置夹爪方向：`grasp_spin_deg` 转为弧度，不能与积木自身方向混用。

例如成品积木朝向90°、计划夹爪方向0°、散料朝向30°，
抓取夹爪应为-60°，放置夹爪为0°；积木随夹爪旋转60°，最终到达90°。

旧 `tasksh` 格式保留 `grasp_spin`（度）、`blueprint_yaw`（弧度）、
`pick/place.orientation`（xyzw）并提供 `grasp_offset_rad`。
源积木位置必须来自所用观测，旧适配器的默认 `pick.pos` 不是视觉定位结果。

## 受限搜索与释放模式

规划优先保留直接拆解逆序，对小规模结构可搜索可达拆解状态，减少宽边夹持。
当直接放置的夹爪通道受阻时，`plan_with_release_above` 可以选择明确标记的
`release_above_press`，在上方松爪后从顶部压合；该路径使用不同的释放几何，
不能描述为可逆直接抓取。执行中仍需通过释放、接触和坐实验收。

支撑链评分和抓取通道检查是几何启发式，不证明动态稳定、全身路径可达或真实抓持力足够。
更大结构采用有界规划，避免状态搜索的指数开销。

## 生成计划

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/plan_assembly.py examples/products/catalog/final_product_hammer.yaml \
  --output runs/hammer-angle-plan.json
```

完整执行还会保存 `assembly_plan.json`。控制器变化、初态变化或动作模式变化后，
应重新评测；仅计划存在或角度相同不能解释为物理执行成功。
