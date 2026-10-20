# 抓取角度规划

规划核心为 `mj_bridge.assembly_planner`，沿用原 `panda_pick/src/myplanner.py`
的 assembly-by-disassembly 流程：从成品中选择可以向上取出的积木，检查
90°、0° 两种夹爪方向，记录拆解步骤；反转步骤得到装配顺序。
原 `myplanner.py` 的 `tasksh` 导出和仿真执行器共用这个核心。

检查使用积木的旋转后轮廓以及手指从闭合到张开的扫掠范围，不再只判断
相邻积木中心是否位于同一条线上。上方重叠的积木会阻挡取出。
两种方向都不可用时返回规划失败，不强制抓取。重复 ID 和实体重叠的目标也会拒绝。

## 角度约定

- `grasp_spin_deg`：成品坐标系中的夹爪方向，只取 0° 或 90°。当前仿真的成品坐标系与世界坐标系轴向相同。
- `grasp_offset_rad`：夹爪相对积木的角度，等于规划夹爪角度减去成品积木角度。
- 散料抓取角度：观测到的散料角度加上 `grasp_offset_rad`。
- 放置夹爪角度：`grasp_spin_deg` 转为弧度。夹爪方向与积木自身方向不能混用。

例如成品积木朝向90°、规划夹爪方向0°、散料朝向30°时，抓取夹爪方向应为-60°，
放置夹爪方向为0°；积木随夹爪转过60°，最终朝向90°。

旧 `tasksh` 格式仍保留 `grasp_spin`（度）、`blueprint_yaw`（弧度）以及
`pick/place.orientation`（xyzw），并增加 `grasp_offset_rad`。
源零件位置来自实际观测；旧适配器中的默认 `pick.pos` 不是视觉定位结果。

## 只生成计划

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/plan_assembly.py examples/products/catalog/final_product_hammer.yaml \
  --output runs/hammer-angle-plan.json
```

静态检查覆盖原始36个设计及7个独立修正版：40个能够生成角度计划，3个因实体
重叠而拒绝。29个原始几何有效设计及7个独立修正版均通过角度复核。
另4个原始设计虽有角度解，仍有其他几何问题，不能据此认定结构有效。
本地逐任务证据为 `runs/grasp-angle-planning/report.json`。

角度检查采用当前Panda手指尺寸的平面包络，并留0.5mm间隙。它不验证机械臂
全身路径、动态接触、支撑稳定性或真实夹持力；这些不属于本轮角度规划验收。
