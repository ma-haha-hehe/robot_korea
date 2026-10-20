# Workbenchmark 导入与验证

Workbenchmark 的 400 个任务使用 `brick_2x2` 和 `brick_4x2` 两种零件，共 2009 个零件实例。原始任务保留在独立仓库；导入器不修改原文件。

## 导入原始散料布局

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/import_workbenchmark.py --repository ../Workbenchmark \
  --output runs/workbenchmark-recorded --initial-layout recorded
python -m mj_bridge.benchmark_cli run \
  --product runs/workbenchmark-recorded/tier1_task_001.yaml \
  --connection-mode physics --executor oracle-baseline --headless --robot-base-x=-.05 \
  --output-dir runs/workbenchmark-recorded-tier1
```

`recorded` 保留 `initial_blocks` 的世界坐标 XY 和朝向，按 ID 校验初始与成品的类型、颜色清单。源文件的统一 Z=0.0495 是旧模型的中心高度；导入器将其解释为桌面散料，运行时根据模型底面计算身体原点高度（当前 hollow 模型为 0.0586 m）。不接受未知高度或倾斜初始姿态。

成品只做平移：XY 包围盒中心移到装配区原点，最低目标 Z 移到零层。块间相对位置、朝向和 19.1 mm 层距均不改动。`import_manifest.json` 记录源提交、文件哈希和每个平移量。导入器拒绝覆盖不同内容的目标文件。

省略 `--initial-layout` 时保持 `seeded` 模式，使用随机散料，不能称为原始布局重放。`recorded` 模式下 seed 不改变散料布局。

## 验证的边界

- 初始布局验证逐块检查桌面范围、ID 完整性和二维轮廓重叠。通过不等于机械臂无碰撞可达。
- 成品几何检查与 0°/90° 抓取角度规划是静态检查，不是执行成功证据。
- `check_target_stability.py` 在目标位置直接初始化积木，再以重力静置。其结果只诊断目标与模型的稳定性，不能当作机器人装配结果。
- 正式执行必须同时满足末态位置、朝向、直立与接触穿透门槛。松爪后需要双指实际达到开度、速度稳定且脱离目标，才能撤离。
- 当前 hollow 刚体模型没有标定塑料弹性和扣合力。悬挑成品在该模型里倒落，不足以判定原成品设计错误。内部支撑筋实验仍未通过实际装配验收，不属于正式模型。

真实视觉 GPU 推理与实机操作未验收。不得把接口可用、静态检查或历史 snap 末态结果写成全任务物理装配成功。

## 外轮廓间隙

当前 hollow 模型的外轮廓为 31.8×31.8 mm 或 63.8×31.8 mm，
留出相邻块之间 0.2 mm 的名义间隙。16 mm 凸点节距和成品目标坐标不变。
此前外轮廓等于完整网格宽度，相邻目标没有任何间隙；约 50–70 µm 的
放置误差就能使下降中的长砖碰到邻块的硬边并翻转。该间隙是仿真近似，
不是实物测量或制造商 CAD。旧版零间隙静置统计不能用于新版模型验收。

## 工位基座位置

原始散料包含接近机械臂基座的点位。可用 `--robot-base-x=-.05` 将仿真基座
沿世界 X 轴后移 5 cm；默认仍为 0。初始积木和成品的世界坐标不变。
运行清单、结果和生成模型哈希记录这项配置，不能混合不同工位配置的结果。
此配置目前用于 baseline 执行器；外部 MoveIt 的坐标系尚未同步，因此 CLI
拒绝对 external 执行器使用非零偏移。工位配置不等于全部点位的可达性证明。

抓取前使用竖直路径接近。夹爪实测位置误差须不超过 1 mm，姿态误差不超过
2°，否则等待最多 1 秒后明确失败并取消闭爪。规划仍只选 0°/90°，在几何
安全的候选中优先选较小开口，方砖相同时保持原有 90° 优先。
