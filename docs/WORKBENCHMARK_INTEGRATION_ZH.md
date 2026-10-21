# Workbenchmark 导入与验证

Workbenchmark 使用 `brick_2x2` 和 `brick_4x2` 两种零件。任务保留在独立的 [benchmark_lego 仓库](https://github.com/ma-haha-hehe/benchmark_lego)；导入器不修改源文件。最新版包含 400 个任务，各 Tier 100 个。旧数据与最新版分别使用以下导入入口。

## 导入最新版三文件任务

最新版使用 `versions/latest/tasks/<类别>/<实例>/` 下的 `final_product.yaml`、`scene_init.yaml`、`metadata.yaml`。在本仓库根目录运行，将 `--tasks-root` 替换为 benchmark_lego 本地副本中的实际路径：

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/import_workbenchmark_latest.py \
  --tasks-root ../benchmark_lego/versions/latest/tasks \
  --output runs/workbenchmark-latest
python scripts/import_workbenchmark_latest.py \
  --output runs/workbenchmark-latest --verify-task two_brick_stack_001.yaml
python -m mj_bridge.benchmark_cli run \
  --product runs/workbenchmark-latest/two_brick_stack_001.yaml \
  --connection-mode physics --contact-profile plastic --executor oracle-baseline \
  --headless --robot-base-x=-.05 --output-dir runs/workbenchmark-latest-tier1
```

导入器检查三文件清单、显式单位、直角朝向、20 mm 层距和元数据，并保留源文件快照及 SHA-256。目标 XY 相对底层整体包络中心平移，源数据的 20 mm 层索引显式映射到仿真模型的 19.2 mm 层距；XY 尺度和每块砖的相对朝向不变。初始布局整体平移到仿真料盘，不逐块重排。该几何适配须与原始任务结果分别记录。

输出目录必须不存在。运行前执行 `--verify-task`，可检测导入产物、冻结快照及仍可访问的源任务是否变化；源任务不可访问时会明确标记仅使用冻结快照。导入通过只证明数据转换和完整性，不证明物理装配成功。

## 导入旧版原始散料布局

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/import_workbenchmark.py --repository ../benchmark_lego \
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
- 可选 plastic 模式使用柔顺内筋、凸点干涉和六维摩擦接触，参数尚未由实物 ABS 标定。松配合 hollow 与 plastic 模式的证据分别记录。各模式的能力与评测口径见[能力范围](RELEASE_STATUS_ZH.md)。

真实视觉 GPU 推理与实机操作未验收。不得把接口可用、静态检查或历史 snap 末态结果写成全任务物理装配成功。

## 外轮廓间隙

当前 hollow 模型的外轮廓为 31.8×31.8 mm 或 63.8×31.8 mm，
留出相邻块之间 0.2 mm 的名义间隙。16 mm 凸点节距和成品目标坐标不变。
该间隙用于表示相邻零件的制造间隙，属于仿真近似，
不是实物测量或制造商 CAD。几何版本不同的结果需要分别评测。

## 工位基座位置

原始散料包含接近机械臂基座的点位。可用 `--robot-base-x=-.05` 将仿真基座
沿世界 X 轴后移 5 cm；默认仍为 0。初始积木和成品的世界坐标不变。
运行清单、结果和生成模型哈希记录这项配置，不能混合不同工位配置的结果。
此配置目前用于 baseline 执行器；外部 MoveIt 的坐标系尚未同步，因此 CLI
拒绝对 external 执行器使用非零偏移。工位配置不等于全部点位的可达性证明。

抓取前使用竖直路径接近。夹爪实测位置误差须不超过 1 mm，姿态误差不超过
2°，否则等待最多 1 秒后明确失败并取消闭爪。规划仍只选 0°/90°，在几何
安全的候选中优先选较小开口，方砖相同时保持原有 90° 优先。
