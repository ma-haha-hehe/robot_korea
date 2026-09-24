# 塑料扣合接触候选

`mj_bridge.plastic_contact` 为自建空心积木增加内筋、管壁过盈和 2 mm 导入斜面。扣合阻力来自柔顺接触与库仑摩擦，不创建焊接约束、不固定积木位姿。

`hollow_plastic_clutch_v2` 将每段管壁、内筋和导入斜面做成一个连续凸碰撞体，消除分离碰撞体接缝处的内部端面。旧 v1 场景可以直接回放已保存的 XML；重新生成须使用新版配置。

当前已接入正式生成和执行入口，通过 `--connection-mode physics --contact-profile plastic` 显式启用；默认仍是原有松配合模型。参数没有实物测量标定，不能称为真实 ABS 材料模型。整体自动装配尚未验收。

| 参数 | 当前值 |
|---|---:|
| 内筋与对齐凸点的名义过盈 | 0.05 mm |
| 内部滑动摩擦系数 | 0.3 |
| 扭转摩擦长度 | 0.001 m |
| 滚动摩擦长度 | 0.00005 m |
| 接触时间常数 / 阻尼比 | 0.03 s / 1 |
| 接触阻抗 `solimp` | 0.9 0.95 0.001 |

这些是 MuJoCo 求解器参数，不是杨氏模量。参数含义见 [MuJoCo 接触参数说明](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters)。外壳硬接触及原有夹爪力限幅保持独立，不能通过增大允许穿透来接受失败插入。

夹具与机器人场景均使用 Newton 求解器、pyramidal 摩擦锥、0.5 ms 步长。椭圆与棱锥摩擦锥具有不同的柔性接触动力学；参数验证须使用相同配置，详见 [MuJoCo 接触模型](https://mujoco.readthedocs.io/en/latest/computation/)。结果报告包含实际求解器设置。

## 轴向夹具验证

```bash
source enter_sim_env.sh
export PYTHONPATH="$PWD/src/mj_bridge:$PYTHONPATH"
python scripts/check_plastic_contact.py --output runs/plastic-contact-check.json
```

夹具固定下块，限制上块的侧向与转动自由度，仅沿插入轴施加力。3 N 压入后撤力，再以 2 N/s 递增拔出力，最大 12 N。积木位置由动力学积分得到。它用于接触模型诊断，不能代替自由放置或机械臂验证。

本机v2 夹具结果：2×2 / 2×4 均完全坐实；撤力后高度误差小于 3 µm；对应拔出力为 3.0 / 5.9 N，最大内部接触形变约 0.05 mm。这些值只适用于该夹具、尺寸和求解器配置，不能当作实物测量值。

机械臂隔离候选使用最多 4 mm 的反馈插入行程，要求高度误差小于 0.4 mm、倾斜不超过 3°，随后依旧确认双指完全张开并解除目标接触。三块任务已通过这一候选检查；正式入口的三块任务已通过；较复杂任务仍在验证，不能据此认定全部 Workbenchmark 通过。

## 执行与零件导出

```bash
python -m mj_bridge.benchmark_cli run --product runs/workbenchmark-recorded/tier2_task_001.yaml --seed 42 --output-dir runs/plastic-example --connection-mode physics --contact-profile plastic --executor oracle-baseline --robot-base-x=-.05 --headless
python scripts/export_workbenchmark_parts.py --repository ../Workbenchmark --output runs/plastic-parts --contact-profile plastic
```

塑料接触仅允许与 `physics` 配合。生成清单和执行结果记录接触配置，执行结果另记录模块及实际生成模型的 SHA256。压入最多增加 4 mm 行程，横向偏差超过 1 mm 或倾斜超过 3°时停止；坐实高度误差须小于 0.4 mm。充分张开双指并解除目标接触的撤离条件保持不变。

桌面、地面和底板实体表面使用硬支撑接触；内筋与凸点之间保持柔顺接触。这样可避免内部扣合参数不当地软化桌面支撑。夹爪力限仍为每指 5 N。

## 底板安装与放置反馈

导入器将成品整体平移到装配区，保持积木间的相对位置和角度。有些成品居中后会偏离底板凸点网格半格。塑料模式根据首层积木网格调整固定底板的安装位置，单轴最多移动 8 mm；成品目标和零件初始位置不变。偏移写入 `assembly_fixture.position_offset_xy_m`，回放时使用同一安装位置。首层不能落在同一网格上时直接报告配置错误。

最终插入对支撑面积不足 75% 的上层积木减速。Oracle 模式还在每个物理步监测宽边抓取的倾斜：积木离目标高度超过 20 mm 且倾斜超过 1° 时暂停原下降轨迹，根据实际手中位姿校正，再以不超过 2 mm 的步长下降。恢复阶段保留 3° 倾斜限制和最终坐实检查；视觉模式尚未验证手中跟踪，不能使用这项 Oracle 反馈作为视觉验收证据。

桌面接触保留 1 µm 的碰撞检测边距，以避免共面凸碰撞体在本机求解器中出现错误的深接触。该边距不改变零件尺寸、目标位置或允许穿透上限。
