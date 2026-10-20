# 塑料扣合接触候选

`mj_bridge.plastic_contact` 为自建空心积木增加内筋、管壁过盈和 2 mm 导入斜面。扣合阻力来自柔顺接触与库仑摩擦，不创建焊接约束、不固定积木位姿。

`hollow_plastic_clutch_v2` 将每段管壁、内筋和导入斜面做成一个连续凸碰撞体，消除分离碰撞体接缝处的内部端面。旧 v1 场景可以直接回放已保存的 XML；重新生成须使用新版配置。

当前已接入正式生成和执行入口，通过 `--connection-mode physics --contact-profile plastic` 显式启用；通用 CLI 默认仍是原有松配合模型。日常 DUPLO 仿真推荐运行 `bash run_panda_duplo.sh`，它显式选择塑料卡扣及相应超时配置。参数没有实物测量标定，不能称为真实 ABS 材料模型。最新逐任务验收与失败对照见 [本轮审核记录](PANDA_DUPLO_AUDIT_20261014.txt)，不能推广为整个任务集已通过。

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

当前机械臂控制使用最多 4 mm 的反馈插入行程，要求高度误差小于 0.4 mm、倾斜不超过 3°，随后确认双指完全张开并解除目标接触。按倾斜状态卸压的主线已通过指定的三块、四块、八块、九块及两种十二块完整任务；八个指定复杂回归仍有三例失败，不能据此认定全部 Workbenchmark 通过。逐任务和随机布局结果见本轮审核记录。

## 执行与零件导出

```bash
python -m mj_bridge.benchmark_cli run --product runs/workbenchmark-recorded/tier2_task_001.yaml --seed 42 --output-dir runs/plastic-example --connection-mode physics --contact-profile plastic --executor oracle-baseline --robot-base-x=-.05 --headless
python scripts/export_workbenchmark_parts.py --repository ../Workbenchmark --output runs/plastic-parts --contact-profile plastic
```

密集装配请使用根目录 `run_panda_duplo.sh`；直接调用 CLI 时，可显式设置
`MJ_BRIDGE_BENCHMARK_CONFIG=$PWD/examples/config/plastic_benchmark.yaml`。
原有 3/5 的桌面/底板实体面滑动摩擦系数仍缺少实物标定依据；降低至 0.6 的整机对照发生退化，未直接替换默认值。
不要将接触稳定或夹具通过解释为材料参数已经真实化。

塑料接触仅允许与 `physics` 配合。生成清单和执行结果记录接触配置，执行结果另记录模块及实际生成模型的 SHA256。压入最多增加 4 mm 行程，横向偏差超过 1 mm 或倾斜超过 3°时停止；坐实高度误差须小于 0.4 mm。充分张开双指并解除目标接触的撤离条件保持不变。

桌面、地面和底板实体表面使用硬支撑接触；内筋与凸点之间保持柔顺接触。这样可避免内部扣合参数不当地软化桌面支撑。夹爪力限仍为每指 5 N。
