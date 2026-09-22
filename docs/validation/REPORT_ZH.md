# 仿真验证报告

记录时间（UTC）：2026-09-22T18:06:50.841353+00:00

代码回归：69 项通过。主产品测试：87/87 成功。
补充测试（按下表独立产品名称统计）：7/7 成功。

范围：Oracle 观测、MuJoCo 实体夹爪、显式 snap 扣合。未验证实机。
视觉推理未验证：缺少 CUDA 环境及 SAM/FoundationPose 权重。RGB-D 渲染已单独验证。

## 原目标存在问题的历史产品

这些产品保留原始目标，未计入装配成功数量。带 `_supported` 后缀的替代设计是独立目标，其成功不代表原目标成功。

| 产品 | 几何问题 |
|---|---|
| final_product | 支撑不足、层高不匹配 |
| final_product2 | 支撑不足、层高不匹配 |
| final_product_burger | 支撑不足、实体重叠 |
| final_product_door | 支撑不足、层高不匹配、实体重叠 |
| final_product_flower | 支撑不足、层高不匹配 |
| final_product_simple | 支撑不足、层高不匹配 |
| final_product_t | 实体重叠 |

## 实际执行结果

| 产品 | Seed | 结果 |
|---|---:|---|
| final_product2_supported | 42 | 成功 |
| final_product_2x4x2_orange_top | 0 | 成功 |
| final_product_2x4x2_orange_top | 17 | 成功 |
| final_product_2x4x2_orange_top | 42 | 成功 |
| final_product_3x2x4_2x2top | 0 | 成功 |
| final_product_3x2x4_2x2top | 17 | 成功 |
| final_product_3x2x4_2x2top | 42 | 成功 |
| final_product_battery | 0 | 成功 |
| final_product_battery | 17 | 成功 |
| final_product_battery | 42 | 成功 |
| final_product_blue_single | 0 | 成功 |
| final_product_blue_single | 17 | 成功 |
| final_product_blue_single | 42 | 成功 |
| final_product_burger2 | 0 | 成功 |
| final_product_burger2 | 17 | 成功 |
| final_product_burger2 | 42 | 成功 |
| final_product_burger_supported | 42 | 成功 |
| final_product_carrot | 0 | 成功 |
| final_product_carrot | 17 | 成功 |
| final_product_carrot | 42 | 成功 |
| final_product_door_supported | 42 | 成功 |
| final_product_estop | 0 | 成功 |
| final_product_estop | 17 | 成功 |
| final_product_estop | 42 | 成功 |
| final_product_flower_supported | 42 | 成功 |
| final_product_green_white_stack | 0 | 成功 |
| final_product_green_white_stack | 17 | 成功 |
| final_product_green_white_stack | 42 | 成功 |
| final_product_hammer | 0 | 成功 |
| final_product_hammer | 17 | 成功 |
| final_product_hammer | 42 | 成功 |
| final_product_icecream | 0 | 成功 |
| final_product_icecream | 17 | 成功 |
| final_product_icecream | 42 | 成功 |
| final_product_magnet | 0 | 成功 |
| final_product_magnet | 17 | 成功 |
| final_product_magnet | 42 | 成功 |
| final_product_orange_green_blue_white_simple | 0 | 成功 |
| final_product_orange_green_blue_white_simple | 17 | 成功 |
| final_product_orange_green_blue_white_simple | 42 | 成功 |
| final_product_real_simple | 0 | 成功 |
| final_product_real_simple | 17 | 成功 |
| final_product_real_simple | 42 | 成功 |
| final_product_simple_supported | 42 | 成功 |
| final_product_small_tree | 0 | 成功 |
| final_product_small_tree | 17 | 成功 |
| final_product_small_tree | 42 | 成功 |
| final_product_stack_3x_2x4 | 0 | 成功 |
| final_product_stack_3x_2x4 | 17 | 成功 |
| final_product_stack_3x_2x4 | 42 | 成功 |
| final_product_supported | 42 | 成功 |
| final_product_t_supported | 42 | 成功 |
| final_product_tier1_single_2x4 | 0 | 成功 |
| final_product_tier1_single_2x4 | 17 | 成功 |
| final_product_tier1_single_2x4 | 42 | 成功 |
| final_product_tier2_stack_3x_2x4 | 0 | 成功 |
| final_product_tier2_stack_3x_2x4 | 17 | 成功 |
| final_product_tier2_stack_3x_2x4 | 42 | 成功 |
| final_product_tier3_bridge_plus_red_yellow | 0 | 成功 |
| final_product_tier3_bridge_plus_red_yellow | 17 | 成功 |
| final_product_tier3_bridge_plus_red_yellow | 42 | 成功 |
| final_product_tier3_white_bridge_3x_2x4 | 0 | 成功 |
| final_product_tier3_white_bridge_3x_2x4 | 17 | 成功 |
| final_product_tier3_white_bridge_3x_2x4 | 42 | 成功 |
| final_product_tier4_bridge_plus_red_yellow | 0 | 成功 |
| final_product_tier4_bridge_plus_red_yellow | 17 | 成功 |
| final_product_tier4_bridge_plus_red_yellow | 42 | 成功 |
| final_product_trafficlight | 0 | 成功 |
| final_product_trafficlight | 17 | 成功 |
| final_product_trafficlight | 42 | 成功 |
| final_product_wb_tier4_007 | 0 | 成功 |
| final_product_wb_tier4_007 | 17 | 成功 |
| final_product_wb_tier4_007 | 42 | 成功 |
| final_product_white_blue_green_stack | 0 | 成功 |
| final_product_white_blue_green_stack | 17 | 成功 |
| final_product_white_blue_green_stack | 42 | 成功 |
| final_product_white_blue_orange | 0 | 成功 |
| final_product_white_blue_orange | 17 | 成功 |
| final_product_white_blue_orange | 42 | 成功 |
| final_product_white_bridge_3x_2x4 | 0 | 成功 |
| final_product_white_bridge_3x_2x4 | 17 | 成功 |
| final_product_white_bridge_3x_2x4 | 42 | 成功 |
| final_product_white_bridge_plus_red_yellow | 0 | 成功 |
| final_product_white_bridge_plus_red_yellow | 17 | 成功 |
| final_product_white_bridge_plus_red_yellow | 42 | 成功 |
| final_product_white_cross_tower_6x_2x4 | 0 | 成功 |
| final_product_white_cross_tower_6x_2x4 | 17 | 成功 |
| final_product_white_cross_tower_6x_2x4 | 42 | 成功 |
| final_product_white_ladder_5x_2x4 | 0 | 成功 |
| final_product_white_ladder_5x_2x4 | 17 | 成功 |
| final_product_white_ladder_5x_2x4 | 42 | 成功 |
| final_product_white_orange_blue_stack | 0 | 成功 |
| final_product_white_orange_blue_stack | 17 | 成功 |
| final_product_white_orange_blue_stack | 42 | 成功 |

[完整数据、逐块评分、实际末态及初始 manifest](report.json)

这些结果只覆盖记录的产品、种子和连接模式，不构成任意随机布局的成功保证。
