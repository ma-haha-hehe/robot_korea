# Legacy: RealSense + 写死手眼标定 备份(已弃用)

> **这是相机换成 ifm O3P 之前、基于 Intel RealSense + 手动写死手眼外参的那一整套的备份。**
> 仅作存档/参考。**当前 pipeline 已不用这些**(见最下面"映射到新 O3P 方案")。
> 备份时间: 2026-06-27。

---

## 1. 这套是什么 / 为什么弃用

之前视觉相机是 **Intel RealSense**,手眼标定那一套有两个特点:

1. **靠 RealSense SDK 直接抓图**(`pyrealsense2`)。
2. **手眼外参的平移是"写死"的手量值**,不是标定出来的——因为当时标定结果不可靠(标出来 `[0.010, -0.065, 0.277]m` 把物体算到桌子底下了),只好保留标定的旋转、平移改用粗略手量 `[0.030, 0.000, 0.030]m`。这就是"写死手眼"。

**弃用原因**: 相机换成 **ifm O3P ToF**。
- RealSense 抓图脚本(`calib_grab.py` 用 `pyrealsense2`)在 O3P 上**直接报错**。
- `calib_append.py` 把 **RealSense 内参写死**(`fx≈605, cx≈318 @640×480`),O3P 内参完全不同。
- O3P 是**鱼眼**且有独立的深度/彩色内参,需要全新的标定链。

---

## 2. 文件清单

| 文件 | 作用 | 为什么对 O3P 不能用 |
|---|---|---|
| `calib_grab.py` | 容器内用 `pyrealsense2` 抓一帧彩色、检测棋盘 | **直接依赖 RealSense 硬件/SDK**,O3P 没有 |
| `calib_append.py` | 解析 print_pose 的 T_base_tool,配对图像写 samples.yaml | **写死 RealSense 内参** `CAM_MATRIX=[[605.29,...]]` |
| `calib_capture.sh` | 采一个标定样本(抓图+读机器人位姿+配对) | 调用上面 RealSense 版 `calib_grab.py` |
| `calib_run.sh` | 跑 `handeye_calibrate.py` 做标定(tsai) | 标定器本身相机无关,但喂的是 RealSense 内参/帧 |
| `calib_apply.sh` | 把标定结果应用到 `camera_to_tool.yaml`(自动备份) | 通用,新方案也复用了它 |
| `handeye_calibrate.py` | 通用手眼标定器: 棋盘/charuco 检测 → solvePnP → calibrateHandEye | 通用,但用 RealSense 针孔内参、无鱼眼/无深度系组合 |
| `old_camera_to_tool.yaml` | **旧的手眼结果**(RealSense): `T_tool_camera` 平移=写死的 `[0.03,0,0.03]`、旧 RealSense 内参 | 相机/安装已变 |
| `old_vision_execution_bridge.yaml` | **旧的换算配置**: 含**过期的** `observe_T_base_tool`、`pick_pre_base_xyz`、为补 base/base_link 的 `flip_x/y=true`、旧 `pick_offset/final_pick_offset/place` 等 | observe/pregrasp 关节改过、相机换了,这些值全过期 |

### 旧手眼值(写死的,存档参考)
`old_camera_to_tool.yaml` 里 `T_tool_camera` 平移 = `[0.03, 0.00, 0.03] m`(手量,非标定);旋转是 RealSense 时代标定的;内参 `fx≈571, cx≈292`(RealSense 彩色)。

---

## 3. 映射到新的 O3P 方案(当前在用的)

| 旧(本备份, RealSense) | 新(当前, O3P) |
|---|---|
| `calib_grab.py`(pyrealsense2) | `src/my_robot_vision/my_robot_vision/calib_grab_o3p.py`(读 grabber 的 `color_hi.png`) |
| `calib_append.py`(写死RS内参) | `src/my_robot_vision/calib_append_o3p.py`(从 `camera_params.json` 读 O3P 内参) |
| `calib_capture.sh` | `calib_capture_o3p.sh` |
| `calib_run.sh` + `handeye_calibrate.py` | `calib_run_o3p.sh` + `handeye_calibrate_o3p.py`(鱼眼去畸变 + 组合回深度系 + 4×6棋盘自动纠180°翻转) |
| `calib_apply.sh` | 复用同一个 |
| 写死平移 `[0.03,0,0.03]` | 重标得 `T_tool_camera` 平移 `[55.6,-11.4,25.9]mm`, std 1-3mm |
| 过期 `observe_T_base_tool`/`pick_pre_base_xyz` | 已用当前 observe 位实测 print_pose 重写 |

新方案详见仓库根的 `calib_*_o3p.sh` 和 `src/my_robot_vision/.../*_o3p.py`,以及记忆 `o3p-handeye-calibration`。

---

## 4. 如果要回退到 RealSense(不建议)
1. 物理换回 RealSense 相机。
2. 把 `old_camera_to_tool.yaml` / `old_vision_execution_bridge.yaml` 拷回 `src/my_robot_vision/config/`。
3. 用本目录的 `calib_*.py/.sh` 重新采样标定(需 `pyrealsense2` + 容器内 RealSense)。
