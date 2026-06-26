# O3P ROS Sample

This package provides a ROS node that publishes depth, IR, RGB, point cloud, and IMU data from an O3P device.

The package is compatible with ROS 1 (tested with Noetic) and ROS 2 (tested with Humble).

## Workspace layout

The sample is delivered as a standard ROS workspace:

```
ros_ws/
    src/
        o3p_node/     ← the ROS package (CMakeLists.txt + package.xml)
    README.md
```

## 1. The Node

### ROS 2 (Humble)

**Prerequisites**
- ROS 2 Humble (sourced in your shell: `source /opt/ros/humble/setup.bash`)
- CMake ≥ 3.24
- OpenCV 4
- `tf2_ros` and `geometry_msgs` (`sudo apt install ros-humble-tf2-ros ros-humble-geometry-msgs`)
- O3P API installed

**Build**
```bash
cd <O3P_INSTALL_DIR>/examples/ros_ws
colcon build --cmake-args -DO3P_DIR="<O3P_INSTALL_DIR>"
source install/setup.bash
```

**Run**
```bash
ros2 launch o3p_node camera_node.launch.py
# or directly:
ros2 run o3p_node camera_node
```

The node accepts the following parameters:

| Parameter                 | Default  | Description                                                                                                   |
|---------------------------|----------|---------------------------------------------------------------------------------------------------------------|
| `serial_no`               | `""`     | Serial number of the camera to open. Empty selects the first available camera.                                |
| `camera_name`             | `camera` | Prefix used for all topics and TF frames, so several cameras can be told apart.                               |
| `recording`               | `""`     | Path to a recording (`.bag`) to play back instead of opening a live camera. Empty uses a live camera.         |
| `loop`                    | `true`   | Whether to loop the playback once the end of the recording is reached.                                        |
| `use_timestamps`          | `true`   | Whether to delay frame delivery according to the recorded timestamps.                                         |
| `wait_for_device_timeout` | `-1`     | Seconds to wait for the camera at startup. Negative = wait indefinitely, `0` = exit immediately if not found. |
| `reconnect_timeout`       | `-1`     | Seconds to keep retrying after a USB disconnect. Negative = retry indefinitely, `0` = no reconnect.           |

Open a specific camera and give it a custom name:
```bash
ros2 launch o3p_node camera_node.launch.py serial_no:=<SERIAL> camera_name:=front
# or directly:
ros2 run o3p_node camera_node --ros-args -p serial_no:=<SERIAL> -p camera_name:=front
```

Play back a recording instead of a live camera:
```bash
ros2 launch o3p_node camera_node.launch.py recording:=/path/to/recording.bag
# or directly:
ros2 run o3p_node camera_node --ros-args -p recording:=/path/to/recording.bag
```

**Multiple cameras**

Use the multi-camera launch file to open several cameras at once. When no serial numbers are
supplied, **all connected cameras are discovered and opened automatically** (one node per
camera). Optionally provide matching names, otherwise `camera1`, `camera2`, ... are used:
```bash
# Open all connected cameras automatically:
ros2 launch o3p_node multi_camera.launch.py

# Open specific cameras and give them names:
ros2 launch o3p_node multi_camera.launch.py serial_nos:=<SERIAL_A>,<SERIAL_B> camera_names:=front,back
```
Each camera then publishes under its own prefix, e.g. `/front/frame_depth` and `/back/frame_depth`.
List the available serial numbers with `o3pViewer` or the `deviceInfo` example.

---

### ROS 1 (Noetic)

**Prerequisites**
- ROS 1 Noetic (sourced in your shell: `source /opt/ros/noetic/setup.bash`)
- CMake ≥ 3.24
- OpenCV 4
- O3P API installed

**Build**
```bash
mkdir -p ~/catkin_ws/src
cp -r <O3P_INSTALL_DIR>/examples/ros_ws/src/o3p_node ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make -DO3P_DIR="<O3P_INSTALL_DIR>"
source devel/setup.bash
```

**Run**
```bash
# Start the ROS master in a separate terminal first:
roscore

# Then in another terminal:
rosrun o3p_node camera_node
```

The node accepts the private parameters `serial_no` (default `""`, empty selects the first
camera) and `camera_name` (default `camera`, prefix for topics and TF frames). To open a
specific camera under a custom name:
```bash
rosrun o3p_node camera_node _serial_no:=<SERIAL> _camera_name:=front
```
Start several instances with different `_serial_no` / `_camera_name` values to stream from
multiple cameras at once.

To play back a recording instead of a live camera, set the private parameter `recording` to the
path of a `.bag` file. The playback behaviour can be tuned with `loop` (default `true`) and
`use_timestamps` (default `true`):
```bash
rosrun o3p_node camera_node _recording:=/path/to/recording.bag
```

---

## 2. The Rviz Panel

A custom Rviz panel is built by default alongside the node. It provides a quick view of RGB, IR, and depth frames plus IMU data.

When several camera nodes are running, the panel shows a **Camera** combobox at the top listing every camera that is currently publishing (by its `camera_name`). Selecting a camera switches all views, the IMU readout, and the *Enable AI* control to that camera. The list refreshes automatically as cameras appear or disappear.

Similar to the O3PViewer, the panel also offers **Preset**, **FPS**, **HDR** and **Range** comboboxes to change the camera use case at runtime. Each combobox shows the current state of the selected camera and the available options. Changing a value applies it to the device; if a combination is rejected by the firmware the comboboxes revert to the state the camera actually uses. The controls reflect the camera the panel is currently connected to and update automatically when the camera state changes.

**ROS 2**
```bash
source <O3P_INSTALL_DIR>/examples/ros_ws/install/setup.bash
rviz2
```
In Rviz2: click **Panels → Add New Panel** and select **O3PRvizPanel**.

**ROS 1**
```bash
cd ~/catkin_ws && source devel/setup.bash
rviz
```
In Rviz: click **Panels → Add New Panel** and select **O3PRvizPanel**.

To visualize the point cloud:
- Set **Fixed Frame** to `map` (or `<camera_name>_frame`)
- In the **Displays** panel click **Add → PointCloud2**
- Set **Topic** to `/camera/frame_pc` and **Color Transformer** → **Axis** → `Z`

The point cloud is published in `<camera_name>_depth_optical_frame`, which is linked
into the TF tree below `map`, so it appears upright (Z up, X forward) instead of being
rotated. See [§3 Coordinate frames (TF)](#3-coordinate-frames-tf) for the full tree.

### Point cloud visualization in rviz step by step

#### ROS 2 (rviz2)

1. Start the node: `ros2 run o3p_node camera_node`
2. In a second terminal: `rviz2`
3. In the **Global Options** section of the **Displays** panel, set **Fixed Frame** to `map`.
4. Click **Add** (bottom-left of Displays panel) → **By topic** tab → select `/camera/frame_pc [PointCloud2]` → **OK**.
5. In the newly added **PointCloud2** display:
   - **Reliability Policy**: `Best Effort` (matches the publisher)
   - **Color Transformer**: `AxisColor`, **Axis**: `Z`, or `Intensity` using the `conf` field
   - **Size (m)**: `0.005` – `0.01` works well for most scenes
   - **Style**: `Points` or `Flat Squares`
6. The view should open looking from the **front** of the scene (camera facing you). If the
   cloud appears rotated, check that **Fixed Frame** is `map` and that `ros2 topic echo /tf_static`
   shows the `camera_depth_optical_frame` entry.

To also add the RGB overlay:
- **Add → By topic → `/camera/frame_rgb` → Image** to get a 2-D RGB preview.
- For a combined 3-D RGB point cloud, use the **PointCloud2** display with **Color Transformer** → `RGB8`
  (requires the cloud to carry colour — the current node publishes XYZ + confidence only, so `AxisColor` is recommended instead).

#### ROS 1 (rviz)

1. Start `roscore`, then `rosrun o3p_node camera_node`.
2. In a second terminal: `rviz`
3. Under **Global Options**, set **Fixed Frame** to `map`.
4. Click **Add → By topic → /camera/frame_pc → PointCloud2 → OK**.
5. In the **PointCloud2** display set **Color Transformer** to `AxisColor`, **Axis** to `Z`.
6. The point cloud should appear with Z pointing up and the camera facing forward.

> **Tip – default view angle:** rviz starts with a perspective camera looking along −Y of
> the fixed frame. Because the node publishes `map` at the origin with X forward, the
> initial view looks at the scene from the **side**. To get a front view, either:
> - In rviz press `F` (focus on selected) after clicking a point in the cloud, or
> - Use the **Views** panel → set **Type** to `XYOrbit` and reset the view with the **Zero**
>   button, then orbit to taste.

## 3. Coordinate frames (TF)

The node publishes a TF tree that follows the ROS conventions (REP-103/REP-105). Every prefix is the `camera_name` parameter (default `camera`).

```
map
└── <camera_name>_frame                        (camera link; REP-103 body: X forward, Y left, Z up)
    ├── <camera_name>_depth_frame              (identity w.r.t. the link)
    │   └── <camera_name>_depth_optical_frame  (optical: X right, Y down, Z forward)
    └── <camera_name>_color_frame              (offset by the device ToF→RGB extrinsics)
        └── <camera_name>_color_optical_frame  (optical: X right, Y down, Z forward)
```

- **Body frames** (`*_frame`) use the REP-103 convention (X forward, Y left, Z up).
- **Optical frames** (`*_optical_frame`) use the camera convention (X right, Y down,
  Z forward). All image and point-cloud data is expressed in an optical frame, since
  that is the convention in which the device produces the data.
- The body→optical rotation is the fixed transform RPY = (−π/2, 0, −π/2), quaternion
  (x, y, z, w) = (−0.5, 0.5, −0.5, 0.5).
- The `<camera_name>_frame → <camera_name>_color_frame` transform is derived from the
  device ToF→RGB extrinsics (reported column-major), so the color and depth sensors are
  placed at their real physical offset.

Which data is published in which frame:

| Data                                             | Frame                              |
|--------------------------------------------------|------------------------------------|
| `frame_pc` (point cloud)                         | `<camera_name>_depth_optical_frame`|
| `frame_depth`, `depth/image_raw` + `camera_info` | `<camera_name>_depth_optical_frame`|
| `frame_ir`                                       | `<camera_name>_depth_optical_frame`|
| `aligned_depth_to_color/*`                       | `<camera_name>_depth_optical_frame`|
| `frame_rgb` + `camera_info`                      | `<camera_name>_color_optical_frame`|
| `imu/accel`, `imu/gyro`                          | `<camera_name>_imu`                |

In ROS 2 the static transforms are published once on `/tf_static` (latched). In ROS 1
the tree is re-broadcast on `/tf` with every frame.

> **Note:** the node publishes the `map → <camera_name>_frame` transform itself so the
> sample is self-contained. When integrating into a larger system that already provides
> a `map` (or world) frame, remove or remap this transform to avoid two publishers for
> the same edge.

---

## 4. Published topics

All topics are prefixed with the `camera_name` parameter (default `camera`).

| Topic                                               | Type                                  |
|-----------------------------------------------------|---------------------------------------|
| `/<camera_name>/frame_depth`                        | `sensor_msgs/Image` (BGR8, colorized) |
| `/<camera_name>/depth/image_raw`                    | `sensor_msgs/Image` (16UC1, mm)       |
| `/<camera_name>/depth/camera_info`                  | `sensor_msgs/CameraInfo`              |
| `/<camera_name>/aligned_depth_to_color/image_raw`   | `sensor_msgs/Image` (16UC1, mm)       |
| `/<camera_name>/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo`              |
| `/<camera_name>/frame_pc`                           | `sensor_msgs/PointCloud2`             |
| `/<camera_name>/frame_ir`                           | `sensor_msgs/Image` (BGR8)            |
| `/<camera_name>/frame_rgb`                          | `sensor_msgs/Image` (BGR8)            |
| `/<camera_name>/camera_info`                        | `sensor_msgs/CameraInfo`              |
| `/<camera_name>/imu/accel`                          | `sensor_msgs/Imu`                     |
| `/<camera_name>/imu/gyro`                           | `sensor_msgs/Imu`                     |
| `/<camera_name>/serial_number`                      | `std_msgs/String` (latched)           |
| `/<camera_name>/metadata`                           | `std_msgs/String` (JSON)              |
| `/<camera_name>/parameters`                         | `std_msgs/String` (latched)           |
| `/<camera_name>/set_parameters`                     | `std_msgs/String`                     |
| `/diagnostics`                                      | `diagnostic_msgs/DiagnosticArray`     |

The `camera_info` and `depth/camera_info` topics carry the RGB and depth camera
intrinsics respectively. `depth/image_raw` publishes the raw 16-bit depth values in
millimetres. `aligned_depth_to_color/image_raw` is the depth image reprojected onto
the color camera plane (same resolution as the RGB frame) — every pixel corresponds
to the same 3-D point as the matching RGB pixel.

The `parameters` topic carries the current use case parameters (preset list, fps, hdr,
range) and the available options as a latched `std_msgs/String`, formatted as one
`key=value` pair per line (list values are `|`-separated). It is what feeds the rviz
panel comboboxes. To change a parameter, publish a single `key=value` command on
`set_parameters` (`key` is one of `preset`, `fps`, `hdr`, `range`); the node applies it
and re-publishes the updated `parameters` state:
```bash
# ROS 2
ros2 topic pub --once /camera/set_parameters std_msgs/msg/String '{data: "fps=15"}'
# ROS 1
rostopic pub -1 /camera/set_parameters std_msgs/String '{data: "hdr=1"}'
```

---

## 5. Retrieving the camera serial number

The node exposes the serial number of the connected device in several ways:

1. **As a parameter** – after the node connects, the actual serial number is written
   back to the `serial_no` parameter, so it can be read at runtime:
   ```bash
   # ROS 2
   ros2 param get /o3p_node serial_no
   # ROS 1
   rosparam get /camera_publisher/serial_no
   ```
   The same parameter can also be used to *select* a specific camera at start-up
   (see the parameter table above).

2. **On a latched topic** – the serial number is published once on the latched
   `/<camera_name>/serial_number` (`std_msgs/String`) topic. Subscribers that connect
   later still receive it:
   ```bash
   # ROS 2
   ros2 topic echo /camera/serial_number
   # ROS 1
   rostopic echo /camera/serial_number
   ```

3. **Via the `device_info` service** – returns the serial number together with the
   other device information (firmware version, etc.):
   ```bash
   # ROS 2
   ros2 service call /camera/device_info std_srvs/srv/Trigger
   # ROS 1
   rosservice call /camera/device_info
   ```

4. **On the `/diagnostics` topic** – the serial number is published as the
   `hardware_id` of the diagnostic status entries (`diagnostic_msgs/DiagnosticArray`):
   ```bash
   # ROS 2
   ros2 topic echo /diagnostics
   # ROS 1
   rostopic echo /diagnostics
   ```

The rviz panel additionally displays the serial number (**S/N**) of the currently
selected camera next to the camera combobox, reading it from the latched
`serial_number` topic.

---

## 6. Available services

All services are prefixed with the `camera_name` parameter (default `camera`).

| Service                      | Type               | Description                                                |
|------------------------------|--------------------|------------------------------------------------------------|
| `/<camera_name>/enable_ai`   | `std_srvs/SetBool` | Enable or disable AI post-processing.                      |
| `/<camera_name>/device_info` | `std_srvs/Trigger` | Returns serial number and other device info as a string.   |
| `/<camera_name>/hw_reset`    | `std_srvs/Empty`   | Stops, re-initialises and restarts the pipeline.           |

**Examples (ROS 2)**
```bash
# Enable AI
ros2 service call /camera/enable_ai std_srvs/srv/SetBool '{data: true}'
# Query device info
ros2 service call /camera/device_info std_srvs/srv/Trigger
# Hardware reset
ros2 service call /camera/hw_reset std_srvs/srv/Empty
```

**Examples (ROS 1)**
```bash
rosservice call /camera/enable_ai '{data: true}'
rosservice call /camera/device_info
rosservice call /camera/hw_reset
```

---

## 7. Automatic reconnection

If the camera is disconnected via USB while the node is running, the node catches the
disconnect exception and re-initialises the pipeline automatically.

- **`reconnect_timeout`** (default `-1`): a negative value means the node keeps trying
  indefinitely, which is the recommended setting for deployed robots. Set to `0` to
  disable automatic reconnect entirely. A positive value is the maximum number of
  seconds to keep retrying before the node exits.

```bash
# Retry for up to 30 seconds, then exit:
ros2 run o3p_node camera_node --ros-args -p reconnect_timeout:=30.0
```

## 8. Wait-for-device at startup

If the camera is not connected when the node starts, the `wait_for_device_timeout`
parameter controls how long to wait:

- **`-1`** (default): wait indefinitely.
- **`0`**: exit immediately if the camera is not found.
- Positive value: maximum seconds to wait before exiting.

```bash
# Wait up to 60 s for the camera, then exit:
ros2 run o3p_node camera_node --ros-args -p wait_for_device_timeout:=60.0
```

## 9. Per-frame metadata

Every depth frame triggers a JSON message on `/<camera_name>/metadata`
(`std_msgs/String`). The fields available depend on firmware; a typical message looks
like:

```json
{"timestamp":1718000000000000000,"frame_number":42,"exposure_time_1":1000,"exposure_time_2":500,"temperature":42.5}
```

Field names follow the O3P `MetadataType` enum: `timestamp` (ns), `frame_number`,
`exposure_time_1` / `exposure_time_2` (µs), `temperature` (°C), `analog_gain`, `white_balance`.

