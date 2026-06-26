# O3P API Architecture

The O3P API provides access to configuration, control, and streaming of data from pmd 3D Time-of-Flight cameras. Developers can get started quickly with the high-level API, or take full control using the low-level device interface:

1. **High-Level Pipeline API**
   The Pipeline interface configures the O3P device with recommended settings and manages hardware resources and threading. It handles frame synchronization, automatic point cloud generation, and recording. Use this API when you need the camera to work without fine-tuning individual sensor settings or managing streaming threads. **Recommended for Application Developers.**

2. **Low-Level Device API**
   The Low-Level Device interface enables direct control of individual device sensors, fine-tuning of all camera settings, and management of streaming threads, calibration, and spatial mapping. **Recommended for Advanced Researchers, Framework and Tool Developers.**

> **Relation to RealSense:**
> The O3P API follows a layered architecture conceptually similar to [Intel's librealsense API](https://github.com/IntelRealSense/librealsense/blob/master/doc/api_arch.md). Both SDKs offer a high-level Pipeline for quick integration and a low-level Device/Sensor layer for full control. O3P extends this model with domain-specific features for pmd Time-of-Flight sensors, such as measurement range control, HDR modes, firmware presets, and plugin communication. The table below summarizes the correspondence:

| Concept | RealSense (librealsense) | O3P API |
|---|---|---|
| Device Discovery | `rs2::context` | `o3p::Context` |
| Streaming Orchestration | `rs2::pipeline` | `o3p::Pipeline` |
| Pipeline Configuration | `rs2::config` | `o3p::PipelineConfig` |
| Active Configuration | `rs2::pipeline_profile` | `o3p::PipelineProfile` |
| Hardware Abstraction | `rs2::device` | `o3p::Device` / `o3p::DeviceO3P` |
| Sensor Control | `rs2::sensor` | `o3p::Sensor` / `o3p::SensorComposite` |
| Stream Description | `rs2::stream_profile` | `o3p::Stream` / `o3p::VideoStream` |
| Frame Container | `rs2::frameset` | `o3p::FrameSet` |
| Depth Colorization | `rs2::colorizer` | `o3p::Colorizer` |
| Point Cloud Generation | `rs2::pointcloud` | `o3p::Depth2Pcl` (integrated into Pipeline) |
| Device Options | `rs2::options` | `o3p::OptionsInterface` / `o3p::Option` |
| Recording / Playback | `rs2::recorder` / `rs2::playback` | `o3p::Recording` / `o3p::DevicePlayback` |
| Device Information | `RS2_CAMERA_INFO_*` | `o3p::CameraInfo` enums |
| Firmware Update | N/A (separate tool) | `o3p::FwUpdateDevice` |
| Presets | JSON profiles | `o3p::Preset` (JSON-serializable) |

---

## High-Level Pipeline API

The `Pipeline` class is the primary entry point for most applications. It selects the best camera settings, acquires and activates the device, manages the different stream threads, and provides time-synchronized frames. The pipeline also provides access to the underlying low-level device, so sensor information and fine-tuning capabilities remain available.

### Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                    Application                      │
├─────────────────────────────────────────────────────┤
│              Pipeline (High-Level API)              │
│  ┌──────────┐ ┌──────────┐ ┌───────────────────┐    │
│  │  Config  │ │ Profile  │ │ Frame Acquisition │    │
│  │& Presets │ │& Streams │ │ & Point Cloud Gen │    │
│  └──────────┘ └──────────┘ └───────────────────┘    │
├─────────────────────────────────────────────────────┤
│              Device (Low-Level API)                 │
│  ┌──────────┐ ┌──────────┐ ┌───────────────────┐    │
│  │  Sensor  │ │ Options  │ │   Info / Calib    │    │
│  │Composite │ │ System   │ │   & Firmware      │    │
│  └──────────┘ └──────────┘ └───────────────────┘    │
├─────────────────────────────────────────────────────┤
│             Platform Abstraction Layer              │
│  ┌───────────────────┐  ┌───────────────────────┐   │
│  │ V4L2 / UVC (Linux)│  │ WinUSB / MF (Windows) │   │
│  └───────────────────┘  └───────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

### Pipeline Workflow

A typical application follows this pattern:

```cpp
#include <o3p/o3p.hpp>

int main() {
    // 1. Create and initialize the pipeline
    o3p::Pipeline pipeline;
    auto profile = pipeline.init();

    // 2. Start streaming
    pipeline.start();

    // 3. Acquire frames
    while (running) {
        const auto& frameSet = pipeline.waitForFrames();

        auto* depth = frameSet.getDepthFrame();        // Depth in mm
        auto* color = frameSet.getColorFrame();        // RGB image
        auto* pcl   = frameSet.getPointCloudFrame();   // Auto-generated XYZ
    }

    // 4. Stop and clean up
    pipeline.stop();
}
```

### Pipeline Components

#### PipelineConfig
Configures the pipeline before initialization. Supports:
- **Live camera**: `enableDevice(serialNumber)` to target a specific device
- **Playback**: `setPlaybackFile(path)` to stream from a ROS bag recording
- **Playback looping**: `setLoopPlayback(true/false)` to control whether playback restarts at end-of-file
- **Playback timing**: `setUseTimestampsPlayback(true/false)` to control whether frame delivery respects original recording timestamps
- **Disconnect handling**: `setDisconnectCallback(callback)` for USB disconnect events

#### PipelineProfile
Returned by `Pipeline::init()`, represents the active configuration:
- `getStreams()` — lists all active streams (depth, color, IR, etc.)
- `getSerialNumber()` — serial number of the connected device
- `getDevice()` — access to the underlying `Device` object

#### Frame Acquisition
`Pipeline::waitForFrames()` is a blocking call that returns a `FrameSet` containing all available frames for the current capture cycle. The pipeline automatically:
- Synchronizes frames across sensors
- Generates point clouds from depth data using `Depth2Pcl`
- Records frames if recording is active

#### Camera Parameters
The pipeline exposes high-level camera controls:
- **FPS**: `setFPS()` / `getFPS()` — frame rate control
  - `getAvailableFPS()` — frame rates valid with current settings; `getAllAvailableFPS()` — all supported frame rates
- **HDR**: `setHDR()` / `getHDR()` — High Dynamic Range mode
  - `getAvailableHDR()` — HDR modes valid with current settings; `getAllAvailableHDR()` — all supported HDR modes
- **Measurement Range**: `setMeasurementRange()` — measurement range identifier (string, e.g. `"short"`, `"mid"`)
  - `getMeasurementRange()` — current range in meters; `getAvailableMeasurementRanges()` / `getAllAvailableMeasurementRanges()`
- **Distance Offset**: `setDistanceOffset()` / `getDistanceOffset()` — calibration offset in meters
  - `getAvailableDistanceOffsets()` / `getAllAvailableDistanceOffsets()`
- **Presets**: `loadPreset()` / `getAvailablePresets()` — apply or query configuration presets
- **Atomic Parameter Setting**: `setCameraParameters(fps, hdr, range, distanceOffset)` — set all use-case parameters at once, avoiding intermediate invalid states; `checkCameraParameters()` validates a combination without applying it

#### Firmware & Device Interaction
- `sendPluginMessage(message)` — send a byte-vector to a device plugin and receive a reply
- `getConfigValue(config)` / `setConfigValue(config, value)` — read/write firmware configuration values by name
- `writeEeprom(value)` / `readEeprom()` — write/read the device EEPROM
- `getLicenseInfo()` — retrieve license information from the camera

#### Recording & Playback
- `startRecording(filename)` — record frames to a ROS bag file
- `stopRecording()` — stop recording
- `setRecordingFps(fps)` — limit the recording frame rate (0.0 = record all frames)
- `getNumberOfFramesInRecording()` — total frames in playback recording
- `getActiveProfile()` — retrieve the current `PipelineProfile` after initialization
- Playback via `PipelineConfig::setPlaybackFile()` using `DevicePlayback`

> **Comparison with RealSense:**
> Like `rs2::pipeline`, `o3p::Pipeline` abstracts device discovery, stream configuration, and frame delivery. Unlike RealSense, O3P integrates point cloud generation directly into the pipeline (rather than using a separate `rs2::pointcloud` processing block) and offers built-in ToF-specific controls (measurement range, HDR, use cases) that reflect pmd sensor capabilities.

---

## Low-Level Device API

O3P devices combine multiple sensors — typically a Time-of-Flight depth module, an optional RGB camera, and potentially IMU and ultrasonic sensors:

```
┌─────────────────────────────────────────┐
│              O3P Device                 │
│  ┌─────────────┐   ┌─────────────────┐  │
│  │  ToF Sensor │   │  RGB Sensor     │  │
│  │  (Depth+IR) │   │  (Color)        │  │
│  └─────────────┘   └─────────────────┘  │
│  ┌──────────────┐  ┌─────────────────┐  │
│  │  IMU Sensor  │  │  Ultrasonic     │  │
│  │  (Accel+Gyro)│  │  Sensor         │  │
│  └──────────────┘  └─────────────────┘  │
│  ┌───────────────────────────────────┐  │
│  │  Plugin Interface                 │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

The Low-Level Device API gives you direct control over these individual components.

### Device Hierarchy

```
          ┌─────────────┐
          │InfoInterface│
          └──────┬──────┘
                 │
          ┌────────────────┐
          │OptionsInterface│
          └──────┬─────────┘
                 │
          ┌──────┴───────┐
          │    Device    │  (abstract base)
          └──────┬───────┘
       ┌─────────┼────────────────┐
       │         │                │
┌──────┴───┐ ┌───┴──────────┐ ┌───┴──────────┐
│DeviceO3P │ │DevicePlayback│ │FwUpdateDevice│
│(Hardware)│ │ (Recording)  │ │  (Firmware)  │
└──────────┘ └──────────────┘ └──────────────┘
```

- **Device** — Abstract base class defining the streaming, configuration, and info interfaces.
- **DeviceO3P** — Implementation for live pmd hardware cameras. Manages a `SensorComposite` that handles multi-sensor data acquisition over USB.
- **DevicePlayback** — Plays back recorded ROS bag files, emulating a live device for offline development and testing.
- **FwUpdateDevice** — Specialized device for firmware updates with progress reporting.

### Sensor Architecture

```
        ┌────────────┐
        │   Sensor   │  (abstract base)
        └─────┬──────┘
              │
    ┌─────────┴───────────┐
    │  SensorComposite    │
    │  (Multi-Sensor Hub) │
    └─────────────────────┘
```

The `SensorComposite` manages composite frame reception from pmd devices that transmit multiple sensor data streams over a single USB endpoint. It handles:
- Buffer configuration for multiple data channels (ToF, RGB, IR, Plugin)
- Calibration data storage (intrinsics & extrinsics)
- USB disconnect detection and callback notification

Platform-specific sensor implementations handle the low-level video capture and communication:
- **Linux**: `V4l2Helper` (Video4Linux2 + udev monitoring for hot-plug), plus `LibUsbHelper` for non-UVC data transfer
- **Windows**: `WMFHelper` (Media Foundation) or `DSHelper` (DirectShow), plus `WinusbHelper` for non-UVC data transfer

> **Comparison with RealSense:**
> RealSense's `rs2::sensor` model treats each sensor independently with separate power management and streaming control. O3P's `SensorComposite` reflects the architecture of pmd ToF modules where multiple data types (depth, IR, RGB) are multiplexed in a single composite USB frame, requiring unified management rather than independent sensor control.

### Stream Types

Each sensor can produce one or more data streams:

| Stream Type | Description | Frame Type |
|---|---|---|
| `O3P_STREAM_DEPTH` | Distance data (mm) | `DepthFrame` |
| `O3P_STREAM_COLOR` | RGB image | `VideoFrame` |
| `O3P_STREAM_INFRARED` | IR amplitude image | `VideoFrame` |
| `O3P_STREAM_IMU` | Accelerometer + Gyroscope | `ImuFrame` |
| `O3P_STREAM_ULTRASONIC` | Ultrasonic range | `UltrasonicFrame` |
| `O3P_STREAM_PLUGIN` | Custom plugin data | `PluginFrame` |
| `O3P_STREAM_CONFIDENCE` | Per-pixel confidence | `VideoFrame` |
| `O3P_STREAM_POSE` | Device pose | — |

`VideoStream` extends `Stream` with spatial properties:
- **Resolution**: `width()`, `height()`
- **Intrinsics**: Focal length, principal point, and distortion model (`Intrinsics`)
- **Extrinsics**: Rotation and translation between streams (`Extrinsics`)

### Frame Data Model

```
         ┌───────────┐
         │   Frame   │  (base: timestamp + metadata)
         └─────┬─────┘
    ┌──────┬───┴───┬───────┬────────────┬────────────┐
    │      │       │       │            │            │
┌───┴──┐┌──┴───┐┌──┴──┐┌───┴───┐┌───────┴───┐┌───────┴──────┐
│Video ││Depth ││ IMU ││Ultra- ││Pointcloud ││   Plugin     │
│Frame ││Frame ││Frame││sonic  ││  Frame    ││   Frame      │
│      ││      ││     ││Frame  ││           ││              │
└──────┘└──────┘└─────┘└───────┘└───────────┘└──────────────┘
```

All frames carry:
- **Timestamp** — hardware timestamp (nanoseconds, via `getTimestamp()`)
- **Metadata** — per-frame information (frame number, exposure times, temperature) queryable via `supportsMetadata()` / `getMetadata<T>()`

**Key frame types:**

| Frame Type | Data Format | Key Methods |
|---|---|---|
| `VideoFrame` | Raw pixel buffer (8/16-bit) | `getWidth()`, `getHeight()`, raw data access |
| `DepthFrame` | `uint16_t*` (millimeters) | `getDistance(x, y)` returns meters |
| `PointcloudFrame` | `float*` XYZC (4 floats/pixel) | `getX(x,y)`, `getY(x,y)`, `getZ(x,y)`, `getConfidence(x,y)` |
| `ImuFrame` | 3-axis accel + gyro | `acceleration[3]`, `angularVelocity[3]` |
| `UltrasonicFrame` | Distance + signal + temp | `getDistance()`, `getAmplitude()`, `getSignalWidth()`, `getTemperature()`, `getTimestamp()` |
| `PluginFrame` | Opaque byte buffer | `getData()`, `getSize()` |

The `FrameSet` is the container returned by `Pipeline::waitForFrames()`. It aggregates all frames from a single capture cycle and provides typed accessors (`getDepthFrame()`, `getColorFrame()`, etc.).

> **Comparison with RealSense:**
> RealSense uses `rs2::frame` as a polymorphic base with `rs2::video_frame`, `rs2::depth_frame`, and `rs2::motion_frame`. O3P follows the same pattern with explicit C++ classes. O3P adds `UltrasonicFrame` and `PluginFrame` types specific to pmd sensor capabilities not present in RealSense.

---

## Options System

O3P provides a type-safe, observable option system for device configuration:

```
┌──────────────────┐     ┌──────────────────┐
│ OptionsInterface │◄────│     Device       │
│ (query + access) │     │  (implements)    │
└────────┬─────────┘     └──────────────────┘
         │
    ┌────┴─────┐
    │  Option  │  ──── value, range, description, observers
    └────┬─────┘
         │
  ┌──────┴────────┐
  │ BackendOption │  ──── communicates with firmware
  └───────────────┘
```

### Option Types

| Option | Description |
|---|---|
| `O3P_OPTION_TOF_ENABLE_AUTO_EXPOSURE` | Enable/disable ToF auto-exposure |
| `O3P_OPTION_TOF_MANUAL_EXPOSURE_TIME_1` | ToF exposure time (µs) |
| `O3P_OPTION_TOF_MANUAL_EXPOSURE_TIME_2` | ToF second exposure time (HDR) |
| `O3P_OPTION_COLOR_SCHEME` | Depth colorization scheme |
| `O3P_OPTION_COLOR_MIN_DEPTH` / `MAX_DEPTH` | Depth colorization range |
| `O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION` | Enable histogram equalization |
| `O3P_OPTION_RGB_ENABLE_AUTO_EXPOSURE` | RGB auto-exposure control |
| `O3P_OPTION_RGB_MANUAL_EXPOSURE_TIME_1` | RGB exposure time |
| `O3P_OPTION_RGB_ENABLE_AUTO_ANALOG_GAIN` | RGB auto-gain control |
| `O3P_OPTION_RGB_MANUAL_ANALOG_GAIN` | RGB manual analog gain value |
| `O3P_OPTION_RGB_ENABLE_AUTO_WHITE_BALANCE` | RGB auto white balance control |
| `O3P_OPTION_RGB_MANUAL_WHITE_BALANCE` | RGB manual white balance value |
| `O3P_OPTION_RGB_MANUAL_EXPOSURE_TIME_2` | RGB second exposure time (HDR) |

Each `Option` carries:
- **OptionValue** — Type-safe value (int, float, bool, or string)
- **OptionRange** — Abstract validation interface (`OptionValueRange`) with `isValid()` for checking values; concrete subclasses define range-specific constraints (e.g. numeric min/max/step or enumerated sets)
- **OptionDescription** — Human-readable name and description
- **Observer pattern** — UI components can register as `OptionObserver` for change notifications

`BackendOption` extends `Option` to communicate with the camera firmware via the `FirmwareOptionInterface`, transparently reading and writing hardware registers.

> **Comparison with RealSense:**
> RealSense uses `RS2_OPTION_*` enums with float-only values and min/max/step/default tuples. O3P's option system is richer: it supports multiple value types (int, float, bool, string), includes an observer pattern for reactive UI updates, and separates validation (`OptionRange`) and metadata (`OptionDescription`) into dedicated classes.

---

## Processing Utilities

### Colorizer (Depth Visualization)

The `Colorizer` converts raw depth data into RGB images for visualization. It supports multiple color schemes:

| Scheme | Description |
|---|---|
| `TURBO` | Google Turbo colormap (perceptually uniform) |
| `ROYALE` | pmd Royale-style gradient |
| `CLASSIC` | Traditional rainbow |
| `GRAYSCALE` / `INV_GRAYSCALE` | Monochrome depth mapping |
| `BIOMES` | Nature-inspired palette |
| `COLD` / `WARM` | Temperature-inspired palettes |
| `QUANTIZED` | Discrete color bands |
| `PATTERN` / `HUE` | Alternative visualizations |

The colorizer integrates with the option system — `O3P_OPTION_COLOR_SCHEME`, `O3P_OPTION_COLOR_MIN_DEPTH`, `O3P_OPTION_COLOR_MAX_DEPTH`, and `O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION` control its behavior.

### Depth2Pcl (Point Cloud Generation)

The `Depth2Pcl` class reprojects depth images into 3D point clouds using camera intrinsics:

```cpp
auto pcl = Depth2Pcl::create(lensParams, lensModel, rows, cols);
pcl->calculatePointCloud(depthData, outputXYZC);
```

- Input: `uint16_t*` (mm) or `float*` (meters) depth array
- Output: 4 floats per pixel (X, Y, Z in meters, C = confidence 0 or 1)
- Supports multiple lens models via `LensModelType`
- Automatically integrated into `Pipeline::waitForFrames()`

### Align (Color-to-Depth Alignment)

The `alignColorToDepth()` function aligns the RGB color frame to the depth frame resolution, so that every depth pixel has a corresponding color pixel:

```cpp
o3p::alignColorToDepth(frameSet, tofToRgbExtrinsics, rgbIntrinsics, tofIntrinsics);
```

- When a `PointcloudFrame` is available in the `FrameSet`, it is used directly for 3D positions; otherwise depth values are back-projected using the ToF intrinsics
- After calling this function, `frameSet.getColorFrame()` returns an aligned color frame with the same dimensions as the depth frame
- Extrinsics and intrinsics are obtained from the stream objects via `getExtrinsics()` and `getIntrinsics()`

---

## Preset System

Presets encapsulate complete device configurations as JSON:

```json
{
    "version": 1,
    "name": "Short Range Indoor",
    "fps": 30,
    "hdr": false,
    "measurement_range": 2,
    "distance_offset": 0,
    "config_values": { "key": "value" },
    "options": { "option_name": "value" }
}
```

`Preset::fromJson()` and `Preset::toJson()` enable serialization. `Pipeline::getAvailablePresets()` queries the device firmware for built-in presets, and `Pipeline::loadPreset()` applies them atomically.

---

## Recording & Playback

O3P records and plays back camera sessions using the **ROS bag** format, ensuring compatibility with the ROS ecosystem.

### Recording Topics

| Topic | Content |
|---|---|
| `/device_0/sensor_0/Depth_0/image/data` | Depth frames (16-bit, mm) |
| `/device_0/sensor_1/Color_0/image/data` | RGB frames |
| `/device_0/sensor_0/Ir_0/image/data` | Infrared frames |
| `/device_0/sensor_2/Accel_0/imu/data` | Accelerometer data |
| `/device_0/sensor_2/Gyro_0/imu/data` | Gyroscope data |
| `/device_0/sensor_3/US_0/range/data` | Ultrasonic distance |
| `/device_0/sensor_4/Plugin_0/data` | Plugin data |
| `/device_0/serial` | Device serial number |
| `/device_0/usecase` | Active use case |
| `/device_0/fps`, `range`, `hdr` | Camera parameters |

Playback via `DevicePlayback` emulates a live device, making it possible to develop and test applications without hardware present.

> **Comparison with RealSense:**
> Both SDKs use ROS bag files for recording. RealSense provides separate `rs2::recorder` and `rs2::playback` device wrappers, while O3P integrates `Recording` and `DevicePlayback` as a transparent device substitute.

---

## Device Information

The `InfoInterface` provides queryable device metadata:

| Info Key | Description |
|---|---|
| `O3P_CAMERA_INFO_SERIAL_NUMBER` | Device serial number |
| `O3P_CAMERA_INFO_CPU_ID` | Processor identifier |
| `O3P_CAMERA_INFO_CPU_CODE` | Processor code |
| `O3P_CAMERA_INFO_CPU_VERSION` | Processor version |
| `O3P_CAMERA_INFO_FW_VERSION` | Firmware version string |
| `O3P_CAMERA_INFO_FW_VERSION_BUILD` | Firmware build number |
| `O3P_CAMERA_INFO_FW_VERSION_DATE` | Firmware build date |
| `O3P_CAMERA_INFO_FW_VERSION_HASH` | Firmware source hash |
| `O3P_CAMERA_INFO_FW_VERSION_CUSTOMER` | Customer-specific version |
| `O3P_CAMERA_INFO_HAS_ULTRASONIC_SENSOR` | Whether an ultrasonic sensor is available (`"1"` or `"0"`) |
| `O3P_CAMERA_INFO_TOF_SENSOR_SERIAL` | ToF sensor serial number |

Device information is populated from on-device info blocks (`DeviceInfoV1`, `DeviceInfoV2`, `DeviceInfoV3`). Newer devices using `DeviceInfoV3` expose the ultrasonic sensor availability and ToF sensor serial fields.

---

## Firmware Logger

The `FwLogger` class provides access to the device's firmware log for advanced debugging. It is accessible via `Device::getFwLogger()`, which returns `nullptr` if the device does not support firmware logging.

---

## Device Discovery

The `Context` class provides device discovery:
- `Context::getAvailableCameras()` — returns a list of serial number strings for all connected cameras
- `Context::getConnectedDevices()` — returns a list of `std::shared_ptr<Device>` for all connected devices, ready for direct low-level use

The `Context` singleton is automatically initialized when `<o3p/o3p.hpp>` is included.

---

## Platform Support

| Platform | Video Capture | USB Communication |
|---|---|---|
| **Linux x86_64** | Video4Linux2 (V4L2) | libusb |
| **Linux ARM64** | Video4Linux2 (V4L2) | libusb |
| **Windows x86_64** | Media Foundation / DirectShow | WinUSB |

Hot-plug detection is supported on all platforms (udev on Linux, device notification on Windows).

---

## Language Bindings

- **C++** — Native API (header-only include via `<o3p/o3p.hpp>`)
- **Python** — SWIG-based bindings (`o3py`) with NumPy integration for zero-copy array access

---

## Getting Started

### Minimal C++ Example

```cpp
#include <o3p/o3p.hpp>
#include <iostream>

int main() {
    std::cout << "O3P API version: " << o3p::getApiVersionString() << std::endl;

    // Discover connected cameras
    const auto cameras = o3p::Context::getAvailableCameras();
    if (cameras.empty()) {
        std::cerr << "No cameras found." << std::endl;
        return 1;
    }

    // Initialize and start the pipeline
    o3p::Pipeline pipeline;
    auto profile = pipeline.init();
    pipeline.start();

    // Capture 100 frames
    for (int i = 0; i < 100; i++) {
        const auto& frames = pipeline.waitForFrames();

        if (auto* depth = frames.getDepthFrame()) {
            float centerDist = depth->getDistance(
                depth->getWidth() / 2, depth->getHeight() / 2);
            std::cout << "Center distance: " << centerDist << " m" << std::endl;
        }
    }

    pipeline.stop();
    return 0;
}
```

### Playback from Recording

```cpp
o3p::Pipeline pipeline;
o3p::PipelineConfig config;
config.setPlaybackFile("recording.bag");
config.setLoopPlayback(true);
config.setUseTimestampsPlayback(true); // deliver frames at original recording speed

auto profile = pipeline.init(config);
pipeline.start();
// ... use identically to live camera ...
```

---

## Additional Links

- [Examples](../examples/) — Sample applications (hello, OpenCV, point cloud, options, etc.)
- [ROS Bag Topics](rosbag_topics.md) — Detailed recording format documentation
- [Changelog](../Changelog.md) — Version history
