# o3p_api 0.14.0.2441

Host API for the O3P camera — providing access to configuration, control, and streaming of data from pmd 3D Time-of-Flight cameras.

## Package Contents

| Directory / File | Description |
|---|---|
| `bin/` | Shared library (`o3p.dll` / `libo3p.so`), example executables, and tools |
| `lib/` | Import / static library (`o3p.lib` on Windows) |
| `lib/cmake/` | CMake integration files (`o3pConfig.cmake`, `o3pConfigVersion.cmake`, `o3pTargets.cmake`) |
| `include/o3p/` | Public C++ headers |
| `doc/html/` | This API reference documentation |
| `examples/` | Example source code with a ready-to-use `CMakeLists.txt` |
| `python/` | Python wrapper module (O3PY) |
| `udev/` | Linux udev rules for device access (Linux packages only) |
| `ThirdPartySoftware.txt` | Third-party license notices |
| `Changelog.md` | Release history |

## Prerequisites

- **CMake** >= 3.24
- **C++ compiler** — Visual Studio 2022 (MSVC v143) on Windows, GCC on Linux

Optional, required only for specific examples and tools:

- **OpenCV** — required for the `opencv-o3p` example

## Integrating the Library into Your Project

The package ships with CMake config files so you can consume it with a standard `find_package` call.

```cmake
find_package(o3p REQUIRED)

target_link_libraries(your_target PRIVATE o3p::o3p)
```

Point CMake at the package by setting `o3p_DIR` to the `lib/cmake` directory inside this package:

```
cmake -Do3p_DIR=/path/to/package/lib/cmake ..
```

Headers are in `include/o3p/`. The main entry point is `o3p/Pipeline.hpp` for the high-level Pipeline API, or `o3p/Context.hpp` for the low-level Device API.

## API Overview

The API offers two levels of access:

- **High-Level Pipeline API** — configures the device with recommended settings, manages threading and frame synchronization. Recommended for application developers.
- **Low-Level Device API** — direct control of individual sensors, fine-tuning of camera settings, and management of streaming threads. Recommended for advanced users.

See the [API Reference](index.html) for the full class documentation.

## Building the Examples

The `examples/` directory contains a self-contained `CMakeLists.txt` that builds all examples against this installed package.

```bash
cd examples
mkdir build && cd build
cmake -Do3p_DIR=../../lib/cmake ..
cmake --build .
```

Available examples:

| Example | Description |
|---|---|
| `hello-o3p` | Minimal example — connect, stream one frame, disconnect |
| `measure-o3p` | Distance measurement with confidence filtering |
| `pointcloud-o3p` | Real-time 3D point cloud visualization |
| `opencv-o3p` | Depth frame integration with OpenCV |
| `recording-o3p` | Record and play back frame sequences |
| `motion-o3p` | IMU motion data streaming |
| `multiDevice-o3p` | Streaming from multiple devices simultaneously |
| `deviceInfo-o3p` | Query device information and capabilities |
| `options-o3p` | Read and write device options |

## Tools

Pre-built tools are located in `bin/`:

| Tool | Description |
|---|---|
| `o3pViewer` | Qt-based viewer for live depth and point cloud visualization |
| `o3p-fw-update` | Firmware update utility |
| `o3p-fw-logger` | Firmware log capture utility |

## Python Wrapper

See the `python/` directory for the module and usage examples.

## Linux: udev Rules

On Linux, install the udev rules to allow non-root access to the device:

```bash
sudo cp udev/11-o3p.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## License

See [ThirdPartySoftware.txt](ThirdPartySoftware.txt) for third-party license terms.
