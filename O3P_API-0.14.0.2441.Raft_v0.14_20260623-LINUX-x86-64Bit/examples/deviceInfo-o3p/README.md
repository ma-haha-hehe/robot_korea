# Tutorial - "DeviceInfo" Example

This tutorial explains the "DeviceInfo" sample for the O3P, which retrieves and prints all device information exposed by the API.

## Installation
This example does not require any additional libraries beyond the O3P API.

## Starting
Connect the O3P device to your computer and execute the executable file.

## Code Overview
The example first lists all cameras found by `o3p::Context::getAvailableCameras()`, then initializes a pipeline with the default configuration and iterates over every `o3p::CameraInfo` value to print the ones supported by the connected device.

```cpp
const auto cameras = o3p::Context::getAvailableCameras();
for (auto i = 0u; i < cameras.size(); ++i) {
    std::cout << "Camera " << i << " : " << cameras[i] << std::endl;
}

o3p::Pipeline p;
o3p::PipelineProfile profile = p.init();

for (auto i = 0; i < o3p::O3P_CAMERA_INFO_COUNT; ++i) {
    o3p::CameraInfo infoType = static_cast<o3p::CameraInfo>(i);
    if (profile.getDevice()->supportsInfo(infoType)) {
        std::cout << o3p::cameraInfoToString(infoType) << " : "
                  << profile.getDevice()->getInfo(infoType) << std::endl;
    }
}
```

Typical output includes the device serial number, firmware version, hardware revision, and other model-specific properties.
