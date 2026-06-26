# Tutorial - "Firmware Update" Example

This tutorial explains the "fwUpdate" sample for the O3P, which demonstrates how to perform a firmware update on a connected device without using the viewer application.

## Installation
This example does not require any additional libraries beyond the O3P API.

## Starting
Connect the O3P device to your computer and execute the example with the path to the firmware file:

```
fwUpdate-o3p <path_to_firmware_file>
```

## Code Overview
The example performs a firmware update in the following steps:

1. Read the firmware binary file into memory using `o3p::readFileToStdVector`.
2. Find the first connected device that supports firmware updates via the `o3p::Updatable` interface.
3. Enter recovery mode by calling `enterUpdateState()` on the device.
4. Wait for the device to re-enumerate in recovery mode as an `o3p::FwUpdateDevice`.
5. Perform the update by calling `update()` with a progress callback.

```cpp
auto fwData = o3p::readFileToStdVector<uint8_t>(firmwareFile);

o3p::Context &ctx = o3p::Context::getInstance();
for (auto &device : ctx.getConnectedDevices()) {
    if (auto updatable = std::dynamic_pointer_cast<o3p::Updatable>(device)) {
        updatable->enterUpdateState();
        break;
    }
}

// After device re-enumerates in recovery mode:
updateDevice->update(fwData, [](const float progress) {
    std::cout << "\rUpdate progress: " << static_cast<int>(progress) << "%" << std::flush;
});
```

After the update completes, the device automatically reboots into normal operation mode with the new firmware.
