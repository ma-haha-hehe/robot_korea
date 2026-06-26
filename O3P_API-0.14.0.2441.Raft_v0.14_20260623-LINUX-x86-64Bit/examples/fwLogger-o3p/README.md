# Tutorial - "Firmware Logger" Example

This tutorial explains the "fwLogger" sample for the O3P, which demonstrates how to retrieve firmware logs from a connected device.

## Installation
This example does not require any additional libraries beyond the O3P API.

## Starting
Connect the O3P device to your computer and execute the executable file:

```
fwLogger-o3p [options]
```

### Options
- `-l, --list-devices` — List connected devices that support firmware logging
- `-s, --serial <serial>` — Target a specific device by serial number
- `-c, --collect-crash-logs` — Collect stored crash logs instead of live logs
- `-o, --out <file>` — Write logs to a file instead of stdout
- `-h, --help` — Print usage information

## Code Overview
The example connects to the first available device (or one specified by serial number), obtains an `o3p::FwLogger` instance, and either streams live logs or collects stored crash logs.

### Live logging
```cpp
fwLogger->startCollecting([&](const char *log) {
    std::cout << log << std::endl;
});
std::cin.get(); // Press Enter to stop
fwLogger->stopCollecting();
```

### Crash log collection
```cpp
fwLogger->collectCrashLogs([&](const char *log) {
    std::cout << log << std::endl;
});
```

Both modes support redirecting output to a file via the `-o` option.
