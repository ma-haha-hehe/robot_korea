v0.14.0 - 2026-06-15
--------------------

### Host API

#### Changes

- Transformed fwupdate and fwlogger into examples
- Added rotate buttons to the different streams
- Added possibility to set the streams that should be opened for
  every new source
- Added possibility to zoom into the streams
- Added button to save the current configuration to a preset
- Added possibility to save current camera parameters into the persistent
  memory of the camera
- Several small adaptions of the viewer

#### Bugfixes

- Fixed a bug where, depending on which camera you removed from
  a multicamera system, the API crashed or not
- Fixed setting of RGB exposure time in the viewer
- Fixed memory leak when closing sources in the viewer

### Device Firmware

#### Changes

- Added Royale with Spectre 7.3.1
- Added support for CONFIG_SAVE message
- Improved device boot up time
- Added support for analog gain and white balance metadata
- Allow setting of min/max for RGB exposure, gain and white balance

#### Bugfixes

- Reduced UVC max packet size to 2048 to fix compatibility with WSL
- Fixed a bug that prevented the RGB exposure time to go above the max.
  exposure time used for 15fps

v0.13.0 - 2026-05-19
--------------------

### Host API

#### Changes

- Added access functions for the Extrinsics for Python
- Presets will be ordered according to their name
- Added recording/playback of configuration keys
- Some small changes for better touchscreen compatibility

#### Bugfixes

- Fixed intrinsics order in the calibration view of the viewer
- Fixed Python recording example
- Changed interpretation of extrinsics from JGR calibration

### Device Firmware

#### Changes

- Added Royale with Spectre 7.3
- Added watchdog implementation

#### Bugfixes

- Moved readout of JGR calibration to an earlier stage so that
  it is available on first boot after a new image was flashed

v0.12.0 - 2026-05-04
--------------------

### Host API

#### Changes

- Adapted ROS node example 
- Added alignment of 2D and 3D
- Added toggling of frustum and viewing presets
- Added pixel info for IR and RGB
- Updated documentation
- Added downloading of license information
- Updated maximum message size
- Added a warning in the viewer when USB2 is used
- Added parsing of local host presets
- Added rrf recording and downloading to the viewer for extended options

#### Bugfixes

- Fixed viewer crash when unplugging the camera
- Fixed playback behavior when looping was off

### Device Firmware

#### Changes

- Updated standard processing parameters
- Added collection of licensing information

#### Bugfixes

- Fixed version handling
- Fixed setting of USB speed after init

v0.11.0 - 2026-04-17
--------------------

### Host API

#### Changes

- Adapted range handling
- Added measure example
- Added readout of sensor serial
- Added point cloud topic to ROS1/2
- Added info if ultrasonic sensor is available

#### Bugfixes

- Fixed handling of extrinsics
- Fixed installation of udev rules file

### Device Firmware

#### Changes

- Added Royale with Spectre 7.2
- Added info about USB speed 

#### Bugfixes

- Fixed USB settings for 30fps
- Fixed config manager handling of bools

v0.10.0 - 2026-03-27
--------------------

### Host API

#### Changes

- Added new use case handling
- Added preset system
- Added Numpy output for the Python wrapper
- Improved the readout speed on Linux
- Added support for live logging and crash log readout from the device
- Implemented handling of RGB intrinsics and extrinsics
- Added source code version of Depth2PCL to remove dependency
- Adapted the intrinsics struct
- Various viewer improvements

#### Bugfixes

- Fixed a bug where metadata in bag files was delivered at the wrong times

### Device Firmware

#### Changes

- Added handling of second exposure time for HDR
- Added a password for the UART connection
- Stop the pipeline in case the device is disconnected but still powered
- Added support for logging to the host
- Reworked Royale implementation that is used to include RGB stream
- Updated to newest Royale/Spectre version
- Added readout of JGR information from the EEPROM to the file system
- Adapted the IMU configuration
- Added a reboot in case the middleware crashes
- Limited recording of debug rrf to a certain size

#### Bugfixes

- Fixed startup values in the JSON file
- Fixed rebooting into recovery mode

v0.9.0 - 2026-02-13
--------------------

### Host API

#### Changes

- Added white balance controls
- Some fixes in the viewer
- Added options to loop or not loop the playbacks and to changes
  the playback speed
- Added option to specify the FPS of a recording
- Added possibility to read out the device firmware version
- Adapted EEPROM header

### Device Firmware

#### Changes

- Added possibility to read out the device firmware version
- Added support for white balance controls
- Added support for coded modulation HDR use cases
- Fixed number of phases for HDR use cases
- Fixed memory leak in the device middleware
- Fixed race condition that happened when quickly connecting/disconnecting
- Updated parameters for use cases

v0.8.1 - 2025-11-28
--------------------

### Host API

#### Changes

- Fixed recording of bag files
- Fixed library search path under Linux

### Device Firmware

#### Changes

- Added option to enable/disable auto exposure

v0.8.0 - 2025-11-21
--------------------

### Host API

#### Changes

- Added firmware update functionality
- Added functionality to access the device EEPROM
- Added Numpy support for the Python wrapper
- Added support for Linux ARM 64Bit
- Added colorizer
- Added motion and pointcloud example
- Added ROS1/2 example node
- Switched to using DirectShow instead of Windows Media Foundation
- Added metadata to the recorded bag file
- Many smaller fixes and changes for the viewer application

### Device Firmware

#### Changes

- Added firmware update and recovery functionality
- Added firmware options manager
- Reduced the USB bandwidth that is requested

v0.7.0 - 2025-09-25
--------------------

### Host API

#### Changes

- Added new viewer application
- Added support for device metadata
- Added support for the EEPROM reading and writing
- Added support for exclusive camera access
- Added function to retrieve API version

### Device Firmware

#### Changes

- Updated to Royale 5.14.1
- Added frame numbers to the metadata
- Added sensor and system temperatures to metadata
- Added exposure times to metadata
- Added functions to read and write the EEPROM
- Added support for configuring the UVC bandwidth
- Adapted the VID/PID


v0.6.1 - 2025-07-07
--------------------

### Host API

#### Changes

- Fixed timestamps
- Added Python wrapper for Linux

### Device Firmware

#### Changes

- Fixed auto exposure
- Limited ultrasonic data to 2m


v0.6.0 - 2025-07-04
--------------------

### Host API

#### Changes

- Fixed usage of Depth2PCL
- Added support for multiple devices on one host
- Added a Python example for usage of multiple cameras
- Added the possibility to define the device which should be openend
- Fixed the installation of the Windows .lib
- Added custom slider for the ultrasonic data in the viewer

### Device Firmware

#### Changes

- Added Royale 5.14/Spectre 7
- Added possibility to record rrfs on the device
- Fixed timestamping
- Changed from 2 UVC streams to 1 UVC stream
- Switched to different standard use case
- Added support for disabling sensors for testing
- Changed RGB resolution to 1344*1008
- Reduced rootfs size
- Added a print of the backtrace when a segfault happens
