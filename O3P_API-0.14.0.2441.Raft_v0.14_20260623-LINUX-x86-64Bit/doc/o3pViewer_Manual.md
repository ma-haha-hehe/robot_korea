# O3P Viewer - User Manual

The O3P Viewer is a graphical demonstration and diagnostics application built on top of
the O3P API. It allows you to connect to an O3P device, visualize its depth, infrared,
RGB and IMU streams, inspect device information, record and replay data, and tune
camera parameters.

This manual describes the user-visible features of the viewer.

---

## 1. Getting started

1. Connect an O3P device to your computer (USB).
2. Launch `o3pViewer` (`o3pViewer.exe` on Windows).
3. On the welcome screen you have two choices:
   - **Start Camera** - connect to the device and start streaming.
   - **Replay** - open a previously recorded `.bag` file and play it back.

You can also drag a `.bag` file onto the application window to start playback directly.

### 1.1 Command line options

The viewer accepts a few command line options:

| Option                      | Description                                             |
| --------------------------- | ------------------------------------------------------- |
| `-e`, `--extended`          | Start the viewer in **extended mode** (see below).      |
| `-b <file>`, `--bag <file>` | Open the given `.bag` recording immediately on startup. |
| `-h`, `--help`              | Print the list of available options and exit.           |

Example - start the viewer in extended mode on Windows:

```cmd
o3pViewer.exe --extended
```

Example - open a recording in extended mode:

```cmd
o3pViewer.exe -e -b C:\recordings\scene.bag
```

On Linux the executable is invoked the same way:

```bash
./o3pViewer --extended
```

**Extended mode** unlocks features that are hidden in the default UI, including the
*Extended Options* and *Record RRF* buttons in the top toolbar, the *Advanced
Settings* entry in the source's *More* menu, and the *Extended Range (7m)* range
option. It is intended for advanced users and developers.

## 2. Main window layout

- **Top toolbar** - global actions (Add Source, Extended Options, Record RRF,
  Settings, Help).
- **Left panel (Source view)** - one tab per connected device or open recording.
  Contains all camera controls and stream selection for that source. A small
  arrow button on the right edge of the panel collapses or expands it.
- **Center area** - the welcome screen, or the live stream windows
  (depth / IR / RGB / IMU / ultrasonic) arranged in a grid.
- **Bottom panel (Debug console)** - log messages from the device and the
  application. The splitter above the console can be dragged to resize it.

## 3. Top toolbar

| Button             | Description                                                                                                             |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| Add Source         | Add an additional camera or recording. Hidden until at least one source is open.                                        |
| Extended Options   | Toggles extended mode, which exposes advanced features such as advanced camera controls. Only visible in extended mode. |
| Record RRF         | Records an RRF file directly on the device and downloads it via ADB. Only visible in extended mode.                     |
| Settings (icon)    | Opens the settings menu (firmware update, recording options, stream options, console options).                          |
| Help (icon)        | Opens the help panel (documentation, contact information, license info).                                                |

## 4. Source view (left panel)

Each source has its own tab with the following controls.

### 4.1 Header buttons

| Button | Description                                                                                  |
| ------ | -------------------------------------------------------------------------------------------- |
| Stop   | Stops or restarts the camera pipeline. For a recording, pauses and resumes playback.         |
| Record | Starts or stops recording all enabled streams to a `.bag` file. Disabled during playback.    |
| Info   | Opens a dialog with device information (serial, model, firmware, ToF-RGB baseline, ...).     |
| More   | Opens a menu with additional actions (see below).                                            |

The **More** menu contains:

- **Show / Hide FPS** - toggles a live frames-per-second counter.
- **Advanced Settings** - opens the advanced camera parameter dialog (extended mode only).
- **Show Calibration Data** - opens a dialog with the device's calibration matrices.
- **Show Live Logs** - enables streaming of device log messages into the debug console
  (live devices only).
- **Collect Crash Logs** - downloads crash logs from the device (live devices only).
- **Save Camera Settings** - writes the current camera configuration to the device's
  non-volatile memory (live devices only).
- **Export PLY/PNG** - saves the current point cloud as a PLY file and the current RGB
  frame as a PNG file to the configured save directory.

### 4.2 Camera settings

- **Preset** - selects a predefined use case. The preset typically sets a matching FPS,
  range and HDR mode. The save icon button next to the preset dropdown saves the
  current configuration as a custom preset file.
- **FPS** - frame rate of the depth stream.
- **HDR** - high dynamic range mode.
- **Range** - measurement range (e.g. near, mid, far).
- **Distance Offset** - distance offset calibration.

### 4.3 Depth visualization

- **Colormap** - color palette used to render depth values (e.g. Jet, Turbo, Gray).
- **Depth Limits (mm)** - minimum and maximum depth used for the colormap.
- **Clamp depth** - if enabled, pixels outside the depth limits are drawn black; if
  disabled, they are clamped to the closest limit color.
- **Auto scale depth image** - automatically rescales the colormap to the
  current frame's min/max depth. When enabled, the Depth Limits controls are
  disabled.

### 4.4 Temperatures

Two read-only indicators show the live SoC and ToF sensor temperatures.

### 4.5 Streams

Each available stream is shown as a **stream card** — a compact panel with an inline
toggle, live metadata, and collapsible controls:

| Element             | Description                                                             |
| ------------------- | ----------------------------------------------------------------------- |
| Toggle checkbox     | Single click enables/disables the stream visualization.                 |
| Title               | Stream name (Depth, RGB, IR, IMU, Ultrasonic).                          |
| FPS badge           | Shows live frames-per-second when the stream is active.                 |
| Metadata line       | Always-visible summary (e.g. frame number, exposure time).              |
| Controls section    | Collapsible per-stream controls (click "Controls" to expand/collapse).  |

Stream cards and their controls:

- **Depth Stream** - auto exposure toggle and manual exposure time sliders.
- **RGB Stream** - auto exposure, manual exposure, auto/manual analog gain, auto/manual
  white balance.
- **IR Stream** - auto exposure and manual exposure time sliders.
- **IMU** - enables the IMU graphs.
- **Ultrasonic** - enables the ultrasonic display (only on devices that support it).

Keyboard shortcuts for quick toggling: `D` = Depth, `C` = RGB, `I` = IR, `A` = IMU.

## 5. Stream windows (center area)

Enabling a stream opens a corresponding window in the central grid. Windows can be
moved and resized; closing them disables the stream.

### 5.1 Depth window

Header controls:

- **Rotate CCW / CW** - rotates the image by 90 degrees.
- **COL / RGB** - switches between colorized depth and RGB-aligned coloring.
- **2D / 3D** - switches between the 2D depth image and the 3D point cloud.
- **3D options menu** (only in 3D mode):
  - Show / Hide Frustum
  - Frontal View
  - Top View
  - Side View

Hovering the mouse over the image shows pixel coordinates and the depth value in
millimeters in the status bar at the bottom of the main window.

#### Zoom and pan (2D mode)

- **Mouse wheel up** - zoom in (centered on the cursor position).
- **Mouse wheel down** - zoom out (minimum is 100%, i.e. the original fit-to-window
  size).
- **Right mouse button drag** - pan the view when zoomed in.

When zoomed in, a **minimap** is displayed in the bottom-left corner of the stream
window. It shows a thumbnail of the full image with a red rectangle indicating the
currently visible region.

Hover information (pixel coordinates and depth value) is always reported in original
image coordinates, regardless of the current zoom level.

### 5.2 IR window

Shows the infrared image. Header contains rotation buttons. Hover information shows
the IR intensity.

Zoom and pan work identically to the depth window (see above).

### 5.3 RGB window

Shows the color video stream. Header contains rotation buttons. Hover information
shows the pixel color.

Zoom and pan work identically to the depth window (see above).

### 5.4 3D point cloud

When the depth window is switched to 3D mode, depth data is rendered as a point cloud
that can be navigated interactively:

- **Left mouse button** - rotate (arcball).
- **Middle mouse button**, or left + right together - pan the view.
- **Mouse wheel** - zoom in / out.

Points are colored either by depth (using the selected colormap) or by aligned RGB,
depending on the **COL / RGB** toggle.

### 5.5 IMU view

Two side-by-side plots scroll in real time:

- **Angular velocity** (gyroscope) - X (red), Y (green), Z (blue).
- **Linear acceleration** - X (red), Y (green), Z (blue).

### 5.6 Ultrasonic view

A read-only vertical gauge shows the current ultrasonic measurement together with a
numeric label.

## 6. Recording and playback

### 6.1 Recording

Press the **Record** button on a source to start a recording of all currently enabled
streams. Press it again to stop. Recordings are written to the directory configured
under Settings > Recording Options.

### 6.2 Playback

Open a recording either via the **Replay** button on the welcome screen, by using
**Add Source** > replay, or by dragging a `.bag` file onto the application window.

A playback source behaves like a live device, except that the Record button and the
device-only actions (firmware update, live logs, crash logs) are disabled. Use the
**Stop** button on the source to pause and resume playback.

## 7. Calibration data

Opened via **More > Show Calibration Data**. The dialog shows:

- Depth camera intrinsics (3x3 matrix).
- Depth camera distortion model and coefficients.
- Depth-to-color extrinsics (rotation and translation).
- Color camera intrinsics.
- Color camera distortion model and coefficients.
- Color-to-depth extrinsics.

The **Export to CSV** button writes all matrices to a CSV file in the configured save
directory.

## 8. Advanced settings (extended mode)

Available via **More > Advanced Settings** when extended mode is enabled. The dialog
lets you read and write low-level camera configuration parameters by key. Use this
only if you know which parameters your device supports.

## 9. Export

**More > Export PLY/PNG** saves:

- The current depth frame as a point cloud in PLY format
  (`O3P_Pointcloud_YYYYMMDD_HHmmss.ply`).
- The current RGB frame as a PNG image
  (`O3P_RGB_YYYYMMDD_HHmmss.png`).

Files are written to the directory configured under Settings > Recording Options.

## 10. Settings menu

- **Firmware Update** - select a `.swu` file to update the connected device. The
  device automatically reboots into recovery mode and progress is shown in a dialog.
- **Recording Options** - configure the following options for recordings and playback:
  - *Save Directory* - the directory where recordings and exported files are written.
  - *Custom Filename* - an optional filename prefix (default: `O3P_Recording`). Files
    are saved as `<filename>_YYYYMMDD_HHMMSS.bag`.
  - *Loop playbacks* - whether playback restarts automatically when it reaches the end.
  - *Playback speed* - play back at the original recorded speed or as fast as possible.
  - *FPS of recordings* - limit the frame rate stored in a recording (0 = all frames).
- **Stream Options** - configure which streams are opened automatically when a new
  source (camera or recording) is connected. Available streams are Depth, IR, RGB
  and IMU. If no stream is selected, the Depth stream is opened by default.
  Changes take effect for the next source that is connected; they do not affect
  already-open sources.
- **Console Options** - choose which log levels (Info, Warning, Error) are shown in
  the debug console.

## 11. Help panel

The help panel is a drop-down opened by the Help button in the top toolbar. It
contains the following sections:

- **Manual** - a link to `Documentation.pdf`.
- **Contact** - email (`info@ifm.com`) and website (`www.ifm.com`) links to ifm.
- **Regional contact** - a country dropdown to display local ifm contact details.
- **Save License Info...** - saves the license information from the connected device
  to a text file.

## 12. Debug console

The bottom panel displays log messages from the application and (when enabled) the
device:

- Info messages are shown in the default text color.
- Warnings are shown in yellow.
- Errors are shown in red.

Use the checkboxes at the top of the console (or Settings > Console Options) to
filter the displayed log levels.

## 13. Keyboard shortcuts

The following shortcuts act on the currently active source (the source whose tab is
open in the left panel). They are ignored while a text field, spin box or combo box
has the keyboard focus.

| Key   | Action                                                                  |
| ----- | ----------------------------------------------------------------------- |
| `D`   | Toggle the depth stream on or off.                                      |
| `C`   | Toggle the RGB (color) stream on or off.                                |
| `I`   | Toggle the IR stream on or off.                                         |
| `2`   | Switch the depth window to 2D mode (when depth is enabled).             |
| `3`   | Switch the depth window to 3D point cloud mode (when depth is enabled). |
| `R`   | Start or stop recording (only when recording is available).             |
| `A`   | Toggle the IMU display on or off.                                       |

## 14. Troubleshooting

- **No device shown on the welcome screen** - check the USB cable and that the
  device has finished booting. On Linux, verify the udev rules are installed (see
  `udev/README.md`).
- **Streams stay black** - make sure the corresponding stream is enabled in the
  source view and the camera pipeline is running (Stop button shows "running" state).
- **Depth image looks flat or saturated** - adjust the Depth Limits, try a different
  colormap, or enable **Auto scale depth image**.
- **Frame rate is lower than expected** - lower the FPS or disable streams you do
  not need; check the SoC and ToF temperatures.
