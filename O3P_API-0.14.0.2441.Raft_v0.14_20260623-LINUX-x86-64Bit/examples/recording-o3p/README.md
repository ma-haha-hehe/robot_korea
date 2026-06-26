# Tutorial - "Recording" Example

This tutorial explains how you can record and play back data with the O3P API.

## Installation
To build and run this code you need a version of the O3P API and also OpenCV installed on your system.

## Starting
Connect the O3P device to your computer and execute the executable file. You will be prompted to choose between record mode and playback mode.

## Modes

### Record mode (`r`)
The pipeline is initialized with a connected device and started. Recording begins immediately and frames are written to `recorded_file.bag` in the current working directory. Press `ESC` or `q` to stop recording and exit.

### Playback mode (`p`)
The pipeline is initialized from a `.bag` file. You can pass the file path as a command-line argument, or the example will look for `recorded_file.bag` in the current working directory. You will also be asked whether to loop the playback.

```bash
# Record
./recording-o3p
# then type: r

# Play back a specific file
./recording-o3p /path/to/my_recording.bag
# then type: p
```

## Visualization
A colorized depth image is shown in an OpenCV window. In both record and playback mode the same key bindings apply:

| Key | Action |
|---|---|
| `c` / `C` | Cycle to the next colormap |
| `v` / `V` | Cycle to the previous colormap |
| `Space` | Pause / resume playback |
| `ESC` / `q` | Stop recording (if active) and quit |

## Code Overview
The mode selection drives which `PipelineConfig` is constructed. In record mode the default config is used and `startRecording()` is called after `start()`; in playback mode the config is given the path to the `.bag` file.

```cpp
o3p::Pipeline p;
o3p::PipelineConfig config;

if (mode == "p") {
    config.setPlaybackFile(cwd + "/recorded_file.bag");
    config.setLoopPlayback(loop == "yes");
}

p.init(config);
p.start();

if (mode == "r") {
    p.startRecording("recorded_file.bag");
}
```

Each frame the depth image is colorized and displayed via OpenCV:

```cpp
o3p::FrameSet frames = p.waitForFrames();
auto depth = frames.getDepthFrame();

colorizer.colorize(depth->m_data, coloredDepthData, width, height, 0.001f);
cv::Mat rgbImg(height, width, CV_8UC3, coloredDepthData.data());
cv::cvtColor(rgbImg, bgrImg, cv::COLOR_RGB2BGR);
cv::imshow("colored depth", bgrImg);
```