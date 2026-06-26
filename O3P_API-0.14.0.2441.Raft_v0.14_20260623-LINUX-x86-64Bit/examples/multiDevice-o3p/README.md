# Tutorial - "MultiDevice" Example

This tutorial explains the "MultiDevice" sample for the O3P, which demonstrates how to stream from multiple connected O3P cameras simultaneously.

## Installation
This example does not require any additional libraries beyond the O3P API.

## Starting
Connect two or more O3P devices to your computer and execute the executable file. If no cameras are found the program exits with an error.

## Code Overview
The example uses `o3p::Context::getAvailableCameras()` to enumerate all connected devices and creates one `Pipeline` per camera. Each pipeline is configured to target a specific device via `PipelineConfig::enableDevice()`.

```cpp
const auto cameras = o3p::Context::getAvailableCameras();

std::vector<std::shared_ptr<o3p::Pipeline>> pipelines;

for (auto i = 0u; i < cameras.size(); ++i) {
    auto p = std::make_shared<o3p::Pipeline>();

    o3p::PipelineConfig config;
    config.enableDevice(cameras[i]);
    p->init(config);
    p->start();

    pipelines.push_back(std::move(p));
}
```

After all pipelines are started the example runs a loop for 5 seconds. In each iteration it queries a depth frame from every pipeline and prints the distance to the object at the center pixel for each camera.

```cpp
while ((now() - start) < loopTime) {
    for (auto i = 0u; i < cameras.size(); ++i) {
        o3p::FrameSet frames = pipelines[i]->waitForFrames();

        auto depth = frames.getDepthFrame();
        float distToCenter = depth->getDistance(depth->getWidth() / 2, depth->getHeight() / 2);

        std::cout << "Camera " << cameras[i] << " distance : " << distToCenter << " meters away" << std::endl;
    }
}
```
