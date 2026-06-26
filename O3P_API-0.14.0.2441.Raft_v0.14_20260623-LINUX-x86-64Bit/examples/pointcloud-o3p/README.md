# Tutorial - Pointcloud Visualization

This tutorial demonstrates how to retrieve pointcloud data and display it as a 3D scatter plot. The application opens a window showing all depth points in 3D space, colored by their distance from the camera. The view can be manipulated with the mouse, including zooming, panning and rotating.

## Starting
Connect the O3P device to your computer and execute the executable file.

## Visualization
Each depth pixel is rendered as a 3D point. Points are colored using `o3p::Colorizer`, mapping depth values to a color scale from near (warm) to far (cool).

## Code Overview
The main loop retrieves the pointcloud frame each iteration and passes it to `draw_pointcloud()` from `rs-example.hpp`, which colorizes the depth values and renders each point in 3D space.

```cpp
    // Create a colorizer to visualize depth data
    o3p::Colorizer colorizer;

    pipe.start();

    while (app) {
        // Block program until frames arrive
        o3p::FrameSet frames = pipe.waitForFrames();

        // Get the pointcloud frame
        auto pc = frames.getPointCloudFrame();

        // Draw the pointcloud
        draw_pointcloud(&colorizer, app.width(), app.height(), app_state, pc);
    }

    pipe.stop();
```