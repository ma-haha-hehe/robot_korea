# Tutorial - Distance Measurement

This tutorial demonstrates how to measure the real-world 3D distance between two points in a depth scene using the O3P camera. The application opens a window showing a colorized depth image with an infrared (IR) overlay and a draggable ruler that lets the user pick two points on the scene.

## Starting
Connect the O3P device to your computer and execute the executable file. 

## Visualization
The grayscale infrared frame is blended on top at ~45% opacity, providing spatial context (edges, surface texture) that aligns pixel-perfectly with the depth image. A white ruler line connects the two draggable endpoints, and the 3D distance is shown as a text label in centimeters at the midpoint of the ruler.

## Code Overview
The main loop retrieves infrared, depth, and pointcloud frames each iteration and renders them in order:

```cpp
    pipe.start();

    while (app) {
        o3p::FrameSet frames = pipe.waitForFrames();

        auto *ir    = frames.getInfraredFrame();
        auto *depth = frames.getDepthFrame();
        auto *pc    = frames.getPointCloudFrame();

        const rect full_window{0.f, 0.f, (float)app.width(), (float)app.height()};

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Render colorized depth as the base layer
        depth_display.render(depth, full_window);

        // Overlay IR (grayscale, semi-transparent) on top of depth
        if (ir->getWidth() > 0 && ir->getHeight() > 0)
            ir_display.render(ir, full_window);

        // Render the ruler line and distance label
        render_simple_distance(depth, pc, app_state, app);

        // Render the ruler endpoint toggles
        app_state.ruler_start.render(app);
        app_state.ruler_end.render(app);
    }

    pipe.stop();
```

**IR overlay rendering** uses `GL_LUMINANCE` with an alpha value of `0.45f` so the depth colorization remains visible through the IR layer:

```cpp
glColor4f(1.0f, 1.0f, 1.0f, alpha);  // alpha = 0.45f by default
glEnable(GL_TEXTURE_2D);
// ... draw quad covering full_window
```

The 3D distance is calculated from the pointcloud frame, which provides pre-computed Cartesian (X, Y, Z) coordinates in meters for every pixel — no manual deprojection is required:

```cpp
float dist_3d(o3p::PointcloudFrame *pc, pixel u, pixel v)
{
    float ux = pc->getX(u.first, u.second);
    float uy = pc->getY(u.first, u.second);
    float uz = pc->getZ(u.first, u.second);

    float vx = pc->getX(v.first, v.second);
    float vy = pc->getY(v.first, v.second);
    float vz = pc->getZ(v.first, v.second);

    return sqrt(pow(ux - vx, 2.f) + pow(uy - vy, 2.f) + pow(uz - vz, 2.f));
}
```

