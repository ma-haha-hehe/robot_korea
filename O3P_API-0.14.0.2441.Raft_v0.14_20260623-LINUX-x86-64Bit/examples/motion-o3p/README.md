# Tutorial - IMU Visualization

This tutorial demonstrates how to retrieve IMU data and display it graphically on the screen. The application opens a window with a 3D model of the camera, providing a rough idea of its physical orientation. The view can be manipulated with the mouse, including zooming, panning and rotating.

## Starting
Connect the O3P device to your computer and execute the executable file, then move the camera device to display the movement on the screen.

## Visualization
The viewer shows a 3D model of the camera rotating in real time based on IMU data. The orientation axes are colored as follows: red = x-direction, green = y-direction, blue = z-direction.

## Code Overview
The main loop retrieves IMU frames each iteration and uses them to update and render the camera orientation.

```cpp
    pipe.start();

    while (app) {
        // Block program until frames arrive
        o3p::FrameSet frames = pipe.waitForFrames();

        // Get the IMU frame
        auto imu = frames.getImuFrame();

        // Handle accelerometer data
        accel_data.x = imu->acceleration[0];
        accel_data.y = imu->acceleration[1];
        accel_data.z = imu->acceleration[2];

        // Handle gyroscope data
        gyro_data.x = imu->angularVelocity[0];
        gyro_data.y = imu->angularVelocity[1];
        gyro_data.z = imu->angularVelocity[2];

        // Draw the camera model at the computed orientation
        camera.renderCamera(algo.getTheta());
    }

    pipe.stop();
```