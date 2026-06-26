# Tutorial - "OpenCV" Example

This tutorial explains how you can use OpenCV together with the O3P API.

## Installation
To build and run this code you need a version of the O3P API and also OpenCV installed on your system.

## Code Explanation
We begin by initializing a pipeline which also initializes the camera. Then we start the pipeline. 
```cpp
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    o3p::Pipeline p;

    try {
        p.init();
    } catch (std::exception &e) {
        std::cerr << "Error initializing camera : " << e.what() << std::endl;
        return 1;
    }

    // Configure and start the pipeline
    p.start();
```

Now we configure a loop to run for 10 seconds. Inside the loop we wait for frames from the camera and store them in the `frames` variable. Then we use OpenCV to display the depth, infrared and rgb frames. We also print the data we get from the IMU.

**Display the depth frame**

To display the depth frame we check that its width and height are greater than 0 and then store its data in a OpenCV matrix. To display it we convert this matrix to 8 bit and apply a colormap. 
```cpp
        {
            // Try to get a frame of a depth image
            auto depth = frames.getDepthFrame();

            // Get the depth frame's dimensions
            auto width = depth->getWidth();
            auto height = depth->getHeight();

            if (width > 0 && height > 0) {
                colorizer.colorize(depth->m_data, coloredDepthData, width, height, 0.001f);
                // Convert to colored depth image for viewing purposes only
                cv::Mat rgbImg(height, width, CV_8UC3, coloredDepthData);
                // OpenCV uses BGR by default, so convert RGB to BGR
                cv::Mat bgrImg;
                cv::cvtColor(rgbImg, bgrImg, cv::COLOR_RGB2BGR);
                cv::resize(bgrImg, bgrImg, cv::Size(), 2, 2);
                imshow("colored depth", bgrImg);
            }
        }
```

To try different colormaps for visualizing the depth data, press the keys 0 to 9.

**Display the infrared frame**

Displaying the infrared frame is similar to displaying the depth frame. We also check that width and height are unequal to 0 and store the data in a matrix. This time we convert the matrix from Gray to BGR and then use `imshow` to display the image. 
```cpp
        {
            auto ir = frames.getInfraredFrame();
            auto width = ir->getWidth();
            auto height = ir->getHeight();

            if (width > 0 && height > 0) {
                cv::Mat irImage = cv::Mat(height, width, CV_8UC1, (uint8_t *)ir->m_data);
                cv::Mat irImageBGR;
                cv::cvtColor(irImage, irImageBGR, COLOR_GRAY2BGR);
                imshow("imageIR", irImageBGR);
            }
        }
```

**Display the rgb frame**

To display the rgb frame we also check that its dimensions are not 0 and store it in a matrix. Then we convert the color from YUV NV12 to BGRA and display the result image. 
```cpp
        {
            // Try to get a frame of a color image
            auto color = frames.getColorFrame();

            // Get the color frame's dimensions
            auto width = color->getWidth();
            auto height = color->getHeight();

            if (width > 0 && height > 0) {
                cv::Mat picYV12 = cv::Mat(height * 3 / 2, width, CV_8UC1, color->m_data);
                cv::Mat picBGR;
                cv::cvtColor(picYV12, picBGR, COLOR_YUV2BGRA_NV12);

                cv::namedWindow("imageRGB", 0);
                cv::resizeWindow("imageRGB", 640, 480);
                imshow("imageRGB", picBGR);
            }
            waitKey(1);
        }
```

**Getting the IMU data**

The imu frame holds the following data: angular velocity in x, y and z direction and timestamp and acceleration in x, y and z direction and timestamp. 
We use simple print statements to display this data here.
```cpp
        {
            auto imu = frames.getImuFrame();

            std::cout << "Angular Velocity X = " << imu->angularVelocity[0] << std::endl;
            std::cout << "Angular Velocity Y = " << imu->angularVelocity[1] << std::endl;
            std::cout << "Angular Velocity Z = " << imu->angularVelocity[2] << std::endl;
            std::cout << "Angular Velocity timestamp = " << imu->angularTimestamp << std::endl;

            std::cout << "Acceleration X = " << imu->acceleration[0] << std::endl;
            std::cout << "Acceleration Y = " << imu->acceleration[1] << std::endl;
            std::cout << "Acceleration Z = " << imu->acceleration[2] << std::endl;
            std::cout << "Acceleration timestamp = " << imu->accelerationTimestamp << std::endl;
        }
```
