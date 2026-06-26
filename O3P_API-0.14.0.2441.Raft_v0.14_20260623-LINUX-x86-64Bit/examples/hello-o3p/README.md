# Tutorial - "Hello" Example

This tutorial explains the "Hello" Sample for the O3P which can be used as a first starting point when working with our O3P API. 

# Installation

This code does not require any additional libraries so you should be able to build run it if you have the API installed.

# Code Explanation
The first thing that is done is the creation of a Pipeline which serves as top-level API for streaming and processing frames. 
Initializing the camera happens during initalization of the pipeline which is done by calling `p.init()`. This function returns the Pipeline profile from which we can get the streams with `profile.getStreams()`.

```cpp
int main(int argc, char *argv[]) {
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    o3p::Pipeline p;
    o3p::PipelineProfile profile;

    try {
        profile = p.init();
    } catch (std::exception &e) {
        std::cerr << "Error initializing camera : " << e.what() << std::endl;
        return 1;
    }

    auto streams = profile.getStreams();

    std::cout << streams.size() << " streams" << std::endl;
``` 

Now follows a loop in which we print some parameters like width and height of the frames in the stream or lens parameters.
In this loop we iterate over all streams and filter for the depth and color streams. 
Then we interpret the stream as VideoStream and use the `getIntrinsics()` function to get the intrinsic parameters. 
Now we are ready to print stream index, stream type, dimensions and intrinsic parameters like principal point, focal length and distortion coefficients. 

```cpp
    for (auto i = 0u; i < streams.size(); ++i) {
        if (streams[i]->streamType() == o3p::StreamType::O3P_STREAM_DEPTH ||
            streams[i]->streamType() == o3p::StreamType::O3P_STREAM_COLOR) {
            auto vstream = reinterpret_cast<o3p::VideoStream *>(streams[i].get());

            auto lensParams = vstream->getIntrinsics();

            std::cout << "Stream " << i << std::endl;
            std::cout << "Type : " << streamType2String(streams[i]->streamType()) << std::endl;
            std::cout << "Width : " << vstream->width() << " Height : " << vstream->height() << std::endl;
            std::cout << "Lens parameters : " << std::endl;
            std::cout << "Principal point cx : " << lensParams.ppx
                      << " cy : " << lensParams.ppy << std::endl;
            std::cout << "Focal length fx : " << lensParams.fx
                      << " fy : " << lensParams.fy << std::endl;
            std::cout << "Distortion coefficients (Brown-Conrady: k1, k2, p1, p2, k3) : " << std::endl;
            for (auto j = 0; j < 5; ++j) {
                std::cout << "coeffs[" << j << "] : " << lensParams.coeffs[j] << std::endl;
            }
        }
    }
```

Now we start the Pipeline by calling `start()` and enter a loop for 5 seconds. If you want you can run the loop longer by changing the value of `loopTime`.
Inside the loop we save the frames we get from the pipeline and store the depth frame int the `depth` variable. Then we use the `getDistance` function to get the distance at the center pixel and print this. 

```cpp
    p.start();

    using namespace std::chrono_literals;
    auto now = std::chrono::steady_clock::now;
    auto start = now();
    auto loopTime = 5s;

    while ((now() - start) < loopTime) {
        // Block program until frames arrive
        o3p::FrameSet frames = p.waitForFrames();

        // Try to get a frame of a depth image
        auto depth = frames.getDepthFrame();

        // Get the depth frame's dimensions
        auto width = depth->getWidth();
        auto height = depth->getHeight();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth->getDistance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
    }
```