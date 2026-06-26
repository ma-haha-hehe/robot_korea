/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#include <chrono>
#include <iostream>
#include <o3p/Align.hpp>
#include <o3p/o3p.hpp>

#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char *argv[]) {
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    o3p::Pipeline p;

    // Use the PipelineConfig to load a recorded file
    o3p::PipelineConfig config;

    if (argc > 1) {
        config.setPlaybackFile(argv[1]); // Load a recorded file
    }

    o3p::PipelineProfile profile;
    try {
        profile = p.init(config);
    } catch (std::exception &e) {
        std::cerr << "Error initializing camera : " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Extract intrinsics and extrinsics from streams
    o3p::Intrinsics tofIntrinsics{}, rgbIntrinsics{};
    o3p::Extrinsics tofToRgbExtrinsics;
    bool hasColorStream = false;

    auto streams = profile.getStreams();
    for (auto &s : streams) {
        auto *vs = dynamic_cast<o3p::VideoStream *>(s.get());
        if (vs) {
            if (vs->streamType() == o3p::O3P_STREAM_DEPTH) {
                tofIntrinsics = vs->getIntrinsics();
                tofToRgbExtrinsics = vs->getExtrinsics();
            } else if (vs->streamType() == o3p::O3P_STREAM_COLOR) {
                rgbIntrinsics = vs->getIntrinsics();
                hasColorStream = true;
            }
        }
    }

    bool alignEnabled = false;

    // Create a colorizer to visualize depth data
    o3p::Colorizer colorizer;
    auto &options = colorizer.getOptions();

    if (options.supportsOption(o3p::O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION)) {
        auto histOption = options.getOption(o3p::O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION);
        histOption->setValue(o3p::OptionValue(true));
    } else {
        std::cout << "Histogram equalization option not supported." << std::endl;
    }

    // Configure and start the pipeline
    p.start();

    // Example : plugin data to be sent
    // std::vector<uint8_t> pluginMessage(1024, 1);

    bool running = true;
    while (running) {

        try {
            // Block program until frames arrive
            o3p::FrameSet frames = p.waitForFrames();

            // Example : send a plugin message and receive a reply
            // std::vector<uint8_t> reply = p.sendPluginMessage(pluginMessage);

            // Apply alignment if enabled
            if (alignEnabled && hasColorStream && frames.getColorFrame()->m_width > 0) {
                o3p::alignColorToDepth(frames, tofToRgbExtrinsics, rgbIntrinsics, tofIntrinsics);
            }

            // Depth
            {
                // Try to get a frame of a depth image
                auto depth = frames.getDepthFrame();

                // Vector to hold the colored depth data
                std::vector<uint8_t> coloredDepthData;

                // Get the depth frame's dimensions
                auto width = depth->getWidth();
                auto height = depth->getHeight();

                if (width > 0 && height > 0) {
                    colorizer.colorize(depth->m_data, coloredDepthData, width, height, 0.001f);
                    // Convert to colored depth image for viewing purposes only
                    cv::Mat rgbImg(height, width, CV_8UC3, coloredDepthData.data());
                    // OpenCV uses BGR by default, so convert RGB to BGR
                    cv::Mat bgrImg;
                    cv::cvtColor(rgbImg, bgrImg, cv::COLOR_RGB2BGR);
                    cv::resize(bgrImg, bgrImg, cv::Size(), 2, 2);
                    imshow("colored depth", bgrImg);

                    std::cout << "Depth timestamp = " << depth->getTimestamp() << std::endl;
                }
            }

            // Infrared
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

            // RGB
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
                    std::cout << "Color timestamp = " << color->getTimestamp() << std::endl;
                }
            }

            /*
                    // IMU
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

                    // Plugin
                    {
                        auto plugin = frames.getPluginFrame();
                        // auto data = plugin->getData(); // Plugin defined output
                        std::cout << "Plugin Frame Size (Bytes) = " << plugin->getSize() << std::endl;
                    }
            */

            // Change color scheme on key press, ESC to exit
            int key = cv::waitKey(1);
            if (key == 27) { // ESC
                running = false;
            } else if (key == 'm' || key == 'M') {
                alignEnabled = !alignEnabled;
                std::cout << "Alignment " << (alignEnabled ? "enabled" : "disabled") << std::endl;
            } else if (key >= '0' && key <= '9') {
                int schemeIdx = key - '0';
                if (schemeIdx < static_cast<int>(o3p::ColorScheme::COUNT)) {
                    if (options.supportsOption(o3p::O3P_OPTION_COLOR_SCHEME)) {
                        auto schemeOption = options.getOption(o3p::O3P_OPTION_COLOR_SCHEME);
                        schemeOption->setValue(o3p::OptionValue(schemeIdx));
                        std::cout << "Switched to colormap " << schemeIdx << std::endl;
                    }
                }
            }
        } catch (std::exception &e) {
            std::cerr << "Error waiting for frames: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
