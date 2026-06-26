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
#include <o3p/o3p.hpp>
#include <vector>

#include <filesystem>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Note that the default usecase of the camera is used in this example.
// But this might not be optimal to show the detection of the nearest point in the depth image.

int main(int argc, char *argv[]) {
    // Get the current Folder to save the recorded file there
    std::string cwd = std::filesystem::current_path().string();

    std::string mode;
    std::string loop;
    int cmap_idx = 0; // Index for colormap selection
    cv::Point idx;

    std::cout << "Do you want to record or play the last recording? (r/p): " << endl;

    //Mode for record and play
    std::cout << "Choose Mode: [r] Record | [p] Play: ";
    cin >> mode;

    if (mode != "r" && mode != "p") {
        cerr << "Invalid mode selected. Please choose 'r' for Record or 'p' for Play." << endl;
        return EXIT_FAILURE;
    }
    o3p::Pipeline p;
    o3p::PipelineConfig config;

    // load the recorded file and ask if the user wants to loop the playback
    if (mode == "p") {
        if (argc > 1) {
            config.setPlaybackFile(argv[1]); // Load a recorded file
        } else {
            config.setPlaybackFile(cwd + "/recorded_file.bag"); // Load recording from current directory
        }
        std::cout << "Loaded recording file: " << config.getPlaybackFile() << std::endl;
        std::cout << "Do you want to loop the playback? (yes/no): ";
        cin >> loop;
        if (loop == "yes") {
            config.setLoopPlayback(true);
        } else if (loop == "no") {
            config.setLoopPlayback(false);
        }
    }

    try {
        p.init(config);
    } catch (std::exception &e) {
        std::cerr << "Error initializing camera : " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Create a colorizer to visualize depth data
    o3p::Colorizer colorizer;
    auto &options = colorizer.getOptions();

    if (options.supportsOption(o3p::O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION)) {
        auto histOption = options.getOption(o3p::O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION);
        histOption->setValue(o3p::OptionValue(true));
    } else {
        std::cout << "Histogram equalization option not supported." << std::endl;
    }

    // Get the color scheme option of the colorizer
    auto colorSchemeOption = colorizer.getOptions().getOption(o3p::O3P_OPTION_COLOR_SCHEME);

    // Configure and start the pipeline
    p.start();

    // Start recording
    if (mode == "r") {
        p.startRecording("recorded_file.bag"); // Set the output file for recording
    }

    std::string colormaps[static_cast<int>(o3p::ColorScheme::COUNT)];
    for (int i = 0; i < static_cast<int>(o3p::ColorScheme::COUNT); ++i) {
        colormaps[i] = o3p::colorScheme2String(static_cast<o3p::ColorScheme>(i));
    }

    cout << "Press: [c] next Colormap | [v] last Colormap | [space] pause replay | [ESC or q] Quit" << endl;

    // Create window early to ensure it has focus
    cv::namedWindow("colored depth", cv::WINDOW_AUTOSIZE);

    while (true) {
        // First check for key input with longer timeout
        int key = cv::waitKey(30);

        // Handle key input immediately
        if (key == 32) { // 32 is the ASCII code for space
            cout << "PAUSED" << endl;
            cv::waitKey(0); // Pause until any key is pressed
            cout << "RESUMED" << endl;
        }
        if (key == 27 || key == 'q' || key == 'Q') { // ESC, q, or Q to quit
            cout << "Exit Program" << endl;
            p.stopRecording();
            p.stop();
            break;
        }
        if (key == 'c' || key == 'C') { // next Colormap
            cmap_idx = (cmap_idx + 1) % static_cast<int>(o3p::ColorScheme::COUNT);
        }

        if (key == 'v' || key == 'V') { // previous Colormap
            cmap_idx = (cmap_idx - 1 + static_cast<int>(o3p::ColorScheme::COUNT)) % static_cast<int>(o3p::ColorScheme::COUNT);
        }

        try {
            // Block program until frames arrive
            o3p::FrameSet frames = p.waitForFrames();

            // Depth
            {
                // Try to get a frame of a depth image
                auto depth = frames.getDepthFrame();

                // Pointer to hold the colored depth data
                std::vector<uint8_t> coloredDepthData;

                // Get the depth frame's dimensions
                auto width = depth->getWidth();
                auto height = depth->getHeight();

                if (width > 0 && height > 0) {
                    colorizer.colorize(depth->m_data, coloredDepthData, width, height, 0.001f);
                    cv::Mat rgbImg(height, width, CV_8UC3, coloredDepthData.data());
                    cv::Mat zImage(height, width, CV_16UC1, depth->m_data);

                    // Find the nearest point in the depth image, i.e. smallest non-zero value
                    int min = INT_MAX;
                    for (int i = 0; i < zImage.rows; ++i) {
                        for (int j = 0; j < zImage.cols; ++j) {
                            float value = zImage.at<uint16_t>(i, j); // Assuming first channel is depth value
                            if (value > 0.0f && value < static_cast<float>(min)) {
                                min = static_cast<int>(value);
                                idx = cv::Point(j, i); // (x, y)
                            }
                        }
                    }

                    // draw green circle around the nearest point in the colored depth image
                    rgbImg.at<cv::Vec3b>(idx.y, idx.x) = cv::Vec3b(0u, 255u, 0u);
                    cv::circle(rgbImg, cv::Point(idx.x, idx.y), 10, cv::Scalar(0, 255, 0));

                    // Set the color scheme option to the selected colormap index
                    colorSchemeOption->setValue(o3p::OptionValue(cmap_idx));

                    cv::Mat bgrImg;
                    cv::cvtColor(rgbImg, bgrImg, cv::COLOR_RGB2BGR);
                    cv::resize(bgrImg, bgrImg, cv::Size(), 2, 2);
                    cv::imshow("colored depth", bgrImg);
                }
            }
        }

        catch (const o3p::BagFileEndReachedWarning &) {
            std::cout << "Reached end of the recording. Program finished. " << std::endl;
            break;
        } catch (std::exception &e) {
            std::cerr << "Error waiting for frames: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}