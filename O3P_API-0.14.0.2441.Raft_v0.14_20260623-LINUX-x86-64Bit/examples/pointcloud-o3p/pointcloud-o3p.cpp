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

#include "realsense-example-helper/rs-example.hpp"
#include <o3p/Align.hpp>
#include <o3p/o3p.hpp>


int main(int argc, char *argv[]) {
    const auto cameras = o3p::Context::getAvailableCameras();
    if (cameras.size() > 0) {
        try {
            // Create a Pipeline - this serves as a top-level API for streaming and processing frames
            o3p::Pipeline pipe;
            o3p::PipelineProfile profile;

            o3p::PipelineConfig config;

            try {
                profile = pipe.init(config);
            } catch (std::exception &e) {
                std::cerr << "Error initializing camera : " << e.what() << std::endl;
                return EXIT_FAILURE;
            }

            auto streams = profile.getStreams();

            std::cout << streams.size() << " streams" << std::endl;

            // Extract intrinsics and extrinsics from streams
            o3p::Intrinsics tofIntrinsics{}, rgbIntrinsics{};
            o3p::Extrinsics tofToRgbExtrinsics;
            bool hasColorStream = false;

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

            // Initialize window for rendering
            window app(1280, 960, "O3P Pointcloud Example");

            // Construct an object to manage view state
            glfw_state app_state(0.0, 0.0);

            // Register callbacks to allow manipulation of the pointcloud
            register_glfw_callbacks(app, app_state);

            // Create a colorizer as fallback when no color stream is available
            o3p::Colorizer colorizer;

            pipe.start();

            while (app) {
                // Block program until frames arrive
                o3p::FrameSet frames = pipe.waitForFrames();

                // Try to get a frame of PointCloud
                auto pc = frames.getPointCloudFrame();

                if (hasColorStream && frames.getColorFrame()->m_width > 0) {
                    // Align the color frame to the depth frame resolution
                    o3p::alignColorToDepth(frames, tofToRgbExtrinsics, rgbIntrinsics, tofIntrinsics);

                    // Draw the pointcloud with real camera colors
                    draw_pointcloud_colored(app.width(), app.height(), app_state,
                                            frames.getPointCloudFrame(), frames.getColorFrame());
                } else {
                    // Fallback: draw with depth-based colorization
                    draw_pointcloud(&colorizer, app.width(), app.height(), app_state, pc);
                }
            }
            // Stop the pipeline
            pipe.stop();

        } catch (std::exception &e) {
            std::cerr << "Error : " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    } else {
        std::cerr << "Error finding a camera" << std::endl;
        return EXIT_FAILURE;
    }
}