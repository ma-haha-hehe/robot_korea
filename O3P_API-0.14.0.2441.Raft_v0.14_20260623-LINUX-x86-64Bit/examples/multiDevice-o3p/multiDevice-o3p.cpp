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

int main(int argc, char *argv[]) {

    const auto cameras = o3p::Context::getAvailableCameras();

    if (cameras.empty()) {
        std::cerr << "No cameras found" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Available cameras : " << std::endl;
    for (auto i = 0u; i < cameras.size(); ++i) {
        std::cout << "Camera " << i << " : " << cameras[i] << std::endl;
    }

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    std::vector<std::shared_ptr<o3p::Pipeline>> pipelines;

    for (auto i = 0u; i < cameras.size(); ++i) {
        std::shared_ptr<o3p::Pipeline> p = std::make_shared<o3p::Pipeline>();

        o3p::PipelineConfig config;
        config.enableDevice(cameras[i]);
        try {
            p->init(config);
        } catch (std::exception &e) {
            std::cerr << "Error initializing camera : " << e.what() << std::endl;
            return EXIT_FAILURE;
        }

        // Configure and start the pipeline
        p->start();

        pipelines.push_back(std::move(p));
    }

    using namespace std::chrono_literals;
    auto now = std::chrono::steady_clock::now;
    auto start = now();
    auto loopTime = 5s;

    while ((now() - start) < loopTime) {
        for (auto i = 0u; i < cameras.size(); ++i) {
            try {
                // Block program until frames arrive
                o3p::FrameSet frames = pipelines[i]->waitForFrames();

                // Try to get a frame of a depth image
                auto depth = frames.getDepthFrame();

                // Get the depth frame's dimensions
                auto width = depth->getWidth();
                auto height = depth->getHeight();

                // Query the distance from the camera to the object in the center of the image
                float distToCenter = depth->getDistance(width / 2, height / 2);

                // Print the distance
                std::cout << "Camera " << cameras[i] << " distance : " << distToCenter << " meters away" << std::endl;
            } catch (std::exception &e) {
                std::cerr << "Error waiting for frames: "
                          << "Camera " << cameras[i] << ": " << e.what() << std::endl;
                return EXIT_FAILURE;
            }
        }
    }

    return EXIT_SUCCESS;
}
