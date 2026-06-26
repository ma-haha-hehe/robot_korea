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

    std::cout << "O3P API version : " << o3p::getApiVersionString() << std::endl;

    const auto cameras = o3p::Context::getAvailableCameras();
    std::cout << "Available cameras : " << std::endl;
    for (auto i = 0u; i < cameras.size(); ++i) {
        std::cout << "Camera " << i << " : " << cameras[i] << std::endl;
    }

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    o3p::Pipeline p;
    o3p::PipelineProfile profile;

    // Use the PipelineConfig to load a recorded file
    o3p::PipelineConfig config;

    try {
        profile = p.init(config);
    } catch (std::exception &e) {
        std::cerr << "Error initializing camera : " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (profile.getDevice()->supportsInfo(o3p::O3P_CAMERA_INFO_CPU_ID)) {
        auto cpuId = profile.getDevice()->getInfo(o3p::O3P_CAMERA_INFO_CPU_ID);
        std::cout << "Connected to camera with CPU ID : " << cpuId << std::endl;
    }

    auto streams = profile.getStreams();

    std::cout << streams.size() << " streams" << std::endl;

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

            switch (lensParams.model) {
            case LensModelType::PIN_HOLE:
            case LensModelType::PIN_HOLE_REAL:
            case LensModelType::PIN_HOLE_VIRTUAL:
                std::cout << "Distortion coefficients (Brown-Conrady: k1, k2, p1, p2, k3) : " << std::endl;
                break;
            case LensModelType::FISH_EYE:
                std::cout << "Distortion coefficients (F-Theta Fish-eye: k1, k2, k3, k4) : " << std::endl;
                break;
            default:
                std::cout << "Distortion coefficients (unknown model) : " << std::endl;
                break;
            }
            for (auto j = 0; j < 5; ++j) {
                std::cout << "coeffs[" << j << "] : " << lensParams.coeffs[j] << std::endl;
            }
        }
    }

    // Configure and start the pipeline
    p.start();

    using namespace std::chrono_literals;
    auto now = std::chrono::steady_clock::now;
    auto start = now();
    auto loopTime = 5s;

    // Example : plugin data to be sent
    // std::vector<uint8_t> pluginMessage(1024, 1);

    while ((now() - start) < loopTime) {
        try {
            // Block program until frames arrive
            o3p::FrameSet frames = p.waitForFrames();

            // Example : send a plugin message and receive a reply
            // std::vector<uint8_t> reply = p.sendPluginMessage(pluginMessage);

            // Try to get a frame of a depth image
            auto depth = frames.getDepthFrame();

            // Get the depth frame's dimensions
            auto width = depth->getWidth();
            auto height = depth->getHeight();

            // Query the distance from the camera to the object in the center of the image
            float distToCenter = depth->getDistance(width / 2, height / 2);

            if (depth->supportsMetadata(o3p::O3P_METADATA_TIMESTAMP)) {
                auto timestamp = depth->getTimestamp();
                std::cout << "Timestamp: " << timestamp << std::endl;
            } else {
                std::cout << "Timestamp not supported." << std::endl;
            }

            if (depth->supportsMetadata(o3p::O3P_METADATA_FRAME_NUMBER)) {
                auto frameNumber = depth->getMetadata<uint32_t>(o3p::O3P_METADATA_FRAME_NUMBER);
                std::cout << "Frame number: " << frameNumber << std::endl;
            } else {
                std::cout << "Frame number not supported." << std::endl;
            }

            // Print the distance
            std::cout << "TOF center pixel distance : " << distToCenter << " meters away" << std::endl;

        } catch (std::exception &e) {
            std::cerr << "Error waiting for frames: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
