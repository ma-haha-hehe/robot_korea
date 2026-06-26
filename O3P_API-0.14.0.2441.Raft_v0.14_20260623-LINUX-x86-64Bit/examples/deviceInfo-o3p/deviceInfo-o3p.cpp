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

    std::cout << std::endl
              << "Device Information :" << std::endl;
    std::cout << "-------------------" << std::endl;

    for (auto i = 0; i < o3p::O3P_CAMERA_INFO_COUNT; ++i) {
        o3p::CameraInfo infoType = static_cast<o3p::CameraInfo>(i);
        if (profile.getDevice()->supportsInfo(infoType)) {
            auto infoValue = profile.getDevice()->getInfo(infoType);
            std::cout << o3p::cameraInfoToString(infoType) << " : " << infoValue << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
