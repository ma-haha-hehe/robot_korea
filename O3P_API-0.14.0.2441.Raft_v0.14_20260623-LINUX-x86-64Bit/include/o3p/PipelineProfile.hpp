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

#ifndef PIPELINE_PROFILE_HPP
#define PIPELINE_PROFILE_HPP

#include <memory>
#include <string>
#include <vector>

#include <o3p/Device.hpp>

namespace o3p {

/**
 * @brief Defines the profile of the pipeline
 */
class PipelineProfile {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API PipelineProfile() {
    }

    /**
     * @brief Constructor that initializes the PipelineProfile based on the given device, vector of streams
     * and serial number
     *
     * @param device The device that will be used for initialization
     * @param streams The vector of streams that will be used for initialization
     * @param id The serial number of the device
     */
    O3P_API PipelineProfile(std::shared_ptr<Device> device,
                            std::vector<std::shared_ptr<Stream>> streams,
                            std::string id) : m_device(device),
                                              m_streams(std::move(streams)),
                                              m_id(std::move(id)) {
        for (const auto &stream : m_streams) {
            // Associate the stream with the device id
            stream->setId(m_id);
        }
    }

    /**
     * @brief Get the streams
     *
     * @return A pointer to the streams
     */
    O3P_API const std::vector<std::shared_ptr<Stream>> &getStreams() const {
        return m_streams;
    }

    /**
     * @brief Get the serial number of the device
     *
     * @return The serial number of the device
     */
    O3P_API const std::string &getSerialNumber() const {
        return m_id;
    }

    /**
     * @brief Get the device
     *
     * @return A pointer to the device
     */
    O3P_API std::shared_ptr<Device> getDevice() const {
        return m_device;
    }

  private:
    std::shared_ptr<Device> m_device;
    std::vector<std::shared_ptr<Stream>> m_streams;
    std::string m_id;
};

} // namespace o3p

#endif
