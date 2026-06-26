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

#ifndef DEVICEPLAYBACK_HPP
#define DEVICEPLAYBACK_HPP

#include <o3p/Device.hpp>

#include <map>

namespace o3p {

/**
 * @brief Represents the camera device
 */
class DevicePlayback : public Device {
  public:
    /**
     * @brief Constructor with loop and timestamp options
     */
    O3P_API DevicePlayback(const std::string &playbackFile, bool loop = true, bool useTimestamps = true);

    /**
     * @brief Default destructor
     */
    O3P_API ~DevicePlayback() override;

    /**
     * @brief Initialize the sensor
     */
    O3P_API void init() override;

    /**
     * @brief Start the sensor
     */
    O3P_API void start() override;

    /**
     * @brief Stop the sensor
     */
    O3P_API void stop() override;

    /**
     * @brief Close the device
     */
    O3P_API void closeDevice() override;

    /**
     * @brief Waits for the frames
     */
    O3P_API void waitForFrames(FrameSet *frameSet) override;

    /**
     * @brief Get the streams
     */
    O3P_API std::vector<std::shared_ptr<Stream>> getStreams() override;

    O3P_API std::vector<uint8_t> sendPluginMessage(const std::vector<uint8_t> &message) override;
    O3P_API void setConfigValue(const std::string &config, const std::string &value) override;
    O3P_API std::string getConfigValue(const std::string &config) override;
    O3P_API void saveConfig() override;
    O3P_API void writeEeprom(const std::vector<uint8_t> &value) override;
    O3P_API std::vector<uint8_t> readEeprom() override;
    O3P_API std::string getLicenseInfo() override;

    O3P_API std::string getUsecase();

    O3P_API void setDisconnectCallback(DisconnectCallback cb) override;

    O3P_API uint32_t getNumberOfFrames() override;

    O3P_API std::shared_ptr<FwLogger> getFwLogger() override;

  private:
    std::string m_playbackFile;

    struct Impl;
    std::unique_ptr<Impl> m_pImpl;

    std::vector<std::shared_ptr<Stream>> m_streams;

    std::vector<uint8_t> m_depthData;
    std::vector<uint8_t> m_colorData;
    std::vector<uint8_t> m_irData;
    std::vector<int8_t> m_pluginData;

    std::vector<uint8_t> m_depthMetadataBuffer;
    std::vector<uint8_t> m_colorMetadataBuffer;
    std::vector<uint8_t> m_irMetadataBuffer;
    std::vector<uint8_t> m_imuMetadataBuffer;
    std::vector<uint8_t> m_usMetadataBuffer;

    std::unordered_map<o3p::MetadataType, size_t> m_depthMetadataLayout;
    std::unordered_map<o3p::MetadataType, size_t> m_colorMetadataLayout;
    std::unordered_map<o3p::MetadataType, size_t> m_irMetadataLayout;
    std::unordered_map<o3p::MetadataType, size_t> m_imuMetadataLayout;
    std::unordered_map<o3p::MetadataType, size_t> m_usMetadataLayout;

    uint32_t m_curFrameCounter;

    bool m_loop;          // loop the playback
    bool m_useTimestamps; // delay the delivery of new frames based on the timestamps

    bool m_ultrasonicSensorChecked;
    std::map<std::string, std::string> m_keys;
};

} // namespace o3p

#endif
