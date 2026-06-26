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

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <memory>
#include <stdexcept>

#include <o3p/FrameSet.hpp>
#include <o3p/Stream.hpp>

#include <o3p/FwLogger.hpp>
#include <o3p/UvcDevice.hpp>
#include <o3p/info/InfoContainer.hpp>
#include <o3p/option/OptionsContainer.hpp>

namespace o3p {

/**
 * @brief Represents the camera device
 */
class Device : public InfoInterface, public OptionsInterface {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API Device() = default;

    /**
     * @brief Default destructor
     */
    O3P_API virtual ~Device() = default;

    /**
     * @brief Initialize the sensor
     */
    O3P_API virtual void init() = 0;

    /**
     * @brief Start the sensor
     */
    O3P_API virtual void start() = 0;

    /**
     * @brief Stop the sensor
     */
    O3P_API virtual void stop() = 0;

    /**
     * @brief Close the device
     */
    O3P_API virtual void closeDevice() = 0;

    /**
     * @brief Waits for the frames
     */
    O3P_API virtual void waitForFrames(FrameSet *frameSet) = 0;

    /**
     * @brief Get the streams
     */
    O3P_API virtual std::vector<std::shared_ptr<Stream>> getStreams() = 0;

    /**
     * @brief Send a message to a plugin. Pipeline must be started first.
     *
     * @param message Data that is processed by a plugin
     *
     * @return Data reply from plugin
     */
    O3P_API virtual std::vector<uint8_t> sendPluginMessage(const std::vector<uint8_t> &message) = 0;

    /**
     * @brief Set a firmware configuration value
     *
     * @throw std::runtime_error if pipeline is not started or config is rejected by firmware
     *
     * @param config The name of the config value
     * @param value The value to set
     */
    O3P_API virtual void setConfigValue(const std::string &config, const std::string &value) = 0;

    /**
     * @brief Get a firmware configuration value
     *
     * @return String representation of config value. Empty string if config does not exist.
     */
    O3P_API virtual std::string getConfigValue(const std::string &config) = 0;

    /**
     * @brief Save the current configuration to the device's non-volatile memory
     *
     * @throw std::runtime_error if pipeline is not initialized or if saving fails
     */
    O3P_API virtual void saveConfig() = 0;

    /**
     * @brief Writes values to the EEPROM of the camera.
     *
     * @throw std::runtime_error if pipeline is not initialized
     *
     * @param value The value to set
     */
    O3P_API virtual void writeEeprom(const std::vector<uint8_t> &value) = 0;

    /**
     * @brief Reads the EEPROM of the camera.
     *
     * @return Byte vector representation of the EEPROM data.
     */
    O3P_API virtual std::vector<uint8_t> readEeprom() = 0;

    /**
     * @brief Reads the license information from the camera.
     *
     * @return String representation of the license data.
     */
    O3P_API virtual std::string getLicenseInfo() = 0;

    /**
     * @brief Returns the camera ID
     */
    [[nodiscard]] O3P_API const std::string &getId() const { return m_id; }

    /**
     * @brief Set the disconnect callback to handle device disconnection
     */
    O3P_API virtual void setDisconnectCallback(DisconnectCallback cb) = 0;

    /**
     * @brief Get the number of frames in the playback
     */
    O3P_API virtual uint32_t getNumberOfFrames() = 0;

    /**
     * @brief Get the firmware logger for this device. Returns nullptr if no logger is available.
     */
    O3P_API virtual std::shared_ptr<FwLogger> getFwLogger() = 0;

    /**
     * @brief Returns the number of frames dropped since the last call and resets the counter.
     *
     * Only meaningful on Linux (V4L2) live devices; playback and Windows return 0.
     */
    O3P_API virtual uint64_t getAndResetDroppedFrames() { return 0; }

    /**
     * @brief Check if an info is supported
     */
    O3P_API bool supportsInfo(CameraInfo info) const override;

    /**
     * @brief Get the info value
     * @throw std::runtime_error if info is not supported
     */
    O3P_API std::string getInfo(CameraInfo info) const override;

    /**
     * @brief Check if an option is supported
     */
    O3P_API bool supportsOption(OptionType optionType) const override;

    /**
     * @brief Get an option by type
     * @throw std::runtime_error if option is not supported
     */
    O3P_API std::shared_ptr<Option> getOption(OptionType optionType) override;

  protected:
    bool m_initialized = false;
    std::string m_id = "";
    InfoContainer m_infoContainer;
    OptionsContainer m_optionsContainer;
};

} // namespace o3p

#endif
