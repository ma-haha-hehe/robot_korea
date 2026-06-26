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

#ifndef DEVICEO3P_HPP
#define DEVICEO3P_HPP

#include <o3p/Device.hpp>
#include <o3p/SensorComposite.hpp>
#include <o3p/Update.hpp>
#include <o3p/option/BackendOption.hpp>

namespace o3p {

/**
 * @brief Represents the camera device
 */
class DeviceO3P : public Device, public Updatable, public FirmwareOptionInterface, public OptionObserver {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API DeviceO3P(const std::string &id);

    /**
     * @brief Default destructor
     */
    O3P_API ~DeviceO3P() override;

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

    virtual uint32_t getNumberOfFrames() override;

    O3P_API std::vector<uint8_t> sendPluginMessage(const std::vector<uint8_t> &message) override;
    O3P_API void setConfigValue(const std::string &config, const std::string &value) override;
    O3P_API std::string getConfigValue(const std::string &config) override;
    O3P_API void saveConfig() override;
    O3P_API void writeEeprom(const std::vector<uint8_t> &value) override;
    O3P_API std::vector<uint8_t> readEeprom() override;
    O3P_API std::string getLicenseInfo() override;

    O3P_API void setDisconnectCallback(DisconnectCallback cb) override;

    /**
     * @brief Get the firmware logger for this device
     */
    O3P_API std::shared_ptr<FwLogger> getFwLogger() override;

    /**
     * @brief Returns V4L2 dropped-frame count since last call (Linux only; 0 otherwise).
     */
    O3P_API uint64_t getAndResetDroppedFrames() override;

    O3P_API void enterUpdateState() override;

    O3P_API void onOptionUpdated(const Option &option) override;

  protected:
    O3P_API virtual OptionInfo requestOptionInfo(OptionType optionType) override;
    O3P_API virtual OptionValue requestOptionValue(OptionType optionType) override;
    O3P_API virtual bool requestOptionValueUpdate(Option &option, const OptionValue &value) override;

    /**
     * @brief Load current options from device firmware. If an option is already present, it will be refreshed.
     */
    void loadOptions();

    /**
     * @brief Load a specific option from device firmware. If the option is already present, it will be refreshed.
     */
    void loadOption(OptionType optionType);

    std::unique_ptr<SensorComposite> m_sensorComposite;
    std::string m_reqId;
    std::shared_ptr<FwLogger> m_fwLogger;
};

} // namespace o3p

#endif
