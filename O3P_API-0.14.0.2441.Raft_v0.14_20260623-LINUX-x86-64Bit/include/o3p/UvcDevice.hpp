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

#ifndef UVCDEVICE_HPP
#define UVCDEVICE_HPP

#include <cstdint>
#include <o3p/Common.hpp>
#include <o3p/Message.hpp>

namespace o3p {

/**
 * @brief Structure representing a buffer.
 */
struct Buffer {
    uint8_t *start;     ///< Pointer to the start of the buffer.
    size_t length;      ///< Length of the buffer in bytes.
    uint64_t timestamp; ///< The time when the buffer was received
};

/**
 * @brief Abstract class representing a UVC device.
 */
class UvcDevice {
  public:
    virtual ~UvcDevice() = default;

    /**
     * @brief Creates a camera with the specified parameters.
     *
     * @param vid Vendor ID of the camera.
     * @param pid Product ID of the camera.
     * @param mi Interface number of the camera.
     * @param xu GUID of the extension unit.
     * @param id Identifier of the camera.
     */
    virtual void createCamera(uint32_t vid, uint32_t pid, uint32_t mi, const Guid &xu, std::string id) = 0;

    /**
     * @brief Gets the value of an extension unit.
     *
     * @param dataId Identifier of the extension data.
     * @param data Pointer to the buffer where the data will be stored.
     * @param len Length of the data buffer.
     */
    virtual void getExtensionUnitValue(ExtensionDataId dataId, uint8_t *data, uint16_t len) = 0;

    /**
     * @brief Sets the value of an extension unit.
     *
     * @param dataId Identifier of the extension data.
     * @param data Pointer to the data to be set.
     * @param len Length of the data.
     */
    virtual void setExtensionUnitValue(ExtensionDataId dataId, const uint8_t *data, uint16_t len) = 0;

    /**
     * @brief Sets a firmware configuration value
     *
     * @throw std::runtime_error if pipeline is not started or config is rejected by firmware
     *
     * @param key The name of the config value
     * @param value The value to set
     */
    virtual void setConfigValue(const std::string &key, const std::vector<uint8_t> &value);

    /**
     * @brief Get a firmware configuration value
     *
     * @return Byte vector representation of config value. Empty if config does not exist.
     */
    virtual std::vector<uint8_t> getConfigValue(const std::string &key);

    /**
     * @brief Save the current configuration to the device's non-volatile memory
     *
     * @throw std::runtime_error if saving fails
     */
    virtual void saveConfig();

    /**
     * @brief Writes values to the EEPROM of the camera.
     *
     * @throw std::runtime_error if pipeline is not initialized
     *
     * @param value The value to set
     */
    virtual void writeEeprom(const std::vector<uint8_t> &value);

    /**
     * @brief Reads the EEPROM of the camera.
     *
     * @return Byte vector representation of the EEPROM data.
     */
    virtual std::vector<uint8_t> readEeprom();
    virtual std::string getLicenseInfo();

    /**
     * @brief Send a low level message to the device
     *
     * @param msg Message to send
     */
    virtual void sendMessage(const message::Message &msg);

    /**
     * @brief Receive a low level message from the device
     *
     * @return The received message
     */
    virtual message::Message receiveMessage();

    /**
     * @brief Closes the camera.
     */
    virtual void closeCamera() = 0;

    /**
     * @brief Reads a sample from the camera.
     */
    virtual void readSample() = 0;

    [[nodiscard]] const std::string &getId() const { return m_id; } ///< Returns the camera ID.

    /**
     * @brief Starts the camera stream.
     */
    virtual void startCamera() = 0;

    /**
     * @brief Stops the camera stream.
     */
    virtual void stopCamera() = 0;

    /**
     * @brief Returns the number of frames dropped since the last call and resets the counter.
     *
     * Only meaningful on Linux (V4L2); other backends return 0.
     */
    virtual uint64_t getAndResetDroppedFrames() { return 0; }

    /**
     * @brief Register a callback that is invoked when the device is disconnected
     *
     * @param cb Callback to invoke on disconnect
     */
    void setDisconnectCallback(DisconnectCallback cb);
    DisconnectCallback m_onDisconnect;

    Buffer m_dataBuffer; ///< Buffer for storing data.

    uint32_t m_width = 0u;  ///< Width of the camera image.
    uint32_t m_height = 0u; ///< Height of the camera image.

    std::string m_id; ///< Identifier of the camera.
};
} // namespace o3p

#endif // UVCDEVICE_HPP
