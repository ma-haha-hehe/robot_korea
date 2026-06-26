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

#ifndef O3P_FW_UPDATE_DEVICE_HPP
#define O3P_FW_UPDATE_DEVICE_HPP

#include "o3p/Common.hpp"
#include "o3p/Device.hpp"
#include "o3p/FwUpdate.hpp"
#include "o3p/usb/UsbDevice.hpp"
#include "o3p/usb/UsbRequest.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>

namespace o3p {

class FwUpdateDevice : public o3p::Device {
  public:
    FwUpdateDevice(std::shared_ptr<UsbDevice> usbDevice);

    virtual ~FwUpdateDevice();

    void init() override;

    void start() override {
        throw std::runtime_error("Not implemented");
    }

    void stop() override {
        throw std::runtime_error("Not implemented");
    }

    void closeDevice() override {
        throw std::runtime_error("Not implemented");
    }

    void waitForFrames(o3p::FrameSet *frameSet) override {
        throw std::runtime_error("Not implemented");
    }

    std::vector<std::shared_ptr<o3p::Stream>> getStreams() override {
        throw std::runtime_error("Not implemented");
    }

    virtual std::vector<uint8_t> sendPluginMessage(const std::vector<uint8_t> &message) override {
        throw std::runtime_error("Not implemented");
    }

    virtual void setConfigValue(const std::string &config, const std::string &value) override {
        throw std::runtime_error("Not implemented");
    }

    virtual std::string getConfigValue(const std::string &config) override {
        throw std::runtime_error("Not implemented");
    }

    virtual void saveConfig() override {
        throw std::runtime_error("Not implemented");
    }

    virtual void writeEeprom(const std::vector<uint8_t> &value) override {
        throw std::runtime_error("Not implemented");
    }

    virtual std::vector<uint8_t> readEeprom() override {
        throw std::runtime_error("Not implemented");
    }

    virtual std::string getLicenseInfo() override {
        throw std::runtime_error("Not implemented");
    }

    O3P_API void update(std::vector<uint8_t> fw_image, std::function<void(const float progress)> progressCallback);

    virtual void setDisconnectCallback(o3p::DisconnectCallback cb) override;

    virtual uint32_t getNumberOfFrames() override;

    O3P_API std::shared_ptr<FwLogger> getFwLogger() override { return nullptr; }

  private:
    void sendStartUpdateCommand(uint64_t updateFileSize);
    void sendCancelUpdateCommand();
    void sendFwImage(const std::vector<uint8_t> &fw_image);
    void sendUpdateMessage(UpdateMessage &msg, bool waitForAck = true);
    void handleUpdateMessage(const UpdateMessage &msg);
    void handleStatusNotification(const StatusNotification &status);
    void handleProgressNotification(const ProgressNotification &progress);
    void handleErrorNotification(const ErrorNotification &error);
    UpdateStatus requestStatus();
    uint32_t requestProgress();

    std::shared_ptr<UsbDevice> m_usbDevice;
    std::shared_ptr<UsbMessenger> m_usbMessenger;
    std::shared_ptr<UsbRequest> m_usbReadRequest;
    std::shared_ptr<UsbRequest> m_usbWriteRequest;

    // Transaction management (send a message and wait for response)
    uint32_t m_transactionId;
    bool m_isSendingMessage;
    std::condition_variable m_transactionCondition;
    std::mutex m_transactionMutex;
    std::atomic<uint32_t> m_lastReceivedTransactionId;

    std::atomic<bool> m_isUpdating;
    UpdateStatus m_currentStatus;
    uint32_t m_currentProgress;
    std::function<void(const float progress)> m_progressCallback;
};

} // namespace o3p

#endif /* O3P_FW_UPDATE_DEVICE_HPP */