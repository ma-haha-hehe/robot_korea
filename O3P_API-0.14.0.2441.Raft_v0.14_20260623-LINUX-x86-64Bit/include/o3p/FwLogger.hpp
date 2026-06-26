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

#ifndef O3P_FW_LOGGER_HPP
#define O3P_FW_LOGGER_HPP

#include <o3p/Definitions.hpp>
#include <o3p/usb/UsbDevice.hpp>
#include <o3p/usb/UsbCommon.hpp>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <condition_variable>

namespace o3p {

enum FwLoggerMessageType {
    FW_LOG_MSG_REQUEST_LOGS = 1,
    FW_LOG_MSG_LOG_DATA = 2,
    FW_LOG_MSG_REQUEST_CRASH_LOGS = 3,
    FW_LOG_MSG_CRASH_LOG_DATA = 4
};

#define O3P_FW_LOG_MSG_MAGIC 0xAA
#define O3P_FW_LOG_MSG_MAX_SIZE 1024

#pragma pack(push, 1)
struct FwLoggerMessage {
    struct {
        const uint8_t magic = O3P_FW_LOG_MSG_MAGIC;
        uint8_t type;
        uint8_t reserved[3];
        uint32_t length;
    } header;
    uint8_t payload[O3P_FW_LOG_MSG_MAX_SIZE - sizeof(header)];
};
#pragma pack(pop)

class FwLogger {
public:
    O3P_API FwLogger(std::shared_ptr<UsbDevice> usbDevice, uint8_t intf);
    O3P_API ~FwLogger();

    /**
     * Starts collecting live firmware logs from the device. The provided callback will be called for each log message received.
     * This is an asynchronous function call that returns immediately. Logs will be received in the callback until stopCollecting 
     * is called.
     * 
     * @param logCallback Callback function to receive log messages. The log message is provided as a null-terminated C string.
     * @param pollIntervalMs Interval in milliseconds at which to poll the device for new logs. Default is 100ms. Setting this too 
     * low may cause increased CPU usage.
     * @return True if log collection was successfully started, false if it was already running or if there was an error starting 
     * log collection.
     */
    O3P_API bool startCollecting(std::function<void(const char* log)> logCallback, uint32_t pollIntervalMs = 100);

    /**
     * Stops collecting live firmware logs from the device. After calling this function, the log callback provided to startCollecting 
     * will no longer be called.
     */
    O3P_API void stopCollecting();

    /**
     * Collects crash logs from the device. The provided callback will be called for each crash log message received. 
     * This function will block until all crash logs have been collected.
     * 
     * @param logCallback Callback function to receive crash log messages. The log message is provided as a null-terminated C string.
     * @return True if crash log collection was successfully started, false if it was already running or if there was an error starting 
     * crash log collection.
     */
    O3P_API bool collectCrashLogs(std::function<void(const char* log)> logCallback);

private:
    void handleLogDataMessage(const FwLoggerMessage* msg);
    void handleCrashLogDataMessage(const FwLoggerMessage* msg);
    bool startReadLogsLoop();
    void stopReadLogsLoop();
    bool shouldStopReadLogsLoop();
    
    std::shared_ptr<UsbDevice> m_usbDevice;
    std::shared_ptr<UsbMessenger> m_usbMessenger;
    std::shared_ptr<UsbRequest> m_usbReadRequest;
    std::shared_ptr<UsbEndpoint> m_writeEp;
    std::shared_ptr<UsbEndpoint> m_readEp;

    std::mutex m_readLoopMutex;
    bool m_readLoopRunning;

    std::mutex m_liveLogCollectionMutex;
    bool m_collectingLiveLogs;
    std::function<void(const char* log)> m_liveLogCallback;
    std::thread m_liveLogQueryThread;
    
    std::mutex m_crashLogCollectionMutex;
    bool m_collectingCrashLogs;
    std::function<void(const char* log)> m_crashLogCallback;
    std::condition_variable m_crashLogCollectionFinishedCv;
};

} // namespace o3p

#endif /* O3P_FW_LOGGER_HPP */