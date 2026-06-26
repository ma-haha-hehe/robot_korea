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

#ifndef V4L2HELPER_HPP
#define V4L2HELPER_HPP

#include <o3p/Device.hpp>
#include <o3p/UvcDevice.hpp>

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <libudev.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <memory>
#include <mutex>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace o3p {

/**
 * @brief V4L2 UVC device helper with asynchronous frame capture.
 *
 * A background thread continuously dequeues frames from the V4L2 driver and
 * copies the latest one into an internal buffer.  readSample() blocks until
 * a new frame is available, matching the DSHelper/DirectShow behaviour.
 */
class V4l2Helper : public UvcDevice {

  public:
    /**
     * @brief Construct a new V4l2Helper instance
     */
    V4l2Helper();

    /**
     * @brief Destroy the V4l2Helper, stopping any background threads
     */
    ~V4l2Helper();

    /**
     * @brief Open the V4L2 UVC camera identified by the given parameters
     *
     * @param vid USB vendor id
     * @param pid USB product id
     * @param mi Multi-interface index
     * @param xu GUID of the extension unit
     * @param id Identifier of the camera
     */
    void createCamera(uint32_t vid, uint32_t pid, uint32_t mi, const Guid &xu, std::string id);

    /**
     * @brief Get the value of an extension unit
     *
     * @param dataId Extension data identifier
     * @param data Buffer that receives the data
     * @param len Length of the buffer in bytes
     */
    void getExtensionUnitValue(ExtensionDataId dataId, uint8_t *data, uint16_t len);

    /**
     * @brief Set the value of an extension unit
     *
     * @param dataId Extension data identifier
     * @param data Pointer to the data to write
     * @param len Length of the data in bytes
     */
    void setExtensionUnitValue(ExtensionDataId dataId, const uint8_t *data, uint16_t len);

    /**
     * @brief Close the camera and stop the capture thread
     */
    void closeCamera();

    /**
     * @brief Block until a new frame is available and copy it into the data buffer
     */
    void readSample();

    /**
     * @brief Get all o3p devices currently connected via V4L2
     */
    static std::vector<std::shared_ptr<o3p::Device>> getConnectedDevices();

    /**
     * @brief Get the identifiers of all available cameras via V4L2
     */
    static std::vector<std::string> getAvailableCameras();

    /**
     * @brief Start the camera stream
     */
    void startCamera() override;

    /**
     * @brief Stop the camera stream
     */
    void stopCamera() override;

    /**
     * @brief Returns the number of frames dropped since the last call to this function.
     */
    uint64_t getAndResetDroppedFrames() override;

  protected:
    bool getResolution(int fd, uint32_t &width, uint32_t &height);

    int openUVCDevice(uint32_t vid, uint32_t pid, uint32_t interfaceNumber, std::string id);

    bool initMmap(int fd);

    /** Background capture loop executed in m_captureThread. */
    void captureLoop();

    /** Background udev monitor loop executed in m_monitorThread. */
    void usbMonitorLoop();

    // Number of mmap capture buffers requested from the driver.
    static constexpr int NUM_BUFFERS = 8;

    int m_fd;
    uint32_t m_extensionUnitId;
    std::vector<Buffer> m_mappedBuffers;

    // --- async capture state ---
    std::thread m_captureThread;
    std::atomic<bool> m_captureRunning{false};

    std::mutex m_frameMutex;
    std::condition_variable m_frameReady;
    std::vector<uint8_t> m_internalBuffer; ///< copy of latest frame data
    uint64_t m_internalTimestamp{0};
    size_t m_internalLength{0};
    bool m_newFrameAvailable{false};

    std::atomic<uint64_t> m_droppedFrames{0};

    std::thread m_monitorThread;
    std::atomic<bool> m_monitorRunning{false};
};

} // namespace o3p

#endif
