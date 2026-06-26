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

#ifndef DSHELPER_HPP
#define DSHELPER_HPP

#include <windows.h>

#include <dshow.h>

#include <atomic>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>

#include <dbt.h>

#include <atlbase.h>
#include <vidcap.h>

#include <o3p/Device.hpp>
#include <o3p/UvcDeviceWin.hpp>
#include <o3p/WindowsCommon.hpp>

#include <o3p/SampleGrabberCallback.hpp>

namespace o3p {

class SampleGrabberCallback;

/**
 * @brief Helper class to open Uvc Devices using Direct Show
 */
class DsHelper : public UvcDeviceWin {
  public:
    /**
     * @brief Basic constructor
     */
    DsHelper();

    /**
     * @brief Basic destructor
     */
    ~DsHelper();

    /**
     * @brief Create Camera from given VID, PID, MI, GUID and ID
     */
    void createCamera(uint32_t vid, uint32_t pid, uint32_t mi, const Guid &xu, std::string id) override;

    /**
     * @brief Close camera
     */
    void closeCamera() override;

    /**
     * @brief Read Sample from device
     */
    void readSample() override;

    /**
     * @brief Set Extension unit value
     */
    void setExtensionUnitValue(ExtensionDataId dataId, const uint8_t *data, uint16_t len) override;

    /**
     * @brief Get Extension unit value
     */
    void getExtensionUnitValue(ExtensionDataId dataId, uint8_t *data, uint16_t len) override;

    /**
     * @brief Get all available cameras
     */
    static std::vector<std::string> getAvailableCameras();

    /**
     * @brief Get all currently connected o3p devices via DirectShow
     */
    static std::vector<std::shared_ptr<o3p::Device>> getConnectedDevices();

    /**
     * @brief Start camera
     */
    void startCamera() override;

    /**
     * @brief Stop Camera
     */
    void stopCamera() override;

    /**
     * @brief Get dimensions, i.e. width and height
     */
    void getDimensions(uint32_t &width, uint32_t &height);

    /**
     * @brief Find extension node id based on GUID
     */
    DWORD findExtensionNodeId(const Guid &xuGuid);

    /**
     * @brief Handle a USB disconnect notification
     *
     * @param hwnd Window handle that received the notification
     * @param devicePath Device path of the disconnected device
     */
    void OnUSBDisconnected(HWND hwnd, LPCWSTR devicePath);

    static void registerInstance(DsHelper *instance);
    static void unregisterInstance(DsHelper *instance);
    static std::unordered_set<DsHelper *> s_instances;
    static std::mutex s_instancesMutex;

  private:
    IGraphBuilder *m_pGraph = nullptr;
    ICaptureGraphBuilder2 *m_pBuilder = nullptr;
    IBaseFilter *m_pCaptureFilter = nullptr;
    ISampleGrabber *m_pGrabber = nullptr;
    IMediaControl *m_pControl = nullptr;
    Guid m_xuGuid;
    CComPtr<IKsControl> m_control;
    KSP_NODE m_extensionNode;
    DWORD m_extensionNodeId = 0;

    HANDLE m_hSampleReadyEvent;
    SampleGrabberCallback *m_callback;
};

} // namespace o3p

#endif
