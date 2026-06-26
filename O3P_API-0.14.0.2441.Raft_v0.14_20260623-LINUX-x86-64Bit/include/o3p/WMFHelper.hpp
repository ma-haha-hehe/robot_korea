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

#ifndef WMFHELPER_HPP
#define WMFHELPER_HPP

#include <Windows.h>

#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ks.h>
#include <ksproxy.h>
#include <mfidl.h>

#include <vidcap.h>

#include <Mferror.h>
#include <atlbase.h>
#include <mfapi.h>
#include <mfreadwrite.h>

#include <comdef.h>
#include <ksmedia.h>
#include <sstream>

#include <o3p/Device.hpp>
#include <o3p/UvcDeviceWin.hpp>
#include <o3p/WindowsCommon.hpp>

namespace o3p {

/**
 * @brief Base class to work with Windows Media Foundation
 */
class WmfHelper : public UvcDeviceWin {
  public:
    /**
     * @brief The constructor
     */
    WmfHelper();

    /**
     * @brief The destructor
     */
    ~WmfHelper() override;

    /**
     * @brief Create the camera identified by the given ids and extension unit GUID
     *
     * @param vid USB vendor id
     * @param pid USB product id
     * @param mi Multi-interface index
     * @param xu GUID of the extension unit
     * @param id Identifier of the camera
     */
    void createCamera(uint32_t vid, uint32_t pid, uint32_t mi, const Guid &xu, std::string id) override;

    /**
     * @brief Close the currently opened camera
     */
    void closeCamera() override;

    /**
     * @brief Get dimensions
     *
     * @param width The width
     * @param height The height
     */
    void getDimensions(uint32_t &width, uint32_t &height);

    /**
     * @brief Read a sample from the device into the data buffer
     */
    void readSample() override;

    /**
     * @brief Set the value of an extension unit
     *
     * @param dataId Extension data identifier
     * @param data Pointer to the data to write
     * @param len Length of the data in bytes
     */
    void setExtensionUnitValue(ExtensionDataId dataId, const uint8_t *data, uint16_t len) override;

    /**
     * @brief Get the value of an extension unit
     *
     * @param dataId Extension data identifier
     * @param data Buffer to receive the data
     * @param len Length of the buffer in bytes
     */
    void getExtensionUnitValue(ExtensionDataId dataId, uint8_t *data, uint16_t len) override;

    /**
     * @brief Get all o3p devices currently exposed via Windows Media Foundation
     */
    static std::vector<std::shared_ptr<o3p::Device>> getConnectedDevices();

    /**
     * @brief Get the identifiers of all available cameras via Windows Media Foundation
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
     * @brief Handle a USB disconnect notification
     *
     * @param hwnd Window handle that received the notification
     * @param devicePath Device path of the disconnected device
     */
    void OnUSBDisconnected(HWND hwnd, LPCWSTR devicePath);

  protected:
    /**
     * @brief Extract Vendor, Product and Interface from given Device ID
     *
     * @param deviceId Device ID
     * @param vid Vendor ID
     * @param pid Product ID
     * @param mi Interface ID
     */
    static void GetDeviceModelId(const std::string &deviceId, uint32_t &vid, uint32_t &pid, uint32_t &mi);

    /**
     * @brief Searches for given vendor extension
     *
     * @param mediaSource Source
     * @param vendorExtension The requested vendor extension
     * @param vendorExtensionNode The node of the vendor extension
     * @param xu The GUID
     *
     * @return Status that indicates success or failure
     */
    static HRESULT searchForVendorExtension(IUnknown *mediaSource,
                                            CComPtr<IKsControl> &vendorExtension,
                                            KSP_NODE &vendorExtensionNode,
                                            const Guid &xu);

    std::vector<uint8_t> m_internalBuffer;

    long m_nRefCount; // Reference count.

    IMFSourceReader *m_sourceReader;
    IMFMediaSource *m_mediaSource;
    CRITICAL_SECTION m_critsec;

    std::mutex m_acquisitionMutex;
    std::condition_variable m_acquisitionCv;

    CComPtr<IKsControl> m_vendorExtension;
    KSP_NODE m_vendorExtensionNode;
};

} // namespace o3p

#endif
