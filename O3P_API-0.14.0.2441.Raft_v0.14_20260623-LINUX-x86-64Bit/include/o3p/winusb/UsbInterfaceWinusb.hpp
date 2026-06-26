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

#ifndef O3P_USB_INTERFACE_WINUSB_HPP
#define O3P_USB_INTERFACE_WINUSB_HPP

#include "o3p/usb/UsbInterface.hpp"

#include <string>
#include <windows.h>
#include <winusb.h>

namespace o3p {

/**
 * @brief WinUSB based implementation of UsbInterface
 */
class UsbInterfaceWinusb : public UsbInterface {
  public:
    /**
     * @brief Construct a WinUSB based interface wrapper
     *
     * @param handle WinUSB interface handle
     * @param info USB interface descriptor
     * @param device_path Windows device path of the underlying device
     */
    UsbInterfaceWinusb(WINUSB_INTERFACE_HANDLE handle, USB_INTERFACE_DESCRIPTOR info, const std::wstring &device_path);

    virtual ~UsbInterfaceWinusb() = default;

    virtual uint8_t getNumber() const override { return m_info.bInterfaceNumber; }
    virtual uint8_t getClass() const override { return m_info.bInterfaceClass; }
    virtual uint8_t getSubClass() const override { return m_info.bInterfaceSubClass; }
    virtual const std::vector<std::shared_ptr<UsbEndpoint>> getEndpoints() const override { return m_endpoints; }
    virtual const std::shared_ptr<UsbEndpoint> firstEndpoint(const UsbEndpointDirection direction, const UsbEndpointType type = UsbEndpointType::USB_ENDPOINT_BULK) const override;

    /**
     * @brief Get the Windows device path of the device this interface belongs to
     */
    const std::wstring &getDevicePath() const { return m_devicePath; }

  private:
    std::wstring m_devicePath;
    USB_INTERFACE_DESCRIPTOR m_info;
    std::vector<std::shared_ptr<UsbEndpoint>> m_endpoints;
};

} // namespace o3p

#endif // O3P_USB_INTERFACE_WINUSB_HPP