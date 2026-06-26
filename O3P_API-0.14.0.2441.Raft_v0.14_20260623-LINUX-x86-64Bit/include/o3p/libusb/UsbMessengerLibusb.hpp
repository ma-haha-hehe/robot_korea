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

#ifndef O3P_USB_MESSENGER_LIBUSB_HPP
#define O3P_USB_MESSENGER_LIBUSB_HPP

#include "o3p/libusb/LibusbHandle.hpp"
#include "o3p/libusb/UsbDeviceLibusb.hpp"
#include "o3p/usb/UsbMessenger.hpp"

#include <memory>

namespace o3p {

/**
 * @brief libusb based implementation of UsbMessenger
 */
class UsbMessengerLibusb : public UsbMessenger {
  public:
    /**
     * @brief Construct a messenger for the given device and handle
     *
     * @param device The libusb device the messenger operates on
     * @param handle An opened/claimed libusb handle for the interface
     */
    UsbMessengerLibusb(const std::shared_ptr<UsbDeviceLibusb> &device, const std::shared_ptr<LibusbHandle> &handle);
    virtual ~UsbMessengerLibusb() = default;

    virtual bool bulkTransfer(const std::shared_ptr<UsbEndpoint> &endpoint, uint8_t *buffer, uint32_t length, uint32_t &transferred, uint32_t timeout_ms) override;
    virtual bool submitRequest(std::shared_ptr<UsbRequest> request) override;
    virtual bool cancelRequest(std::shared_ptr<UsbRequest> request) override;
    virtual std::shared_ptr<UsbRequest> createRequest(const std::shared_ptr<UsbEndpoint> &endpoint) override;

  private:
    std::shared_ptr<UsbDeviceLibusb> m_device;
    std::shared_ptr<LibusbHandle> m_handle;
};

} // namespace o3p

#endif // O3P_USB_MESSENGER_LIBUSB_HPP