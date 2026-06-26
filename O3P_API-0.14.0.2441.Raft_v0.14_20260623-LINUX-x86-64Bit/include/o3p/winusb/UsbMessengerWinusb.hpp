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

#ifndef O3P_USB_MESSENGER_WINUSB_HPP
#define O3P_USB_MESSENGER_WINUSB_HPP

#include "o3p/concurrency/Concurrency.hpp"
#include "o3p/usb/UsbMessenger.hpp"
#include "o3p/winusb/UsbDeviceWinusb.hpp"
#include "o3p/winusb/WinusbHandle.hpp"

#include <memory>

namespace o3p {

/**
 * @brief WinUSB based implementation of UsbMessenger
 */
class UsbMessengerWinusb : public UsbMessenger {
  public:
    /**
     * @brief Construct a messenger for the given device and handle
     *
     * @param device The WinUSB backed device
     * @param handle Open WinUSB handle for the device
     */
    UsbMessengerWinusb(const std::shared_ptr<UsbDeviceWinusb> &device, const std::shared_ptr<WinusbHandle> &handle);
    virtual ~UsbMessengerWinusb();

    virtual bool bulkTransfer(const std::shared_ptr<UsbEndpoint> &endpoint, uint8_t *buffer, uint32_t length, uint32_t &transferred, uint32_t timeout_ms) override;
    virtual bool submitRequest(std::shared_ptr<UsbRequest> request) override;
    virtual bool cancelRequest(std::shared_ptr<UsbRequest> request) override;
    virtual std::shared_ptr<UsbRequest> createRequest(const std::shared_ptr<UsbEndpoint> &endpoint) override;

  private:
    std::shared_ptr<WorkQueue> getWorkQueue(uint8_t endpoint);
    std::shared_ptr<WinusbHandle> m_handle;
    std::shared_ptr<UsbDeviceWinusb> m_device;
    std::map<uint8_t, std::shared_ptr<WorkQueue>> m_workQueues;
};

} // namespace o3p

#endif // O3P_USB_MESSENGER_WINUSB_HPP