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

#ifndef O3P_USB_REQUEST_WINUSB_HPP
#define O3P_USB_REQUEST_WINUSB_HPP

#include "o3p/usb/UsbDevice.hpp"
#include "o3p/usb/UsbRequest.hpp"

#include <Windows.h>
#include <winusb.h>

namespace o3p {

/**
 * @brief WinUSB based implementation of UsbRequest
 */
class UsbRequestWinusb : public UsbRequest {
  public:
    /**
     * @brief Construct a WinUSB request bound to the given device and endpoint
     *
     * @param device The owning USB device
     * @param endpoint The endpoint the request operates on
     */
    UsbRequestWinusb(std::shared_ptr<UsbDevice> device, const std::shared_ptr<UsbEndpoint> &endpoint);
    virtual ~UsbRequestWinusb();

    void *getNativeRequest() const override { return m_overlapped.get(); }

  private:
    std::shared_ptr<OVERLAPPED> m_overlapped;
    std::shared_ptr<HANDLE> m_handle;
};

} // namespace o3p

#endif // O3P_USB_REQUEST_WINUSB_HPP