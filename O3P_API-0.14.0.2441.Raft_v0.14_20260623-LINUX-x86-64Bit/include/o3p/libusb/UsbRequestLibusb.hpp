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

#ifndef O3P_USB_REQUEST_LIBUSB_HPP
#define O3P_USB_REQUEST_LIBUSB_HPP

#include "o3p/libusb/LibusbHandle.hpp"
#include "o3p/usb/UsbRequest.hpp"
#include <libusb-1.0/libusb.h>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>

namespace o3p {

/**
 * @brief libusb based implementation of UsbRequest
 */
class UsbRequestLibusb : public UsbRequest, public std::enable_shared_from_this<UsbRequestLibusb> {
  public:
    /**
     * @brief Create a libusb request bound to the given endpoint and handle
     *
     * @param endpoint Endpoint the request operates on
     * @param handle Open libusb handle to use for the transfer
     */
    UsbRequestLibusb(const std::shared_ptr<UsbEndpoint> &endpoint, const std::shared_ptr<LibusbHandle> &handle);
    virtual ~UsbRequestLibusb();

    void setBuffer(const std::vector<uint8_t> &buffer) override;

    void *getNativeRequest() const override { return m_transfer.get(); }

    /**
     * @brief Mark the request as currently in-flight or finished
     *
     * @param active true if the request is currently in-flight
     */
    void setActive(bool active) { m_active = active; }
    void waitUntilInactive() override;

    /**
     * @brief Store a weak self reference for use in libusb callbacks
     *
     * @param ptr Shared pointer to this request
     */
    void setShared(const std::shared_ptr<UsbRequest> &ptr) {
        m_weakPtrThis = ptr;
    }

    /**
     * @brief Lock and return the stored self reference
     */
    std::shared_ptr<UsbRequest> getShared() {
        return m_weakPtrThis.lock();
    }

  private:
    std::atomic<bool> m_active{false};
    std::mutex m_inactiveMutex;
    std::condition_variable m_inactiveCv;
    std::shared_ptr<LibusbHandle> m_handle;
    std::shared_ptr<libusb_transfer> m_transfer;
    std::weak_ptr<UsbRequest> m_weakPtrThis;
};

} // namespace o3p

#endif /* O3P_USB_REQUEST_LIBUSB_HPP */
