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

#ifndef O3P_LIB_USB_HANDLE_HPP
#define O3P_LIB_USB_HANDLE_HPP

#include "o3p/libusb/LibusbContext.hpp"
#include "o3p/libusb/UsbInterfaceLibusb.hpp"
#include <libusb-1.0/libusb.h>
#include <stdexcept>

namespace o3p {

/**
 * @brief RAII handle for an opened and claimed libusb interface
 */
class LibusbHandle {
  public:
    /**
     * @brief Open the device and claim the given interface
     *
     * @param context The libusb context to operate in
     * @param device The libusb device to open
     * @param intf The interface to claim
     * @throws std::invalid_argument if any argument is null
     * @throws std::runtime_error if opening or claiming fails
     */
    LibusbHandle(std::shared_ptr<LibusbContext> context, libusb_device *device, std::shared_ptr<UsbInterfaceLibusb> intf) : m_context(context), m_interface(intf) {
        if (!context || !device || !intf) {
            throw std::invalid_argument("Invalid argument(s) provided to LibusbHandle constructor");
        }

        if (libusb_open(device, &m_handle) != LIBUSB_SUCCESS) {
            throw std::runtime_error("Failed to open device");
        }

        // On Linux the kernel may have a driver attached to the interface (e.g. uvcvideo
        // or a generic driver). Ask libusb to detach it automatically when we claim so
        // that claim_interface succeeds. The kernel driver is re-attached on release.
        // This call returns LIBUSB_ERROR_NOT_SUPPORTED on platforms that don't need it,
        // which is safe to ignore.
        libusb_set_auto_detach_kernel_driver(m_handle, 1);

        // Claim the interface
        if (libusb_claim_interface(m_handle, intf->getNumber()) != LIBUSB_SUCCESS) {
            libusb_close(m_handle);
            m_handle = nullptr;
            throw std::runtime_error("Failed to claim interface");
        }

        m_context->startEventHandler();
    }

    ~LibusbHandle() {
        if (m_handle) {
            m_context->stopEventHandler();
            libusb_release_interface(m_handle, m_interface->getNumber());
            libusb_close(m_handle);
        }
    }

    /**
     * @brief Get the underlying libusb device handle
     */
    libusb_device_handle *getNativeHandle() const { return m_handle; }

  private:
    std::shared_ptr<LibusbContext> m_context;
    std::shared_ptr<UsbInterfaceLibusb> m_interface;
    libusb_device_handle *m_handle = nullptr;
};

} // namespace o3p

#endif // O3P_LIB_USB_HANDLE_HPP
