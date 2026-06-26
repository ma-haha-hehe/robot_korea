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

#ifndef O3P_USB_INTERFACE_LIBUSB_HPP
#define O3P_USB_INTERFACE_LIBUSB_HPP

#include "o3p/usb/UsbInterface.hpp"
#include <libusb-1.0/libusb.h>
#include <memory>
#include <vector>

namespace o3p {

/**
 * @brief libusb based implementation of UsbInterface
 */
class UsbInterfaceLibusb : public UsbInterface {
  public:
    /**
     * @brief Wrap a libusb interface
     *
     * @param inf The libusb interface to wrap
     */
    UsbInterfaceLibusb(libusb_interface inf);
    virtual ~UsbInterfaceLibusb();

    virtual uint8_t getNumber() const override { return m_desc.bInterfaceNumber; }
    virtual uint8_t getClass() const override { return m_desc.bInterfaceClass; }
    virtual uint8_t getSubClass() const override { return m_desc.bInterfaceSubClass; }
    virtual const std::vector<std::shared_ptr<UsbEndpoint>> getEndpoints() const override { return m_endpoints; }
    virtual const std::shared_ptr<UsbEndpoint> firstEndpoint(const UsbEndpointDirection direction, const UsbEndpointType type = UsbEndpointType::USB_ENDPOINT_BULK) const override;

  private:
    libusb_interface_descriptor m_desc;
    std::vector<std::shared_ptr<UsbEndpoint>> m_endpoints;
};

} // namespace o3p

#endif // O3P_USB_INTERFACE_LIBUSB_HPP