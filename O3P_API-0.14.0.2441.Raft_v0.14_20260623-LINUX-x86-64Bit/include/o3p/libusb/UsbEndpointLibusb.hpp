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

#ifndef O3P_USB_ENDPOINT_LIBUSB_HPP
#define O3P_USB_ENDPOINT_LIBUSB_HPP

#include "o3p/usb/UsbCommon.hpp"
#include "o3p/usb/UsbEndpoint.hpp"
#include <libusb-1.0/libusb.h>

namespace o3p {

/**
 * @brief libusb based implementation of UsbEndpoint
 */
class UsbEndpointLibusb : public UsbEndpoint {
  public:
    /**
     * @brief Wrap a libusb endpoint descriptor
     *
     * @param desc The endpoint descriptor
     * @param interface_number Number of the interface this endpoint belongs to
     */
    UsbEndpointLibusb(libusb_endpoint_descriptor desc, uint8_t interface_number) : m_desc(desc), m_interfaceNumber(interface_number) {}

    virtual ~UsbEndpointLibusb() = default;

    virtual uint8_t getAddress() const override { return m_desc.bEndpointAddress; }
    virtual UsbEndpointType getType() const override { return static_cast<UsbEndpointType>(m_desc.bmAttributes); }
    virtual uint8_t getInterfaceNumber() const override { return m_interfaceNumber; }
    virtual UsbEndpointDirection getDirection() const override {
        return m_desc.bEndpointAddress >= USB_ENDPOINT_DIRECTION_READ ? USB_ENDPOINT_DIRECTION_READ : USB_ENDPOINT_DIRECTION_WRITE;
    }

    /**
     * @brief Get the wrapped libusb endpoint descriptor
     */
    libusb_endpoint_descriptor getDescriptor() const { return m_desc; }

  private:
    libusb_endpoint_descriptor m_desc;
    uint8_t m_interfaceNumber;
};

} // namespace o3p

#endif // O3P_USB_ENDPOINT_LIBUSB_HPP