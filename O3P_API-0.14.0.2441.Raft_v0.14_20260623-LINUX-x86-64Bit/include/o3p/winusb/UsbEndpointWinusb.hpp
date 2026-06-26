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

#ifndef O3P_USB_ENDPOINT_WINUSB_HPP
#define O3P_USB_ENDPOINT_WINUSB_HPP

#include "o3p/usb/UsbEndpoint.hpp"

#include <windows.h>
#include <winusb.h>

namespace o3p {

/**
 * @brief WinUSB based implementation of UsbEndpoint
 */
class UsbEndpointWinusb : public UsbEndpoint {
  public:
    /**
     * @brief Wrap a WinUSB pipe information block
     *
     * @param info Pipe information returned by WinUSB
     * @param interface_number Number of the interface this endpoint belongs to
     */
    UsbEndpointWinusb(WINUSB_PIPE_INFORMATION info, uint8_t interface_number) : m_address(info.PipeId), m_type(static_cast<UsbEndpointType>(info.PipeType)), m_interfaceNumber(interface_number) {}

    virtual ~UsbEndpointWinusb() = default;

    virtual uint8_t getAddress() const override { return m_address; }
    virtual UsbEndpointType getType() const override { return m_type; }
    virtual UsbEndpointDirection getDirection() const override {
        return USB_ENDPOINT_DIRECTION_IN(m_address) ? USB_ENDPOINT_DIRECTION_READ : USB_ENDPOINT_DIRECTION_WRITE;
    }
    virtual uint8_t getInterfaceNumber() const override { return m_interfaceNumber; }

  private:
    uint8_t m_address;
    UsbEndpointType m_type;
    UsbEndpointDirection m_direction;
    uint8_t m_interfaceNumber;
};

} // namespace o3p

#endif // O3P_USB_ENDPOINT_WINUSB_HPP