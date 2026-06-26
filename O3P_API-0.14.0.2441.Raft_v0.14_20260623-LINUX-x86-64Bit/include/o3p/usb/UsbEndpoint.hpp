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

#ifndef O3P_USB_ENDPOINT_HPP
#define O3P_USB_ENDPOINT_HPP

#include "o3p/usb/UsbCommon.hpp"
#include <cstdint>

namespace o3p {

/**
 * @brief Abstract representation of a USB endpoint
 */
class UsbEndpoint {
  public:
    virtual ~UsbEndpoint() = default;

    /**
     * @brief Get the endpoint address
     */
    virtual uint8_t getAddress() const = 0;

    /**
     * @brief Get the transfer type of the endpoint
     */
    virtual UsbEndpointType getType() const = 0;

    /**
     * @brief Get the data direction of the endpoint
     */
    virtual UsbEndpointDirection getDirection() const = 0;

    /**
     * @brief Get the number of the interface this endpoint belongs to
     */
    virtual uint8_t getInterfaceNumber() const = 0;
};

} // namespace o3p

#endif // O3P_USB_ENDPOINT_HPP