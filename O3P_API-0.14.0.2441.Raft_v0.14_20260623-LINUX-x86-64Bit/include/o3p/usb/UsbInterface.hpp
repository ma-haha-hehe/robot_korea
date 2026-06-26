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

#ifndef O3P_USB_INTERFACE_HPP
#define O3P_USB_INTERFACE_HPP

#include "o3p/usb/UsbEndpoint.hpp"
#include <cstdint>
#include <memory>
#include <vector>

namespace o3p {

/**
 * @brief Abstract representation of a USB interface
 */
class UsbInterface {
  public:
    UsbInterface() = default;
    virtual ~UsbInterface() = default;

    /**
     * @brief Get the interface number
     */
    virtual uint8_t getNumber() const = 0;

    /**
     * @brief Get the USB class identifier of the interface
     */
    virtual uint8_t getClass() const = 0;

    /**
     * @brief Get the USB subclass identifier of the interface
     */
    virtual uint8_t getSubClass() const = 0;

    /**
     * @brief Get all endpoints belonging to this interface
     */
    virtual const std::vector<std::shared_ptr<UsbEndpoint>> getEndpoints() const = 0;

    /**
     * @brief Find the first endpoint matching the given direction and type
     *
     * @param direction Required endpoint direction
     * @param type Required endpoint type (defaults to bulk)
     */
    virtual const std::shared_ptr<UsbEndpoint> firstEndpoint(const UsbEndpointDirection direction, const UsbEndpointType type = UsbEndpointType::USB_ENDPOINT_BULK) const = 0;
};

} // namespace o3p

#endif // O3P_USB_INTERFACE_HPP