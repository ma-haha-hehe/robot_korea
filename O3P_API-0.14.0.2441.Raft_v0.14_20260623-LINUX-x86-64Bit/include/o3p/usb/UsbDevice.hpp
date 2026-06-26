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

#ifndef O3P_USB_DEVICE_HPP
#define O3P_USB_DEVICE_HPP

#include "o3p/usb/UsbCommon.hpp"
#include "o3p/usb/UsbInterface.hpp"
#include "o3p/usb/UsbMessenger.hpp"

#include <cstdint>
#include <sstream>
#include <string>

namespace o3p {

/**
 * @brief Information describing a USB device
 */
struct UsbDeviceInfo {
    std::string id;          /**< Backend specific device identifier */
    uint16_t vid;            /**< USB vendor id */
    uint16_t pid;            /**< USB product id */
    uint16_t mi;             /**< Multi-interface index */
    std::string unique_id;   /**< Unique identifier for the device */
    std::string serial;      /**< Serial number string */
    UsbClass usb_class;      /**< USB class identifier */
    uint8_t usb_subclass;    /**< USB subclass identifier */
    UsbSpec usb_spec;        /**< USB specification version */

    /**
     * @brief Convert this info to a printable string
     */
    operator std::string() {
        std::stringstream s;

        s << "id- " << id << "\nvid- " << std::hex << vid << "\npid- " << std::hex << pid << "\nmi- " << mi << "\nunique_id- " << unique_id
        << "\nserial- " << serial << "\nusb_class- " << static_cast<int>(usb_class) << "\nusb_subclass- " << static_cast<int>(usb_subclass) << "\nusb_spec- " << static_cast<int>(usb_spec);

        return s.str();
    }
};

/**
 * @brief Abstract representation of a USB device
 */
class UsbDevice {
  public:
    virtual ~UsbDevice() = default;

    /**
     * @brief Get the descriptive information of the device
     */
    virtual const UsbDeviceInfo &getInfo() const = 0;

    /**
     * @brief Get all interfaces exposed by the device
     */
    virtual const std::vector<std::shared_ptr<UsbInterface>> getInterfaces() const = 0;

    /**
     * @brief Get a specific interface by its number
     *
     * @param interface_number The interface number
     */
    virtual const std::shared_ptr<UsbInterface> getInterface(uint8_t interface_number) const = 0;

    /**
     * @brief Open the device on the given interface and return a messenger
     *
     * @param interface_number Interface number to claim
     * @return Messenger that can be used to communicate over the interface
     */
    virtual const std::shared_ptr<UsbMessenger> open(uint8_t interface_number) = 0;

    /**
     * @brief Get all raw descriptors reported by the device
     */
    virtual const std::vector<UsbDescriptor> getDescriptors() const = 0;
};

} // namespace o3p

#endif // O3P_USB_DEVICE_HPP