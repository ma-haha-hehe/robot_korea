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

#ifndef O3P_USB_COMMON_HPP
#define O3P_USB_COMMON_HPP

#include <cstdint>
#include <vector>

namespace o3p {

/**
 * @brief USB endpoint transfer types
 */
enum UsbEndpointType {
    USB_ENDPOINT_CONTROL,
    USB_ENDPOINT_ISOCHRONOUS,
    USB_ENDPOINT_BULK,
    USB_ENDPOINT_INTERRUPT
};

/**
 * @brief USB endpoint data direction
 */
enum UsbEndpointDirection {
    USB_ENDPOINT_DIRECTION_WRITE = 0x00,
    USB_ENDPOINT_DIRECTION_READ = 0x80
};

// https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/supported-usb-classes#microsoft-provided-usb-device-class-drivers
/**
 * @brief USB device class identifiers
 */
enum UsbClass : uint8_t {
    O3P_USB_CLASS_UNSPECIFIED = 0x00,
    O3P_USB_CLASS_AUDIO = 0x01,
    O3P_USB_CLASS_COM = 0x02,
    O3P_USB_CLASS_HID = 0x03,
    O3P_USB_CLASS_PID = 0x05,
    O3P_USB_CLASS_IMAGE = 0x06,
    O3P_USB_CLASS_PRINTER = 0x07,
    O3P_USB_CLASS_MASS_STORAGE = 0x08,
    O3P_USB_CLASS_HUB = 0x09,
    O3P_USB_CLASS_CDC_DATA = 0x0A,
    O3P_USB_CLASS_SMART_CARD = 0x0B,
    O3P_USB_CLASS_CONTENT_SECURITY = 0x0D,
    O3P_USB_CLASS_VIDEO = 0x0E,
    O3P_USB_CLASS_PHDC = 0x0F,
    O3P_USB_CLASS_AV = 0x10,
    O3P_USB_CLASS_BILLBOARD = 0x11,
    O3P_USB_CLASS_DIAGNOSTIC_DEVICE = 0xDC,
    O3P_USB_CLASS_WIRELESS_CONTROLLER = 0xE0,
    O3P_USB_CLASS_MISCELLANEOUS = 0xEF,
    O3P_USB_CLASS_APPLICATION_SPECIFIC = 0xFE,
    O3P_USB_CLASS_VENDOR_SPECIFIC = 0xFF
};

/**
 * @brief USB specification version identifiers (BCD encoded)
 */
enum UsbSpec {
    usb_undefined = 0,
    usb1_type = 0x0100,
    usb1_1_type = 0x0110,
    usb2_type = 0x0200,
    usb2_01_type = 0x0201,
    usb2_1_type = 0x0210,
    usb3_type = 0x0300,
    usb3_1_type = 0x0310,
    usb3_2_type = 0x0320,
};

/**
 * @brief Raw USB descriptor as returned by the device
 */
struct UsbDescriptor {
    uint8_t length;            /**< Length of the descriptor in bytes */
    uint8_t type;              /**< Descriptor type */
    std::vector<uint8_t> data; /**< Descriptor payload */
};

#define O3P_DEVICE_USB_VID 0x1C28
#define O3P_DEVICE_USB_PID 0xF003
#define O3P_FW_UPDATE_USB_VID 0x1C28
#define O3P_FW_UPDATE_USB_PID 0xF004
#define O3P_FW_UPDATE_USB_CLASS O3P_USB_CLASS_VENDOR_SPECIFIC
#define O3P_FW_UPDATE_USB_SUBCLASS 0x00
#define O3P_FW_LOGGER_USB_DEVICE_CLASS O3P_USB_CLASS_VENDOR_SPECIFIC
#define O3P_FW_LOGGER_USB_DEVICE_SUBCLASS 0x00

} // namespace o3p

#endif // O3P_USB_COMMON_HPP