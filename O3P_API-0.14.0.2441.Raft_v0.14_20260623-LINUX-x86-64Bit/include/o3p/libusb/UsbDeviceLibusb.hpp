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

#ifndef O3P_USB_DEVICE_LIBUSB_HPP
#define O3P_USB_DEVICE_LIBUSB_HPP

#include "o3p/libusb/LibusbContext.hpp"
#include "o3p/usb/UsbDevice.hpp"
#include <libusb-1.0/libusb.h>
#include <vector>

namespace o3p {

/**
 * @brief libusb based implementation of UsbDevice
 */
class UsbDeviceLibusb : public UsbDevice, public std::enable_shared_from_this<UsbDeviceLibusb> {
  public:
    /**
     * @brief Construct a libusb backed device wrapper
     *
     * @param device The native libusb device pointer
     * @param desc The device descriptor returned by libusb
     * @param info Higher level descriptive information about the device
     * @param context The libusb context the device belongs to
     */
    UsbDeviceLibusb(libusb_device *device, const libusb_device_descriptor &desc, const UsbDeviceInfo &info, std::shared_ptr<LibusbContext> context);
    virtual ~UsbDeviceLibusb();

    const UsbDeviceInfo &getInfo() const override { return m_info; }
    virtual const std::vector<std::shared_ptr<UsbInterface>> getInterfaces() const override { return m_interfaces; }
    virtual const std::shared_ptr<UsbInterface> getInterface(uint8_t interface_number) const override;
    virtual const std::shared_ptr<UsbMessenger> open(uint8_t interface_number) override;
    virtual const std::vector<UsbDescriptor> getDescriptors() const override { return m_descriptors; }

  private:
    libusb_device *m_device;
    libusb_device_descriptor m_desc;
    UsbDeviceInfo m_info;
    std::shared_ptr<LibusbContext> m_context;

    std::vector<std::shared_ptr<UsbInterface>> m_interfaces;
    std::vector<UsbDescriptor> m_descriptors;
};

} // namespace o3p

#endif // O3P_USB_DEVICE_LIBUSB_HPP
