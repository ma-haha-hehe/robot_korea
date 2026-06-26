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

#ifndef O3P_USB_ENUMERATOR_HPP
#define O3P_USB_ENUMERATOR_HPP

#include "o3p/Definitions.hpp"
#include "o3p/usb/UsbDevice.hpp"
#include <memory>
#include <vector>

namespace o3p {

/**
 * @brief Static helpers to enumerate and create USB devices
 */
class UsbEnumerator {
  public:
    /**
     * @brief Create a UsbDevice instance from device information
     *
     * @param info Device information as returned by queryDeviceInfo()
     * @return Shared pointer to the created UsbDevice
     */
    O3P_API static std::shared_ptr<UsbDevice> createUsbDevice(const UsbDeviceInfo &info);

    /**
     * @brief Query information about all currently connected USB devices
     *
     * @return List of device information for every connected device
     */
    O3P_API static std::vector<UsbDeviceInfo> queryDeviceInfo();
};

} // namespace o3p

#endif // O3P_USB_ENUMERATOR_HPP