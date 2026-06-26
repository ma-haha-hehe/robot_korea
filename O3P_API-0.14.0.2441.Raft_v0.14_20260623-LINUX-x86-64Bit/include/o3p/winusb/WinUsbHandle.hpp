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

#ifndef O3P_WIN_USB_HANDLE_HPP
#define O3P_WIN_USB_HANDLE_HPP

#include <map>
#include <stdexcept>
#include <string>
#include <windows.h>
#include <winusb.h>

namespace o3p {

/**
 * @brief RAII wrapper around a WinUSB device handle and its interface handles
 */
class WinusbHandle {
  public:
    /**
     * @brief Open the device at the given path and initialize all WinUSB interfaces
     *
     * @param path Windows device path of the USB device
     * @throws std::runtime_error if opening or initialization fails
     */
    WinusbHandle(const std::wstring &path) {
        if (m_deviceHandle != INVALID_HANDLE_VALUE) {
            CloseHandle(m_deviceHandle);
            m_deviceHandle = INVALID_HANDLE_VALUE;
        }
        m_deviceHandle = CreateFileW(path.c_str(),
                                     GENERIC_WRITE | GENERIC_READ,
                                     FILE_SHARE_WRITE | FILE_SHARE_READ,
                                     NULL,
                                     OPEN_EXISTING,
                                     FILE_FLAG_OVERLAPPED,
                                     NULL);
        if (m_deviceHandle == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("Failed to open device: " + std::to_string(GetLastError()));
        }

        WINUSB_INTERFACE_HANDLE interface_handle;
        if (WinUsb_Initialize(m_deviceHandle, &interface_handle) == FALSE) {
            CloseHandle(m_deviceHandle);
            m_deviceHandle = INVALID_HANDLE_VALUE;
            throw std::runtime_error("Failed to initialize WinUSB: " + std::to_string(GetLastError()));
        }

        USB_INTERFACE_DESCRIPTOR interface_descriptor;
        if (WinUsb_QueryInterfaceSettings(interface_handle, 0, &interface_descriptor) == FALSE) {
            WinUsb_Free(interface_handle);
            CloseHandle(m_deviceHandle);
            m_deviceHandle = INVALID_HANDLE_VALUE;
            throw std::runtime_error("Failed to query interface settings: " + std::to_string(GetLastError()));
        }

        for (UCHAR interface_number = 0; true; interface_number++) {
            WINUSB_INTERFACE_HANDLE h;
            USB_INTERFACE_DESCRIPTOR descriptor;

            if (!WinUsb_GetAssociatedInterface(interface_handle, interface_number, &h)) {
                auto error = GetLastError();
                if (error != ERROR_NO_MORE_ITEMS)
                    throw std::runtime_error("WinUsb action failed, last error: " + std::to_string(error));
                break;
            }

            if (!WinUsb_QueryInterfaceSettings(h, 0, &descriptor)) {
                throw std::runtime_error("WinUsb action failed, last error: " + std::to_string(GetLastError()));
            }

            m_handles[descriptor.bInterfaceNumber] = h;
            m_descriptors[descriptor.bInterfaceNumber] = descriptor;
        }

        m_handles[interface_descriptor.bInterfaceNumber] = interface_handle;
        m_descriptors[interface_descriptor.bInterfaceNumber] = interface_descriptor;
    }

    ~WinusbHandle() {
        for (auto &&handle : m_handles) {
            WinUsb_Free(handle.second);
        }
        m_handles.clear();

        if (m_deviceHandle != INVALID_HANDLE_VALUE) {
            CloseHandle(m_deviceHandle);
            m_deviceHandle = INVALID_HANDLE_VALUE;
        }
    }

    /**
     * @brief Get the raw Windows device handle
     */
    const HANDLE getDeviceHandle() { return m_deviceHandle; }

    /**
     * @brief Get the map of interface number to WinUSB interface handle
     */
    const std::map<int, WINUSB_INTERFACE_HANDLE> getHandles() { return m_handles; }

    /**
     * @brief Get the map of interface number to USB interface descriptor
     */
    const std::map<int, USB_INTERFACE_DESCRIPTOR> getDescriptors() { return m_descriptors; }

    /**
     * @brief Get the WinUSB handle of the first interface
     */
    const WINUSB_INTERFACE_HANDLE getFirstInterfaceHandle() { return m_handles.begin()->second; }

    /**
     * @brief Get the WinUSB handle of a specific interface
     *
     * @param interfaceNumber Number of the interface
     * @throws std::runtime_error if no such interface exists
     */
    const WINUSB_INTERFACE_HANDLE getInterfaceHandle(int interfaceNumber) {
        auto it = m_handles.find(interfaceNumber);
        if (it == m_handles.end()) {
            throw std::runtime_error("get_interface_handle failed, interface not found");
        }
        return it->second;
    }

  private:
    HANDLE m_deviceHandle = INVALID_HANDLE_VALUE;
    std::map<int, WINUSB_INTERFACE_HANDLE> m_handles;
    std::map<int, USB_INTERFACE_DESCRIPTOR> m_descriptors;
};

} // namespace o3p

#endif // O3P_WIN_USB_HANDLE_HPP