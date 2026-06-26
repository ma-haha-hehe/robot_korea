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

#include <chrono>
#include <iostream>
#include <o3p/FwUpdateDevice.hpp> // Provides the FwUpdateDevice class for recovery-mode devices
#include <o3p/Update.hpp>         // Provides the Updatable interface and readFileToStdVector helper
#include <o3p/o3p.hpp>
#include <thread>

// This example demonstrates a complete firmware update workflow:
// 1. Load a firmware binary from disk
// 2. Find an updatable device on the USB bus
// 3. Reboot the device into recovery mode
// 4. Wait for re-enumeration and flash the new firmware
// 5. The device automatically reboots into normal mode after a successful update

int main(int argc, char *argv[]) {

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <firmware_file>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string firmwareFile = argv[1];

    // Read the entire firmware binary into a contiguous memory buffer.
    // The file is typically a .swu file.
    auto fwData = o3p::readFileToStdVector<uint8_t>(firmwareFile);
    if (fwData.empty()) {
        std::cerr << "Failed to read firmware file: " << firmwareFile << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Firmware file: " << firmwareFile << " (" << fwData.size() << " bytes)" << std::endl;

    // Get the singleton context which manages device discovery.
    // Iterate over connected devices and find the first one that implements
    // the Updatable interface, indicating it supports firmware updates.
    o3p::Context &ctx = o3p::Context::getInstance();
    auto devices = ctx.getConnectedDevices();

    std::shared_ptr<o3p::Device> targetDevice;
    std::string serialNumber;

    for (auto &device : devices) {
        if (std::dynamic_pointer_cast<o3p::Updatable>(device)) {
            targetDevice = device;
            if (device->supportsInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER)) {
                serialNumber = device->getInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER);
            }
            break;
        }
    }

    if (!targetDevice) {
        std::cerr << "No updatable O3P device found." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Found updatable device with serial: " << serialNumber << std::endl;

    // Instruct the device to reboot into recovery/update mode.
    // After this call the device disconnects from the USB bus and
    // re-enumerates as an FwUpdateDevice.
    auto updatable = std::dynamic_pointer_cast<o3p::Updatable>(targetDevice);
    updatable->enterUpdateState();
    std::cout << "Device rebooting into recovery mode..." << std::endl;

    // Poll for the device to re-appear on the bus in recovery mode.
    // The device typically takes a few seconds to reboot; we retry up to 5 times
    // with a 5-second delay between attempts (25 seconds total timeout).
    std::shared_ptr<o3p::FwUpdateDevice> updateDevice;
    for (int attempt = 0; attempt < 5; ++attempt) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "Waiting for device to re-enumerate..." << std::endl;

        for (auto &device : ctx.getConnectedDevices()) {
            if (device->supportsInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER) &&
                device->getInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER) == serialNumber) {
                updateDevice = std::dynamic_pointer_cast<o3p::FwUpdateDevice>(device);
                break;
            }
        }

        if (updateDevice) {
            break;
        }
    }

    if (!updateDevice) {
        std::cerr << "Device did not re-enumerate in recovery mode." << std::endl;
        return EXIT_FAILURE;
    }

    // Flash the firmware. The update() method writes the binary data to the device
    // and accepts a progress callback that reports completion percentage (0-100).
    // On success the device will automatically reboot into normal operation mode.
    // Note: the first boot after a firmware update may take longer than usual as
    // the device initializes the new firmware for the first time.
    std::cout << "Starting firmware update..." << std::endl;

    try {
        updateDevice->update(fwData, [](const float progress) {
            std::cout << "\rUpdate progress: " << static_cast<int>(progress) << "%" << std::flush;
        });
        std::cout << std::endl;
        std::cout << "Firmware update completed successfully." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << std::endl;
        std::cerr << "Firmware update failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
