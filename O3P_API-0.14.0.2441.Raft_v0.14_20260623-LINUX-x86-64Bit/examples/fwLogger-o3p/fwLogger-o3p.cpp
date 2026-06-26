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

#include <o3p/o3p.hpp>

#include <fstream>
#include <string>

// Helper function to retrieve the serial number from a device.
// Returns an empty string if the device does not expose a serial number.
std::string getDeviceSerialNumber(std::shared_ptr<o3p::Device> device) {
    if (device->supportsInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER)) {
        return device->getInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER);
    } else {
        return "";
    }
}

void printUsage() {
    std::cout << "O3P Firmware Logger Example\n"
              << "Usage: fwLogger-o3p [options]\n\n"
              << "Options:\n"
              << "  -l, --list-devices       List connected O3P devices\n"
              << "  -s, --serial <serial>    Device serial number (defaults to the first connected device)\n"
              << "  -c, --collect-crash-logs Collect crash logs from the device instead of live logs\n"
              << "  -o, --out <file>         Output file for logs (defaults to stdout)\n"
              << "  -h, --help               Print help\n";
}

int main(int argc, char *argv[]) {
    std::string serialNumber;
    std::string outputFilePath;
    std::ofstream outputFile;
    bool collectCrashLogs = false;
    bool listDevices = false;
    std::shared_ptr<o3p::FwLogger> fwLogger;

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage();
            return 0;
        } else if (arg == "-l" || arg == "--list-devices") {
            listDevices = true;
        } else if (arg == "-c" || arg == "--collect-crash-logs") {
            collectCrashLogs = true;
        } else if ((arg == "-s" || arg == "--serial") && i + 1 < argc) {
            serialNumber = argv[++i];
        } else if ((arg == "-o" || arg == "--out") && i + 1 < argc) {
            outputFilePath = argv[++i];
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage();
            return 1;
        }
    }

    // List all connected devices that support firmware logging and exit
    if (listDevices) {
        auto devices = o3p::Context::getInstance().getConnectedDevices();
        if (devices.empty()) {
            std::cout << "No O3P devices connected." << std::endl;
        } else {
            std::cout << "Connected O3P devices (supports firmware logging):" << std::endl;
            for (const auto &device : devices) {
                if (!device->getFwLogger()) {
                    continue;
                }
                std::cout << " - Serial: " << getDeviceSerialNumber(device) << std::endl;
            }
        }
        return 0;
    }

    if (serialNumber.empty()) {
        std::cout << "No serial number provided. Attempting to retrieve firmware logs from the first connected device." << std::endl;
    } else {
        std::cout << "Looking for device with serial number: " << serialNumber << std::endl;
    }

    // Find an O3P device matching the provided serial number (or first device if no serial provided)
    for (auto device : o3p::Context::getInstance().getConnectedDevices()) {
        if (serialNumber.empty()) {
            serialNumber = getDeviceSerialNumber(device);
        } else if (getDeviceSerialNumber(device) != serialNumber) {
            continue;
        }
        fwLogger = device->getFwLogger();
        break;
    }

    if (!fwLogger) {
        if (serialNumber.empty()) {
            std::cout << "No connected O3P devices that support firmware logging were found." << std::endl;
        } else {
            std::cout << "No device found with serial number: " << serialNumber << " that supports firmware logging." << std::endl;
        }
        return 1;
    }

    // Open the output file if specified, otherwise logs go to stdout
    if (!outputFilePath.empty()) {
        std::cout << "Output file specified: " << outputFilePath << std::endl;
        outputFile.open(outputFilePath, std::ios::out | std::ios::trunc);
        if (!outputFile.is_open()) {
            std::cerr << "Failed to open output file: " << outputFilePath << std::endl;
            return 1;
        }
    } else {
        std::cout << "No output file specified. Logs will be printed to stdout." << std::endl;
    }

    // Use either the file stream or stdout as the output destination
    std::ostream &out = outputFile.is_open() ? outputFile : std::cout;

    // Collect crash logs: reads stored crash log entries from the device in one shot.
    // Live logs: streams real-time firmware log output until the user presses Enter.
    if (collectCrashLogs) {
        std::cout << "Collecting crash logs.." << std::endl;
        out << "Crash logs for device with serial number: " << serialNumber << "\n"
            << std::endl;
        fwLogger->collectCrashLogs([&](const char *log) {
            out << log << std::endl;
        });
        std::cout << "Finished collecting crash logs." << std::endl;
    } else {
        std::cout << "Collecting live logs. Press Enter to stop..." << std::endl;
        out << "Live logs for device with serial number: " << serialNumber << "\n"
            << std::endl;
        fwLogger->startCollecting([&](const char *log) {
            out << log << std::endl;
        });
        std::cin.get();
        fwLogger->stopCollecting();
        std::cout << "Stopped collecting live logs." << std::endl;
    }

    if (outputFile.is_open()) {
        outputFile.close();
    }

    return 0;
}
