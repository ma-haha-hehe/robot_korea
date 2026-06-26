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

#include <algorithm>
#include <iostream>
#include <string>

// Helper functions for acquiring user input via console
int getUserIntegerInput(const std::string &prompt);
float getUserFloatInput(const std::string &prompt);
bool getUserBooleanInput(const std::string &prompt);
std::string getUserStringInput(const std::string &prompt);

// Helper functions for printing and modifying device options
void printAllDeviceOptions(std::shared_ptr<o3p::Device> device);
void printDeviceOptionDetails(std::shared_ptr<o3p::Option> option);
void modifyDeviceOption(std::shared_ptr<o3p::Option> option);
void modifyIntegerOption(std::shared_ptr<o3p::Option> option);
void modifyFloatOption(std::shared_ptr<o3p::Option> option);
void modifyBooleanOption(std::shared_ptr<o3p::Option> option);
void modifyStringOption(std::shared_ptr<o3p::Option> option);

int main(int argc, char *argv[]) {
    o3p::Pipeline p;
    o3p::PipelineProfile profile;
    std::shared_ptr<o3p::Device> device;

    try {
        profile = p.init();
    } catch (std::exception &e) {
        std::cerr << "Error initializing camera : " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    device = profile.getDevice();

    if (!device || !device->supportsInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER)) {
        std::cerr << "No valid device found" << std::endl;
        return EXIT_FAILURE;
    } else {
        auto serialNumber = device->getInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << "Connected to camera with S/N : " << serialNumber << "\n"
                  << std::endl;
    }

    // Start of interactive option exploration and modification
    while (true) {
        printAllDeviceOptions(device);

        uint32_t optionType = getUserIntegerInput("Enter the option type number to get details or modify (0 to exit): ");
        if (optionType == 0) {
            break;
        }

        if (device->supportsOption(static_cast<o3p::OptionType>(optionType))) {
            auto option = device->getOption(static_cast<o3p::OptionType>(optionType));
            std::cout << "Selected Option:" << std::endl;
            printDeviceOptionDetails(option);

            if (option->isReadOnly() || !option->isEnabled()) {
                continue;
            }

            while (true) {
                std::string modifyChoice;
                std::cout << "Do you want to modify this option? (y/n): ";
                std::cin >> modifyChoice;
                std::cout << std::endl;

                if (modifyChoice == "y" || modifyChoice == "Y") {
                    try {
                        modifyDeviceOption(option);
                        std::cout << "\nUpdated Option:" << std::endl;
                        printDeviceOptionDetails(option);

                        std::cout << "Returning to main menu...\n"
                                  << std::endl;
                        break;
                    } catch (std::exception &e) {
                        std::cout << "Error setting option value: " << e.what() << std::endl;
                    }
                } else if (modifyChoice == "n" || modifyChoice == "N") {
                    break;
                } else {
                    std::cout << "Invalid choice. Please enter 'y' or 'n'." << std::endl;
                }
            }
        } else {
            std::cout << "Option " << o3p::optionTypeToString(static_cast<o3p::OptionType>(optionType)) << " not supported by this device!\n"
                      << std::endl;
            break;
        }
    }

    return EXIT_SUCCESS;
}

int getUserIntegerInput(const std::string &prompt) {
    std::cout << prompt;
    int value;
    std::string stringValue;
    std::cin >> stringValue;

    try {
        value = std::stoi(stringValue);
    } catch (const std::exception &e) {
        throw std::runtime_error("Invalid integer value \"" + stringValue + "\": " + std::string(e.what()));
    }

    return value;
}

float getUserFloatInput(const std::string &prompt) {
    std::cout << prompt;
    float value;
    std::string stringValue;
    std::cin >> stringValue;

    try {
        value = std::stof(stringValue);
    } catch (const std::exception &e) {
        throw std::runtime_error("Invalid float value \"" + stringValue + "\": " + std::string(e.what()));
    }

    return value;
}

bool getUserBooleanInput(const std::string &prompt) {
    std::cout << prompt;
    std::string stringValue;
    std::cin >> stringValue;
    bool value;

    std::transform(stringValue.begin(), stringValue.end(), stringValue.begin(), ::tolower);

    if (stringValue == "true") {
        value = true;
    } else if (stringValue == "false") {
        value = false;
    } else {
        throw std::runtime_error("Invalid boolean value \"" + stringValue + "\": Please enter 'true' or 'false'.");
    }

    return value;
}

std::string getUserStringInput(const std::string &prompt) {
    std::cout << prompt;
    std::string value;
    std::cin >> value;

    return value;
}

void printAllDeviceOptions(std::shared_ptr<o3p::Device> device) {
    std::cout << "All Device Options:" << std::endl;
    std::cout << "=========================" << std::endl;

    for (int optionType = o3p::O3P_OPTION_NONE + 1; optionType < o3p::O3P_OPTION_MAX; ++optionType) {
        if (device->supportsOption(static_cast<o3p::OptionType>(optionType))) {
            std::cout << optionType << ": " << o3p::optionTypeToString(static_cast<o3p::OptionType>(optionType));
            auto option = device->getOption(static_cast<o3p::OptionType>(optionType));
            if (!option->isEnabled()) {
                std::cout << " [Disabled]";
            } else if (option->isReadOnly()) {
                std::cout << " [Read-Only]";
            }
            std::cout << std::endl;
        }
    }

    std::cout << "=========================\n"
              << std::endl;
}

void printDeviceOptionDetails(std::shared_ptr<o3p::Option> option) {
    std::cout << "=========================" << std::endl;
    std::cout << "Name: " << option->getName() << std::endl;
    std::cout << "Description: " << option->getDescription() << std::endl;
    std::cout << "Type: " << o3p::optionTypeToString(option->getType()) << std::endl;
    std::cout << "Current Value: " << option->getValue() << std::endl;

    if (option->getRange()) {
        std::cout << "Valid Range: " << *option->getRange() << std::endl;
    } else {
        std::cout << "No range defined for this option." << std::endl;
    }

    std::cout << "Enabled: " << (option->isEnabled() ? "true" : "false") << std::endl;
    std::cout << "Read-Only: " << (option->isReadOnly() ? "true" : "false") << std::endl;

    std::cout << "=========================\n"
              << std::endl;
}

void modifyDeviceOption(std::shared_ptr<o3p::Option> option) {
    if (option->getValueType() == o3p::O3P_OPTION_VALUE_TYPE_INT) {
        modifyIntegerOption(option);
    } else if (option->getValueType() == o3p::O3P_OPTION_VALUE_TYPE_FLOAT) {
        modifyFloatOption(option);
    } else if (option->getValueType() == o3p::O3P_OPTION_VALUE_TYPE_BOOL) {
        modifyBooleanOption(option);
    } else if (option->getValueType() == o3p::O3P_OPTION_VALUE_TYPE_STRING) {
        modifyStringOption(option);
    } else {
        std::cout << "Unsupported option value type for modification." << std::endl;
    }
}

void modifyIntegerOption(std::shared_ptr<o3p::Option> option) {
    if (option->getValueType() != o3p::O3P_OPTION_VALUE_TYPE_INT) {
        throw std::runtime_error("Option is not of integer type.");
    }

    int newValue = getUserIntegerInput("Enter new integer value for the option: ");

    // Optionally check against allowed values if an IntOptionRange is defined
    std::shared_ptr<o3p::IntOptionRange> range = std::dynamic_pointer_cast<o3p::IntOptionRange>(option->getRange());
    if (range) {
        if (!range->isValid(o3p::OptionValue(newValue))) {
            throw std::runtime_error("Value out of range for this option.");
        }

        //// Alternatively, you can also check the range manually:
        // if (newValue < range->getMin() || newValue > range->getMax()) {
        //     throw std::runtime_error("Value out of range for this option.");
        // }
        //// Also check step if needed
        // if (range->getStep() > 0) {
        //     int delta = newValue - range->getMin();
        //     if (delta % range->getStep() != 0) {
        //         throw std::runtime_error("Value does not align with step for this option.");
        //     }
        // }
        // auto defaultValue = range->getDefaultValue();
    }

    option->setValue(o3p::OptionValue(newValue));
}

void modifyFloatOption(std::shared_ptr<o3p::Option> option) {
    if (option->getValueType() != o3p::O3P_OPTION_VALUE_TYPE_FLOAT) {
        throw std::runtime_error("Option is not of float type.");
    }

    float newValue = getUserFloatInput("Enter new float value for the option: ");

    // Optionally check against allowed values if a FloatOptionRange is defined
    std::shared_ptr<o3p::FloatOptionRange> range = std::dynamic_pointer_cast<o3p::FloatOptionRange>(option->getRange());
    if (range) {
        if (!range->isValid(o3p::OptionValue(newValue))) {
            throw std::runtime_error("Value out of range for this option.");
        }

        //// Alternatively, you can also check the range manually:
        // if (newValue < range->getMin() || newValue > range->getMax()) {
        //     throw std::runtime_error("Value out of range for this option.");
        // }
        //// Also check step if needed
        // if (range->getStep() > 0.0f) {
        //     float delta = newValue - range->getMin();
        //     if (delta % range->getStep() != 0) {
        //         throw std::runtime_error("Value does not align with step for this option.");
        //     }
        // }
        // auto defaultValue = range->getDefaultValue();
    }

    option->setValue(o3p::OptionValue(newValue));
}

void modifyBooleanOption(std::shared_ptr<o3p::Option> option) {
    if (option->getValueType() != o3p::O3P_OPTION_VALUE_TYPE_BOOL) {
        throw std::runtime_error("Option is not of boolean type.");
    }
    bool newValue = getUserBooleanInput("Enter new boolean value for the option (true/false): ");

    // Bool options are not expected to have a range
    option->setValue(o3p::OptionValue(newValue));
}

void modifyStringOption(std::shared_ptr<o3p::Option> option) {
    if (option->getValueType() != o3p::O3P_OPTION_VALUE_TYPE_STRING) {
        throw std::runtime_error("Option is not of string type.");
    }
    std::string newValue = getUserStringInput("Enter new string value for the option: ");

    // Optionally check against allowed values if a StringOptionRange is defined
    std::shared_ptr<o3p::StringOptionRange> range = std::dynamic_pointer_cast<o3p::StringOptionRange>(option->getRange());
    if (range) {
        if (!range->isValid(o3p::OptionValue(newValue))) {
            throw std::runtime_error("Value not allowed for this option.");
        }

        // Alternatively, you can also check the allowed values manually:
        // auto allowedValues = range->getAllowedValues();
        // if (std::find(allowedValues.begin(), allowedValues.end(), newValue) == allowedValues.end()) {
        //     throw std::runtime_error("Value not allowed for this option.");
        // }
        // auto defaultValue = range->getDefaultValue();
    }

    option->setValue(o3p::OptionValue(newValue));
}
