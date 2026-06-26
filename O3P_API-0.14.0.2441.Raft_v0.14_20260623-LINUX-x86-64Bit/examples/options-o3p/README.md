# Tutorial - "o3p-options" Example

This tutorial explains the "o3p-options" example for the O3P which can be used as a starting point when learning to work
with O3P device options.

# Installation

This code does not require any additional libraries so you should be able to build run it if you have the API installed.

# Code Explanation
The first thing that is done is the creation of a Pipeline which serves as top-level API for streaming and processing frames. 
Initializing the camera happens during initalization of the pipeline which is done by calling `p.init()`. This function returns the Pipeline profile from which we can get a handle to an O3P device.

```cpp
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
``` 

It is a good idea to discover which Options are supported by the device.

```cpp
void printAllDeviceOptions(std::shared_ptr<o3p::Device> device) {
    std::cout << "All Device Options:" << std::endl;
    std::cout << "=========================" << std::endl;
    
    for (int optionType = o3p::O3P_OPTION_NONE + 1; optionType < o3p::O3P_OPTION_MAX; ++optionType) {
        std::cout << optionType << ": " << o3p::optionTypeToString(static_cast<o3p::OptionType>(optionType));
        if (device->supportsOption(static_cast<o3p::OptionType>(optionType))) {
            std::cout << std::endl;
        }
        else {
            std::cout << " (not supported by this device)" << std::endl;
        }

    }

    std::cout << "=========================\n" << std::endl;
}
```

Get an Option from a device and discover its properties.

```cpp
if (device->supportsOption(static_cast<o3p::OptionType>(optionType))) {
    auto option = device->getOption(static_cast<o3p::OptionType>(optionType));
    std::cout << "Selected Option:" << std::endl;
    printDeviceOptionDetails(option);

    ...

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

    std::cout << "=========================\n" << std::endl;
}
```

Once a valid Option is retrieved, its value can be modified. Note that an option can support a 
range(e.g. min,max,step,default). This can be useful for discovering which new values may be
acceptable or which value is the default.

```cpp
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
```