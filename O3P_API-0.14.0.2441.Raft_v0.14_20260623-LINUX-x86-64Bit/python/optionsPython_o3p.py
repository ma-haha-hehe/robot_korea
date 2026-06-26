#!/usr/bin/python3

# Copyright (C) 2026 pmdtechnologies gmbh
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys

import o3py


def print_all_device_options(device):
    """Print all supported device options."""
    print("All Device Options:")
    print("=========================")

    for option_type in range(o3py.O3P_OPTION_NONE + 1, o3py.O3P_OPTION_MAX):
        if device.supportsOption(option_type):
            name = o3py.optionTypeToString(option_type)
            option = device.getOption(option_type)
            status = ""
            if not option.isEnabled():
                status = " [Disabled]"
            elif option.isReadOnly():
                status = " [Read-Only]"
            print(f"{option_type}: {name}{status}")

    print("=========================\n")


def print_device_option_details(option):
    """Print detailed information about a single option."""
    print("=========================")
    print(f"Name: {option.getName()}")
    print(f"Description: {option.getDescription()}")
    print(f"Type: {o3py.optionTypeToString(option.getType())}")
    print(f"Current Value: {option.getValue()}")

    opt_range = option.getRange()
    if opt_range:
        print(f"Valid Range: {opt_range}")
    else:
        print("No range defined for this option.")

    print(f"Enabled: {option.isEnabled()}")
    print(f"Read-Only: {option.isReadOnly()}")
    print("=========================\n")


def modify_device_option(option):
    """Modify an option based on its value type."""
    value_type = option.getValueType()

    if value_type == o3py.O3P_OPTION_VALUE_TYPE_INT:
        modify_integer_option(option)
    elif value_type == o3py.O3P_OPTION_VALUE_TYPE_FLOAT:
        modify_float_option(option)
    elif value_type == o3py.O3P_OPTION_VALUE_TYPE_BOOL:
        modify_boolean_option(option)
    elif value_type == o3py.O3P_OPTION_VALUE_TYPE_STRING:
        modify_string_option(option)
    else:
        print("Unsupported option value type for modification.")


def modify_integer_option(option):
    """Modify an integer-typed option."""
    if option.getValueType() != o3py.O3P_OPTION_VALUE_TYPE_INT:
        raise RuntimeError("Option is not of integer type.")

    new_value = int(input("Enter new integer value for the option: "))

    # Optionally check against allowed values if an IntOptionRange is defined
    opt_range = option.getRange()
    int_range = o3py.IntOptionRange.fromRange(opt_range) if opt_range else None
    if int_range:
        if not int_range.isValid(o3py.OptionValue(new_value)):
            raise RuntimeError("Value out of range for this option.")

        ## Alternatively, you can also check the range manually:
        # if new_value < int_range.getMin() or new_value > int_range.getMax():
        #     raise RuntimeError("Value out of range for this option.")
        ## Also check step if needed
        # if int_range.getStep() > 0:
        #     delta = new_value - int_range.getMin()
        #     if delta % int_range.getStep() != 0:
        #         raise RuntimeError("Value does not align with step for this option.")
        # default_value = int_range.getDefaultValue()

    option.setValue(o3py.OptionValue(new_value))


def modify_float_option(option):
    """Modify a float-typed option."""
    if option.getValueType() != o3py.O3P_OPTION_VALUE_TYPE_FLOAT:
        raise RuntimeError("Option is not of float type.")

    new_value = float(input("Enter new float value for the option: "))

    # Optionally check against allowed values if a FloatOptionRange is defined
    opt_range = option.getRange()
    float_range = o3py.FloatOptionRange.fromRange(opt_range) if opt_range else None
    if float_range:
        if not float_range.isValid(o3py.OptionValue(new_value)):
            raise RuntimeError("Value out of range for this option.")

        ## Alternatively, you can also check the range manually:
        # if new_value < float_range.getMin() or new_value > float_range.getMax():
        #     raise RuntimeError("Value out of range for this option.")
        # default_value = float_range.getDefaultValue()

    option.setValue(o3py.OptionValue(new_value))


def modify_boolean_option(option):
    """Modify a boolean-typed option."""
    if option.getValueType() != o3py.O3P_OPTION_VALUE_TYPE_BOOL:
        raise RuntimeError("Option is not of boolean type.")

    user_input = input("Enter new boolean value for the option (true/false): ").strip().lower()
    if user_input == "true":
        new_value = True
    elif user_input == "false":
        new_value = False
    else:
        raise RuntimeError(f"Invalid boolean value \"{user_input}\": Please enter 'true' or 'false'.")

    # Bool options are not expected to have a range
    option.setValue(o3py.OptionValue(new_value))


def modify_string_option(option):
    """Modify a string-typed option."""
    if option.getValueType() != o3py.O3P_OPTION_VALUE_TYPE_STRING:
        raise RuntimeError("Option is not of string type.")

    new_value = input("Enter new string value for the option: ")

    # Optionally check against allowed values if a StringOptionRange is defined
    opt_range = option.getRange()
    string_range = o3py.StringOptionRange.fromRange(opt_range) if opt_range else None
    if string_range:
        if not string_range.isValid(o3py.OptionValue(new_value)):
            raise RuntimeError("Value not allowed for this option.")

        ## Alternatively, you can also check the allowed values manually:
        # allowed_values = string_range.getAllowedValues()
        # if new_value not in allowed_values:
        #     raise RuntimeError("Value not allowed for this option.")
        # default_value = string_range.getDefaultValue()

    option.setValue(o3py.OptionValue(new_value))


def main():
    pipe = o3py.Pipeline()

    try:
        profile = pipe.init()
    except RuntimeError as e:
        print(f"Error initializing camera: {e}")
        return 1

    device = profile.getDevice()

    if not device or not device.supportsInfo(o3py.O3P_CAMERA_INFO_SERIAL_NUMBER):
        print("No valid device found")
        return 1

    serial_number = device.getInfo(o3py.O3P_CAMERA_INFO_SERIAL_NUMBER)
    print(f"Connected to camera with S/N: {serial_number}\n")

    # Start of interactive option exploration and modification
    while True:
        print_all_device_options(device)

        try:
            option_type = int(input("Enter the option type number to get details or modify (0 to exit): "))
        except ValueError:
            print("Invalid input. Please enter a number.")
            continue

        if option_type == 0:
            break

        if device.supportsOption(option_type):
            option = device.getOption(option_type)
            print("Selected Option:")
            print_device_option_details(option)

            if option.isReadOnly() or not option.isEnabled():
                continue

            while True:
                modify_choice = input("Do you want to modify this option? (y/n): ").strip()

                if modify_choice in ("y", "Y"):
                    try:
                        modify_device_option(option)
                        print("\nUpdated Option:")
                        print_device_option_details(option)
                        print("Returning to main menu...\n")
                        break
                    except RuntimeError as e:
                        print(f"Error setting option value: {e}")
                elif modify_choice in ("n", "N"):
                    break
                else:
                    print("Invalid choice. Please enter 'y' or 'n'.")
        else:
            print(f"Option {o3py.optionTypeToString(option_type)} not supported by this device!\n")
            break

    return 0


if __name__ == "__main__":
    sys.exit(main())
