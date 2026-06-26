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

#ifndef O3P_OPTION_OPTION_HPP
#define O3P_OPTION_OPTION_HPP

#include <o3p/option/OptionInfo.hpp>
#include <o3p/option/OptionValue.hpp>
#include <o3p/option/OptionRange.hpp>
#include <o3p/option/OptionDescription.hpp>
#include <o3p/Definitions.hpp>

#include <memory>
#include <string>
#include <vector>

namespace o3p {

class OptionObserver;

/**
 * @brief Represents a configurable option in the system
 */
class Option {
  public:
    /**
     * @brief Gets the current value of the option
     */
    O3P_API OptionValue getValue() const;

    /**
     * @brief Sets a new value for the option
     * @param newValue The new value to set
     * @throws std::runtime_error if the value type does not match or if the value is out of range
     */
    O3P_API virtual void setValue(const OptionValue &newValue);

    /**
     * @brief Gets the name of the option
     */
    O3P_API const std::string &getName() const;

    /**
     * @brief Gets the description of the option
     */
    O3P_API const std::string &getDescription() const;

    /**
     * @brief Gets the type of the option
     */
    O3P_API OptionType getType() const;

    /**
     * @brief Gets the value type of the option
     */
    O3P_API OptionValueType getValueType() const;

    /**
     * @brief Gets the valid range of the option value, if any. Returns nullptr if no range is defined.
     */
    O3P_API std::shared_ptr<OptionValueRange> getRange() const;

    /**
     * @brief Gets the OptionInfo associated with this option
     */
    O3P_API OptionInfo getInfo() const;

    /**
     * @brief Checks if the option is enabled
     */
    O3P_API bool isEnabled() const;

    /**
     * @brief Checks if the option is read-only
     */
    O3P_API bool isReadOnly() const;

    /**
     * @brief Registers an observer to be notified when the option value changes
     * @param observer The observer to register
     */
    O3P_API void registerObserver(OptionObserver *observer);

    /**
     * @brief Unregisters an observer
     * @param observer The observer to unregister
     */
    O3P_API void unregisterObserver(OptionObserver *observer);

    O3P_API friend std::ostream& operator<<(std::ostream& os, const Option& obj);

  protected:
    /**
     * @brief Call @ref OptionObserver::onOptionUpdated on all registered observers
     */
    O3P_API void notifyObservers();

    OptionInfo m_info;
    OptionValue m_value;
    std::vector<OptionObserver*> m_observers;
};

class OptionObserver {
  public:
    O3P_API virtual ~OptionObserver() = default;

    /**
     * @brief Called when the observed option is updated
     * @param option The updated option
     */
    O3P_API virtual void onOptionUpdated(const Option &option) = 0;
};

enum OptionCommandType : uint8_t {
    OPTION_CMD_INVALID = 0,
    OPTION_CMD_SET_VALUE = 1,
    OPTION_CMD_GET_VALUE = 2,
    OPTION_CMD_GET_INFO = 3,
    OPTION_CMD_MAX
};

const char* optionCommandTypeToString(OptionCommandType type);

class OptionCommand {
  public:
    /**
     * @brief Creates a command to set the value of an option
     * @param optionType The type of the option
     * @param value The new value to set
     */
    O3P_API static OptionCommand CreateSetValueCommand(OptionType optionType, const OptionValue &value);

    /**
     * @brief Creates a command to get the value of an option
     * @param optionType The type of the option
     */
    O3P_API static OptionCommand CreateGetValueCommand(OptionType optionType);

    /**
     * @brief Creates a command to get the info of an option
     * @param optionType The type of the option
     */
    O3P_API static OptionCommand CreateGetInfoCommand(OptionType optionType);

    /**
     * @brief Gets the value associated with the command (only valid for SET_VALUE commands)
     */
    O3P_API OptionValue getValue() const;

    /**
     * @brief Gets the command type
     */
    O3P_API OptionCommandType getCommandType() const;

    /**
     * @brief Gets the option type associated with the command
     */
    O3P_API OptionType getOptionType() const;

    /**
     * @brief Serializes the OptionCommand into a byte vector suitable for transmission
     */
    O3P_API std::vector<uint8_t> serialize() const;

    /**
     * @brief Deserializes a byte vector into an OptionCommand
     * @param data The byte data
     * @param size The size of the data
     * @return The deserialized OptionCommand
     */
    O3P_API static OptionCommand deserialize(const uint8_t *data, size_t size);

    O3P_API friend std::ostream& operator<<(std::ostream& os, const OptionCommand& obj);

  private:
    OptionCommand() = default;

    OptionCommandType m_commandType;
    OptionType m_optionType;
    OptionValue m_value;
};

} // namespace o3p

#endif // O3P_OPTION_OPTION_HPP