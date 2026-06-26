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

#ifndef O3P_OPTION_OPTIONINFO_HPP
#define O3P_OPTION_OPTIONINFO_HPP

#include <o3p/option/OptionDescription.hpp>
#include <o3p/option/OptionRange.hpp>
#include <o3p/option/OptionValue.hpp>
#include <o3p/Definitions.hpp>

namespace o3p {

/**
 * @brief Types of options
 * Note: The actual option types supported may vary by device and should be queried at runtime.
 * When adding new option types, ensure to update the optionTypeToString and stringToOptionType functions accordingly.
 */
enum OptionType : uint8_t {
    O3P_OPTION_NONE,
    O3P_OPTION_TOF_ENABLE_AUTO_EXPOSURE,
    O3P_OPTION_TOF_MANUAL_EXPOSURE_TIME_1,
    O3P_OPTION_TOF_MANUAL_EXPOSURE_TIME_2, // For HDR modes if supported
    O3P_OPTION_COLOR_SCHEME,
    O3P_OPTION_COLOR_HISTOGRAM_EQUALIZATION,
    O3P_OPTION_COLOR_MIN_DEPTH,
    O3P_OPTION_COLOR_MAX_DEPTH,
    O3P_OPTION_RGB_ENABLE_AUTO_EXPOSURE,
    O3P_OPTION_RGB_MANUAL_EXPOSURE_TIME_1,
    O3P_OPTION_RGB_MANUAL_EXPOSURE_TIME_2, // For HDR modes if supported
    O3P_OPTION_RGB_ENABLE_AUTO_ANALOG_GAIN,
    O3P_OPTION_RGB_MANUAL_ANALOG_GAIN,
    O3P_OPTION_RGB_ENABLE_AUTO_WHITE_BALANCE,
    O3P_OPTION_RGB_MANUAL_WHITE_BALANCE,
    O3P_OPTION_MAX
};

O3P_API const char *optionTypeToString(OptionType type);
O3P_API OptionType stringToOptionType(const std::string &str);

/**
 * @brief Contains metadata about an Option
 */
class OptionInfo {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API OptionInfo();

    /**
     * @brief Parameterized constructor
     * @param type The type of the option
     * @param valueType The type of the option value
     * @param range The valid range for the option value (pass nullptr if not applicable)
     * @param description The description of the option
     * @param enabled Whether the option is enabled
     * @param readOnly Whether the option is read-only
     */
    O3P_API OptionInfo(OptionType type,
                       OptionValueType valueType,
                       std::shared_ptr<OptionValueRange> range,
                       OptionDescription description,
                       bool enabled,
                       bool readOnly);

    O3P_API virtual ~OptionInfo() = default;
    O3P_API bool operator==(const OptionInfo &other) const;
    O3P_API bool operator!=(const OptionInfo &other) const;

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
     * @brief Gets the description of the option
     */
    O3P_API const std::string &getDescription() const;

    /**
     * @brief Gets the name of the option
     */
    O3P_API const std::string &getName() const;

    /**
     * @brief Serializes the OptionInfo to a byte vector suitable for transmission/storage.
     */
    O3P_API std::vector<uint8_t> serialize() const;

    /**
     * @brief Deserializes an OptionInfo from a byte array.
     * @param data Pointer to the byte array
     * @param size Size of the byte array
     * @param bytesRead Reference to size_t to store the number of bytes read
     * @throws std::runtime_error if the data is invalid or insufficient
     */
    O3P_API static OptionInfo deserialize(const uint8_t *data, size_t size, size_t &bytesRead);

    /**
     * @brief Checks if the option is enabled
     */
    O3P_API bool isEnabled() const;

    /**
     * @brief Checks if the option is read-only
     */
    O3P_API bool isReadOnly() const;

    O3P_API friend std::ostream &operator<<(std::ostream &os, const OptionInfo &obj);

  private:
    OptionType m_type;
    OptionValueType m_valueType;
    std::shared_ptr<OptionValueRange> m_range;
    OptionDescription m_description;
    bool m_enabled;
    bool m_readOnly;
};

} // namespace o3p

#endif // O3P_OPTION_OPTIONINFO_HPP