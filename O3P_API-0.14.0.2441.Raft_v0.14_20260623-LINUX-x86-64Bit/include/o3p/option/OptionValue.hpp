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

#ifndef O3P_OPTION_OPTIONVALUE_HPP
#define O3P_OPTION_OPTIONVALUE_HPP

#include <o3p/Definitions.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace o3p {

/**
 * @brief Types of option values
 */
enum OptionValueType : uint8_t {
    O3P_OPTION_VALUE_TYPE_NONE,
    O3P_OPTION_VALUE_TYPE_INT,
    O3P_OPTION_VALUE_TYPE_FLOAT,
    O3P_OPTION_VALUE_TYPE_STRING,
    O3P_OPTION_VALUE_TYPE_BOOL,
    O3P_OPTION_VALUE_TYPE_MAX
};

/**
 * @brief Converts an OptionValueType to its string representation
 */
const char *optionValueTypeToString(OptionValueType type);

/**
 * @brief Represents a value for an option
 */
class OptionValue {
  public:
    /** 
     * @brief Default constructor
     */
    O3P_API OptionValue();

    O3P_API bool operator==(const OptionValue &other) const;
    O3P_API bool operator!=(const OptionValue &other) const;

    /** 
     * @brief Constructor for OptionValue of type int
     */
    O3P_API OptionValue(const int value);

    /** 
     * @brief Constructor for OptionValue of type float
     */
    O3P_API OptionValue(const float value);

    /** 
     * @brief Constructor for OptionValue of type bool
     */
    O3P_API OptionValue(const bool value);

    /** 
     * @brief Constructor for OptionValue of type string
     */
    O3P_API OptionValue(const std::string &value);

    /** 
     * @brief Constructor for OptionValue of type string from C-string
     */
    O3P_API OptionValue(const char* value);

    /** 
     * @brief Cast operator to get int value from this OptionValue
     * @throws std::runtime_error if the stored type is not int
     */
    O3P_API operator int() const;

    /** 
     * @brief Cast operator to get float value from this OptionValue
     * @throws std::runtime_error if the stored type is not float
     */
    O3P_API operator float() const;

    /** 
     * @brief Cast operator to get bool value from this OptionValue
     * @throws std::runtime_error if the stored type is not bool
     */
    O3P_API operator bool() const;

    /** 
     * @brief Cast operator to get string value from this OptionValue
     * @throws std::runtime_error if the stored type is not string
     */
    O3P_API operator std::string() const;

    /** 
     * @brief Gets the type of the stored value
     */
    O3P_API OptionValueType getType() const;

    /** 
     * @brief Serializes the OptionValue into a byte vector suitable for transmission/storage
     */
    O3P_API std::vector<uint8_t> serialize() const;

    /** 
     * @brief Deserializes an OptionValue from a byte array
     * @param data Pointer to the byte array
     * @param size Size of the byte array
     * @param bytesRead Reference to size_t to store the number of bytes read
     * @throws std::runtime_error if the data is invalid or insufficient
     */
    O3P_API static OptionValue deserialize(const uint8_t *data, size_t size, size_t &bytesRead);

    /**
     * @brief Converts the OptionValue to a JSON string representation
     */
    O3P_API std::string toJsonValue() const;

    O3P_API friend std::ostream& operator<<(std::ostream& os, const OptionValue& obj);

  private:
    OptionValueType m_type;
    int m_intValue;
    float m_floatValue;
    bool m_boolValue;
    std::string m_stringValue;
};
} // namespace o3p

#endif // O3P_OPTION_OPTIONVALUE_HPP