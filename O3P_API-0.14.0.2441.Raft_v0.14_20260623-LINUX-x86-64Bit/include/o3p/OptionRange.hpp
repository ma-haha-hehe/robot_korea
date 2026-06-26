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

#ifndef O3P_OPTION_OPTIONRANGE_HPP
#define O3P_OPTION_OPTIONRANGE_HPP

#include <o3p/option/OptionValue.hpp>
#include <o3p/Definitions.hpp>

#include <iostream>
#include <cstdint>
#include <memory>
#include <string>
#include <stdexcept>

namespace o3p {

/**
 * @brief Base class for defining valid ranges for option values
 */
class OptionValueRange {
  public:
    O3P_API bool operator==(const OptionValueRange &other) const;
    O3P_API bool operator!=(const OptionValueRange &other) const;

    /**
     * @brief Serializes the OptionValueRange to a byte vector suitable for transmission/storage.
     */
    O3P_API virtual std::vector<uint8_t> serialize() const = 0;

    /**
     * @brief Checks if the given OptionValue is valid within this range
     * @param value The OptionValue to check
     * @return true if valid, false otherwise
     */
    O3P_API virtual bool isValid(const OptionValue &value) const = 0;

    /**
     * @brief Gets the type of values this range supports
     */
    O3P_API OptionValueType getType() const;

    O3P_API friend std::ostream &operator<<(std::ostream &os, const OptionValueRange &obj);

  protected:
    /**
     * @brief Protected constructor to prevent direct instantiation (abstract class)
     * @param type The type of values this range supports
     */
    OptionValueRange(OptionValueType type) : m_valueType(type) {}

    /**
     * @brief Compares this range with another for equality
     * @param other The other OptionValueRange to compare with
     * @return true if equal, false otherwise
     */
    O3P_API virtual bool equals(const OptionValueRange &other) const = 0;

    /**
     * @brief Provides a debug string representation of the range
     * @return A string describing the range
     */
    virtual std::string toDebugString() const = 0;

    OptionValueType m_valueType;
};

/**
 * @brief Represents a valid range for integer option values
 */
class IntOptionRange : public OptionValueRange {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API IntOptionRange();

    /**
     * @brief Parameterized constructor
     * @param min Minimum valid value
     * @param max Maximum valid value
     * @param defaultValue Default value
     * @param step Step size between valid values
     */
    O3P_API IntOptionRange(int min, int max, int defaultValue, int step);

    /**
     * @brief Gets the minimum valid value
     */
    O3P_API int getMin() const;

    /**
     * @brief Gets the maximum valid value
     */
    O3P_API int getMax() const;

    /**
     * @brief Gets the default value
     */
    O3P_API int getDefaultValue() const;

    /**
     * @brief Gets the step size between valid values
     */
    O3P_API int getStep() const;

    /**
     * @brief Serializes the IntOptionRange to a byte vector suitable for transmission/storage.
     */
    O3P_API std::vector<uint8_t> serialize() const override;

    /**
     * @brief Deserializes an IntOptionRange from a byte array.
     * @param data Pointer to the byte array
     * @param size Size of the byte array
     * @param bytesRead Reference to size_t to store the number of bytes read
     * @throws std::runtime_error if the data is invalid or insufficient
     */
    O3P_API static std::shared_ptr<IntOptionRange> deserialize(const uint8_t *data, size_t size, size_t &bytesRead);

    /**
     * @brief Checks if the given OptionValue is valid within this range
     * @param value The OptionValue to check
     * @return true if valid, false otherwise
     */
    O3P_API bool isValid(const OptionValue &value) const override;

  private:
    /**
     * @brief Compares this range with another for equality
     * @param other The other OptionValueRange to compare with
     * @return true if equal, false otherwise
     */
    O3P_API bool equals(const OptionValueRange &other) const override;

    /**
     * @brief Provides a debug string representation of the range
     * @return A string describing the range
     */
    O3P_API std::string toDebugString() const override;

    int m_min;
    int m_max;
    int m_defaultValue;
    int m_step;
};

/**
 * @brief Represents a valid range for float option values
 */
class FloatOptionRange : public OptionValueRange {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API FloatOptionRange();

    /**
     * @brief Parameterized constructor
     * @param min Minimum valid value
     * @param max Maximum valid value
     * @param defaultValue Default value
     * @param step Step size between valid values
     */
    O3P_API FloatOptionRange(float min, float max, float defaultValue, float step);

    /**
     * @brief Gets the minimum valid value
     */
    O3P_API float getMin() const;

    /**
     * @brief Gets the maximum valid value
     */
    O3P_API float getMax() const;

    /**
     * @brief Gets the default value
     */
    O3P_API float getDefaultValue() const;

    /**
     * @brief Gets the step size between valid values
     */
    O3P_API float getStep() const;

    /**
     * @brief Serializes the FloatOptionRange to a byte vector suitable for transmission/storage.
     */
    O3P_API std::vector<uint8_t> serialize() const override;

    /**
     * @brief Deserializes a FloatOptionRange from a byte array.
     * @param data Pointer to the byte array
     * @param size Size of the byte array
     * @param bytesRead Reference to size_t to store the number of bytes read
     * @throws std::runtime_error if the data is invalid or insufficient
     */
    O3P_API static std::shared_ptr<FloatOptionRange> deserialize(const uint8_t *data, size_t size, size_t &bytesRead);

    /**
     * @brief Checks if the given OptionValue is valid within this range
     * @param value The OptionValue to check
     * @return true if valid, false otherwise
     */
    O3P_API bool isValid(const OptionValue &value) const override;

  private:
    /**
     * @brief Compares this range with another for equality
     * @param other The other OptionValueRange to compare with
     * @return true if equal, false otherwise
     */
    O3P_API bool equals(const OptionValueRange &other) const override;

    /**
     * @brief Provides a debug string representation of the range
     * @return A string describing the range
     */
    O3P_API std::string toDebugString() const override;

    float m_min;
    float m_max;
    float m_defaultValue;
    float m_step;
};

/**
 * @brief Represents a valid range for string option values
 */
class StringOptionRange : public OptionValueRange {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API StringOptionRange();

    /**
     * @brief Parameterized constructor
     * @param defaultVal Default value
     * @param allowedVals List of allowed string values
     */
    O3P_API StringOptionRange(const std::string &defaultVal, const std::vector<std::string> &allowedVals);

    /**
     * @brief Gets the default value
     */
    O3P_API std::string getDefaultValue() const;

    /**
     * @brief Gets the list of allowed string values
     */
    O3P_API std::vector<std::string> getAllowedValues() const;

    /**
     * @brief Serializes the StringOptionRange to a byte vector suitable for transmission/storage.
     */
    O3P_API std::vector<uint8_t> serialize() const override;

    /**
     * @brief Deserializes a StringOptionRange from a byte array.
     * @param data Pointer to the byte array
     * @param size Size of the byte array
     * @param bytesRead Reference to size_t to store the number of bytes read
     * @throws std::runtime_error if the data is invalid or insufficient
     */
    O3P_API static std::shared_ptr<StringOptionRange> deserialize(const uint8_t *data, size_t size, size_t &bytesRead);

    /**
     * @brief Checks if the given OptionValue is valid within this range
     * @param value The OptionValue to check
     * @return true if valid, false otherwise
     */
    O3P_API bool isValid(const OptionValue &value) const override;

  private:
    /**
     * @brief Compares this range with another for equality
     * @param other The other OptionValueRange to compare with
     * @return true if equal, false otherwise
     */
    O3P_API bool equals(const OptionValueRange &other) const override;

    /**
     * @brief Provides a debug string representation of the range
     * @return A string describing the range
     */
    O3P_API std::string toDebugString() const override;

    std::string m_defaultValue;
    std::vector<std::string> m_allowedValues;
};

/**
 * @brief Deserializes an OptionValueRange from a byte array. The returned OptionValueRange
 * may be of type IntOptionRange, FloatOptionRange, StringOptionRange, or nullptr. Use @ref OptionValueRange::getType()
 * on the returned object to determine its actual type.
 *
 * @param data Pointer to the byte array
 * @param size Size of the byte array
 * @param bytesRead Reference to size_t to store the number of bytes read
 * @throws std::runtime_error if the data is invalid or insufficient
 */
std::shared_ptr<OptionValueRange> deserializeOptionValueRange(const uint8_t *data, size_t size, size_t &bytesRead);

} // namespace o3p

#endif // O3P_OPTION_OPTIONRANGE_HPP