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

#ifndef O3P_OPTION_OPTIONDESCRIPTION_HPP
#define O3P_OPTION_OPTIONDESCRIPTION_HPP

#include <o3p/Definitions.hpp>

#include <string>
#include <vector>
#include <cstdint>

namespace o3p {

/**
 * @brief Describes an option with a name and description
 */
class OptionDescription {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API OptionDescription();

    /**
     * @brief Parameterized constructor
     * @param name The name of the option
     * @param description The description of the option
     */
    O3P_API OptionDescription(const std::string &name, const std::string &description);

    O3P_API virtual ~OptionDescription() = default;
    O3P_API bool operator==(const OptionDescription &other) const;
    O3P_API bool operator!=(const OptionDescription &other) const;

    /**
     * @brief Gets the name of the option
     */
    O3P_API const std::string &getName() const;

    /**
     * @brief Gets the description of the option
     */
    O3P_API const std::string &getDescription() const;

    /**
     * @brief Serializes the OptionDescription to a byte vector suitable for transmission/storage.
     * The format is:
     * [nameLength (2 bytes)][name (nameLength bytes)][descriptionLength (2 bytes)][description (descriptionLength bytes)]
     */
    O3P_API std::vector<uint8_t> serialize() const;

    /**
     * @brief Deserializes an OptionDescription from a byte array.
     * @param data Pointer to the byte array
     * @param size Size of the byte array
     * @param bytesRead Reference to size_t to store the number of bytes read
     * @throws std::runtime_error if the data is invalid or insufficient
     */
    O3P_API static OptionDescription deserialize(const uint8_t *data, size_t size, size_t& bytesRead);

    O3P_API friend std::ostream& operator<<(std::ostream& os, const OptionDescription& obj);

  private:
    std::string m_name;
    std::string m_description;
};

} // namespace o3p

#endif // O3P_OPTION_OPTIONDESCRIPTION_HPP