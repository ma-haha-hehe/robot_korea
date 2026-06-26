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

#ifndef O3P_OPTION_BACKEND_OPTION_HPP
#define O3P_OPTION_BACKEND_OPTION_HPP

#include <o3p/option/Option.hpp>

namespace o3p {

class BackendOption;

/**
 * @brief Interface for communicating Option related requests to the firmware
 */
class FirmwareOptionInterface {
  public:
    O3P_API virtual ~FirmwareOptionInterface() = default;

    /**
     * @brief Requests a BackendOption for the given OptionType. Will construct a BackendOption
     * with results provided by @ref requestOptionInfo and @ref requestOptionValue.
     * @param optionType The type of the option to request
     * @return A shared pointer to the BackendOption
     */
    O3P_API virtual std::shared_ptr<BackendOption> requestOption(OptionType optionType);

    /**
     * @brief Requests to update the option value in the firmware
     * @param option The option to update
     * @param value The new value to set
     * @return true if the update was successful, false otherwise
     */
    O3P_API virtual bool requestOptionValueUpdate(Option &option, const OptionValue &value) = 0;

    /**
     * @brief Requests the OptionInfo for the given OptionType from the firmware
     * @param optionType The type of the option
     * @return The OptionInfo
     */
    O3P_API virtual OptionInfo requestOptionInfo(OptionType optionType) = 0;

    /**
     * @brief Requests the OptionValue for the given OptionType from the firmware
     * @param optionType The type of the option
     * @return The OptionValue
     */
    O3P_API virtual OptionValue requestOptionValue(OptionType optionType) = 0;
        
};

/**
 * @brief Represents an Option which communicates with the firmware via @ref FirmwareOptionInterface
 */
class BackendOption : public Option {
  public:
    /**
     * @brief Constructor
     * @param optionType The type of the option
     * @param firmwareInterface The firmware interface to use for communication
     */
    O3P_API BackendOption(OptionType optionType, FirmwareOptionInterface *firmwareInterface);

    /**
     * @brief Sets a new value for the option
     * @param newValue The new value to set
     */
    O3P_API void setValue(const OptionValue &newValue) override;

    /**
     * @brief Queries the firmware for the latest option info and value, updating internal state if changed
     */
    O3P_API void refresh();

  private:
    FirmwareOptionInterface *m_firmwareInterface;
};

} // namespace o3p

#endif // O3P_OPTION_BACKEND_OPTION_HPP