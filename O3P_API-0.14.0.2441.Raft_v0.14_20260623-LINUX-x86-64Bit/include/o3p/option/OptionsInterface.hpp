#ifndef O3P_OPTION_OPTIONSINTERFACE_HPP
#define O3P_OPTION_OPTIONSINTERFACE_HPP

#include <o3p/option/Option.hpp>
#include <memory>

namespace o3p
{

/**
 * @brief Interface for accessing options
 */
class OptionsInterface
{
  public:
    virtual ~OptionsInterface() = default;

    /**
     * @brief Get an option by its type
     * @param optionType The type of the option to retrieve
     * @return A shared pointer to the Option
     */
    virtual std::shared_ptr<Option> getOption(OptionType optionType) = 0;

    /**
     * @brief Check if an option is supported
     * @param optionType The type of the option to check
     * @return true if the option is supported, false otherwise
     */
    virtual bool supportsOption(OptionType optionType) const = 0;
};

} // namespace o3p


#endif // O3P_OPTION_OPTIONSINTERFACE_HPP