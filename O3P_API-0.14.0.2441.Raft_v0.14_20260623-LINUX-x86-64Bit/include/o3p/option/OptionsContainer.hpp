#ifndef O3P_OPTION_OPTIONSCONTAINER_HPP
#define O3P_OPTION_OPTIONSCONTAINER_HPP

#include <o3p/Definitions.hpp>
#include <o3p/option/OptionsInterface.hpp>

#include <map>
#include <memory>
#include <stdexcept>

namespace o3p {

/**
 * @brief Container for managing multiple options
 */
class OptionsContainer : public OptionsInterface {
  public:
    virtual ~OptionsContainer() = default;

    /**
     * @brief Get an option by its type
     * @param optionType The type of the option to retrieve
     * @return A shared pointer to the Option
     * @throws std::runtime_error if the option is not found. Use @ref supportsOption to check for existence.
     */
    std::shared_ptr<Option> getOption(OptionType optionType) override {
        auto it = m_options.find(optionType);
        if (it == m_options.end()) {
            // Option not found
            return nullptr;
        }
        return it->second;
    }

    /**
     * @brief Check if an option is supported
     * @param optionType The type of the option to check
     * @return true if the option is supported, false otherwise
     */
    bool supportsOption(OptionType optionType) const override {
        return m_options.find(optionType) != m_options.end();
    }

    /**
     * @brief Adds a new option to the container
     * @param option The option to add
     * @throws std::runtime_error if the option is null or already exists
     */
    void addOption(std::shared_ptr<Option> option) {
        if (!option) {
            throw std::runtime_error("Cannot add null option");
        }

        if (supportsOption(option->getType())) {
            throw std::runtime_error("Option already exists");
        }

        m_options[option->getType()] = option;
    }

    /**
     * @brief Removes an option from the container
     * @param optionType The type of the option to remove
     * @throws std::runtime_error if the option does not exist
     */
    void removeOption(OptionType optionType) {
        if (!supportsOption(optionType)) {
            throw std::runtime_error("Option not found");
        }

        m_options.erase(optionType);
    }

    /**
     * @brief Clears all options from the container
     */
    void clearOptions() {
        m_options.clear();
    }

  private:
    std::map<OptionType, std::shared_ptr<Option>> m_options;
};

} // namespace o3p

#endif // O3P_OPTION_OPTIONSCONTAINER_HPP