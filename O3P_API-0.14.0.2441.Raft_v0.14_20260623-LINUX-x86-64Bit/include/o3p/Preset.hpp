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

#ifndef PRESET_HPP
#define PRESET_HPP

#include <list>
#include <string>
#include <vector>


#include "Definitions.hpp"
#include "option/OptionValue.hpp"

namespace o3p {

/**
 * @brief A preset defines a specific combination of camera parameters that can be applied to the device.
 * 
 * A preset can be converted to and from JSON format for easy storage and retrieval.
 * The JSON format includes the following fields:
 * - version: The version of the preset format (currently 1)
 * - name: The name of the preset
 * - fps: The frames per second setting of the preset
 * - hdr: The HDR mode setting of the preset
 * - measurement_range: The measurement range setting of the preset
 * - distance_offset: The distance offset setting of the preset (optional, default is 0)
 * - config_values: A list of key-value pairs representing additional configuration values (optional)
 * - options: A list of key-value pairs representing additional options (optional)
 */
class Preset {
  public:
    /**
     * @brief Serialize this preset to a JSON string
     *
     * @return The JSON representation of the preset
     */
    O3P_API std::string toJson() const;

    /**
     * @brief Construct a Preset from a JSON string
     *
     * @param jsonString JSON description of a single preset
     * @return The parsed preset
     */
    static O3P_API Preset fromJson(const std::string &jsonString);

    /**
     * @brief Construct a list of Presets from a JSON string
     *
     * @param jsonString JSON description containing multiple presets
     * @return The parsed presets
     */
    static O3P_API std::list<Preset> manyFromJson(const std::string &jsonString);

    /**
     * @brief Set the name of the preset
     *
     * @param name The new name
     */
    void setName(const std::string &name) {
        m_name = name;
    }

    /**
     * @brief Set the human readable description of the preset
     *
     * @param description The new description
     */
    void setDescription(const std::string &description) {
        m_description = description;
    }

    /**
     * @brief Set the frames-per-second of the preset
     *
     * @param fps Frames per second
     */
    void setFps(unsigned fps) {
        m_fps = fps;
    }

    /**
     * @brief Enable or disable HDR for the preset
     *
     * @param hdr true to enable HDR, false to disable
     */
    void setHdr(bool hdr) {
        m_hdr = hdr;
    }

    /**
     * @brief Set the measurement range identifier of the preset
     *
     * @param measurementRange The measurement range identifier
     */
    void setMeasurementRange(const std::string &measurementRange) {
        m_measurementRange = measurementRange;
    }

    /**
     * @brief Set the distance offset of the preset
     *
     * @param distanceOffset The distance offset value
     */
    void setDistanceOffset(unsigned distanceOffset) {
        m_distanceOffset = distanceOffset;
    }

    /**
     * @brief Get the name of the preset
     *
     * @return The name
     */
    const std::string &name() const {
        return m_name;
    }

    /**
     * @brief Get the description of the preset
     *
     * @return The description
     */
    const std::string &description() const {
        return m_description;
    }

    /**
     * @brief Get the frames-per-second of the preset
     *
     * @return Frames per second
     */
    unsigned fps() const {
        return m_fps;
    }

    /**
     * @brief Get the HDR flag of the preset
     *
     * @return true if HDR is enabled, false otherwise
     */
    bool hdr() const {
        return m_hdr;
    }

    /**
     * @brief Get the measurement range identifier of the preset
     *
     * @return The measurement range identifier
     */
    const std::string &measurementRange() const {
        return m_measurementRange;
    }

    /**
     * @brief Get the distance offset of the preset
     *
     * @return The distance offset value
     */
    unsigned distanceOffset() const {
        return m_distanceOffset;
    }

    /**
     * @brief Get the additional firmware configuration values of the preset
     *
     * @return List of (key, value) pairs of configuration values
     */
    const std::list<std::pair<std::string, std::string>> &configValues() const {
        return m_configValues;
    }

    /**
     * @brief Get the additional options of the preset
     *
     * @return List of (option name, option value) pairs
     */
    const std::list<std::pair<std::string, o3p::OptionValue>> &options() const {
        return m_options;
    }

  private:
    std::string m_name;
    std::string m_description;
    unsigned m_fps;
    bool m_hdr;
    std::string m_measurementRange;
    unsigned m_distanceOffset;
    std::list<std::pair<std::string, std::string>> m_configValues;
    std::list<std::pair<std::string, o3p::OptionValue>> m_options;
};

}; // namespace o3p

#endif // PRESET_HPP