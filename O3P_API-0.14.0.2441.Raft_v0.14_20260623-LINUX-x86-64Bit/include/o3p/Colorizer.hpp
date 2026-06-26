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

#ifndef COLORIZER_HPP
#define COLORIZER_HPP

#include <o3p/o3p.hpp>
#include <o3p/option/Option.hpp>
#include <o3p/option/OptionsContainer.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

namespace o3p {

/**
 * @brief Simple 3D vector for RGB colors
 */
struct RgbColor {
    float r, g, b;
    RgbColor() : r(0), g(0), b(0) {}
    RgbColor(float r, float g, float b) : r(r), g(g), b(b) {}

    RgbColor operator*(float t) const { return RgbColor(r * t, g * t, b * t); }
    RgbColor operator+(const RgbColor &other) const { return RgbColor(r + other.r, g + other.g, b + other.b); }
};

/**
 * @brief 3D vector for HSV colors with each channel represented as uint8
 */
typedef struct HsvColor {
    /**
     * The h value defines the intensity of the hue as an integer between 0 and 255.
     * By default the hue value is described as an integer between 0 and 360.
     * to convert the h value to the real hue multiply it with 1.41176:
     * auto hue = static_cast<float> (h) * 1.41176;
     */
    uint8_t h;

    /**
     * The s value defines the intensity of the saturation as an integer between 0 and 255.
     * By default the saturation value is described as a float between 0 and 1.
     * to convert the s value to the real saturation multiply it with 0.00392157:
     * auto saturation = static_cast<float> (s) * 0.00392157;
     */
    uint8_t s;

    /**
     * The v value defines the intensity of the value as an integer between 0 and 255.
     * By default the value value is described as a float between 0 and 1.
     * to convert the v value to the real value multiply it with 0.00392157:
     * auto value = static_cast<float> (v) * 0.00392157;
     */
    uint8_t v;
} HsvColor;

/**
 * @brief Color map mapping a normalized value to an RGB color via control points
 */
class ColorMap {
  public:
    /**
     * @brief Default constructor with empty color list
     */
    O3P_API ColorMap() : m_min(0), m_max(6), m_cacheSize(4000) {}

    /**
     * @brief Construct a color map from a list of control point colors
     *
     * @param colors Control point colors of the map
     * @param cacheSize Number of entries in the lookup cache
     */
    O3P_API ColorMap(const std::vector<RgbColor> &colors, int cacheSize = 4000);

    /**
     * @brief Get the color for a normalized value in [0, 1]
     *
     * @param value Normalized value
     * @return The interpolated color
     */
    O3P_API RgbColor getColor(float value) const;

    /**
     * @brief Get the lower bound of the value range
     */
    O3P_API float getMinValue() const { return m_min; }

    /**
     * @brief Get the upper bound of the value range
     */
    O3P_API float getMaxValue() const { return m_max; }

  private:
    static RgbColor lerp(const RgbColor &a, const RgbColor &b, float t);
    void buildCache(int steps);

    std::map<float, RgbColor> m_controlPoints;
    std::vector<RgbColor> m_cache;
    float m_min, m_max;
    size_t m_cacheSize;
};

/**
 * @brief Available built-in color schemes used by the colorizer
 */
enum class ColorScheme {
    ROYALE,
    CLASSIC,
    GRAYSCALE,
    INV_GRAYSCALE,
    BIOMES,
    COLD,
    WARM,
    QUANTIZED,
    PATTERN,
    HUE,
    TURBO,
    COUNT
};

/**
 * @brief Converts the given ColorScheme type to std::string
 *
 * @param scheme The color scheme that should be converted
 */
inline std::string colorScheme2String(ColorScheme scheme) {
    switch (scheme) {
    case ColorScheme::ROYALE:
        return "ROYALE";
    case ColorScheme::CLASSIC:
        return "CLASSIC";
    case ColorScheme::GRAYSCALE:
        return "GRAYSCALE";
    case ColorScheme::INV_GRAYSCALE:
        return "INV_GRAYSCALE";
    case ColorScheme::BIOMES:
        return "BIOMES";
    case ColorScheme::COLD:
        return "COLD";
    case ColorScheme::WARM:
        return "WARM";
    case ColorScheme::QUANTIZED:
        return "QUANTIZED";
    case ColorScheme::PATTERN:
        return "PATTERN";
    case ColorScheme::HUE:
        return "HUE";
    case ColorScheme::TURBO:
        return "TURBO";
    case ColorScheme::COUNT:
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Option implementation specialized for the Colorizer
 */
class ColorizerOption : public Option {
  public:
    /**
     * @brief Construct a colorizer option from info and value
     *
     * @param info Description of the option
     * @param value Initial value of the option
     */
    ColorizerOption(const OptionInfo &info, const OptionValue &value) {
        m_info = info;
        m_value = value;
    }
};

/**
 * @brief Convert depth data to colorized RGB images using configurable color schemes
 */
class Colorizer : public OptionObserver {
  public:
    /**
     * @brief Default constructor
     */
    O3P_API Colorizer();

    /**
     * @brief Get the options container of the colorizer
     *
     * @return Reference to the colorizer options
     */
    O3P_API OptionsContainer &getOptions();

    /**
     * @brief Called when one of the colorizer options is updated
     *
     * @param option The updated option
     */
    O3P_API void onOptionUpdated(const Option &option) override;

    /**
     * @brief Process depth data and convert it to a packed RGB image
     *
     * @tparam T Type of the depth values (uint16_t for millimeters, float for meters)
     * @param depthData Pointer to the input depth values
     * @param rgbData Output buffer that receives the RGB pixels (resized as needed)
     * @param width Image width in pixels
     * @param height Image height in pixels
     * @param depthScale Conversion factor (e.g. 0.001 for mm to meters)
     */
    template <typename T>
    O3P_API void colorize(const T *depthData, std::vector<uint8_t> &rgbData, int width, int height, float depthScale = 0.001f);

    ColorMap m_colorMap;

  private:
    void initializeOptions();
    // Color schemes
    void initializeColorSchemes();
    ColorMap createColorMap(ColorScheme scheme);
    static RgbColor hsvToRgb(const HsvColor &hsv);
    static std::vector<RgbColor> turboToRgb();

    // Processing functions
    template <typename T>
    void buildHistogram(const T *depthData, int width, int height, float depthScale);

    template <typename T>
    void colorizeFixedRange(const T *depthData, std::vector<uint8_t> &rgbData, int width, int height, float depthScale);

    template <typename T>
    void colorizeHistogramEqualized(const T *depthData, std::vector<uint8_t> &rgbData, int width, int height, float depthScale);

    // Settings
    ColorScheme m_currentScheme;
    bool m_histogramEqualization;
    int m_minDepthFilter, m_maxDepthFilter; // in millimeters

    // Color maps
    std::vector<ColorMap> m_colorMaps;

    // Histogram for equalization
    static const int MAX_DEPTH_VALUES = 0x10000; // 65536 for 16-bit depth
    std::vector<int> m_histogram;
    int *m_histData;

    OptionsContainer m_options;
};

// Template implementation needs to be in header
template <typename T>
void Colorizer::colorize(const T *depthData, std::vector<uint8_t> &rgbData, int width, int height, float depthScale) {
    // Allocate RGB data if needed
    if (rgbData.size() < static_cast<size_t>(width * height * 3)) {
        rgbData.resize(width * height * 3);
    }

    if (m_histogramEqualization) {
        buildHistogram(depthData, width, height, depthScale);
        colorizeHistogramEqualized(depthData, rgbData, width, height, depthScale);
    } else {
        colorizeFixedRange(depthData, rgbData, width, height, depthScale);
    }
}

template <typename T>
void Colorizer::buildHistogram(const T *depthData, int width, int height, float depthScale) {
    // Clear histogram
    std::fill(m_histogram.begin(), m_histogram.end(), 0);

    // Build histogram
    for (int i = 0; i < width * height; ++i) {
        T depth_val = depthData[i];
        if (depth_val > 0) {
            int index = static_cast<int>(depth_val * depthScale * 1000.0f); // Convert to mm
            if (index < MAX_DEPTH_VALUES) {
                m_histogram[index]++;
            }
        }
    }

    // Build cumulative histogram
    for (int i = 1; i < MAX_DEPTH_VALUES; ++i) {
        m_histogram[i] += m_histogram[i - 1];
    }
}

template <typename T>
void Colorizer::colorizeFixedRange(const T *depthData, std::vector<uint8_t> &rgbData, int width, int height, float depthScale) {
    m_colorMap = m_colorMaps[static_cast<int>(m_currentScheme)];

    float minInM = static_cast<float>(m_minDepthFilter) / 1000; // convert to meters
    float maxInM = static_cast<float>(m_maxDepthFilter) / 1000; // convert to meters

    for (int i = 0; i < width * height; ++i) {
        T depth_val = depthData[i];

        if (depth_val > 0) {
            float depth_meters = depth_val * depthScale;

            // Normalize depth to [0,1]
            float normalized = (depth_meters - minInM) / (maxInM - minInM); // fixed range
            if (normalized > 1.0f)
                normalized = 1.0f;
            if (normalized < 0.0f)
                normalized = 0.0f;
            RgbColor color = m_colorMap.getColor(normalized);
            rgbData[i * 3 + 0] = static_cast<uint8_t>(color.r);
            rgbData[i * 3 + 1] = static_cast<uint8_t>(color.g);
            rgbData[i * 3 + 2] = static_cast<uint8_t>(color.b);
        } else {
            // Invalid depth - set to black
            rgbData[i * 3 + 0] = 0;
            rgbData[i * 3 + 1] = 0;
            rgbData[i * 3 + 2] = 0;
        }
    }
}

template <typename T>
void Colorizer::colorizeHistogramEqualized(const T *depthData, std::vector<uint8_t> &rgbData, int width, int height, float depthScale) {
    m_colorMap = m_colorMaps[static_cast<int>(m_currentScheme)];
    int total_pixels = m_histogram[MAX_DEPTH_VALUES - 1];

    for (int i = 0; i < width * height; ++i) {
        T depth_val = depthData[i];

        if (depth_val > 0) {
            int index = static_cast<int>(depth_val * depthScale * 1000.0f); // Convert to mm
            if (index < MAX_DEPTH_VALUES && total_pixels > 0) {
                float normalized = static_cast<float>(m_histogram[index]) / static_cast<float>(total_pixels);
                RgbColor color = m_colorMap.getColor(normalized);
                rgbData[i * 3 + 0] = static_cast<uint8_t>(color.r);
                rgbData[i * 3 + 1] = static_cast<uint8_t>(color.g);
                rgbData[i * 3 + 2] = static_cast<uint8_t>(color.b);
            } else {
                rgbData[i * 3 + 0] = 0;
                rgbData[i * 3 + 1] = 0;
                rgbData[i * 3 + 2] = 0;
            }
        } else {
            // Invalid depth - set to black
            rgbData[i * 3 + 0] = 0;
            rgbData[i * 3 + 1] = 0;
            rgbData[i * 3 + 2] = 0;
        }
    }
}

} // namespace o3p

#endif // COLORIZER_HPP
