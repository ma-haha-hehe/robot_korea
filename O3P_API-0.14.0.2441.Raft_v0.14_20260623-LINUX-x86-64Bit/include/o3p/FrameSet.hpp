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

#ifndef FRAMESET_HPP
#define FRAMESET_HPP

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "Common.hpp"

namespace o3p {

/**
 * @brief Base class for frames
 */
class Frame {
  public:
    O3P_API Frame() : m_timestamp(0) {
    }

    O3P_API virtual ~Frame() = default;

    /**
     * @brief Copy constructor
     *
     * @param other The Frame object to copy from
     */
    O3P_API Frame(const Frame &other) : m_timestamp(other.m_timestamp),
                                        m_metadataBuffer(other.m_metadataBuffer),
                                        m_metadataLayout(other.m_metadataLayout) {
    }

    /**
     * @brief Assignment operator
     *
     * @param other The Frame object to assign from
     * @return Reference to the assigned Frame object
     */
    O3P_API Frame &operator=(const Frame &other) {
        if (&other != this) {
            m_timestamp = other.m_timestamp;
            m_metadataBuffer = other.m_metadataBuffer;
            m_metadataLayout = other.m_metadataLayout;
        }

        return *this;
    }

    /**
     * @brief Gets the current timestamp in nanoseconds
     *
     * @return The current timestamp
     */
    O3P_API virtual uint64_t getTimestamp() const {
        return m_timestamp;
    }

    /**
     * @brief Checks if the frame supports the given metadata type
     *
     * @param type The metadata type to check
     */
    bool supportsMetadata(MetadataType type) const {
        return m_metadataLayout.find(type) != m_metadataLayout.end();
    }

    /**
     * @brief Gets the metadata of the given type
     *
     * @tparam T The type of the metadata
     * @param type The metadata type to get
     * @return The metadata value, or a default value if the metadata is not supported
     */
    template <typename T>
    T getMetadata(o3p::MetadataType type) const {
        T value = T();

        if (supportsMetadata(type)) {
            value = o3p::getMetadata<T>(m_metadataBuffer.data(), type, m_metadataLayout.at(type));
        }

        return value;
    }

    uint64_t m_timestamp;

    std::vector<uint8_t> m_metadataBuffer;
    std::unordered_map<MetadataType, size_t> m_metadataLayout; /**< The metadata layout for the frame */
};

/**
 * @brief Represents videoframes
 *
 * Extends the Frames class for videoframes
 */
class VideoFrame : public Frame {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API VideoFrame() : m_width(0u),
                           m_height(0u),
                           m_bitsPerPixel(0u),
                           m_data(nullptr),
                           m_isCopy(false) {
    }

    /**
     * @brief Copy constructor
     *
     * @param other The VideoFrame object to copy from
     */
    O3P_API VideoFrame(const VideoFrame &other) : Frame(other),
                                                  m_width(other.m_width),
                                                  m_height(other.m_height),
                                                  m_bitsPerPixel(other.m_bitsPerPixel),
                                                  m_isCopy(true) {
        size_t byteSize = static_cast<size_t>(m_width) * m_height * m_bitsPerPixel / 8;
        m_dataCopy.resize(byteSize);
        if (byteSize > 0 && other.m_data != nullptr) {
            memcpy(m_dataCopy.data(), other.m_data, byteSize);
        }
        m_data = m_dataCopy.data();
    }

    /**
     * @brief Assignment operator
     *
     * @param other The VideoFrame object to assign from
     * @return Reference to the assigned VideoFrame object
     */
    O3P_API VideoFrame &operator=(const VideoFrame &other) {
        if (&other != this) {
            Frame::operator=(other);
            m_width = other.m_width;
            m_height = other.m_height;
            m_bitsPerPixel = other.m_bitsPerPixel;
            m_isCopy = true;
            size_t byteSize = static_cast<size_t>(m_width) * m_height * m_bitsPerPixel / 8;
            m_dataCopy.resize(byteSize);
            if (byteSize > 0 && other.m_data != nullptr) {
                memcpy(m_dataCopy.data(), other.m_data, byteSize);
            }
            m_data = m_dataCopy.data();
        }

        return *this;
    }

    /**
     * @brief Gets the width of the frame
     *
     * @return The width
     */
    O3P_API uint16_t getWidth() const {
        return m_width;
    }

    /**
     * @brief Gets the height of the frame
     *
     * @return The height
     */
    O3P_API uint16_t getHeight() const {
        return m_height;
    }

    uint16_t m_width;        /**< The width of the frame */
    uint16_t m_height;       /**< The height of the frame */
    uint16_t m_bitsPerPixel; /**< The number of bits per pixel */

    uint8_t *m_data;                 /**< Pointer to the frame data */
    bool m_isCopy;                   /**< Flag indicating if the frame data is a copy */
    std::vector<uint8_t> m_dataCopy; /**< Copy of the frame data */
};

/**
 * @brief Represents depth frames
 *
 * Extends the Frames class for depth frames.
 * Each pixel is represented by a single uint16_t value in millimeters.
 */
class DepthFrame : public Frame {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API DepthFrame() : m_width(0u),
                           m_height(0u),
                           m_data(nullptr),
                           m_isCopy(false) {
    }

    /**
     * @brief Copy constructor
     *
     * @param other The DepthFrame object to copy from
     */
    O3P_API DepthFrame(const DepthFrame &other) : Frame(other),
                                                  m_width(other.m_width),
                                                  m_height(other.m_height),
                                                  m_isCopy(true) {
        m_dataCopy.resize(m_width * m_height);
        memcpy(m_dataCopy.data(), other.m_data, m_width * m_height * sizeof(uint16_t));
        m_data = m_dataCopy.data();
    }

    /**
     * @brief Assignment operator
     *
     * @param other The DepthFrame object to assign from
     * @return Reference to the assigned DepthFrame object
     */
    O3P_API DepthFrame &operator=(const DepthFrame &other) {
        if (&other != this) {
            Frame::operator=(other);
            m_width = other.m_width;
            m_height = other.m_height;
            m_isCopy = true;
            m_dataCopy.resize(m_width * m_height);
            memcpy(m_dataCopy.data(), other.m_data, m_width * m_height * sizeof(uint16_t));
            m_data = m_dataCopy.data();
        }

        return *this;
    }

    /**
     * @brief Gets the width of the frame
     *
     * @return The width
     */
    O3P_API uint16_t getWidth() const {
        return m_width;
    }

    /**
     * @brief Gets the height of the frame
     *
     * @return The height
     */
    O3P_API uint16_t getHeight() const {
        return m_height;
    }

    /**
     * @brief Gets the distance at (x,y) in meters
     *
     * @param x The x-coordinate
     * @param y The y-coordinate
     * @return The distance in meters, or 0 if (x,y) is out of bounds
     */
    O3P_API float getDistance(uint16_t x, uint16_t y) const {
        if (y * m_width + x < m_width * m_height) {
            return m_data[y * m_width + x] / 1000.0f;
        }

        return 0.0f;
    }

    uint16_t m_width;  /**< The width of the frame */
    uint16_t m_height; /**< The height of the frame */

    uint16_t *m_data;                 /**< Pointer to the frame data */
    bool m_isCopy;                    /**< Flag indicating if the frame data is a copy */
    std::vector<uint16_t> m_dataCopy; /**< Copy of the frame data */
};

/**
 * @brief Represents pointcloud frames
 *
 * Extends the Frames class for pointcloud frames.
 * Each point is represented by 4 floats (x, y, z, confidence)
 * The cartesian coordinates are given in meters.
 */
class PointcloudFrame : public Frame {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API PointcloudFrame() : m_width(0u),
                                m_height(0u),
                                m_data(nullptr),
                                m_isCopy(false) {
    }

    /**
     * @brief Copy constructor
     *
     * @param other The PointcloudFrame object to copy from
     */
    O3P_API PointcloudFrame(const PointcloudFrame &other) : Frame(other),
                                                            m_width(other.m_width),
                                                            m_height(other.m_height),
                                                            m_isCopy(true) {
        m_dataCopy.resize(m_width * m_height * 4);
        memcpy(m_dataCopy.data(), other.m_data, m_width * m_height * 4 * sizeof(float));
        m_data = m_dataCopy.data();
    }

    /**
     * @brief Assignment operator
     *
     * @param other The PointcloudFrame object to assign from
     * @return Reference to the assigned PointcloudFrame object
     */
    O3P_API PointcloudFrame &operator=(const PointcloudFrame &other) {
        if (&other != this) {
            Frame::operator=(other);
            m_width = other.m_width;
            m_height = other.m_height;
            m_isCopy = true;
            m_dataCopy.resize(m_width * m_height * 4);
            memcpy(m_dataCopy.data(), other.m_data, m_width * m_height * 4 * sizeof(float));
            m_data = m_dataCopy.data();
        }

        return *this;
    }

    /**
     * @brief Get the width
     *
     * @return The width
     */
    O3P_API uint16_t getWidth() const {
        return m_width;
    }

    /**
     * @brief Get the height
     *
     * @return The height
     */
    O3P_API uint16_t getHeight() const {
        return m_height;
    }

    /**
     * @brief Get the cartesian x value for pixel (x, y)
     *
     * @param x The x-coordinate
     * @param y The y-coordinate
     * @return The cartesian x value
     */
    O3P_API float getX(uint16_t x, uint16_t y) const {
        return m_data[(y * m_width + x) * 4 + 0];
    }

    /**
     * @brief Get the cartesian y value for pixel (x, y)
     *
     * @param x The x-coordinate
     * @param y The y-coordinate
     * @return The cartesian y value
     */
    O3P_API float getY(uint16_t x, uint16_t y) const {
        return m_data[(y * m_width + x) * 4 + 1];
    }

    /**
     * @brief Get the cartesian z value for pixel (x, y)
     *
     * @param x The x-coordinate
     * @param y The y-coordinate
     * @return The cartesian z value
     */
    O3P_API float getZ(uint16_t x, uint16_t y) const {
        return m_data[(y * m_width + x) * 4 + 2];
    }

    /**
     * @brief Get the confidence for pixel (x, y)
     *
     * @param x The x-coordinate
     * @param y The y-coordinate
     * @return The confidence
     */
    O3P_API float getConfidence(uint16_t x, uint16_t y) const {
        return m_data[(y * m_width + x) * 4 + 3];
    }

    uint16_t m_width;  /**< The width of the frame */
    uint16_t m_height; /**< The height of the frame */

    float *m_data;                 /**< Pointer to the frame data */
    bool m_isCopy;                 /**< Flag indicating if the frame data is a copy */
    std::vector<float> m_dataCopy; /**< Copy of the frame data */
};

/**
 * @brief Represents imu frames
 *
 * Extends the Frames class for imu frames
 */
class ImuFrame : public Frame {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API ImuFrame() : angularVelocity{0.0f, 0.0f, 0.0f},
                         angularTimestamp(0),
                         acceleration{0.0f, 0.0f, 0.0f},
                         accelerationTimestamp(0) {
    }

    float angularVelocity[3];       /**< The angular velocity */
    uint64_t angularTimestamp;      /**< The timestamp for the angular velocity */
    float acceleration[3];          /**< The acceleration */
    uint64_t accelerationTimestamp; /**< The timestamp for the acceleration */
};

/**
 * @brief Represents ultrasonic frames
 *
 * Extends the Frames class for ultrasonic frames
 */
class UltrasonicFrame : public Frame {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API UltrasonicFrame() : m_data{0, 0, 0, 0, 0.0f} {
    }

    /**
     * @brief Get the distance in meters
     *
     * @return The distance in meters
     */
    O3P_API float getDistance() const { return m_data.distance / 1000.0f; }

    /**
     * @brief Get the timestamp
     *
     * @return The timestamp
     */
    O3P_API uint64_t getTimestamp() const override { return m_data.timestamp; }

    /**
     * @brief Get the signal width
     *
     * @return The signal width
     */
    O3P_API uint8_t getSignalWidth() const { return m_data.width; }

    /**
     * @brief Get the signal amplitude
     *
     * @return The signal amplitude
     */
    O3P_API uint8_t getAmplitude() const { return m_data.amplitude; }

    /**
     * @brief Get the temperature
     *
     * @return The temperature
     */
    O3P_API float getTemperature() const { return m_data.temperature; }

    struct UltrasonicData {
        uint16_t distance;  /**< The distance measured by the ultrasonic sensor in millimeters */
        uint64_t timestamp; /**< The timestamp for the ultrasonic frame */
        uint8_t width;      /**< The signal width of the ultrasonic echo */
        uint8_t amplitude;  /**< The signal amplitude of the ultrasonic echo */
        float temperature;  /**< The temperature of the ultrasonic frame */
    } m_data;
};

/**
 * @brief Represents plugin frames
 *
 * Extends the Frames class for plugin frames
 */
class PluginFrame : public Frame {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API PluginFrame() : m_size(0),
                            m_data(nullptr) {
    }

    /**
     * @brief Get the data
     *
     * @return Pointer to the data
     */
    O3P_API uint8_t *getData() const {
        return m_data;
    }

    /**
     * @brief Get the size
     *
     * @return The size
     */
    O3P_API uint16_t getSize() const {
        return m_size;
    }

    uint16_t m_size; /**< The size of the data */
    uint8_t *m_data; /**< Pointer to the data */
};

/**
 * @brief Defines the frame set of the device
 */
class FrameSet {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API FrameSet() {
    }

    O3P_API FrameSet(const FrameSet &other) = default;

    O3P_API FrameSet &operator=(const FrameSet &other);

    /**
     * @brief Get the Color frame
     *
     * @return Pointer to the Color frame
     */
    O3P_API VideoFrame *getColorFrame() {
        return &m_colorFrame;
    }

    /**
     * @brief Get the Infrared frame
     *
     * @return Pointer to the Infrared frame
     */
    O3P_API VideoFrame *getInfraredFrame() {
        return &m_infraredFrame;
    }

    /**
     * @brief Get the depth frame
     *
     * @return Pointer to the depth frame
     */
    O3P_API DepthFrame *getDepthFrame() {
        return &m_depthFrame;
    }

    /**
     * @brief Get the imu frame
     *
     * @return Pointer to the imu frame
     */
    O3P_API ImuFrame *getImuFrame() {
        return &m_imuFrame;
    }

    /**
     * @brief Get the ultrasonic frame
     *
     * @return Pointer to the ultrasonic frame
     */
    O3P_API UltrasonicFrame *getUltrasonicFrame() {
        return &m_ultrasonicFrame;
    }

    /**
     * @brief Get the point cloud frame
     *
     * @return Pointer to the point cloud frame
     */
    O3P_API PointcloudFrame *getPointCloudFrame() {
        return &m_pointcloudFrame;
    }

    /**
     * @brief Get the plugin frame
     *
     * @return Pointer to the plugin frame
     */
    O3P_API PluginFrame *getPluginFrame() {
        return &m_pluginFrame;
    }

    VideoFrame m_colorFrame;           /**< The Color frame */
    VideoFrame m_infraredFrame;        /**< The Infrared frame */
    DepthFrame m_depthFrame;           /**< The depth frame */
    ImuFrame m_imuFrame;               /**< The imu frame */
    PointcloudFrame m_pointcloudFrame; /**< The point cloud frame */
    UltrasonicFrame m_ultrasonicFrame; /**< The ultrasonic frame */
    PluginFrame m_pluginFrame;         /**< The plugin frame */
};

} // namespace o3p

#endif
