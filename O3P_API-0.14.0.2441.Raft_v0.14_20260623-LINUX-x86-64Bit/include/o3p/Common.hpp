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

#ifndef COMMON_HPP
#define COMMON_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Definitions.hpp"
#include "LensModelType.hpp"

namespace o3p {

#pragma pack(push, 1)
/**
 * @brief Types of metadata that can be attached to a frame
 */
typedef enum MetadataType {
    O3P_METADATA_TIMESTAMP,
    O3P_METADATA_FRAME_NUMBER,
    O3P_METADATA_EXPOSURE_TIME_1,
    O3P_METADATA_EXPOSURE_TIME_2,
    O3P_METADATA_TEMPERATURE,
    O3P_METADATA_ANALOG_GAIN,
    O3P_METADATA_WHITE_BALANCE
} MetadataType;

/**
 * @brief Describes a single metadata entry
 */
struct MetadataInfo {
    MetadataType type; /**< Type of the metadata */
    size_t size;       /**< Size of the metadata value in bytes */
    std::string name;  /**< Human readable name of the metadata */
};

// Declare all metadata
static const MetadataInfo METADATA_INFO[] = {
    {O3P_METADATA_TIMESTAMP, sizeof(uint64_t), "timestamp"},
    {O3P_METADATA_FRAME_NUMBER, sizeof(uint32_t), "frame number"},
    {O3P_METADATA_EXPOSURE_TIME_1, sizeof(uint32_t), "exposure time 1"},
    {O3P_METADATA_EXPOSURE_TIME_2, sizeof(uint32_t), "exposure time 2"},
    {O3P_METADATA_TEMPERATURE, sizeof(float), "temperature"},
    {O3P_METADATA_ANALOG_GAIN, sizeof(float), "analog gain"},
    {O3P_METADATA_WHITE_BALANCE, sizeof(uint32_t), "white balance"},
};

/**
 * @brief Describes the location of a metadata value within a metadata buffer
 */
struct MetadataField {
    MetadataType type; /**< Type of the metadata */
    size_t offset;     /**< Offset of the value inside the buffer */
};

/**
 * @brief Write a metadata value into a buffer at the given offset
 *
 * @tparam T Type of the metadata value
 * @param data Pointer to the metadata buffer
 * @param type Type of the metadata to write
 * @param offset Offset within the buffer
 * @param value The value to write
 */
template <typename T>
void setMetadata(uint8_t *data, MetadataType type, size_t offset, const T &value) {
    MetadataInfo info = METADATA_INFO[type];
    if (data && sizeof(T) == info.size) {
        std::memcpy(data + offset, &value, sizeof(T));
    }
}

/**
 * @brief Read a metadata value from a buffer at the given offset
 *
 * @tparam T Type of the metadata value
 * @param data Pointer to the metadata buffer
 * @param type Type of the metadata to read
 * @param offset Offset within the buffer
 * @return The metadata value, or a default-constructed T if data is invalid
 */
template <typename T>
T getMetadata(const uint8_t *data, MetadataType type, size_t offset) {
    MetadataInfo info = METADATA_INFO[type];
    T value{};
    if (data && sizeof(T) == info.size) {
        std::memcpy(&value, data + offset, sizeof(T));
    }
    return value;
}

/**
 * @brief Generate a metadata layout by parsing a metadata buffer
 *
 * @param buffer Pointer to the metadata buffer
 * @param len Number of bytes available in the buffer
 * @return Map from metadata type to its offset within the buffer; empty on parse error
 */
inline std::unordered_map<MetadataType, size_t> getMetadataLayoutFromBuffer(const uint8_t *buffer, size_t len) {
    std::unordered_map<MetadataType, size_t> layout;
    size_t fieldOffset = 0;

    while (len) {
        if (sizeof(MetadataField) >= len) {
            // Error - buffer should never end with field, only values
            layout.clear();
            break;
        }

        MetadataField field;
        memcpy(&field, buffer + fieldOffset, sizeof(MetadataField));
        fieldOffset += sizeof(MetadataField);
        len -= sizeof(MetadataField);

        if (field.type >= sizeof(METADATA_INFO) || field.offset == 0 || METADATA_INFO[field.type].size > len) {
            // Error - Invalid field (invalid type, invalid offset)
            layout.clear();
            break;
        }

        layout[field.type] = field.offset;
        len -= METADATA_INFO[field.type].size;
    }

    return layout;
}

// Generate a metadata layout from a list of metadata types
/**
 * @brief Generate a metadata layout from a list of metadata types
 *
 * @param types The metadata types to include in the layout
 * @return Map from metadata type to its offset within the buffer
 */
inline std::unordered_map<MetadataType, size_t> getMetadataLayout(const std::vector<MetadataType> &types) {
    std::unordered_map<MetadataType, size_t> layout;
    size_t metaValueOffset = types.size() * sizeof(o3p::MetadataField);

    for (auto &type : types) {
        layout[type] = metaValueOffset;
        metaValueOffset += o3p::METADATA_INFO[type].size;
    }

    return layout;
}

// Populates the metadata fields of a buffer but not the values
/**
 * @brief Write the metadata field descriptors to a buffer (without the values)
 *
 * @param buffer Destination buffer
 * @param layout Layout map describing the metadata fields
 */
inline void applyMetadataLayoutToBuffer(uint8_t *buffer, std::unordered_map<MetadataType, size_t> &layout) {
    size_t offset = 0;
    for (const auto &entry : layout) {
        MetadataField field;
        field.type = entry.first;
        field.offset = entry.second;
        memcpy(buffer + offset, &field, sizeof(o3p::MetadataField));
        offset += sizeof(o3p::MetadataField);
    }
}

// Calculate the expected size of a buffer supporting a set of metadata fields and values
/**
 * @brief Calculate the buffer size needed to hold a set of metadata fields and their values
 *
 * @param metadataTypes The metadata types to include
 * @return Required buffer size in bytes
 */
inline size_t calculateMetadataBufferSize(const std::vector<o3p::MetadataType> &metadataTypes) {
    size_t size = 0;
    for (const auto &type : metadataTypes) {
        auto metadataInfo = o3p::METADATA_INFO[type];
        size += sizeof(o3p::MetadataField) + metadataInfo.size;
    }
    return size;
}

#pragma pack(pop)

/**
 * @brief Enum for different Stream types
 */
typedef enum StreamType {
    O3P_STREAM_ANY,
    O3P_STREAM_DEPTH,
    O3P_STREAM_COLOR,
    O3P_STREAM_INFRARED,
    O3P_STREAM_IMU,
    O3P_STREAM_PLUGIN,
    O3P_STREAM_POSE,
    O3P_STREAM_CONFIDENCE,
    O3P_STREAM_ULTRASONIC,
    O3P_STREAM_COUNT
} StreamType;

/**
 * @brief Converts the given stream type to std::string
 *
 * @param type The stream type that should be converted
 */
inline std::string streamType2String(StreamType type) {
    switch (type) {
    case O3P_STREAM_ANY:
        return "O3P_STREAM_ANY";
    case O3P_STREAM_DEPTH:
        return "O3P_STREAM_DEPTH";
    case O3P_STREAM_COLOR:
        return "O3P_STREAM_COLOR";
    case O3P_STREAM_INFRARED:
        return "O3P_STREAM_INFRARED";
    case O3P_STREAM_IMU:
        return "O3P_STREAM_IMU";
    case O3P_STREAM_PLUGIN:
        return "O3P_STREAM_PLUGIN";
    case O3P_STREAM_POSE:
        return "O3P_STREAM_POSE";
    case O3P_STREAM_CONFIDENCE:
        return "O3P_STREAM_CONFIDENCE";
    case O3P_STREAM_ULTRASONIC:
        return "O3P_STREAM_ULTRASONIC";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Defines the Intrinsic parameters
 */
typedef struct Intrinsics {
    float ppx;           /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    float ppy;           /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    float fx;            /**< Focal length of the image plane, as a multiple of pixel width */
    float fy;            /**< Focal length of the image plane, as a multiple of pixel height */
    LensModelType model; /**< Distortion model of the image */
    float coeffs[5];     /**< Distortion coefficients. Order for Brown-Conrady: [k1, k2, p1, p2, k3]. Order for F-Theta Fish-eye: [k1, k2, k3, k4, 0]. Other models are subject to their own interpretations */
} Intrinsics;

/**
 * @brief Defines the Extrinsic parameters
 */
typedef struct Extrinsics {
    Extrinsics() : rotation{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}, translation{0.0f, 0.0f, 0.0f} {}
    float rotation[9];    //!< 3x3 rotation matrix in column-major order
    float translation[3]; //!< Translation vector (x, y, z) in meters
} Extrinsics;

#pragma pack(push, 1)
#define MAX_COMPOSITE_FRAME_BUFFERS 6
/**
 * @brief Describes a single buffer within a composite frame
 */
struct CompositeBufferConfig {
    uint8_t type;          /**< Buffer/stream type identifier */
    uint32_t byteOffset;   /**< Offset of the buffer within the composite frame */
    uint32_t byteSize;     /**< Size of the buffer in bytes */
    uint32_t metadataSize; /**< Size of the metadata block in bytes */
    uint16_t width;        /**< Width of the image in pixels */
    uint16_t height;       /**< Height of the image in pixels */
};
/**
 * @brief Describes the layout of a composite frame containing multiple buffers
 */
struct CompositeFrameConfig {
    uint8_t numBuffers;                                               /**< Number of valid buffer entries */
    CompositeBufferConfig bufferConfigs[MAX_COMPOSITE_FRAME_BUFFERS]; /**< Per-buffer configuration */
};
#pragma pack(pop)

/**
 * @brief Struct that defines the GUID
 */
struct Guid {
    uint32_t data1;
    uint16_t data2, data3;
    uint8_t data4[8];
};

/**
 * @brief Enum that defines the Extension Data ID
 */
enum ExtensionDataId {
    LENS_PARAMETERS = 1,
    MESSAGE_PACKET = 2
};

using DisconnectCallback = std::function<void()>;

/**
 * @brief Exception thrown when communication fails because the USB device disconnected
 */
class UsbDisconnectedException : public std::runtime_error {
  public:
    explicit UsbDisconnectedException(const std::string &message)
        : std::runtime_error("USB Disconnected: " + message) {}
};

/**
 * @brief Warning thrown when the end of a bag file is reached during playback
 */
class BagFileEndReachedWarning : public std::runtime_error {
  public:
    explicit BagFileEndReachedWarning(const std::string &message)
        : std::runtime_error("End of bag file reached: " + message) {}
};

#pragma pack(push, 1)
/**
 * @brief Version 1 of the device information block
 */
struct DeviceInfoV1 {
    char serialNumber[16];
    char cpuId[32];
    char cpuCode[4];
    char cpuVersion;
};
#pragma pack(pop)

#pragma pack(push, 1)
/**
 * @brief Version 2 of the device information block, including firmware version details
 */
struct DeviceInfoV2 {
    char serialNumber[17]; // null terminated serial number string (16 characters + null terminator)
    char cpuId[33];        // null terminated CPU ID string (32 characters + null terminator)
    char cpuCode[5];       // null terminated CPU code string (4 characters + null terminator)
    char cpuVersion;
    char fwVersion[33];          // null terminated version number (e.g. major.minor.patch) up to 32 characters max.
    char fwVersionBuild[33];     // null terminated version build id (e.g. number) up to 32 characters max.
    char fwVersionDate[33];      // null terminated date string (e.g. YYYY-MM-DD) up to 32 characters max.
    char fwVersionHash[33];      // null terminated hash string (e.g. git commit hash) up to 32 characters max.
    char fwVersionCustomer[129]; // null terminated customer string up to 128 characters max.
};
#pragma pack(pop)

#pragma pack(push, 1)
/**
 * @brief Version 3 of the device information block, adding ultrasonic sensor and ToF serial fields
 */
struct DeviceInfoV3 {
    char serialNumber[17]; // null terminated serial number string (16 characters + null terminator)
    char cpuId[33];        // null terminated CPU ID string (32 characters + null terminator)
    char cpuCode[5];       // null terminated CPU code string (4 characters + null terminator)
    char cpuVersion;
    char fwVersion[33];          // null terminated version number (e.g. major.minor.patch) up to 32 characters max.
    char fwVersionBuild[33];     // null terminated version build id (e.g. number) up to 32 characters max.
    char fwVersionDate[33];      // null terminated date string (e.g. YYYY-MM-DD) up to 32 characters max.
    char fwVersionHash[33];      // null terminated hash string (e.g. git commit hash) up to 32 characters max.
    char fwVersionCustomer[129]; // null terminated customer string up to 128 characters max.
    char hasUltrasonicSensor;    // '1' if ultrasonic sensor is available, '0' otherwise
    char tofSensorSerial[20];    // null terminated ToF sensor serial number string (19 characters + null terminator)
};
#pragma pack(pop)

#pragma pack(push, 1)
/**
 * @brief Version 1 of the lens information block (ToF and RGB intrinsics, plus extrinsics)
 */
struct LensInfoV1 {
    float intrinsicsTof[9];
    uint8_t lensModelToF;
    float intrinsicsRgb[9];
    uint8_t lensModelRgb;
    float extrinsicsRgbToTof[7];
};
#pragma pack(pop)

/**
 * @brief Identifiers for the different camera information fields
 */
typedef enum CameraInfo {
    O3P_CAMERA_INFO_SERIAL_NUMBER,
    O3P_CAMERA_INFO_CPU_ID,
    O3P_CAMERA_INFO_CPU_CODE,
    O3P_CAMERA_INFO_CPU_VERSION,
    O3P_CAMERA_INFO_FW_VERSION,
    O3P_CAMERA_INFO_FW_VERSION_BUILD,
    O3P_CAMERA_INFO_FW_VERSION_DATE,
    O3P_CAMERA_INFO_FW_VERSION_HASH,
    O3P_CAMERA_INFO_FW_VERSION_CUSTOMER,
    O3P_CAMERA_INFO_HAS_ULTRASONIC_SENSOR,
    O3P_CAMERA_INFO_TOF_SENSOR_SERIAL,
    O3P_CAMERA_INFO_COUNT
} CameraInfo;

/**
 * @brief Convert a CameraInfo enum value to a human readable string
 *
 * @param info The camera info to convert
 * @return The human readable name
 */
inline std::string cameraInfoToString(const CameraInfo &info) {
    switch (info) {
    case O3P_CAMERA_INFO_SERIAL_NUMBER:
        return "Serial Number";
    case O3P_CAMERA_INFO_CPU_ID:
        return "CPU ID";
    case O3P_CAMERA_INFO_CPU_CODE:
        return "CPU Code";
    case O3P_CAMERA_INFO_CPU_VERSION:
        return "CPU Version";
    case O3P_CAMERA_INFO_FW_VERSION:
        return "Firmware Version";
    case O3P_CAMERA_INFO_FW_VERSION_BUILD:
        return "Firmware Version Build";
    case O3P_CAMERA_INFO_FW_VERSION_DATE:
        return "Firmware Version Date";
    case O3P_CAMERA_INFO_FW_VERSION_HASH:
        return "Firmware Version Hash";
    case O3P_CAMERA_INFO_FW_VERSION_CUSTOMER:
        return "Firmware Version Customer";
    case O3P_CAMERA_INFO_HAS_ULTRASONIC_SENSOR:
        return "Ultrasonic Sensor Available";
    case O3P_CAMERA_INFO_TOF_SENSOR_SERIAL:
        return "ToF Sensor Serial Number";
    default:
        return "Unknown Camera Info";
    }
}

/**
 * @brief Retrieves the size of a given file.
 * @param filename Filename of the file.
 * @return Size of the file.
 */
inline uint64_t getFileSize(const std::string &filename) {
    std::ifstream inputfile;
    inputfile.open(filename.c_str(), std::ios_base::in | std::ios_base::binary);
    if (!inputfile.is_open()) {
        return 0;
    }

    // get length of file:
    inputfile.seekg(0, std::ios::end);
    uint64_t length = static_cast<uint64_t>(inputfile.tellg());
    inputfile.seekg(0, std::ios::beg);

    inputfile.close();

    return length;
}

/**
 * @brief Reads a file into a std::vector.
 * @tparam T Element type of the resulting vector
 * @param filename Filename of the file.
 * @return vector containing the file data.
 */

template <class T>
inline std::vector<T> readFileToStdVector(const std::string &filename) {
    std::vector<T> data;

    uint64_t length = getFileSize(filename);

    if (!length) {
        throw std::runtime_error("Failed to get file size or file is empty.");
    }

    std::ifstream f(filename.c_str(), std::ios_base::in | std::ios_base::binary);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    if (length % sizeof(T) != 0) {
        throw std::runtime_error("File size is not a multiple of container value type size.");
    }

    data.resize(static_cast<size_t>(length) / sizeof(T));
    f.read((char *)&data[0], static_cast<std::streamsize>(length));
    f.close();

    return data;
}

} // namespace o3p

#endif