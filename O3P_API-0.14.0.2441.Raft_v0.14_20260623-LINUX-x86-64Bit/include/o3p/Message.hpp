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

#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <cstdint>
#include <vector>

#include "Definitions.hpp"

namespace o3p {
namespace message {

/**
 * @brief Identifier for the type of a Message
 */
typedef enum MessageId : uint8_t {
    INVALID,             /**< Invalid / uninitialized message */
    SUCCESS,             /**< Acknowledgement of a successful operation */
    CONFIG_GET,          /**< Request or response to read a configuration value */
    CONFIG_SET,          /**< Request or response to write a configuration value */
    PLUGIN,              /**< Plugin specific message */
    EEPROM_GET,          /**< Request or response to read EEPROM data */
    EEPROM_SET,          /**< Request or response to write EEPROM data */
    FIRMWARE_UPDATE,     /**< Firmware update message */
    DEVICE_INFO_GET,     /**< Request or response for device information */
    OPTION_CMD,          /**< Option command message */
    DEVICE_LENSINFO_GET, /**< Request or response for device lens information */
    LICENSES_GET,        /**< Request or response for license information */
    CONFIG_SAVE,         /**< Request or response to save configuration */
    MAX                  /**< Sentinel value, must be the last entry */
} MessageId;

#pragma pack(push, 1)
/** @brief Maximum total size of a single MessagePacket in bytes */
constexpr std::size_t MAX_PACKET_SIZE = 60;
/**
 * @brief A single fixed-size packet that is part of a fragmented Message
 */
struct MessagePacket {
    /**
     * @brief Header of a MessagePacket
     */
    struct Header {
        MessageId id;          /**< Type of the message this packet belongs to */
        uint32_t sequence;     /**< Zero-based index of this packet within the message */
        uint32_t totalPackets; /**< Total number of packets making up the message */
        uint8_t payloadLength; /**< Number of valid payload bytes in this packet */
    } header;
    uint8_t payload[MAX_PACKET_SIZE - sizeof(Header)]; /**< Payload bytes */
};
#pragma pack(pop)

/** @brief Maximum number of payload bytes that fit into a single MessagePacket */
constexpr std::size_t MAX_PAYLOAD_PER_PACKET = MAX_PACKET_SIZE - sizeof(MessagePacket::Header);
/** @brief Maximum total payload size of a Message that can be transmitted via MessagePackets */
constexpr std::size_t MAX_MESSAGE_DATA_SIZE = static_cast<std::size_t>(UINT32_MAX) * MAX_PAYLOAD_PER_PACKET;

/**
 * @brief A logical message consisting of a type identifier and a payload
 */
struct Message {
    MessageId id;              /**< Type of the message */
    std::vector<uint8_t> data; /**< Message payload */
};

/**
 * @brief Split a Message into a sequence of MessagePackets
 *
 * @param msg The message to fragment
 * @return The packets that together represent the message
 */
O3P_API std::vector<MessagePacket> MessageToPackets(const Message &msg);

/**
 * @brief Reassemble a Message from a sequence of MessagePackets
 *
 * @param packets The packets to assemble
 * @return The reconstructed message
 */
O3P_API Message PacketsToMessage(const std::vector<MessagePacket> &packets);

/**
 * @brief Helper to incrementally collect MessagePackets and assemble Messages
 */
class MessagePacketCollector {
  public:
    /**
     * @brief Construct an empty MessagePacketCollector
     */
    MessagePacketCollector() = default;
    /**
     * @brief Add a newly received packet to the collector
     *
     * @param newPacket Packet to add
     * @return true if the packet was accepted, false otherwise
     */
    O3P_API bool handlePacket(const MessagePacket *newPacket);

    /**
     * @brief Check whether a complete Message can be assembled
     *
     * @return true if all packets of the current message have been received
     */
    O3P_API bool messageAvailable();

    /**
     * @brief Assemble the collected packets into a Message
     *
     * @return The assembled Message
     */
    O3P_API Message assemble();

    /**
     * @brief Discard all currently collected packets
     */
    O3P_API void reset();

  private:
    std::vector<MessagePacket> payload;
};

} // namespace message
} // namespace o3p

#endif /* MESSAGE_HPP */
