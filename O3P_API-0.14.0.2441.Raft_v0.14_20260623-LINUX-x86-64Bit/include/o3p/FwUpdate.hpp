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

#ifndef O3P_FW_UPDATE_HPP
#define O3P_FW_UPDATE_HPP

#include <cstdint>

#define O3P_MAX_UPDATE_MESSAGE_SIZE 1024

/**
 * @brief Status of a firmware update operation
 */
enum class UpdateStatus {
    IDLE,
    DOWNLOADING,
    INSTALLING,
    COMPLETED,
    FAILED
};

/**
 * @brief Container for a firmware update message exchanged with the device
 */
struct UpdateMessage {
    /**
     * @brief Identifies the kind of UpdateMessage
     */
    enum type {
        START_UPDATE_CMD,
        CANCEL_UPDATE_CMD,
        STATUS_GET_CMD,
        STATUS_NOTIFICATION,
        PROGRESS_GET_CMD,
        PROGRESS_NOTIFICATION,
        ERROR_NOTIFICATION,
        FILE_CHUNK,
    };

    /**
     * @brief Header of an UpdateMessage
     */
    struct Header {
        uint32_t transactionId;     /**< Identifier of the update transaction */
        UpdateMessage::type msgType;/**< Kind of message */
        uint32_t payloadSize;       /**< Number of valid payload bytes */
    } header;

    uint8_t payload[O3P_MAX_UPDATE_MESSAGE_SIZE - sizeof(header)]; /**< Message payload */
};

/**
 * @brief Payload of a START_UPDATE_CMD message
 */
struct StartUpdateCommand {
    uint64_t hostDateTime;   /**< Current host time (epoch seconds) */
    uint64_t updateFileSize; /**< Total size of the update file in bytes */
};

/**
 * @brief Payload of a CANCEL_UPDATE_CMD message
 */
struct CancelUpdateCommand {
    uint64_t hostDateTime; /**< Current host time (epoch seconds) */
};

/**
 * @brief Payload of a STATUS_GET_CMD message
 */
struct StatusGetCommand {
    // No payload
};

/**
 * @brief Payload of a PROGRESS_GET_CMD message
 */
struct ProgressGetCommand {
    // No payload
};

/**
 * @brief Payload of a STATUS_NOTIFICATION message
 */
struct StatusNotification {
    UpdateStatus status; /**< Current update status */
};

/**
 * @brief Payload of a PROGRESS_NOTIFICATION message
 */
struct ProgressNotification {
    uint32_t progress; /**< Update progress in percent (0-100) */
};

/**
 * @brief Payload of an ERROR_NOTIFICATION message
 */
struct ErrorNotification {
    char errorMessage[256]; /**< Null terminated error description */
};

/**
 * @brief Payload of a FILE_CHUNK message containing a portion of the update file
 */
struct FileChunk {
    /**
     * @brief Header of a FileChunk message
     */
    struct Header {
        uint32_t chunkIndex;  /**< Index of this chunk */
        uint32_t totalChunks; /**< Total number of chunks */
        uint32_t chunkSize;   /**< Number of valid bytes in this chunk */
    } header;
    uint8_t data[sizeof(UpdateMessage::payload) - sizeof(Header)]; /**< Chunk data */
};

#endif /* O3P_FW_UPDATE_HPP */
