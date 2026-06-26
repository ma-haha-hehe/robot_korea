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

#ifndef PIPELINE_CONFIG_HPP
#define PIPELINE_CONFIG_HPP

#include <string>

#include <o3p/Common.hpp>

namespace o3p {

/**
 * @brief Defines the pipeline configuration
 */
class PipelineConfig {

  public:
    /**
     * @brief Basic constructor
     */
    O3P_API PipelineConfig() : m_playbackFile(""),
                               m_id(""),
                               m_disconnectCallback(nullptr),
                               m_loopPlayback(true),
                               m_useTimestampsPlayback(true) {
    }

    /**
     * @brief Constructor that initializes the playback file
     *
     * @param playbackFile The playback file that will be used for initialization
     * @param loopPlayback Whether to loop the playback
     * @param useTimestampsPlayback Whether to delay the delivery of new frames based on the timestamps
     */
    O3P_API PipelineConfig(const std::string &playbackFile, bool loopPlayback, bool useTimestampsPlayback) : m_playbackFile(playbackFile), m_loopPlayback(loopPlayback), m_useTimestampsPlayback(useTimestampsPlayback) {
    }

    /**
     * @brief Sets the playback file that will be used
     *
     * @param playbackFile The playback file
     */
    O3P_API void setPlaybackFile(const std::string &playbackFile) {
        m_playbackFile = playbackFile;
    }

    /**
     * @brief Get the playback file
     *
     * @return The playback file
     */
    O3P_API const std::string &getPlaybackFile() const {
        return m_playbackFile;
    }

    /**
     * @brief Sets whether to loop the playback
     *
     * @param loopPlayback True to loop the playback, false otherwise
     */
    O3P_API void setLoopPlayback(bool loopPlayback) {
        m_loopPlayback = loopPlayback;
    }

    /**
     * @brief Retrieves whether to loop the playback
     *
     * @return True if the playback will be looped, false otherwise
     */
    O3P_API bool getLoopPlayback() {
        return m_loopPlayback;
    }

    /**
     * @brief Sets whether to delay the delivery of new frames based on the timestamps
     *
     * @param useTimestampsPlayback True to use timestamps for playback, false otherwise
     */
    O3P_API void setUseTimestampsPlayback(bool useTimestampsPlayback) {
        m_useTimestampsPlayback = useTimestampsPlayback;
    }

    /**
     * @brief Retrieves whether to delay the delivery of new frames based on the timestamps
     *
     * @return True if timestamps will be used for playback, false otherwise
     */
    O3P_API bool getUseTimestampsPlayback() {
        return m_useTimestampsPlayback;
    }

    /**
     * @brief Sets the serial number of the device to be used
     *
     * @param id The serial of the device
     */
    O3P_API void enableDevice(const std::string &id) {
        m_id = id;
    }

    /**
     * @brief Retrieves the serial number of the device to be used
     *
     * @return The serial of the device
     */
    O3P_API const std::string &getDeviceId() const {
        return m_id;
    }

    /**
     * @brief Sets the function that will be called when the device is disconnected
     *
     * @param callback The callback that will be called
     */
    O3P_API void setDisconnectCallback(DisconnectCallback callback) {
        m_disconnectCallback = std::move(callback);
    }

    /**
     * @brief Retrieves the callback that will be called when the device is disconnected
     *
     * @return The callback function
     */
    O3P_API DisconnectCallback getDisconnectCallback() {
        return m_disconnectCallback;
    }

    /**
     * @brief Comparison operator
     *
     * @param other The other PipelineConfig to compare with
     * @return True if the configs are the same, false otherwise
     */
    bool operator==(const PipelineConfig &other) const {
        return (m_playbackFile == other.m_playbackFile &&
                m_id == other.m_id);
    }

    /**
     * @brief Comparison operator !=
     *
     * @param other The other PipelineConfig to compare with
     * @return True if the configs are different, false otherwise
     */
    bool operator!=(const PipelineConfig &other) const {
        return !(*this == other);
    }

  private:
    std::string m_playbackFile;
    std::string m_id;
    DisconnectCallback m_disconnectCallback;
    bool m_loopPlayback;
    bool m_useTimestampsPlayback;
};

} // namespace o3p

#endif
