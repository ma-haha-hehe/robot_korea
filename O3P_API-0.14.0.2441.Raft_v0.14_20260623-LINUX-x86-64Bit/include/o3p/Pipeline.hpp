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

#ifndef PIPELINE_HPP
#define PIPELINE_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <o3p/Device.hpp>
#include <o3p/PipelineConfig.hpp>
#include <o3p/PipelineProfile.hpp>
#include <o3p/Preset.hpp>

namespace o3p {

class Recording;

/**
 * @brief Defines the pipeline
 */
class Pipeline {

  public:
    /**
     * @brief Basic constructor
     */
    O3P_API Pipeline();

    /**
     * @brief Basic destructor
     */
    O3P_API virtual ~Pipeline();

    /**
     * @brief Initializes the Pipeline with the default configuration
     *
     * @throw UsbDisconnectedException if the USB device is disconnected
     * @throw std::runtime_error on device initialization failure
     *
     * @return The pipeline's profile
     */
    O3P_API PipelineProfile init();

    /**
     * @brief Initializes the Pipeline based on a given configuration
     *
     * @throw std::runtime_error if the specified playback file does not exist
     * @throw UsbDisconnectedException if the USB device is disconnected
     * @throw std::runtime_error on device initialization failure
     *
     * @param config The configuration that should be used
     *
     * @return The pipeline's profile
     */
    O3P_API PipelineProfile init(PipelineConfig config);

    /**
     * @brief Starts the pipeline
     *
     * @throw UsbDisconnectedException if the USB device is disconnected
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void start();

    /**
     * @brief Stops the pipeline
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void stop();

    /**
     * @brief Collect all the frames
     *
     * @throw UsbDisconnectedException if the USB device is disconnected
     * @throw std::runtime_error if the pipeline is not initialized or no frames are received
     *
     * @return Set of all frames
     */
    O3P_API const FrameSet &waitForFrames();

    /**
     * @brief Send a message to a plugin. Pipeline must be started first.
     *
     * @throw std::runtime_error if the pipeline is not initialized
     *
     * @param message Data that is processed by a plugin
     *
     * @return Data reply from plugin
     */
    O3P_API std::vector<uint8_t> sendPluginMessage(const std::vector<uint8_t> &message);

    /**
     * @brief Get a firmware configuration value
     *
     * @throw std::runtime_error if the pipeline is not initialized
     *
     * @return String representation of config value. Empty string if config does not exist.
     */
    O3P_API std::string getConfigValue(const std::string &config);

    /**
     * @brief Set a firmware configuration value
     *
     * @throw std::runtime_error if the pipeline is not initialized or config is rejected by firmware
     *
     * @param config The name of the config value
     * @param value The value to set
     */
    O3P_API void setConfigValue(const std::string &config, const std::string &value);

    /**
     * @brief Writes values to the EEPROM of the camera.
     *
     * @throw std::runtime_error if pipeline is not initialized
     *
     * @param value The value to set
     */
    O3P_API virtual void writeEeprom(const std::vector<uint8_t> &value);

    /**
     * @brief Reads the EEPROM of the camera.
     *
     * @throw std::runtime_error if the pipeline is not initialized
     *
     * @return Byte vector representation of the EEPROM data.
     */
    O3P_API virtual std::vector<uint8_t> readEeprom();

    /**
     * @brief Reads the license information from the camera.
     *
     * @throw std::runtime_error if the pipeline is not initialized
     *
     * @return String representation of the license data.
     */
    O3P_API std::string getLicenseInfo();

    /**
     * @brief Get all available use cases from the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<std::string> getAvailableUseCases();

    /**
     * @brief Get the current usecase from the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::string getUseCase();

    /**
     * @brief Set the usecase of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void setUseCase(const std::string &useCase);

    /**
     * @brief Load a preset use case configuration to the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void loadPreset(const Preset &preset);

    /**
     * @brief Get available presets
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::list<o3p::Preset> getAvailablePresets();

    /**
     * @brief Set all camera use case parameters of the device at the same time (avoiding possible intermediate invalid states). Note that not all combinations of settings may be valid, depending on the device capabilities. In case of an invalid combination, an exception is thrown and no settings are changed.
     *
     * @throw std::runtime_error if the pipeline is not initialized or the parameter combination is rejected by firmware
     */
    O3P_API void setCameraParameters(unsigned fps, bool hdr, const std::string &range, unsigned distanceOffset = 0);

    /**
     * @brief Check if the given camera parameter combination is valid for the current device configuration. Note that this does not check all possible combinations of settings, but only the provided combination.
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API bool checkCameraParameters(unsigned fps, bool hdr, const std::string &range, unsigned distanceOffset = 0);

    /**
     * @brief Save the current camera parameters as a custom preset.
     *
     * @throw std::runtime_error if the pipeline is not initialized or if saving fails
     */
    O3P_API void saveCameraParameters();

    /**
     * @brief Set the frame rate of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void setFPS(unsigned fps);

    /**
     * @brief Get the current frame rate of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized or the fps value cannot be parsed
     */
    O3P_API unsigned getFPS();

    /**
     * @brief Get the currently available frame rates of the device (when all other settings in the use case are unchanged)
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<unsigned> getAvailableFPS();

    /**
     * @brief Get all available frame rates of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<unsigned> getAllAvailableFPS();

    /**
     * @brief Set the HDR mode of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void setHDR(bool on);

    /**
     * @brief Get the HDR mode of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized or the HDR value cannot be parsed
     */
    O3P_API bool getHDR();

    /**
     * @brief Get the currently available HDR modes of the device (when all other settings in the use case are unchanged)
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<bool> getAvailableHDR();

    /**
     * @brief Get all available HDR modes of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<bool> getAllAvailableHDR();

    /**
     * @brief Set the range of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void setMeasurementRange(const std::string &range);

    /**
     * @brief Get the range of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::string getMeasurementRange();

    /**
     * @brief Get the currently available measurement ranges of the device (when all other settings in the use case are unchanged)
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<std::string> getAvailableMeasurementRanges();

    /**
     * @brief Get all available measurement ranges of the device
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<std::string> getAllAvailableMeasurementRanges();

    /**
     * @brief Set the distance offset of the use case
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API void setDistanceOffset(const unsigned offset);

    /**
     * @brief Get the distance offset of the use case
     *
     * @throw std::runtime_error if the pipeline is not initialized or the distance offset value cannot be parsed
     */
    O3P_API unsigned getDistanceOffset();

    /**
     * @brief Get the currently available distance offsets of the use case (when all other settings in the use case are unchanged)
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<unsigned> getAvailableDistanceOffsets();

    /**
     * @brief Get all available distance offsets of the use case
     *
     * @throw std::runtime_error if the pipeline is not initialized
     */
    O3P_API std::vector<unsigned> getAllAvailableDistanceOffsets();

    /**
     * @brief Starts a recording
     *
     * @throw std::runtime_error if the pipeline is not initialized
     * @throw UsbDisconnectedException if the USB device is disconnected
     */
    O3P_API void startRecording(const std::string &filename);

    /**
     * @brief Stops a recording
     */
    O3P_API void stopRecording();

    /**
     * @brief Retrieves the current pipeline profile
     *
     * @throw std::runtime_error if the pipeline is not initialized
     *
     * @return The pipeline's profile
     */
    O3P_API PipelineProfile getActiveProfile();

    /**
     * @brief Retrieves the currently used device
     *
     * @return A pointer to the device
     */
    O3P_API std::shared_ptr<Device> getDevice() {
        return m_activeDevice;
    }

    /**
     * @brief Returns the number of frames dropped by the capture backend since the last call and resets the counter.
     *
     * Only meaningful on Linux (V4L2) live devices; returns 0 during playback and on Windows.
     */
    O3P_API uint64_t getAndResetDroppedFrames() {
        return m_activeDevice ? m_activeDevice->getAndResetDroppedFrames() : 0;
    }

    /**
     * @brief Sets the recorded fps
     *
     * Note that the actual recorded fps may be lower than the specified fps depending on the set usecase.
     *
     * @param recFps The recording fps, 0.0 = all frames are recorded
     */
    O3P_API void setRecordingFps(double recFps);

    /**
     * @brief Get the number of frames in the recording
     *
     * @throw std::runtime_error if the pipeline is not initialized or is not using a playback file
     */
    O3P_API uint32_t getNumberOfFramesInRecording();

    O3P_API bool enableAI(bool enable);

  private:
    void deinit();

    void onUsbDisconnected();

    struct Impl;
    std::unique_ptr<Impl> m_pImpl;

    std::shared_ptr<o3p::Device> m_activeDevice;
    FrameSet m_frameSet;

    std::atomic<bool> m_initialized;
    PipelineProfile m_pipelineProfile;

    std::unique_ptr<Recording> m_recording;
    std::atomic<bool> m_recordingActive;

    PipelineConfig m_pipelineConfig;

    std::mutex m_sensorMutex;
    std::atomic<bool> m_running;

    std::atomic<bool> m_usbIsDisconnected{false};

    std::vector<float> m_calcPointCloud;
    float m_lensParameters[9];

    std::atomic<double> m_recFps{0.0};                    // specifies the recording fps, 0.0 means device fps
    std::chrono::steady_clock::time_point m_lastSaveTime; // time point of last saved frame
};

} // namespace o3p

#endif
