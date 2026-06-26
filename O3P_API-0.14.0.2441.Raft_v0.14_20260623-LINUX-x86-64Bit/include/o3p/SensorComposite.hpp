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

#ifndef SENSORCOMPOSITE_HPP
#define SENSORCOMPOSITE_HPP

#include <o3p/Sensor.hpp>

namespace o3p {

/**
 * @brief Extends the sensor class for composite sensors
 */
class SensorComposite : public o3p::Sensor {

  public:
    /**
     * @brief Basic constructor
     */
    O3P_API SensorComposite(const std::string &id);

    /**
     * @brief Basic destructor
     */
    O3P_API ~SensorComposite() override;

    /**
     * @brief Initialize the sensor
     */
    O3P_API void init() override;

    /**
     * @brief Start the sensor
     */
    O3P_API void start() override;

    /**
     * @brief Stop the sensor
     */
    O3P_API void stop() override;

    /**
     * @brief Close the device
     */
    O3P_API void closeDevice() override;

    /**
     * @brief Wait for the frames
     */
    O3P_API void waitForFrames(FrameSet *frameSet) override;

    /**
     * @brief Get the streams
     */
    O3P_API void getStreams(std::vector<std::shared_ptr<Stream>> &streams) override;

    O3P_API void setDisconnectCallback(DisconnectCallback cb);

  private:
    void updateConfig();

    std::vector<uint16_t> m_tofData;
    std::vector<uint8_t> m_infraredData;
    std::vector<uint8_t> m_pluginData;
    std::vector<uint8_t> m_rgbData;
    Guid m_extensionUnit;

    std::vector<float> m_calcPointCloud;

    Intrinsics m_intrinsicsTof;
    Intrinsics m_intrinsicsRgb;

    Extrinsics m_extrinsicsTofToRgb;
    Extrinsics m_extrinsicsRgbToTof;

    CompositeFrameConfig m_frameConfig;

    uint16_t m_widthTof;
    uint16_t m_heightTof;
    uint16_t m_widthRgb;
    uint16_t m_heightRgb;
};

} // namespace o3p

#endif
