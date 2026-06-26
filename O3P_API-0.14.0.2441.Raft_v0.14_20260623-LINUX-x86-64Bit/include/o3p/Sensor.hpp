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

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <memory>
#include <stdexcept>

#include <o3p/FrameSet.hpp>
#include <o3p/Stream.hpp>
#include <o3p/UvcDevice.hpp>

namespace o3p {

/**
 * @brief Defines the Sensor class
 */
class Sensor {

  public:
    /**
     * @brief Default constructor
     */
    O3P_API Sensor();

    /**
     * @brief Default destructor
     */
    O3P_API virtual ~Sensor() = default;

    /**
     * @brief Initialize the sensor
     */
    O3P_API virtual void init() = 0;

    /**
     * @brief Start the sensor
     */
    O3P_API virtual void start() = 0;

    /**
     * @brief Stop the sensor
     */
    O3P_API virtual void stop() = 0;

    /**
     * @brief Close the device
     */
    O3P_API virtual void closeDevice() = 0;

    /**
     * @brief Waits for the frames
     */
    O3P_API virtual void waitForFrames(FrameSet *frameSet) = 0;

    /**
     * @brief Get the streams
     */
    O3P_API virtual void getStreams(std::vector<std::shared_ptr<Stream>> &streams) = 0;

    /**
     * @brief Retrieve the underlying UVC device
     */
    O3P_API virtual UvcDevice *getDevice() { return m_device.get(); }

    /**
     * @brief Returns the camera ID
     */
    [[nodiscard]] O3P_API const std::string &getId() const { return m_id; }

  protected:
    bool m_initialized = false;

    float m_lensParametersTof[9];
    float m_lensParametersRgb[9];
    float m_extrinsics[7];

    std::unique_ptr<UvcDevice> m_device;
    std::string m_id;
};

} // namespace o3p

#endif
