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

#ifndef RECORDING_HPP
#define RECORDING_HPP

#include <map>
#include <memory>
#include <stdexcept>

#include <o3p/FrameSet.hpp>
#include <o3p/Stream.hpp>

namespace o3p {

/**
 * @brief Defines the class used for recording sensor data
 */
class Recording {

  public:
    /**
     * @brief Default constructor
     */
    O3P_API Recording();

    /**
     * @brief Default destructor
     */
    O3P_API ~Recording();

    /**
     * @brief Start the sensor recording
     */
    O3P_API void startRecording(const std::string &filename, const std::vector<std::shared_ptr<Stream>> &streams,
                                const std::string &serial, bool hasUltrasonicSensor,
                                const std::map<std::string, std::string> &keys);

    /**
     * @brief Stop the sensor recording
     */
    O3P_API void stopRecording();

    /**
     * @brief Record a frame
     */
    O3P_API void recordFrame(FrameSet *frameSet);

  private:
    struct Impl;
    std::unique_ptr<Impl> m_pImpl;

    uint32_t m_recFrameCounter;
};

} // namespace o3p

#endif
