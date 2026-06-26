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

#ifndef UVCDEVICEWIN_HPP
#define UVCDEVICEWIN_HPP

#include <windows.h>

#include <o3p/Device.hpp>
#include <o3p/UvcDevice.hpp>
#include <o3p/WindowsCommon.hpp>

#include <string>
#include <set>

namespace o3p {

/**
 * @brief Windows specific UVC device base class providing camera locking
 */
class UvcDeviceWin : public UvcDevice {
  public:
    UvcDeviceWin();

    ~UvcDeviceWin() override;

    /**
     * @brief Lock camera so other processes cannot access it
     *
     * @param serial Serial number of the camera to lock
     * @return true if the lock was acquired
     */
    bool lockCamera(const std::string &serial);

    /**
     * @brief Release a previously acquired camera lock
     *
     * @param serial Serial number of the camera to unlock
     */
    void unlockCamera(const std::string &serial);

    /**
     * @brief Check whether the camera with the given serial is currently in use
     *
     * @param serial Serial number of the camera to check
     * @return true if the camera is currently locked
     */
    static bool isCameraInUse(const std::string &serial);

  protected:
    HANDLE m_cameraMutex = nullptr;
    static std::set<std::string> s_lockedCameras;
    static std::mutex s_lockedCamerasMutex;
};

} // namespace o3p

#endif // UVCDEVICEWIN_HPP