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

#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include "Device.hpp"
#include <memory>
#include <string>
#include <vector>

#include <o3p/Common.hpp>

namespace o3p {

/**
 * @brief Represents the camera device
 */
class Context {
  public:
    /**
     * @brief Get the identifiers of all available cameras
     *
     * @return A list of identifier strings of all currently available cameras
     */
    O3P_API static std::vector<std::string> getAvailableCameras();

    /**
     * @brief Get all currently connected devices
     *
     * @return A list of shared pointers to all connected o3p::Device instances
     */
    O3P_API static std::vector<std::shared_ptr<o3p::Device>> getConnectedDevices();

    /**
     * @brief Get the singleton instance of the Context
     *
     * @return Reference to the global Context instance
     */
    O3P_API static Context &getInstance() {
        static Context instance;
        return instance;
    }

  private:
    O3P_API Context();
    O3P_API ~Context();

    O3P_API Context(const Context &) = delete;
    O3P_API Context &operator=(const Context &) = delete;

#ifdef _WIN32
#ifdef O3P_USE_DIRECTSHOW
    bool m_comInitialized;
#endif
#endif
};

} // namespace o3p

#endif
