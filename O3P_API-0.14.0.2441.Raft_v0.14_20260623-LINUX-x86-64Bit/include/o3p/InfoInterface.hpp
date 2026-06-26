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

#ifndef O3P_INFO_INFOINTERFACE_HPP
#define O3P_INFO_INFOINTERFACE_HPP

#include "o3p/Common.hpp"
#include <string>

namespace o3p {

/**
 * @brief Interface for objects exposing CameraInfo properties
 */
class InfoInterface {
  public:
    virtual ~InfoInterface() = default;

    /**
     * @brief Check whether a given CameraInfo entry is supported
     *
     * @param info The info entry to check
     * @return true if the info is provided by this object
     */
    virtual bool supportsInfo(CameraInfo info) const = 0;

    /**
     * @brief Get the value of a CameraInfo entry as string
     *
     * @param info The info entry to retrieve
     * @return String representation of the requested info
     */
    virtual std::string getInfo(CameraInfo info) const = 0;
};

} // namespace o3p

#endif // O3P_INFO_INFOINTERFACE_HPP