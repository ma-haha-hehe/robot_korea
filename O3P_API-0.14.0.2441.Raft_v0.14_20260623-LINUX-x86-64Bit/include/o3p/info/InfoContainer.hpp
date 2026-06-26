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

#ifndef O3P_INFO_CONTAINER_HPP
#define O3P_INFO_CONTAINER_HPP

#include "o3p/info/InfoInterface.hpp"
#include <map>
#include <stdexcept>
#include <string>

namespace o3p {

class InfoContainer : public virtual InfoInterface {
  public:
    InfoContainer() = default;
    ~InfoContainer() override = default;

    void addInfo(CameraInfo info, const std::string &value) {
        m_infoMap[info] = value;
    }

    void updateInfo(CameraInfo info, const std::string &value) {
        auto it = m_infoMap.find(info);
        if (it == m_infoMap.end()) {
            throw std::runtime_error("Camera info not found");
        }
        it->second = value;
    }

    bool supportsInfo(CameraInfo info) const override {
        return m_infoMap.find(info) != m_infoMap.end();
    }

    std::string getInfo(CameraInfo info) const override {
        auto it = m_infoMap.find(info);
        if (it == m_infoMap.end()) {
            throw std::runtime_error("Camera info not found");
        }
        return it->second;
    }

  private:
    std::map<CameraInfo, std::string> m_infoMap;
};

} // namespace o3p

#endif // O3P_INFO_CONTAINER_HPP