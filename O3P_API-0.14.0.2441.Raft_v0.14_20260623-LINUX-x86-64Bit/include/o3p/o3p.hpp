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

#ifndef O3P_HPP
#define O3P_HPP

#include "Align.hpp"
#include "Common.hpp"
#include "Context.hpp"
#include "Device.hpp"
#include "FrameSet.hpp"
#include "Pipeline.hpp"
#include "Stream.hpp"
#include <o3p-version.h>
#include "Colorizer.hpp"

namespace {
o3p::Context &_ctxAutoInit = o3p::Context::getInstance();
}

/**
 * This file contains the declarations of the O3P library, which provides
 * functionality for working with O3P devices, framesets, pipelines, and streams.
 * It includes the necessary dependencies such as Common.hpp, Device.hpp,
 * FrameSet.hpp, Pipeline.hpp, and Stream.hpp.
 */

namespace o3p {
/**
 * @brief Get the numeric API version
 *
 * @return The version encoded as MAJOR*10000 + MINOR*100 + PATCH
 */
inline static int getApiVersion() {
    return O3P_VERSION_MAJOR * 10000 + O3P_VERSION_MINOR * 100 + O3P_VERSION_PATCH;
}

/**
 * @brief Get the API version as a string
 *
 * @return The version in the format "MAJOR.MINOR.PATCH.BUILD"
 */
inline static std::string getApiVersionString() {
    return std::to_string(O3P_VERSION_MAJOR) + "." + std::to_string(O3P_VERSION_MINOR) +
           "." + std::to_string(O3P_VERSION_PATCH) + "." + std::to_string(O3P_VERSION_BUILD);
}
} // namespace o3p

#endif
