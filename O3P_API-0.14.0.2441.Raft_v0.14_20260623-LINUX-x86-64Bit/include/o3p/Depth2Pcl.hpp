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

#ifndef DEPTH2PCL_HPP
#define DEPTH2PCL_HPP

#include "o3p/Definitions.hpp"
#include "o3p/LensModelType.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace o3p {

struct Depth2PclImpl;

/**
 * @brief Helper class which can be used to convert a depth array back to a full point cloud
 */
class Depth2Pcl final {
  public:
    O3P_API ~Depth2Pcl();

    /**
     * @brief Calculates the full point cloud based on Z
     *
     * This function assumes that z contains the number of pixels specified in the
     * calibration blob used during constructing this instance.
     *
     * The output array must be allocated for at least 4 * number of pixels.
     *
     * The resulting confidence will be either 0 or 1. Pixels with a depth of z
     * are interpreted as flagged, and get a zero confidence.
     *
     * @param z depth for each pixel (in meter)
     * @param coordinates output destination (xyzc)
     */
    O3P_API void calculatePointCloud(const float *z, float *coordinates) const;

    /**
       In contrast to calculatePointCloud(const float*, float *) this function
       takes z as fixed point values (interpreted in mm).
     */
    O3P_API void calculatePointCloud(const uint16_t *z, float *coordinates) const;

    /**
     * @brief Creates a new Depth2Pcl instance based on the lens parameters + model
     *
     * @param lensParameters lens parameter to use
     * @param lensModelType lens model type to use
     * @param numRows number of rows of full image
     * @param numCols number of columns of full image
     *
     * @return managed pointer to created instance, or nullptr in case of an error
     */
    static O3P_API std::unique_ptr<Depth2Pcl> create(const float *lensParameters, LensModelType lensModelType, uint32_t numRows, uint32_t numCols);

    /**
     * @brief Gets the number of rows expected for input/output
     *
     * @return number of rows
     */
    O3P_API uint32_t numRows() const;

    /**
     * @brief Gets the number of columns expected for input/output
     *
     * @return number of columns
     */
    O3P_API uint32_t numCols() const;

    Depth2Pcl(const Depth2Pcl &) = delete;
    Depth2Pcl &operator=(const Depth2Pcl &) = delete;
    Depth2Pcl &operator=(Depth2Pcl &&) = delete;
    Depth2Pcl(Depth2Pcl &&) = delete;

  private:
    explicit Depth2Pcl(std::unique_ptr<Depth2PclImpl> pimpl);

  private:
    std::unique_ptr<Depth2PclImpl> m_pimpl;
};

} // namespace o3p

#endif
