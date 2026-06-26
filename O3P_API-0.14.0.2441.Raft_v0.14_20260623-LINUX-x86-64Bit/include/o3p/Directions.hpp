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

#ifndef DIRECTIONS_HPP
#define DIRECTIONS_HPP

#include "o3p/Definitions.hpp"
#include "o3p/LensModelType.hpp"
#include <stdint.h>

namespace o3p {

enum DirectionStatus {
    /// The operation failed
    STATUS_ERROR = -1,
    /// The operation succeeded
    STATUS_SUCCESS = 0
};

/**
 * @brief Updates the directions based on the lensParameters and the calibrationROI
 *
 * @param directions destination for directions
 * @param size number of entries in directions (see above)
 * @param lensParameters lensparameters from calibration (array of size 9)
 * @param lensModel lens model type
 * @param numRows number of rows in the calibration ROI
 * @param numCols number of columns in the calibration ROI
 *
 * @return status of operation
 */
O3P_API enum DirectionStatus updateDirections(float *directions, long size,
                                              const float *lensParameters,
                                              enum LensModelType lensModel,
                                              uint32_t numRows, uint32_t numCols);

} // namespace o3p

#endif // DIRECTIONS_HPP
