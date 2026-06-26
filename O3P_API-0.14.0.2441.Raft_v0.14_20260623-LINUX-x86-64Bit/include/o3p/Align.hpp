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

#ifndef ALIGN_HPP
#define ALIGN_HPP

#include "o3p/Common.hpp"
#include "o3p/FrameSet.hpp"

namespace o3p {

/**
 * @brief Aligns the color frame to the depth frame resolution
 *
 * For each depth pixel, this function finds the corresponding color pixel by
 * transforming 3D points from the ToF camera coordinate system to the RGB camera
 * coordinate system and projecting them onto the RGB image plane.
 *
 * When a pointcloud frame is available in the FrameSet, it is used directly for
 * the 3D positions. Otherwise, depth values are back-projected using the ToF intrinsics.
 *
 * After calling this function, the color frame in the FrameSet will have the same
 * width and height as the depth frame, so that every depth point has a matching color point.
 *
 * @param frameSet FrameSet containing color, depth, and optionally pointcloud data.
 *                 The color frame will be replaced with the aligned version.
 * @param tofToRgbExtrinsics Extrinsics that transform points from ToF camera space to RGB camera space
 *                           (obtained from the depth stream via getExtrinsics())
 * @param rgbIntrinsics Intrinsic parameters of the RGB camera
 * @param tofIntrinsics Intrinsic parameters of the ToF camera
 */
O3P_API void alignColorToDepth(FrameSet &frameSet,
                               const Extrinsics &tofToRgbExtrinsics,
                               const Intrinsics &rgbIntrinsics,
                               const Intrinsics &tofIntrinsics);

} // namespace o3p

#endif
