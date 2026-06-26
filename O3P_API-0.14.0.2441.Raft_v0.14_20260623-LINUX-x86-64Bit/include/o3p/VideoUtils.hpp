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

#ifndef VIDEOUTILS_HPP
#define VIDEOUTILS_HPP

#include <algorithm>
#include <cstdint>
#include <vector>

namespace o3p {

/**
 * @brief Convert an NV12 (YUV semi-planar, bpp=12) buffer to packed RGB24.
 *
 * width/height describe the luma plane; the UV plane follows immediately.
 *
 * @param data Pointer to the NV12 source buffer
 * @param width Image width in pixels (luma plane)
 * @param height Image height in pixels (luma plane)
 * @param out Destination vector that will be resized and filled with packed RGB24 data
 */
inline void convertNv12ToRgb(const uint8_t *data, uint16_t width, uint16_t height,
                             std::vector<uint8_t> &out) {
    const size_t numPixels = static_cast<size_t>(width) * height;
    out.resize(numPixels * 3);
    const uint8_t *yData = data;
    const uint8_t *uvData = data + numPixels;

    // Fixed-point coefficients (scaled by 256) for BT.601 YUV-to-RGB
    constexpr int kRv = 359; // 1.402    * 256
    constexpr int kGu = 88;  // 0.344136 * 256
    constexpr int kGv = 183; // 0.714136 * 256
    constexpr int kBu = 454; // 1.772    * 256

    const int w = width;
    const int h = height;
    const int evenW = w & ~1;

    for (int y = 0; y < h; ++y) {
        const int yRowOff = y * w;
        const uint8_t *uvRow = uvData + (y / 2) * w;
        const uint8_t *yRow = yData + yRowOff;
        uint8_t *outRow = out.data() + yRowOff * 3;

        // Process pixel pairs sharing the same UV sample
        for (int x = 0; x < evenW; x += 2) {
            int u = static_cast<int>(uvRow[x]) - 128;
            int v = static_cast<int>(uvRow[x + 1]) - 128;

            int crR = kRv * v;
            int crG = kGu * u + kGv * v;
            int crB = kBu * u;

            // Pixel 0
            {
                int yv = static_cast<int>(yRow[x]) << 8;
                uint8_t *dst = outRow + x * 3;
                dst[0] = static_cast<uint8_t>(std::max(0, std::min(255, (yv + crR) >> 8)));
                dst[1] = static_cast<uint8_t>(std::max(0, std::min(255, (yv - crG) >> 8)));
                dst[2] = static_cast<uint8_t>(std::max(0, std::min(255, (yv + crB) >> 8)));
            }
            // Pixel 1
            {
                int yv = static_cast<int>(yRow[x + 1]) << 8;
                uint8_t *dst = outRow + (x + 1) * 3;
                dst[0] = static_cast<uint8_t>(std::max(0, std::min(255, (yv + crR) >> 8)));
                dst[1] = static_cast<uint8_t>(std::max(0, std::min(255, (yv - crG) >> 8)));
                dst[2] = static_cast<uint8_t>(std::max(0, std::min(255, (yv + crB) >> 8)));
            }
        }

        // Handle potential odd last column
        if (evenW < w) {
            int u = static_cast<int>(uvRow[evenW]) - 128;
            int v = static_cast<int>(uvRow[evenW + 1]) - 128;
            int yv = static_cast<int>(yRow[evenW]) << 8;
            uint8_t *dst = outRow + evenW * 3;
            dst[0] = static_cast<uint8_t>(std::max(0, std::min(255, (yv + kRv * v) >> 8)));
            dst[1] = static_cast<uint8_t>(std::max(0, std::min(255, (yv - kGu * u - kGv * v) >> 8)));
            dst[2] = static_cast<uint8_t>(std::max(0, std::min(255, (yv + kBu * u) >> 8)));
        }
    }
}

} // namespace o3p

#endif // VIDEOUTILS_HPP
