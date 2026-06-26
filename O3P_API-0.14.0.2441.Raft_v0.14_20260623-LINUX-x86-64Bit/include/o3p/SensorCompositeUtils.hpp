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

#ifndef SENSORCOMPOSITEUTILS_HPP
#define SENSORCOMPOSITEUTILS_HPP

#include <o3p/Common.hpp>

namespace o3p {

/**
 * @brief Converts extrinsics array (quaternion + translation) to Extrinsics object
 * @param extrinsics Array of 7 floats: [tx, ty, tz, qx, qy, qz, qw]
 * @return Extrinsics object with 3x3 rotation matrix (column-major) and translation
 */
inline Extrinsics convertExtrinsics(const float extrinsics[7]) {
    Extrinsics result;

    // Extract quaternion components (x, y, z, w)
    float qx = extrinsics[3];
    float qy = extrinsics[4];
    float qz = extrinsics[5];
    float qw = extrinsics[6];

    // Extract translation (tx, ty, tz)
    result.translation[0] = extrinsics[0];
    result.translation[1] = extrinsics[1];
    result.translation[2] = extrinsics[2];

    // Convert quaternion to 3x3 rotation matrix (column-major)
    // Column 0
    result.rotation[0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    result.rotation[1] = 2.0f * (qx * qy + qw * qz);
    result.rotation[2] = 2.0f * (qx * qz - qw * qy);

    // Column 1
    result.rotation[3] = 2.0f * (qx * qy - qw * qz);
    result.rotation[4] = 1.0f - 2.0f * (qx * qx + qz * qz);
    result.rotation[5] = 2.0f * (qy * qz + qw * qx);

    // Column 2
    result.rotation[6] = 2.0f * (qx * qz + qw * qy);
    result.rotation[7] = 2.0f * (qy * qz - qw * qx);
    result.rotation[8] = 1.0f - 2.0f * (qx * qx + qy * qy);

    return result;
}

/**
 * @brief Inverts extrinsics to get the opposite transformation
 * @param extrinsics The original extrinsics (e.g., RGB to ToF)
 * @return Inverted extrinsics (e.g., ToF to RGB)
 */
inline Extrinsics invertExtrinsics(const Extrinsics &extrinsics) {
    Extrinsics result;

    // Transpose the rotation matrix (inverse of rotation matrix is its transpose)
    // Original is column-major, so:
    // Column 0 becomes Row 0
    result.rotation[0] = extrinsics.rotation[0]; // R[0][0]
    result.rotation[3] = extrinsics.rotation[1]; // R[1][0]
    result.rotation[6] = extrinsics.rotation[2]; // R[2][0]

    // Column 1 becomes Row 1
    result.rotation[1] = extrinsics.rotation[3]; // R[0][1]
    result.rotation[4] = extrinsics.rotation[4]; // R[1][1]
    result.rotation[7] = extrinsics.rotation[5]; // R[2][1]

    // Column 2 becomes Row 2
    result.rotation[2] = extrinsics.rotation[6]; // R[0][2]
    result.rotation[5] = extrinsics.rotation[7]; // R[1][2]
    result.rotation[8] = extrinsics.rotation[8]; // R[2][2]

    // Calculate inverse translation: -R^T * t
    result.translation[0] = -(extrinsics.rotation[0] * extrinsics.translation[0] + extrinsics.rotation[1] * extrinsics.translation[1] + extrinsics.rotation[2] * extrinsics.translation[2]);
    result.translation[1] = -(extrinsics.rotation[3] * extrinsics.translation[0] + extrinsics.rotation[4] * extrinsics.translation[1] + extrinsics.rotation[5] * extrinsics.translation[2]);
    result.translation[2] = -(extrinsics.rotation[6] * extrinsics.translation[0] + extrinsics.rotation[7] * extrinsics.translation[1] + extrinsics.rotation[8] * extrinsics.translation[2]);

    return result;
}

} // namespace o3p

#endif // SENSORCOMPOSITEUTILS_HPP
