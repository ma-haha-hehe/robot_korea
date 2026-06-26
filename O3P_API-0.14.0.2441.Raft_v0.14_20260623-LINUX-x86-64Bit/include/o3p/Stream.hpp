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

#ifndef STREAM_HPP
#define STREAM_HPP

#include <o3p/Common.hpp>

namespace o3p {

/**
 * @brief Defines a basic Stream
 */
class Stream {
  public:
    /**
     * @brief Basic constructor
     */
    O3P_API Stream(StreamType streamType) : m_streamType(streamType) {
    }

    O3P_API virtual ~Stream() = default;

    /**
     * @brief Get the type of stream
     *
     * @return The stream's type
     */
    O3P_API StreamType streamType() const {
        return m_streamType;
    }

    /**
     * @brief Set the identifier of the stream
     *
     * @param id The identifier of the device this stream belongs to
     */
    O3P_API void setId(const std::string &id) {
        m_id = id;
    }

    /**
     * @brief Get the identifier of the stream
     *
     * @return The identifier of the device this stream belongs to
     */
    O3P_API const std::string &id() const {
        return m_id;
    }

  protected:
    StreamType m_streamType; // The type of the stream
    std::string m_id;        // Identifier of the device to which this stream belongs
};

/**
 * @brief Extension of the Stream class for Video streams
 */
class VideoStream : public Stream {
  public:
    /**
     * @brief Constructor
     */
    O3P_API VideoStream(uint16_t width, uint16_t height,
                        Intrinsics &intrinsics, Extrinsics &extrinsics,
                        StreamType streamType) : Stream(streamType),
                                                 m_width(width),
                                                 m_height(height),
                                                 m_intrinsics(intrinsics),
                                                 m_extrinsicsToOtherStream(extrinsics) {
    }

    /**
     * @brief Get the width of the stream
     *
     * @return The width
     */
    O3P_API uint16_t width() const {
        return m_width;
    }

    /**
     * @brief Get the height of the stream
     *
     * @return The height
     */
    O3P_API uint16_t height() const {
        return m_height;
    }

    /**
     * @brief Get the intrinsic (lens) parameters
     *
     * @return The lens parameters
     */
    O3P_API const Intrinsics &getIntrinsics() const {
        return m_intrinsics;
    }

    /**
     * @brief Get the extrinsic parameters describing the transform to another stream
     *
     * @param to The target stream to compute the extrinsics to
     * @return The extrinsic parameters from this stream to the given stream
     */
    O3P_API Extrinsics getExtrinsicsTo(const Stream &to) const;

    /**
     * @brief Get the stored extrinsic parameters
     *
     * @return The extrinsic parameters to the associated stream
     */
    O3P_API const Extrinsics &getExtrinsics() const {
        return m_extrinsicsToOtherStream;
    }

  private:
    uint16_t m_width;
    uint16_t m_height;
    Intrinsics m_intrinsics;
    Extrinsics m_extrinsicsToOtherStream;
};

} // namespace o3p

#endif
