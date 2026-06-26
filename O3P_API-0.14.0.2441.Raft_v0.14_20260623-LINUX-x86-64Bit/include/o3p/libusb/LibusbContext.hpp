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

#ifndef O3P_LIBUSB_LIBUSB_CONTEXT_HPP
#define O3P_LIBUSB_LIBUSB_CONTEXT_HPP

#include <libusb-1.0/libusb.h>
#include <mutex>
#include <thread>

namespace o3p {

/**
 * @brief RAII wrapper around a libusb context with a background event handler thread
 */
class LibusbContext {
  public:
    /**
     * @brief Initialize a new libusb context and enumerate the connected devices
     */
    LibusbContext();

    /**
     * @brief Stop the event handler and release the libusb context
     */
    ~LibusbContext();

    /**
     * @brief Get the underlying libusb context pointer
     */
    libusb_context *getNativeContext();

    /**
     * @brief Start the asynchronous event handling thread (refcounted)
     */
    void startEventHandler();

    /**
     * @brief Stop the asynchronous event handling thread (refcounted)
     */
    void stopEventHandler();

    /**
     * @brief Get the number of devices currently enumerated
     */
    size_t deviceCount();

    /**
     * @brief Get the device at the given index
     *
     * @param index Index in the enumerated device list
     */
    libusb_device *getDevice(uint8_t index);

  private:
    std::mutex m_mutex;
    libusb_device **m_list;
    size_t m_deviceCount;
    struct libusb_context *m_ctx;
    std::thread m_eventThread;
    int m_killHandlerThread = 0;
    int m_handlerRequests = 0;
};

} // namespace o3p

#endif // O3P_LIBUSB_LIBUSB_CONTEXT_HPP
