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

#ifndef O3P_USB_MESSENGER_HPP
#define O3P_USB_MESSENGER_HPP

#include "o3p/usb/UsbEndpoint.hpp"
#include "o3p/usb/UsbRequest.hpp"

#include <cstdint>
#include <memory>

namespace o3p {

/**
 * @brief Abstract messenger used to perform USB transfers on a claimed interface
 */
class UsbMessenger {
  public:
    virtual ~UsbMessenger() = default;

    /**
     * @brief Perform a synchronous bulk transfer
     *
     * @param endpoint Endpoint to perform the transfer on
     * @param buffer Pointer to the data buffer
     * @param length Length of the buffer in bytes
     * @param transferred Receives the number of bytes actually transferred
     * @param timeout_ms Timeout in milliseconds
     * @return true on success, false otherwise
     */
    virtual bool bulkTransfer(const std::shared_ptr<UsbEndpoint> &endpoint, uint8_t *buffer, uint32_t length, uint32_t &transferred, uint32_t timeout_ms) = 0;

    /**
     * @brief Submit an asynchronous USB request
     *
     * @param request The request to submit
     * @return true if the request was submitted successfully
     */
    virtual bool submitRequest(std::shared_ptr<UsbRequest> request) = 0;

    /**
     * @brief Cancel a previously submitted request
     *
     * @param request The request to cancel
     * @return true on success
     */
    virtual bool cancelRequest(std::shared_ptr<UsbRequest> request) = 0;

    /**
     * @brief Create a new USB request bound to the given endpoint
     *
     * @param endpoint The endpoint the request will be performed on
     * @return Shared pointer to the new request
     */
    virtual std::shared_ptr<UsbRequest> createRequest(const std::shared_ptr<UsbEndpoint> &endpoint) = 0;
};

} // namespace o3p

#endif // O3P_USB_MESSENGER_HPP