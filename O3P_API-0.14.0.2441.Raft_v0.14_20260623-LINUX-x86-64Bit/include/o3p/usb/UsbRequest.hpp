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

#ifndef O3P_USB_REQUEST_HPP
#define O3P_USB_REQUEST_HPP

#include "o3p/usb/UsbEndpoint.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace o3p {

class UsbRequestCallback;

/**
 * @brief Abstract asynchronous USB request
 */
class UsbRequest {
  public:
    /**
     * @brief Create a request bound to the given endpoint
     *
     * @param endpoint Endpoint the request operates on
     */
    UsbRequest(std::shared_ptr<UsbEndpoint> endpoint) : m_endpoint(endpoint) {}
    virtual ~UsbRequest() = default;

    /**
     * @brief Get the endpoint this request is bound to
     */
    virtual std::shared_ptr<UsbEndpoint> getEndpoint() const { return m_endpoint; }

    /**
     * @brief Access the underlying backend specific request object
     */
    virtual void *getNativeRequest() const = 0;

    /**
     * @brief Set the callback that is invoked when the request completes
     *
     * @param callback Callback to invoke; pass nullptr to clear
     */
    virtual void setCallback(const std::shared_ptr<UsbRequestCallback> &callback) { m_callback = callback; }

    /**
     * @brief Get the currently registered completion callback
     */
    virtual std::shared_ptr<UsbRequestCallback> getCallback() const { return m_callback; }

    /**
     * @brief Get the data buffer associated with the request
     */
    virtual const std::vector<uint8_t> &getBuffer() const { return m_buffer; }

    /**
     * @brief Set the data buffer associated with the request
     *
     * @param buffer Buffer contents
     */
    virtual void setBuffer(const std::vector<uint8_t> &buffer) { m_buffer = buffer; }

    /**
     * @brief Blocks until any in-flight async request has completed or been cancelled.
     */
    virtual void waitUntilInactive() {}

  protected:
    void *m_data;
    std::shared_ptr<UsbRequestCallback> m_callback;
    std::shared_ptr<UsbEndpoint> m_endpoint;
    std::vector<uint8_t> m_buffer;
};

/**
 * @brief Thread safe wrapper for a USB request completion callback
 */
class UsbRequestCallback {
  public:
    /**
     * @brief Construct a callback wrapper around the given std::function
     *
     * @param callback Function to invoke when the request completes
     */
    UsbRequestCallback(std::function<void(std::shared_ptr<UsbRequest>)> callback)
        : m_callback(callback) {}

    ~UsbRequestCallback() {
        cancel();
    }

    /**
     * @brief Disable the callback so that subsequent invocations are no-ops
     */
    void cancel() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_callback = nullptr;
    }

    /**
     * @brief Invoke the wrapped callback if it has not been cancelled
     *
     * @param request The completed USB request to forward to the callback
     */
    void operator()(std::shared_ptr<UsbRequest> request) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_callback) {
            m_callback(request);
        }
    }

  private:
    std::mutex m_mutex;
    std::function<void(std::shared_ptr<UsbRequest>)> m_callback;
};

} // namespace o3p

#endif /* O3P_USB_REQUEST_HPP */
