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

#ifndef SAMPLEGABBERCALLBACK_HPP
#define SAMPLEGABBERCALLBACK_HPP

#include <cstdint>
#include <dshow.h>
#include <mutex>
#include <string>

#include <o3p/QEditCompat.hpp>
#include <o3p/DsHelper.hpp>

namespace o3p {

/**
 * @brief Callback to grab samples from device using Direct Show
 */
class SampleGrabberCallback : public ISampleGrabberCB {
  public:
    /**
     * @brief Basic constructor
     *
     * @param buffer Buffer that receives the sample data
     * @param sampleReadyEvent Event signalled when a sample is available
     */
    SampleGrabberCallback(Buffer &buffer, HANDLE sampleReadyEvent);

    /**
     * @brief Basic destructor
     */
    ~SampleGrabberCallback();

    /**
     * @brief IUnknown::QueryInterface implementation
     */
    STDMETHODIMP QueryInterface(REFIID riid, void **ppv) override;
    /**
     * @brief IUnknown::AddRef implementation
     */
    STDMETHODIMP_(ULONG)
    AddRef() override;
    /**
     * @brief IUnknown::Release implementation
     */
    STDMETHODIMP_(ULONG)
    Release() override;

    /**
     * @brief Called by DirectShow when a new sample is available
     */
    STDMETHODIMP SampleCB(double SampleTime, IMediaSample *pSample) override;
    /**
     * @brief Called by DirectShow when a new buffer is available
     */
    STDMETHODIMP BufferCB(double SampleTime, BYTE *pBuffer, long BufferLen) override;

  private:
    LONG m_refCount;
    Buffer &m_buffer;
    std::vector<uint8_t> m_internalBuffer;
    HANDLE m_sampleReadyEvent;
};

} // namespace o3p

#endif // SAMPLEGABBERCALLBACK_HPP
