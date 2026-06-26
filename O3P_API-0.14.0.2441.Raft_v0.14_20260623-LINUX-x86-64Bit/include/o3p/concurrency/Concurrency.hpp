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

#ifndef O3P_CONCURRENCY_HPP
#define O3P_CONCURRENCY_HPP

#include <o3p/Definitions.hpp>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace o3p {

/**
 * @brief Bounded thread-safe work queue executed on a dedicated worker thread
 */
class WorkQueue {
  public:
    /**
     * @brief Construct a queue with the given capacity
     *
     * @param capacity Maximum number of pending tasks
     */
    O3P_API WorkQueue(size_t capacity);

    /**
     * @brief Stops the worker thread and destroys the queue
     */
    O3P_API ~WorkQueue();

    /**
     * @brief Enqueue a task for asynchronous execution
     *
     * @param task The task to run on the worker thread
     * @param blockIfQueueFull If true, blocks until space is available; otherwise returns false when full
     * @return true if the task was enqueued
     */
    O3P_API bool enqueue(std::function<void()> task, bool blockIfQueueFull = false);

    /**
     * @brief Enqueue a task and block until it has finished executing
     *
     * @param task The task to run on the worker thread
     * @param blockIfQueueFull If true, blocks until space is available; otherwise returns false when full
     * @return true if the task was enqueued and executed
     */
    O3P_API bool enqueueAndWait(std::function<void()> task, bool blockIfQueueFull = false);

    /**
     * @brief Start the worker thread
     */
    O3P_API void start();

    /**
     * @brief Stop the worker thread, discarding any pending tasks
     */
    O3P_API void stop();

    /**
     * @brief Check whether the queue currently contains no pending tasks
     */
    O3P_API bool empty() const;

    /**
     * @brief Get the current number of pending tasks
     */
    O3P_API size_t size() const;

  private:
    size_t m_capacity;
    std::queue<std::function<void()>> m_workQueue;
    mutable std::mutex m_mutex;
    std::condition_variable m_condition;
    std::atomic<bool> m_stopped;
    std::thread workerThread;
};

} // namespace o3p

#endif // O3P_CONCURRENCY_HPP