#ifndef _MESSAGE_MANAGER_H_
#define _MESSAGE_MANAGER_H_

#include <iostream>
#include <list>
#include <future>
#include <memory>
#include <queue>
#include <chrono>

#include "HeaderInfo.h"
#include "Frame.pb.h"

namespace Kinova
{
namespace Api
{
    /**
     * Timeout tracking structure for message callbacks
     * Stores timeout duration and registration time to detect expired callbacks
     */
    struct CallbackTimeoutStruct
    {
        // Timeout duration in milliseconds
        uint32_t m_timeout;

        // Time point when callback was registered
        std::chrono::steady_clock::time_point m_registeredTimePoint;

        /**
         * Constructor
         *
         * @param[in] timeout Timeout duration in milliseconds
         */
        explicit CallbackTimeoutStruct(uint32_t timeout)
            : m_timeout(timeout)
        {
            m_registeredTimePoint = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
        }

        /**
         * Check if callback has expired
         *
         * @return True if elapsed time exceeds timeout, false otherwise
         */
        bool checkExpiry()
        {
            uint32_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_registeredTimePoint).count();
            return (duration >= m_timeout);
        }
    };

    // Callback type for message handling
    typedef std::function<void (const Kinova::Api::Frame&)> MessageCallback;

    // Queue type for timeout tracking by message callback
    typedef std::queue<std::pair<uint32_t, CallbackTimeoutStruct> > TimeoutClockByMessageCallbackQueue;

    /**
     * Frame handler for managing message requests and responses
     *
     * Handles the lifecycle of messages sent through the router, including:
     * - Future-based promises for request/response patterns
     * - Callback-based message handling
     * - Timeout management for callbacks
     * - Exception handling for failed messages
     *
     * @note This is an internal class used by RouterClient
     */
    class FrameHandler
    {
    public:
        /**
         * Constructor
         *
         * @param[in] maxCallbackTimeout Maximum time in milliseconds before callback expires (default: 60000ms = 1 minute)
         */
        explicit FrameHandler(uint32_t maxCallbackTimeout = 60000);

        // Destructor
        ~FrameHandler() = default;

        /**
         * Register a message and get a future for the response
         *
         * @param[in] msgId Message identifier
         * @return Future that will be fulfilled when response arrives
         */
        std::future<Frame> registerMessage(uint32_t msgId);

        /**
         * Register a callback to be invoked when a message response arrives
         *
         * @param[in] msgId Message identifier
         * @param[in] callback Callback to invoke when response arrives
         */
        void registerMessageCallback(uint32_t msgId, const MessageCallback& callback);

        /**
         * Process a received message frame
         *
         * @param[in] msgFrame Frame received from transport layer
         * @return Error status indicating success or failure
         */
        Error manageReceivedMessage(Frame &msgFrame);

        /**
         * Set exception on a message promise due to error
         *
         * @param[in] headerInfo Header information identifying the message
         * @param[in] error Error to set on the promise
         */
        void setMessageException(HeaderInfo& headerInfo, Error& error);

    private:
        // Clean up callbacks that have exceeded their timeout
        void cleanDanglingCallback();

        // Mutex for thread-safe access to internal structures
        std::mutex      m_mutex;

        // Maximum callback timeout in milliseconds
        uint32_t        m_maxCallbackTimeout;

        // Map of message promises indexed by message ID
        std::unordered_map< uint32_t, std::shared_ptr<std::promise<Frame>> > m_messagePromises;

        // Queue for tracking callback timeouts
        TimeoutClockByMessageCallbackQueue m_timeoutClockByCallback;

        // Map of message callbacks indexed by message ID
        std::unordered_map< uint32_t, MessageCallback > m_messageCallbacks;
    };

}
}

#endif // _MESSAGE_MANAGER_H_
