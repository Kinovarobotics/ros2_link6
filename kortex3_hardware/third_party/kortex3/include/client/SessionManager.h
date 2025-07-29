#ifndef _SESSION_MANAGER_H_
#define _SESSION_MANAGER_H_

#include <iostream>
#include <list>
#include <future>
#include <memory>
#include <functional>

#include "IRouterClient.h"
#include "Frame.pb.h"
#include "Session.pb.h"
#include "SessionClientRpc.h"

namespace Kinova
{
namespace Api
{
    /**
     * Session manager for handling client sessions with Kortex devices
     *
     * This class extends the auto-generated SessionClient to provide high-level
     * session management functionality including session creation, validation,
     * and automatic keep-alive mechanisms.
     *
     * @note Sessions must be created before using other Kortex API services
     */
    class SessionManager : public Session::SessionClient
    {
    public:
        // Default constructor is deleted - use the parameterized constructor
        SessionManager() = delete;

        /**
         * Constructor
         *
         * @param[in] router Router to use for communication with the device
         * @param[in] connectionTimeoutCallback Optional callback invoked when connection timeout occurs
         *                                      If nullptr, no callback will be called on timeout
         */
        explicit SessionManager(IRouterClient* router, std::function<void()> connectionTimeoutCallback = nullptr);

        /**
         * Destructor
         * Automatically closes the session if one is active
         */
        virtual ~SessionManager();

        /**
         * Create a new session with the device
         *
         * @param[in] info Session creation information (user credentials, timeout values, etc.)
         * @exception KDetailedException if session creation fails
         * @post A valid session is established and automatic keep-alive is started
         */
        void CreateSession(const Session::CreateSessionInfo& info);

        /**
         * Close the current session
         *
         * @post The session is terminated and keep-alive thread is stopped
         * @note It is safe to call this even if no session is active
         */
        void CloseSession();

        /**
         * Get the list of active connections to the device
         *
         * @return A ConnectionList containing information about all active connections
         * @exception KDetailedException if the query fails
         */
        Session::ConnectionList GetConnections();

    private:
        void Hit(FrameTypes hitType);
        void ThreadSessionValidation();

        std::function<void()>           m_connectionTimeoutCallback;

        std::thread                     m_thread;
        std::atomic<bool>               m_hasBeenSent { false };
        std::atomic<bool>               m_hasBeenReceived { false };
        std::atomic<bool>               m_threadRunning { false };
        Session::CreateSessionInfo      m_sessionInfo;

        void checkTransport();
    };

}
}

#endif // _MESSAGE_MANAGER_H_
