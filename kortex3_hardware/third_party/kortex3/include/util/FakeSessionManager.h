#ifndef __FAKESESSIONMANAGER_H__
#define __FAKESESSIONMANAGER_H__

#include <SessionServerRpc.h>
#include <RouterMQTT.h>

/**
 * Fake session manager for testing and simulation purposes
 *
 * This class provides a simplified session management implementation that automatically
 * grants full system permissions to all connections. It is intended for testing scenarios
 * where authentication is not required.
 *
 * @note This class should NOT be used in production environments
 *
 * @warning All created sessions are granted full system-level permissions
 */
class FakeSessionManager : SessionServer
{
	public:
		/**
		 * Constructor
		 * Initializes the fake session manager with no active sessions
		 */
		FakeSessionManager();

		// Destructor
		virtual ~FakeSessionManager();

		/**
		 * Initialize the fake session manager with a router
		 *
		 * @param[in] pApiRouter Pointer to the router server (must be a RouterMQTT instance)
		 * @return True on success, false on failure
		 * @note The router is cast to RouterMQTT and stored for publishing session status
		 */
		bool Init(IRouterServer* pApiRouter);

		/**
		 * Create a new session with full system permissions
		 *
		 * @param[in] sessionInfo Connection session information
		 * @param[in] pInCreateSessionInfo Session creation parameters (not used in fake implementation)
		 * @param[out] pOutEmpty Empty response message
		 * @return Error object (always returns success)
		 * @post A new session is created with userId=1, full permissions, and logged-in status
		 * @post Session status is published to MQTT network
		 */
		Kinova::Api::Error CreateSession(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Session::CreateSessionInfo* pInCreateSessionInfo, Kinova::Api::Common::Empty* pOutEmpty) override;

		/**
		 * Close an existing session
		 *
		 * @param[in] sessionInfo Session information to close
		 * @param[in] pInEmpty Empty input message
		 * @param[out] pOutEmpty Empty response message
		 * @return Error object (always returns success)
		 * @post The session is marked as inactive and not logged in
		 * @post Session status is published to MQTT network
		 */
		Kinova::Api::Error CloseSession(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Common::Empty* pOutEmpty) override;

		/**
		 * Get the list of all active connections
		 *
		 * @param[in] sessionInfo Current session information (not used)
		 * @param[in] pInEmpty Empty input message
		 * @param[out] pOutConnectionList List of all active connections with their details
		 * @return Error object indicating success or failure
		 */
		Kinova::Api::Error GetConnections(const KeApiRouter::tSessionInfo &sessionInfo, Kinova::Api::Common::Empty* pInEmpty, Kinova::Api::Session::ConnectionList* pOutConnectionList) override;

		/**
		 * Update the internal session map and publish status to MQTT
		 *
		 * @param[in] sessionInfo Session information to update
		 * @post If session is logged in, it is added/updated in the session map
		 * @post If session is not logged in, it is removed from the session map
		 * @post Updated session status is published to MQTT network
		 */
		void UpdateUserSessions(const tSessionInfo &sessionInfo);

    private:
        Kinova::Api::RouterMQTT* m_router;  	 // Router for publishing session status updates
        int m_currentSessionId{0};          	 // Counter for generating unique session IDs
        std::map<int, tSessionInfo> m_sessions;  // Map of active sessions by session ID
};

#endif