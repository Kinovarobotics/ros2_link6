/* ***************************************************************************
 * Kinova inc.
 * Project : 
 *
 * Copyright (c) 2006-2017 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef _KORTEXAPICPP_IROUTERSERVER_H_
#define _KORTEXAPICPP_IROUTERSERVER_H_

// Kortex API includes
#include "ITransportServer.h" // for ClientConnectionHandle
#include "KError.h"

// Protobuf includes
#include <google/protobuf/util/message_differencer.h>
#include "Frame.pb.h"
#include "Common.pb.h"
#include "Errors.pb.h"

// Standard includes
#include <functional>
#include <unordered_map>
#include <string>
#include <chrono>
#include <atomic>
#include <type_traits>

using ChronoClock = typename std::conditional<
    std::chrono::high_resolution_clock::is_steady,
    std::chrono::high_resolution_clock,
    std::chrono::steady_clock
>::type;

/**
 * Return status structure that wraps error information for API operations
 *
 * This structure encapsulates Kortex API error codes and provides utilities
 * for creating error frames and checking success status.
 */
struct ReturnStatus
{
    Kinova::Api::Error  m_error;

    /**
     * Constructor from existing Error object
     *
     * @param[in] error Pre-constructed error object to wrap
     */
    ReturnStatus(Kinova::Api::Error error)
    {
        m_error = error;
    }

    /**
     * Constructor with error codes and optional message
     *
     * @param[in] errorCode Main error code
     * @param[in] errorSubCode Sub-error code for additional detail
     * @param[in] errorSubString Optional human-readable error description
     */
    ReturnStatus(Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorSubString = {})
    {
        setError(errorCode, errorSubCode, errorSubString);
    }

    /**
     * Default constructor - initializes with no error (success status)
     */
    ReturnStatus()
    {
        setError(Kinova::Api::ErrorCodes::ERROR_NONE, Kinova::Api::SubErrorCodes::SUB_ERROR_NONE);
    }

    /**
     * Destructor
     */
    ~ReturnStatus()
    {}

    /**
     * Get the wrapped error object
     *
     * @return Rvalue reference to the error object (moves ownership)
     */
    Kinova::Api::Error&& getError()
    {
        return std::move(m_error);
    }

    /**
     * Set the error information
     *
     * @param[in] errorCode Main error code
     * @param[in] errorSubCode Sub-error code for additional detail
     * @param[in] errorSubString Optional human-readable error description
     */
    void setError(Kinova::Api::ErrorCodes errorCode, uint16_t errorSubCode, std::string errorSubString = {})
    {
        m_error.set_error_code(errorCode);
        m_error.set_error_sub_code(errorSubCode);
        m_error.set_error_sub_string(errorSubString);
    }

    /**
     * Check if the operation was successful (no error)
     *
     * @return True if error code is ERROR_NONE, false otherwise
     */
    bool isSuccessful() const
    {
        return (m_error.error_code() == Kinova::Api::ErrorCodes::ERROR_NONE);
    }

    /**
     * Get the combined error status as a 16-bit value
     *
     * @return Packed error status (error code in upper 4 bits, sub-code in lower 12 bits)
     */
    uint16_t getErrorStatus() const
    {
        return (m_error.error_sub_code()  & 0x0FFF) | ((m_error.error_code()  << 12) & 0xF000);
    }

    /**
     * Create an error message frame from this return status
     *
     * @param[in] refHeader Reference header to copy basic information from
     * @return A message frame containing the error information as payload
     */
    Kinova::Api::Frame createErrorMsgFrame( const Kinova::Api::Header& refHeader )
    {
        Kinova::Api::Frame errorMsgFrame;

        std::string serializedError;
        m_error.SerializeToString(&serializedError);

        auto header = errorMsgFrame.mutable_header();
        header->CopyFrom( refHeader );

        uint32_t DeviceIdMask = refHeader.frame_info() & 0x00FF0000;
        uint32_t frameInfo = ((Kinova::Api::HeaderVersion::CURRENT_VERSION << 28) & 0xF0000000) |
                             ((Kinova::Api::FrameTypes::MSG_FRAME_RESPONSE << 24) & 0x0F000000) |
                             DeviceIdMask | (getErrorStatus() & 0x0000FFFF);
        header->set_frame_info(frameInfo);

        uint32_t payloadInfo = ((0 << 24)           & 0xFF000000) |
                               ((serializedError.size()) & 0x00FFFFFF);
        header->set_payload_info(payloadInfo);

        errorMsgFrame.set_payload(serializedError);

        return errorMsgFrame;
    }

    /**
     * Equality operator overload
     *
     * @param[in] other Other structure to compare with current
     * @return True if the 2 structures are equal, false otherwise
     */
    bool operator==(const ReturnStatus& other)
    {
        return google::protobuf::util::MessageDifferencer::Equals(m_error,  other.m_error);
    }

    /**
     * Non-equality operator overload
     *
     * @param[in] other Other structure to compare with current
     * @return True if the 2 structures are not equal, false otherwise
     */
    bool operator!=(const ReturnStatus& other)
    {
        return !(*this == other);
    }
};

struct tSessionInfo
{
    //
    // Owned by the Server App (read-write)
    //
    uint32_t    userId;
    bool        isLoggedIn;
    bool        active; // true until a client connection is considered inactive (inactivity timeout)
    int         inactivityTimeout_ms;           // session inactivity time before the session times out and closes on its own
    uint32_t    connectionInactivityTimeout_ms;

    //
    // Owned by the API (read-only outside)
    //
    /**
     * @note This has been made 'mutable' in order to be able to modify it
     * during a service call (specifically Kinova.Api.Session.CreateSession
     * and Kinova.Api.Session.CloseSession)
     */
    mutable int         sessionId;

    // should be encapsulated and not accessible via the sessionInfo expose outside api
    ClientConnectionHandle  connectionHandle;
    ChronoClock::time_point lastActivityTimestamp;
    ChronoClock::time_point lastConnectionInactivityCallbackTimestamp;

    // Roles associated to this session
    std::vector<Kinova::Api::Common::UserRole> roles;

    /**
     * Equality operator overload
     *
     * @param[in] other Other structure to compare with current
     * @return True if the 2 structures are equal, false otherwise
     */
    bool operator==(const tSessionInfo& other)
    {
        return userId == other.userId &&
               isLoggedIn == other.isLoggedIn &&
               active == other.active &&
               inactivityTimeout_ms == other.inactivityTimeout_ms &&
               connectionInactivityTimeout_ms == other.connectionInactivityTimeout_ms &&
               sessionId == other.sessionId &&
               connectionHandle == other.connectionHandle;
    }

    /**
     * Non-equality operator overload
     *
     * @param[in] other Other structure to compare with current
     * @return True if the 2 structures are not equal, false otherwise
     */
    bool operator!=(const tSessionInfo& other)
    {
        return !(*this == other);
    }

    /**
     * Helper method to get the unique permissions associated to this session's roles
     *
     * @return Set of unique permissions this session has
     */
    std::set<Kinova::Api::Common::UserPermission> GetPermissions() const
    {
        std::set<Kinova::Api::Common::UserPermission> permissions;
        for (auto role : roles)
        {
            for (int i = 0; i < role.permissions_size(); i++)
            {
                permissions.insert(role.permissions(i));
            }
        }
        return permissions;
    }

};


/**
 * Interface for server-side router implementations
 *
 * This interface defines the contract for routers that act as servers,
 * dispatching RPC requests to registered services and managing client sessions.
 */
class IRouterServer
{
public:
    /**
     * Virtual destructor
     */
    virtual ~IRouterServer() noexcept = default;

    /**
     * Process incoming messages from the transport layer
     *
     * @post All pending messages are read from transport and dispatched to appropriate handlers
     */
    virtual void ProcessReceive() = 0;

    /**
     * Check the health of all active connections
     *
     * @post Inactive or timed-out connections are identified and cleaned up
     */
    virtual void ProcessConnectionsSanity() = 0;

    /**
     * Register a bridging callback
     *
     * @param[in] bridgingCallback The callback to set
     */
    virtual void registerBridgingCallback(std::function<void (uint8_t, Kinova::Api::Frame &)> bridgingCallback) = 0;

    /**
     * Register a new service
     *
     * @param[in] serviceId Id of the service
     * @param[in] onMessageCb Callback to call when receiving a message of specified service
     * @param[in] ns Namespace to initialise the Service within
     *            The namespace will be prepended to the MQTT topics the Service would normally subscribe to
     * @post all messages relative to the service <serviceId> will be sent to the callback <onMessageCb>
     * @return True is the service was successfully registered,
     *         False otherwise (if a service was already registered with the id <serviceId> or if the namespace was incorrect)
     */
    virtual bool RegisterService(uint32_t serviceId, std::function<int(const tSessionInfo&,int,int,int,std::string*)> onMessageCb, const std::string& ns = "") = 0;

    /**
     * Send a message through the transport
     *
     * @param[in] sessionInfo Session Id where to send the message
     * @param[in] frameType Frame type to be sent, NOTIF or Answer to request
     * @param[in] returnStatus Results of the answer or notif
     * @param[in] pStrPayload Buffer containing the message to be sent
     * @param[in] deviceId Device Id used for internal routing
     * @param[in] serviceVersion Version of the service
     * @param[in] serviceId Id of the service
     * @param[in] functionId Id of the service function
     * @param[in] messageId Message ID/Notification Id related to the message or notification
     */
    virtual void Send(const tSessionInfo &sessionInfo, Kinova::Api::FrameTypes frameType, const ReturnStatus& returnStatus, std::string* pStrPayload, int deviceId, int serviceVersion, int serviceId, int functionId, int messageId ) = 0;
    
    /**
     * Register a callback for cleanup notifications
     *
     * @param[in] serviceId Id of the service to call
     * @param[in] functionCb callback
     * @post Until this function is called again, callback <functionCb> will be called whenever a Session is ended,
     *       supplied with the removed ClientConnectionHandle 
     */
    virtual void RegisterCleanupNotification(uint32_t serviceId, std::function<void(const tSessionInfo &sessionInfo)> functionCb) = 0;

    /**
     * Register a callback for errors
     *
     * @param[in] callback The callback
     * @pre Can only be called once. Any subsequent call will fail due to an already-existing callback.
     * @post Callback <callback> will be called whenever an error occurs, supplied with the error. 
     */
    virtual void RegisterErrorCallback(std::function<void(Kinova::Api::KError)> callback) = 0;

    /**
     * Register a callback for connection inactivity 
     *
     * @param[in] inactivityCallback
     * @post Until this function is called again, <inactivityCallback> will be called whenever a connection timeouts because of inactivity 
     */
    virtual void RegisterConnectionsInactivityCallback(std::function<void(const tSessionInfo& sessionInfo)> inactivityCallback) = 0;

    /**
     * Register a callback for session inactivity
     *
     * @param[in] sessionInactivityCallback
     * @post Until this function is called again, <sessionInactivityCallback> will be called whenever a session timeouts because of inactivity 
     */
    virtual void RegisterSessionInactivityCallback(std::function<void(const tSessionInfo& sessionInfo)> sessionInactivityCallback) = 0;

    /**
     * Modify a Session entry with a new entry
     *
     * @param[in] sessionInfo The new session to store
     * @pre The Router must already contain a session info with the same session id as <sessionInfo>. Otherwise this function does nothing
     * @post The session info for session id <sessionInfo.sessionid> is updated to the values of <sessionInfo>.
     */
    virtual void UpdateUserSessionInfo(const tSessionInfo &sessionInfo) = 0;

    /**
     * Remove a Session entry with a new entry
     *
     * @param[in] connectionHandle The client connection handle for which to remove the session
     * @pre The Router must already contain a session info using the client Id  <connectionHandle.clientId>. 
     *      Otherwise this function does nothing
     * @post The session info for client Id <connectionHandle.clientId> is removed from the Router. 
     *       The session inactivity callbacks are called for <connectionHandle>.
     */
    virtual void RemoveSessionInfo( const ClientConnectionHandle& connectionHandle) = 0;

    /**
     * Return the map of all connections of the Router
     *
     * @return A copy of the map of session infos, mapped by their client Id.
     */
    virtual std::unordered_map<uint32_t, tSessionInfo> GetConnectionsMap() const noexcept = 0;

    /**
     * Return the next (unique) notification Id
     *
     * @return The next available id for notification messages
     * @note This is used to ensure notification messages are routed by-clients, using an id 
     */
    virtual uint16_t GetNextNotificationId() = 0;

    /**
     * Send a notification message to a connection
     *
     * @param[in] connectionHandle Connection to send the notification to
     * @param[in] notifId Unique notification id used for this notification
     * @param[in] serviceVersion Version of the service
     * @param[in] serviceId ID of the service
     * @param[in] functionId Id of the service function
     */
    virtual void Notify(ClientConnectionHandle &connectionHandle, int notifId, std::string* pStrOut, int serviceVersion, int serviceId, int functionId) = 0;

    /**
     * Send a bridged message frame
     *
     * @param[in] msgFrame the message frame to send
     */
    virtual void sendBridgedMsgFrame(const Kinova::Api::Frame &msgFrame) = 0;
};

#endif // _KORTEXAPICPP_IROUTERSERVER_H_