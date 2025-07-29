#ifndef IROUTER_CLIENT_SERVER_MQTT
#define IROUTER_CLIENT_SERVER_MQTT

#include "IRouterClientServer.h"

#include "ITransportMQTT.h"

// Protobuf includes
#include "Frame.pb.h"
#include "Errors.pb.h"

#include "CoreBenchmarker.h"

// Standard includes
#include <functional>
#include <future>
#include <memory>
#include <map>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <chrono>

// forward-declarations
namespace mqtt
{
    class client;
    class message;
}


namespace Kinova
{
namespace Api
{

/**
 * Tuple to identify a notification callback (client-side) with the service ID and the namespace
 * Used to make sure notifications for the same service ID but different namespaces are tracked correctly
 */
using ServiceIdWithNamespace = 
    std::tuple<
        uint32_t,   // service Id
        std::string // namespace
>;

/**
 * Service callback type used to intercept a service request
 * The function is as follows:
 *
 * @param[in] sessionInfo
 *
 * @param[in] deviceId
 *
 * @param[in] functionId
 *
 * @param[in] messageId
 *
 * @param[in/out] payload
 *
 * @post <payload> will be used for the response payload, and (typically)
 *       IRouterServer::send() will be called with the response payload 
 *
 * @return The return code of the callback
 *         Zero for success
 *         A non-zero error code for errors
 */
using ServiceCallbackFunction = int(const tSessionInfo&,int,int,int,std::string*);

/**
 * Notification callback function type to intercept a notification
 * The function is as follows:
 *
 * @param[in] frame The message Frame representing the notification
 *
 * @return An error message representing the potential error
 *         Error::error_code() will be ErrorCodes::ERROR_NONE in case of success,
 *         otherwise it will describe the error
 */
using NotificationCallbackFunction = Error(Kinova::Api::Frame&);


// Map type of session infos associated by session id
using SessionInfosMapBySessionId = std::unordered_map<uint32_t, tSessionInfo>;

/**
 * Map type of service callbacks associated by service ids
 *
 * @note Only one service callback can be registered per service id
 */
using ServiceCallbackRoutingMapByServiceId = 
    std::unordered_map<uint32_t, std::function<ServiceCallbackFunction>>;

/**
 * Map type of notification callbacks associated by service ids
 *
 * @note One callback can be registered per (service ID, namespace) tuple
 */
using NotificationCallbacksMapByServiceIdAndNamespace = 
    std::map<ServiceIdWithNamespace, std::map<uint32_t, std::function<NotificationCallbackFunction>>>;

// Map promises for requests associated by response ids
using RequestPromisesMapByResponseId = 
    std::map<ResponseIdentifier, std::promise<Kinova::Api::Frame>>;

// Map callbacks for requests associated by response ids
using RequestCallbacksMapByResponseId = 
    std::map<ResponseIdentifier, MessageCallback>;

/**
 * Map custom client subscriptions callbacks (registered with subscribe) by topic name
 *
 * @note Multiple callbacks can be registered for each topic
 */
using CustomSubscriptionsCallbacksMapByTopicName = 
    std::unordered_map<std::string, std::vector<MessageCallback>>;

// Map type of response topics associated by response identifier
using ResponseTopicByResponseIdentifier = std::map<ResponseIdentifier, std::string>;

// Map type of std::future's for async request handlers and request message frame associated by response identifier
using RequestAsyncHandlersFuturesByResponseIdentifier = 
    std::map<ResponseIdentifier, std::pair<std::future<int>, std::shared_ptr<std::string>>>;

// Router using the Mqtt protocol for supplying both client-side and server-side services
class RouterMQTT final : public IRouterClientServer
{
private:

    ServiceCallbackRoutingMapByServiceId            m_mapServiceRouting;
    std::unordered_map<uint32_t, std::string>       m_mapServiceNamespaces;
    NotificationCallbacksMapByServiceIdAndNamespace m_notificationCallbacks;
    uint32_t                                        m_notificationCallbackId;
    std::function<void(KError)>                     m_errorCallback;
    std::function<void(FrameTypes)>                 m_hitSessionCallback;
    std::function<void(const tSessionInfo &)>       m_pongCallback;
    std::function<void(const tSessionInfo &)>       m_sessionClosedCallback;
    SessionInfosMapBySessionId                      m_mapSessions;
    CustomSubscriptionsCallbacksMapByTopicName      m_mapSubscriptionsCallbacks;
    ResponseTopicByResponseIdentifier               m_mapBridgedResponseTopics;
    std::atomic<int>                                m_lastNotificationId;
    
    bool m_enableAsyncRequestProcessing;
    RequestAsyncHandlersFuturesByResponseIdentifier m_mapOngoingAsyncRequestHandlers;
    std::mutex m_asyncRequestsMapMutex;
    std::chrono::time_point<std::chrono::steady_clock> m_lastAsyncFuturesCleanupTimepoint;

    // Server-side bridging callback
    std::function<void(uint8_t, Kinova::Api::Frame&)>  m_bridgingCallback;

    uint32_t                                    m_sessionId;

    // List of promised responses
    RequestPromisesMapByResponseId              m_requestPromises;
    // List of responses callbacks
    RequestCallbacksMapByResponseId             m_requestCallbacks;

    // Connection information
    std::string m_ip_address;
    int m_port;

    // Benchmarker object used to benchmark API calls.
    std::shared_ptr<Kinova::Api::Util::Benchmarking::CoreBenchmarker> m_benchmarker;

    /**
     * @note There is a known issue with the destruction of the Transport
     *       class under certain circumstances. A temporary fix is to 
     *       prioritise the destruction of the transport before other
     *       members. Until this issue is resolved, it should be ensured
     *       that the Transport REMAINS THE FIRST MEMBER TO BE DESTRUCTED.
     * @see  Story KOR-5235 (https://jira.kinovaapps.com/browse/KOR-5235)
     *       for information about this issue and its fix.
     */
    std::shared_ptr<ITransportMQTT>             m_transport;

public:

    /**
     * Constructor for RouterMQTT with network parameters
     *
     * @param[in] ipAddress IP address of the MQTT broker
     * @param[in] port Port number of the MQTT broker
     * @param[in] clientId Optional MQTT client identifier (if empty, broker will assign one)
     * @param[in] callback Optional error callback function to handle errors
     */
    RouterMQTT(const std::string& ipAddress, unsigned int port, const std::string& clientId = std::string{}, std::function<void(Kinova::Api::KError)> callback = nullptr);

    /**
     * Constructor for RouterMQTT with existing transport
     *
     * @param[in] transport Shared pointer to an existing MQTT transport instance
     * @param[in] callback Optional error callback function to handle errors
     */
    RouterMQTT(std::shared_ptr<Kinova::Api::ITransportMQTT> transport, std::function<void(Kinova::Api::KError)> callback = nullptr);

    /**
     * Destructor - cleans up MQTT connections and resources
     */
    ~RouterMQTT() noexcept;

    /**
     * Starts the incoming message processing thread for this router
     * This will start a thread that will continuously try to consume the
     * next received mqtt message and call RouterMQTT::ProcessReceive 
     * with it
     *
     * @param[in] delay Period (in milliseconds) for the thread to look for messages
     * @post m_processSpinThread will be set to a joinable thread
     *       if m_processSpinThread is already joinable, nothing
     *       happens
     */
    void SpinProcess(const std::chrono::milliseconds& delay = DEFAULT_MQTT_SPIN_PROCESS_DELAY);

    /**
     * Stops the incoming message processing thread for this router
     *
     * @post m_processSpinThread will be stopped
     *       if m_processSpinThread is already stopped, nothing
     *       happens
     */
    void UnspinProcess();

    /**
     * Getter for the boolean flag indicating if async request processing is enabled
     *
     * @see `SetAsyncRequestProcessing` documentation
     */
    bool GetAsyncRequestProcessing() const { return m_enableAsyncRequestProcessing;}

    /**
     * Setter for the boolean flag indicating if async request processing is enabled.
     * This setter is useful for users running the router message processing thread themselves
     * who do not want threads created on the fly. It is enabled by default.
     *
     * @param[in] New boolean value
     */
    void SetAsyncRequestProcessing(bool enable_async_request_processing) { m_enableAsyncRequestProcessing = enable_async_request_processing;}

    // ***** IRouterClient interface *****

    /**
     * Reset the router to its initial state
     *
     * @post All registered callbacks and sessions are cleared
     */
    void reset();

    /**
     * Register a callback for client-side bridging operations
     *
     * @param[in] bridgingCallback Function to be called when a bridged frame is received
     */
    void registerBridgingCallback(std::function<void (Kinova::Api::Frame &)> bridgingCallback);

    /**
     * Register a callback to receive notifications for a specific service
     *
     * @param[in] serviceId Service ID to receive notifications from
     * @param[in] callback Function to be called when a notification is received
     * @param[in] ns Optional namespace to prepend to the MQTT notification topic
     * @return Unique callback identifier that can be used to unregister the callback
     */
    uint32_t registerNotificationCallback( uint32_t serviceId, std::function<Error (Kinova::Api::Frame&)>, const std::string& ns = "") override;

    /**
     * Unregister a previously registered notification callback
     *
     * @param[in] callbackId The callback identifier returned by registerNotificationCallback
     * @param[in] serviceId Service ID of the notification callback
     * @param[in] ns Optional namespace that was used when registering the callback
     */
    void unregisterNotificationCallback( uint32_t callbackId, uint32_t serviceId, const std::string& ns) override;

    /**
     * @param[in] serviceId Service ID of the notification to subscribe to
     * @param[in] functionId Function ID of the notification to subscribe to
     * @param[in] ns Namespace to prepend the MQTT request topic with
     * @note This function does nothing if registerNotificationCallback has not been called prior with serviceId
     * @post The Router will subscribe to the "notif/<serviceId>/<functionId>"
     */
    void registerNotificationFunctionId(uint32_t serviceId, uint32_t functionId, const std::string& ns = "");

    /**
     * @param[in] serviceId Service ID of the notification to unsubscribe to
     * @param[in] functionId Function ID of the notification to unsubscribe to
     * @param[in] ns Namespace to prepend the MQTT request topic with
     * @post The Router will unsubscribe to the "notif/<serviceId>/<functionId>"
     */
    void unregisterNotificationFunctionId(uint32_t serviceId, uint32_t functionId, const std::string& ns = "");

    /**
     * Register a callback to handle errors that occur in the router
     *
     * @param[in] callback Function to be called when an error occurs
     */
    void registerErrorCallback(std::function<void(Kinova::Api::KError)> callback );

    /**
     * Register a callback to be notified when specific frame types are received
     *
     * @param[in] hitSessionCallback Function to be called with the frame type
     */
    void registerHitCallback(std::function<void (Kinova::Api::FrameTypes)> hitSessionCallback);

    /**
     * Send a request message and return a future to the response
     *
     * @param[in] txPayload Serialized payload to send
     * @param[in] serviceVersion Version of the service
     * @param[in] funcId Function identifier
     * @param[in] deviceId Target device identifier
     * @param[in] options Send options (timeout, etc.)
     * @param[in] ns Optional namespace to prepend to the MQTT request topic
     * @return Future that will contain the response frame when it arrives
     */
    std::future<Kinova::Api::Frame> send(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, const RouterClientSendOptions& options, const std::string& ns = "");

    /**
     * Send a request message with an asynchronous callback for the response
     *
     * @param[in] txPayload Serialized payload to send
     * @param[in] serviceVersion Version of the service
     * @param[in] funcId Function identifier
     * @param[in] deviceId Target device identifier
     * @param[in] callback Function to be called when the response arrives
     * @param[in] ns Optional namespace to prepend to the MQTT request topic
     * @return Error status indicating success or failure of sending
     */
    Error sendWithCallback(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, MessageCallback callback, const std::string& ns = "");

    /**
     * Send a pre-constructed message frame
     *
     * @param[in] msgFrame The message frame to send
     * @return Error status indicating success or failure of sending
     */
    Error sendMsgFrame(const Kinova::Api::Frame& msgFrame);

    /**
     * Get the connection identifier for this router instance
     *
     * @return Connection ID
     */
    uint16_t getConnectionId() const;

    /**
     * Set the activation status of the router
     *
     * @param[in] isActive True to activate, false to deactivate
     */
    void SetActivationStatus(bool isActive);

    /**
     * Get the underlying transport client
     *
     * @return Pointer to the transport client
     */
    ITransportClient* getTransport();

    // ***** IRouterServer interface *****

    /**
     * Process incoming MQTT messages and dispatch them to appropriate handlers
     *
     * @see IRouterServer::ProcessReceive
     */
    void ProcessReceive();

    /**
     * Process connection health checks and handle inactive connections
     *
     * @see IRouterServer::ProcessConnectionsSanity
     */
    void ProcessConnectionsSanity();

    /**
     * Register a callback for server-side bridging operations
     *
     * @param[in] bridgingCallback Function to be called with device ID and frame when bridging occurs
     * @see IRouterServer::registerBridgingCallback
     */
    void registerBridgingCallback(std::function<void (uint8_t, Kinova::Api::Frame &)> bridgingCallback);

    /**
     * Register a service with its callback to handle incoming requests
     *
     * @param[in] serviceId Unique service identifier
     * @param[in] onMessageCb Callback function to handle service requests
     * @param[in] ns Optional namespace for MQTT topics
     * @return True if registration was successful, false otherwise
     * @see IRouterServer::RegisterService
     */
    bool RegisterService(uint32_t serviceId, std::function<int(const tSessionInfo&,int,int,int,std::string*)> onMessageCb, const std::string& ns = "");

    /**
     * Send a response or notification to a client
     *
     * @param[in] sessionInfo Session information of the target client
     * @param[in] frameType Type of frame to send (response or notification)
     * @param[in] returnStatus Status code and error information
     * @param[in] pStrPayload Pointer to the payload string
     * @param[in] deviceId Device identifier
     * @param[in] serviceVersion Service version
     * @param[in] serviceId Service identifier
     * @param[in] functionId Function identifier
     * @param[in] messageId Message identifier
     * @see IRouterServer::Send
     */
    void Send(const tSessionInfo &sessionInfo, Kinova::Api::FrameTypes frameType, const ReturnStatus& returnStatus, std::string* pStrPayload, int deviceId, int serviceVersion, int serviceId, int functionId, int messageId );

    /**
     * Register a cleanup callback to be called when a session is closed
     *
     * @param[in] serviceId Service identifier
     * @param[in] functionCb Callback function to execute on session cleanup
     * @see IRouterServer::RegisterCleanupNotification
     */
    void RegisterCleanupNotification(uint32_t serviceId, std::function<void(const tSessionInfo &sessionInfo)> functionCb);

    /**
     * Register a callback to handle server-side errors
     *
     * @param[in] callback Function to be called when an error occurs
     * @see IRouterServer::RegisterErrorCallback
     */
    void RegisterErrorCallback(std::function<void(Kinova::Api::KError)> callback);

    /**
     * Register a callback to be notified when a connection becomes inactive
     *
     * @param[in] inactivityCallback Function to be called with session info when inactivity is detected
     * @see IRouterServer::RegisterConnectionsInactivityCallback
     */
    void RegisterConnectionsInactivityCallback(std::function<void(const tSessionInfo& sessionInfo)> inactivityCallback);

    /**
     * Register a callback to be notified when a user session becomes inactive
     *
     * @param[in] sessionInactivityCallback Function to be called with session info when session inactivity is detected
     * @see IRouterServer::RegisterSessionInactivityCallback
     */
    void RegisterSessionInactivityCallback(std::function<void(const tSessionInfo& sessionInfo)> sessionInactivityCallback);

    /**
     * Update the information for an existing user session
     *
     * @param[in] sessionInfo Updated session information
     * @see IRouterServer::UpdateUserSessionInfo
     */
    void UpdateUserSessionInfo(const tSessionInfo &sessionInfo);

    /**
     * Remove and cleanup a session based on its connection handle
     *
     * @param[in] connectionHandle Handle identifying the connection to remove
     * @see IRouterServer::RemoveSessionInfo
     */
    void RemoveSessionInfo( const ClientConnectionHandle& connectionHandle);

    /**
     * Get a copy of all active connections mapped by session ID
     *
     * @return Map of session IDs to session information
     * @see IRouterServer::GetConnectionsMap
     */
    SessionInfosMapBySessionId GetConnectionsMap() const noexcept;

    /**
     * Get the next available notification identifier
     *
     * @return Unique notification ID
     * @see IRouterServer::GetNextNotificationId
     */
    uint16_t GetNextNotificationId();

    /**
     * Send a notification to a specific client connection
     *
     * @param[in] connectionHandle Connection handle of the target client
     * @param[in] notifId Notification identifier
     * @param[in] pStrOut Pointer to the notification payload
     * @param[in] serviceVersion Service version
     * @param[in] serviceId Service identifier
     * @param[in] functionId Function identifier
     * @see IRouterServer::Notify
     */
    void Notify(ClientConnectionHandle &connectionHandle, int notifId, std::string* pStrOut, int serviceVersion, int serviceId, int functionId);

    /**
     * Send a bridged message frame back to the originating client
     *
     * @param[in] msgFrame The message frame to send back
     * @see IRouterServer::sendBridgedMsgFrame
     */
    void sendBridgedMsgFrame(const Kinova::Api::Frame &msgFrame);
    

    /**
     * Method to register a callback to execute whenever a PONG message comes in
     *
     * @param[in] callback The function to execute when the PONG comes in, which returns void and takes a Frame as input
     * @post The router will subscribe to the 'pong/#' topic, receiving all PONG's in the network
     */
    void registerPongCallback(std::function<void (const tSessionInfo&)> callback);

    /**
     * Method to register a callback to execute whenever a session is closed on the network
     *
     * @param[in] callback The function to execute when a session is closed, which returns void and takes a tSessionInfo as input (the closed session)
     */
    void registerSessionClosedCallback(std::function<void(const tSessionInfo& sessionInfo)> callback);

    // ***** PUB/SUB/UNSUB-related functions *****

    /**
     * Method to publish a custom message (encoded to string) on the MQTT network
     *
     * @param[in] topic Topic on which to send the payload
     * @param[in] payload Payload to send
     * @param[in] is_mqtt_raw Boolean value to indicate if payload must be put in a Kortex Frame (false, default) or not (true)
     *                        False should be used when publishing to other Kortex routers, true should be used when publishing to pure MQTT clients
     * @return True if publication was successful, false otherwise
     */
    bool publish(const std::string& topic, const std::string& payload, bool is_mqtt_raw = false);

    /**
     * Method to subscribe to a given topic on the MQTT network
     *
     * @param[in] topic Topic on which to subscribe
     * @param[in] callback Callback function to register for the given topic, which returns void and takes a Kortex Frame as input
     * @note Some topics are reserved for internal usage
     * @note Subscription to wildcards is not supported for the moment (the topic cannot contain '+' or '#')
     * @return True if subscription was successful, false otherwise
     */
    bool subscribe(const std::string& topic, const MessageCallback& callback);

    /**
     * Method to unsubsribe all callbacks from a given topic on the MQTT network
     *
     * @param[in] topic Topic on which to subscribe
     * @param[in] callback Callback function to register for the given topic
     * @warning This unsubscribes ALL callbacks registered for this topic
     */
    void unsubscribe(const std::string& topic);

    /**
     * Method to send a ping to client participants
     *
     * @param[in] sessionId Session ID of the client we want to ping
     * @post All clients which already created a session will receive a PING message and answer with a PONG message 
     */
    void sendPing(uint32_t sessionId) const;

    /**
     * Get the MQTT client identifier for this router instance
     *
     * @return The client's ID as string on the MQTT network
     * @note If it wasn't supplied when creating the Router, the client ID will be assigned by the broker
     */
    std::string getMQTTClientId() const;

    /**
     * Getter for MQTT broker address
     *
     * @return The broker address
     */
    std::string GetBrokerAddress() const {return m_ip_address;}

    /**
     * Getter for MQTT port
     *
     * @return The port
     */
    int GetBrokerPort() const {return m_port;}

    /**
     * Activate or deactivate performance benchmarking for router and transport layers
     *
     * @param[in] activateRouter True to enable benchmarking on the router layer
     * @param[in] activateTransport True to enable benchmarking on the transport layer
     */
    void activateBenchmarker(bool activateRouter, bool activateTransport);

private:

    // Sending methods

    /**
     * Send a REQUEST message to the MQTT network
     *
     * @param[in] frame The frame to send
     * @param[in] ns Namespace to prepend the MQTT request topic with 
     * @return A future<Kinova::Api::Frame> which will be set to the response frame when it arrives
     * @note This is the version which outputs a future. There is another overload which registers an async calllback.
     */
    std::future<Kinova::Api::Frame> sendRequest(const Kinova::Api::Frame& frame, const std::string& ns = "");

    /**
     * Send a REQUEST message to the MQTT network
     *
     * @param[in] frame The frame to send
     * @param[in] callback The function to execute when the response frame comes back. It returns void and takes the response frame as input.
     * @param[in] ns Namespace to prepend the MQTT request topic with
     * @return A Kinova::Api::Error which reports any error related to sending the message
     * @note This is the version which registers an async callback. There is another overload which outputs a future to the response frame.
     */
    Kinova::Api::Error sendRequest(const Kinova::Api::Frame& frame, const MessageCallback& callback, const std::string& ns = "");

    /**
     * Send a RESPONSE message to the MQTT network
     *
     * @param[in] frame The frame to send
     * @param[in] topic The response topic to send the response to 
     */
    void sendResponse(const Kinova::Api::Frame& frame, const std::string& topic);
    
    /**
     * Send a NOTIFICATION message to the MQTT network
     *
     * @param[in] frame The frame to send
     */
    void sendNotification(const Kinova::Api::Frame& frame);
    
    /**
     * Send a PONG message to the MQTT network
     *
     * @note This is called right after a PING frame was received
     */
    void sendPong();

    // Receiving methods

    /**
     * Process an incoming Frame
     *
     * @param[in] frame The received frame
     * @param[in] notif_namespace The namespace on which the message arrived (only filled for NOTIFICATION message type)
     * @param[in] response_topic The optional response topic bundled with the message
     */
    void onMessage(Kinova::Api::Frame const& frame, const std::string& notif_namespace, const std::string& response_topic);

    /**
     * Process an incoming REQUEST Frame
     *
     * @param[in] frame The received frame
     * @param[in] clientId The clientId who sent the message
     * @note This will call the service callback in <m_mapServiceRouting> associated to the message's service ID
     */
    void onRequest(Kinova::Api::Frame const& frame, const std::string& clientId);

    /**
     * Process an incoming RESPONSE Frame
     *
     * @param[in] frame The received frame
     * @note This will find the promised response in <m_requestPromises> associated to this message and set it
     */
    void onResponse(Kinova::Api::Frame const& frame);

    /**
     * Process an incoming NOTIFICATION Frame
     *
     * @param[in] topic The namespace on which it was received
     * @param[in] frame The received frame
     * @note This will call the notification callback in <m_notificationCallbacks> associated to the service ID and namespace prepending "notif/" in the topic
     */
    void onNotification(const std::string& ns, Kinova::Api::Frame const& frame);

    /**
     * Process an incoming PING Frame
     *
     * @param[in] frame The received frame
     * @post This will register a hit of PING type and send back a PONG message
     */
    void onPing(Kinova::Api::Frame const& frame);

    /**
     * Process an incoming PONG Frame
     *
     * @param[in] frame The received frame
     * @post This will call <m_pongCallback> if it has been set
     */
    void onPong(Kinova::Api::Frame const& frame);

    /**
     * Process an incoming RAW Frame
     *
     * @param[in] frame The received frame
     * @note This will call the callback(s) registered by the user for this topic
     */
    void onRaw(Kinova::Api::Frame const& frame);

    /**
     * Generate a CloseSession Frame for this client
     *
     * @return The REQUEST frame with header and payload correctly set to trigger a CloseSession
     * @note This is called to prepare a CloseSession in the destructor
     */
    Kinova::Api::Frame generateCloseSessionFrame() const noexcept;

    /**
     * Generate a LWT RAW Frame for this client
     *
     * @return The RAW frame with header and payload correctly set to flag the session has closed unexpectedly
     * @note This is called to prepare the Last Will Testament when reconnecting
     */
    Kinova::Api::Frame generateLWTFrame() const;

    // Cleanup method to remove the done futures from the async requests map
    void cleanupAsyncRequestsFutures(bool wait) noexcept;

    // Error handling methods

    /**
     * Set exception in message promise (client side)
     *
     * @param[in] responseId The responseId of the message
     * @param[in] topic The HeaderInfo of the message
     * @param[in] topic The error payload of the message
     */
    void setExceptionOnPromise(const Kinova::Api::ResponseIdentifier& responseId, const HeaderInfo& headerInfo, const Kinova::Api::Error& error);

    std::mutex m_requestPromisesMapLock;
};

} // namespace Api
} // namespace Kinova

#endif //IROUTER_CLIENT_SERVER_MQTT