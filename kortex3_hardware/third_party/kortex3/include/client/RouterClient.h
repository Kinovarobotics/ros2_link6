#ifndef _ROUTER_CLIENT_H_
#define _ROUTER_CLIENT_H_


#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"

#include "ITransportClient.h"
#include "IRouterClient.h"

#include "FrameHandler.h"


namespace Kinova
{
namespace Api
{

    typedef std::map< uint32_t, std::function<Error (Frame&)> > NotificationServices;

    /**
     * Client-side router implementation for TCP/UDP transport
     *
     * This class handles message routing, notification management, and communication
     * with the server using TCP or UDP transport protocols.
     */
    class RouterClient : public IRouterClient
    {
	private:
        ITransportClient* const m_transport;

        NotificationServices            m_notificationServices;
        std::function<void(KError)>     m_errorCallback;
        std::function<void(FrameTypes)> m_hitSessionCallback;
        std::function<void(Frame&)>     m_bridgingCallback;

        FrameHandler            m_frameHandler;
        uint16_t                m_msgId;
        uint16_t                m_sessionId;

        //A flag indicating if the instance can be used.
        bool                    m_isActive;

        std::mutex              m_send_mutex;

    public:
        /**
         * Constructor
         *
         * @param[in] transport Pointer to the transport client (TCP or UDP)
         * @param[in] errorCallback Function to be called when errors occur
         */
        RouterClient(ITransportClient* transport, std::function<void (KError)> errorCallback);

        /**
         * Destructor - cleans up resources and closes connections
         */
        virtual ~RouterClient();

        /**
         * Reset the router to its initial state
         *
         * @post All pending requests and callbacks are cleared
         */
        virtual void reset() override;

        /**
         * Register a callback for bridging operations
         *
         * @param[in] bridgingCallback Function to be called when a bridged frame is received
         */
        virtual void registerBridgingCallback(std::function<void (Frame &)> bridgingCallback) override;

        /**
         * Register a callback to receive notifications for a specific service
         *
         * @param[in] serviceId Service ID to receive notifications from
         * @param[in] callback Function to be called when a notification is received
         * @param[in] ns Namespace parameter (unused in TCP/UDP implementation)
         * @return Unique callback identifier that can be used to unregister the callback
         */
        uint32_t registerNotificationCallback( uint32_t serviceId, std::function<Error (Frame&)>, const std::string& ns = "") override;

        /**
         * Unregister a previously registered notification callback
         *
         * @param[in] callbackId The callback identifier returned by registerNotificationCallback
         * @param[in] serviceId Service ID of the notification callback
         * @param[in] ns Namespace parameter (unused in TCP/UDP implementation)
         */
        void unregisterNotificationCallback( uint32_t callbackId, uint32_t serviceId, const std::string& ns) override;
        /**
         * Not implemented in the TCP/UDP RouterClient class because the concept of "per-function" notification
         * does not exist in the implementation
         */
        virtual void registerNotificationFunctionId(uint32_t serviceId, uint32_t functionId, const std::string& ns = "") override {}
        /**
         * Not implemented in the TCP/UDP RouterClient class because the concept of "per-function" notification
         * does not exist in the implementation
         */
        virtual void unregisterNotificationFunctionId(uint32_t serviceId, uint32_t functionId, const std::string& ns = "") override {}

        /**
         * Register a callback to handle errors that occur in the router
         *
         * @param[in] callback Function to be called when an error occurs
         */
        virtual void registerErrorCallback( std::function<void(KError)> callback ) override;

        /**
         * Register a callback to be notified when specific frame types are received
         *
         * @param[in] hitSessionCallback Function to be called with the frame type
         */
        virtual void registerHitCallback(std::function<void (FrameTypes)> hitSessionCallback) override;

        /**
         * Send a request message and return a future to the response
         *
         * @param[in] txPayload Serialized payload to send
         * @param[in] serviceVersion Version of the service
         * @param[in] funcId Function identifier
         * @param[in] deviceId Target device identifier
         * @param[in] options Send options (timeout, etc.)
         * @param[in] ns Namespace parameter (unused in TCP/UDP implementation)
         * @return Future that will contain the response frame when it arrives
         */
        virtual std::future<Frame> send(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, const RouterClientSendOptions& options, const std::string& ns = "") override;

        /**
         * Send a request message with an asynchronous callback for the response
         *
         * @param[in] txPayload Serialized payload to send
         * @param[in] serviceVersion Version of the service
         * @param[in] funcId Function identifier
         * @param[in] deviceId Target device identifier
         * @param[in] callback Function to be called when the response arrives
         * @param[in] ns Namespace parameter (unused in TCP/UDP implementation)
         * @return Error status indicating success or failure of sending
         */
        virtual Error sendWithCallback(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, MessageCallback callback, const std::string& ns = "") override;

        /**
         * Send a pre-constructed message frame
         *
         * @param[in] msgFrame The message frame to send
         * @return Error status indicating success or failure of sending
         */
        virtual Error sendMsgFrame(const Frame& msgFrame) override;

        /**
         * Get the connection identifier for this router instance
         *
         * @return Connection ID (session ID)
         */
        virtual uint16_t getConnectionId() const override;

        /**
         * Set the activation status of the router
         *
         * @param[in] isActive True to activate, false to deactivate
         * @post When deactivated, the router will reject new operations
         */
        virtual void SetActivationStatus(bool isActive) override;

        /**
         * Get the underlying transport client
         *
         * @return Pointer to the transport client
         */
        virtual ITransportClient* getTransport() override {return m_transport;}

    private:
        uint16_t generateNewMsgId();
        void frameHandler(const void *rxBuffer, uint32_t rxSize);
        uint32_t notificationCallbackId = 1;
    };

} // namespace Api
} // namespace Kinova

#endif // _ROUTER_CLIENT_H_
