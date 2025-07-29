#ifndef _I_ROUTER_CLIENT_H_
#define _I_ROUTER_CLIENT_H_


#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"

#include "ITransportClient.h"
#include "KError.h"

namespace Kinova
{
namespace Api
{
    // Callback type for message handling
    typedef std::function<void (const Kinova::Api::Frame&)> MessageCallback;

    // Options for router client send operations
    typedef struct
    {
        bool andForget;         // If true, don't wait for response
        uint32_t delay_ms;      // Delay before sending in milliseconds
        uint32_t timeout_ms;    // Timeout for response in milliseconds

    } RouterClientSendOptions;

    /**
     * Router client interface for Kortex API communication
     *
     * This interface defines the high-level routing layer that manages
     * message passing, notification callbacks, and session handling between
     * the client application and Kortex services.
     *
     * @note This is an internal interface - users typically interact through
     *       higher-level service clients (ClientService and derived classes)
     */
    class IRouterClient
    {
    public:
        // Virtual destructor
        virtual ~IRouterClient() {}

        // Reset the router to initial state
        virtual void reset() = 0;

        /**
         * Register callback for bridging frames between routers
         *
         * @param[in] bridgingCallback Callback to invoke for bridged frames
         */
        virtual void registerBridgingCallback(std::function<void (Frame &)> bridgingCallback) = 0;

        /**
         * Register callback for service notifications
         *
         * @param[in] serviceId Service identifier to listen for
         * @param[in] callback Callback to invoke when notification arrives
         * @param[in] ns Optional namespace (default: empty)
         * @return Callback identifier for unregistering
         */
        virtual uint32_t registerNotificationCallback( uint32_t serviceId, std::function<Error (Frame&)>, const std::string& ns = "") = 0;

        /**
         * Unregister a notification callback
         *
         * @param[in] callbackId Callback identifier from registerNotificationCallback
         * @param[in] serviceId Service identifier
         * @param[in] ns Optional namespace (default: empty)
         */
        virtual void unregisterNotificationCallback( uint32_t callbackId, uint32_t serviceId, const std::string& ns = std::string{}) = 0;

        /**
         * Register to receive notifications for a specific function
         *
         * @param[in] serviceId Service identifier
         * @param[in] functionId Function identifier to listen for
         * @param[in] ns Optional namespace (default: empty)
         */
        virtual void registerNotificationFunctionId(uint32_t serviceId, uint32_t functionId, const std::string& ns = std::string{}) = 0;

        /**
         * Unregister from notifications for a specific function
         *
         * @param[in] serviceId Service identifier
         * @param[in] functionId Function identifier to stop listening for
         * @param[in] ns Optional namespace (default: empty)
         */
        virtual void unregisterNotificationFunctionId(uint32_t serviceId, uint32_t functionId, const std::string& ns = std::string{}) = 0;

        /**
         * Register callback for error handling
         *
         * @param[in] callback Callback to invoke when errors occur
         */
        virtual void registerErrorCallback( std::function<void(KError)> callback ) = 0;

        /**
         * Register callback for session hit events
         *
         * @param[in] hitSessionCallback Callback to invoke for session events
         */
        virtual void registerHitCallback(std::function<void (FrameTypes)> hitSessionCallback) = 0;

        /**
         * Send a message and get a future for the response
         *
         * @param[in] txPayload Serialized message payload to send
         * @param[in] serviceVersion Version of the service
         * @param[in] funcId Function identifier
         * @param[in] deviceId Device identifier
         * @param[in] options Send options (timeout, delay, fire-and-forget)
         * @param[in] ns Optional namespace (default: empty)
         * @return Future that will contain the response frame
         */
        virtual std::future<Frame> send(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, const RouterClientSendOptions& options, const std::string& ns = "") = 0;

        /**
         * Send a message with callback for response handling
         *
         * @param[in] txPayload Serialized message payload to send
         * @param[in] serviceVersion Version of the service
         * @param[in] funcId Function identifier
         * @param[in] deviceId Device identifier
         * @param[in] callback Callback to invoke when response arrives
         * @param[in] ns Optional namespace (default: empty)
         * @return Error status of the send operation
         */
        virtual Error sendWithCallback(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, MessageCallback callback, const std::string& ns = "") = 0;

        /**
         * Send a pre-constructed message frame
         *
         * @param[in] msgFrame Frame to send
         * @return Error status of the send operation
         */
        virtual Error sendMsgFrame(const Frame& msgFrame) = 0;

        /**
         * Get current connection/session identifier
         *
         * @return Connection ID (0 if not connected)
         */
        virtual uint16_t getConnectionId() const = 0;

        /**
         * Set router activation status
         *
         * @param[in] isActive True to activate, false to deactivate
         */
        virtual void SetActivationStatus(bool isActive) = 0;

        /**
         * Get underlying transport client
         *
         * @return Pointer to transport client
         */
        virtual ITransportClient* getTransport() = 0;
    };

} // namespace Api
} // namespace Kinova

#endif // _I_ROUTER_CLIENT_H_
