#ifndef _NOTIFICATION_HANDLER_H_
#define _NOTIFICATION_HANDLER_H_

#include "ITransportClient.h"
#include "IRouterClient.h"

#include "KBasicException.h"
#include "KDetailedException.h"

#include "Frame.pb.h"

#include <memory>
#include <map>
#include <vector>
#include <functional>
#include <future>

namespace Kinova
{
namespace Api
{

    /**
     * Abstract base class for notification callback functions
     *
     * This provides a type-erased interface for callback functions that handle
     * notification messages with different payload types.
     */
    class AbstractCallbackFunction
    {
    public:
        /**
         * Default constructor
         */
        AbstractCallbackFunction() = default;

        /**
         * Virtual destructor
         */
        virtual ~AbstractCallbackFunction() noexcept = default;

        /**
         * Execute the callback with a notification frame
         *
         * @param[in] msgFrameNotif The notification message frame to process
         * @return Error object indicating success or failure of processing
         */
        virtual Error call(Frame& msgFrameNotif) = 0;
    };

    /**
     * Template class for type-safe notification callbacks
     *
     * This class wraps a callback function that takes a specific protobuf message type,
     * handling deserialization and error reporting.
     *
     * @tparam DataType The protobuf message type this callback expects (must inherit from google::protobuf::Message)
     */
    template <class DataType>
    class CallbackFunction : public AbstractCallbackFunction
    {
    private:
        static_assert(
            std::is_base_of<::google::protobuf::Message, DataType>::value,
            "DataType must inherit from ::google::protobuf::Message");

        std::function<void(DataType)> m_callbackFct;

    public:
        /**
         * Constructor
         *
         * @param[in] callback User-provided function to call with deserialized notification data
         */
        explicit CallbackFunction(std::function<void(DataType)> callback) :
            AbstractCallbackFunction(),
            m_callbackFct{std::move(callback)}
        {}

        /**
         * Destructor
         */
        ~CallbackFunction() noexcept override = default;

        /**
         * Execute the callback by deserializing the frame payload and calling the user function
         *
         * @param[in] msgFrameNotif The notification message frame to process
         * @return Error object with ERROR_NONE on success, or payload decoding error on failure
         */
        Error call(Frame& msgFrameNotif) override
        {
            Error error;
            error.set_error_code(ErrorCodes::ERROR_NONE);

            DataType decodedMsgNotif;
            if( !decodedMsgNotif.ParseFromString(msgFrameNotif.payload()) )
            {
                HeaderInfo headerInfo( msgFrameNotif.header() );

                error.set_error_code(ERROR_PROTOCOL_CLIENT);
                error.set_error_sub_code(PAYLOAD_DECODING_ERR);
                error.set_error_sub_string(std::string("The data payload could not be deserialized : notification for serviceId=") + std::to_string(headerInfo.m_serviceInfo.serviceId) + " \n");
            }
            else
            {
                m_callbackFct(decodedMsgNotif);
            }

            return error;
        }
    };

    // Map type of callbacks associated by their notification ids
    using CallbackMapByNotifId = 
        std::unordered_map< uint32_t, std::unique_ptr<AbstractCallbackFunction>>;

    /**
     * Map type of notification ids associated by function ids
     * This is useful to call all suitable callbacks for a certain function id 
     */
    using NotifIdsMapByFunctionId = std::map<uint32_t, std::vector<uint32_t>>;

    /**
     * Handler for managing notification callbacks on the client side
     *
     * This class manages registration, dispatching, and cleanup of notification callbacks
     * for different function IDs. It supports multiple callbacks per function and is thread-safe.
     */
    class NotificationHandler
    {
        CallbackMapByNotifId     m_callbackMap;
        NotifIdsMapByFunctionId  m_functionNotifs;
        std::mutex               m_mutex;

    public:
        /**
         * Default constructor
         */
        NotificationHandler() = default;

        /**
         * Destructor
         */
        ~NotificationHandler() noexcept = default;


        /**
         * Add a callback to intercept notifications of a specific functionId
         *
         * @param[in] functionId The id of the function to be notified from
         * @param[in] callback The callback to call
         * @return A pair of boolean and notification Id signifying the result of adding.
         *         First is True if this was the first callback for the <functionId>
         *         First is False otherwise
         *         Second is the notification id to use to unsubscribe this callback
         */
        template <class DataType>
        std::pair<bool,int> addCallback( uint32_t functionId, std::function<void(DataType)> callback )
        {
            auto fct = 
                std::unique_ptr<CallbackFunction<DataType>>{new CallbackFunction<DataType>{callback}};
            return addCallback(functionId, std::move(fct));
        }

        /**
         * Add a callback to intercept notifications of a specific functionId
         *
         * @param[in] functionId The id of the function to be notified from
         * @param[in] callback The callback to call
         * @return A pair of boolean and notification Id signifying the result of adding.
         *         First is True if this was the first callback for the <functionId>
         *         First is False otherwise
         *         Second is the notification id to use to unsubscribe this callback
         */
        std::pair<bool,int> addCallback( uint32_t functionId, std::unique_ptr<AbstractCallbackFunction> callback);

        /**
         * Clear the callback associated with the specified notifId
         *
         * @param[in] notifId The notifId for which to clear the callback
         * @pre <notifId> must correspond to an existing callback
         * @post <notifId> will no longer correspond to an existing callback
         * @post The callback previously associated to <notifId> will stop being called
         * @return A pair of booleans signifying the result of erasing.
         *         First is True if the callback was found and removed
         *         First is False otherwise
         *         Second is a functionId if the callback was the last of the function id
         *         Second is zero otherwise
         */
        std::pair<bool,uint32_t> clearIdKeyCallbacks( uint32_t notifId);

        // Clear all callbacks registered
        /**
         * @post All currently associated notification ids are disassociated,
         *       as if NotificationHandler::clearIdKeyCallbacks had been called
         *       for each notification id
         * @warning This may pre-release currently-used notification ids with pending
         *          release calls, resulting in contract violation. USE WITH CARE.
         */
        void clearAll();

        /**
         * Return the next available notification id
         *
         * @return The lowest unused notification id in m_callbackMap
         */
        int nextAvailableNotifId() const;

        /**
         * Dispatch the message frame to the correct callback functions
         *
         * @param[in] msgFrameNotif A notification message frame to dispatch
         *            the frame will be dispatched using its <functionId>
         */
        void call(Frame& msgFrameNotif);

        /**
         * Verify if an incoming message's notification ID is valid 
         *
         * @param[in] msgFrameNotif A notification message frame to dispatch
         *            the frame will be dispatched using its <functionId>
         * @return An error frame representing an error.
         *         ErrorCodes::ERROR_NONE for success
         *         an error code representing the error otherwise
         */
        Error validateNotifId(const Frame& msgFrameNotif);
    };

}
}

#endif // _NOTIFICATION_HANDLER_H_
