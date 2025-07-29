#ifndef _KORTEXAPI_MQTTPROTOCOL_H_
#define _KORTEXAPI_MQTTPROTOCOL_H_

#include "ITransportMQTT.h"

#include <memory>

#include <map>
#include <set>

#include <vector>

namespace Kinova{
namespace Api{

class KortexApiProtocol : public ITransportMQTT
{
private:

    // Underlying transport
    std::unique_ptr<ITransportMQTT> m_transport;

    // Typedef for a pair : first element is function ID, second element is notification handle identifier
    using FunctionIdAndNotificationHandleIdentifier = std::pair<uint16_t, uint16_t>;

    /**
     * Type for a two-level map of connection handles
     * Divides by service id first, for connection handle 
     * identifiers are unique only pertaining a service.
     * Divides by function is second, allowing multiple
     * connection handles for each function id
     */
    using ConnectionHandleByServiceByFunction = 
        std::map<
        uint16_t, // session Id
        std::map<
            uint16_t, // service id
            std::set<FunctionIdAndNotificationHandleIdentifier> // Function IDs and Notification Handles
        >>;
    
    // Map of all current notification connection handles
    ConnectionHandleByServiceByFunction m_NotificationHandles;

    // Set of subscribe function UIDS
    std::set<uint32_t> m_SubscribeUids;

    // Set of unsubscribe function UIDS
    std::set<uint32_t> m_UnsubscribeUids;

    // Session Id for the transport
    unsigned int m_SessionId;

public:

    /**
     * Constructor
     *
     * @param[in] transport The underlying transport
     */
    KortexApiProtocol(std::unique_ptr<ITransportMQTT> transport);

public:

    // @see ITransportMQTT::SpinProcess
    void SpinProcess(const std::chrono::milliseconds& delay = DEFAULT_MQTT_SPIN_PROCESS_DELAY) override;

    // @see ITransportMQTT::UnspinProcess
    void UnspinProcess() override;

    // @see ITransportMQTT::Connect
    void Connect() override;

    // @see ITransportMQTT::Disconnect
    void Disconnect() noexcept override;

    // @see ITransportMQTT::Reconnect
    void Reconnect(Kinova::Api::Frame const& lastWillTestament) override;

    // @see ITransportMQTT::GetClientId
    const std::string GetClientId() const noexcept override;

    // @see ITransportMQTT::IsConnected
    bool IsConnected() const override;

    // @see ITransportMQTT::SendAll
    void SendAll(Kinova::Api::Frame const& msg, const std::string& ns = "") override;

    // @see ITransportMQTT::SendTo
    void SendTo(Kinova::Api::Frame const& msg, const std::string& clientId) override;

    // @see ITransportMQTT::SendTo
    void SendTo(std::string const& msg, const std::string& clientId) override;

    // @see ITransportMQTT::OnMessage
    void OnMessage(std::function<CallbackType> callback) override;

    // @see ITransportMQTT::ProcessNextMessages
    unsigned int ProcessNextMessages() override;

    // @see ITransportMQTT::Subscribe
    void Subscribe(std::string const& topicPattern) override;

    // @see ITransportMQTT::Unsubscribe
    void Unsubscribe(std::string const& topicPattern) override;

    // @see ITransportMQTT::ValidateCustomTopic
    bool ValidateCustomTopic(const std::string& topic) const override;
    
    // @see ITransportMQTT::GenerateTopic
    std::string GenerateTopic(Kinova::Api::HeaderInfo const& info, const std::string& ns = "") const override;

    /**
     * @see ITransportMQTT::ActivateBenchmarker
     * @note purposely not implemented for now
     */
    void ActivateBenchmarker(bool activate) override {}

public:

    /**
     * Set the known set of subscribe function UIDs
     *
     * @param[in] uids Set of function UIDs that represent subscribe operations
     * @post The protocol will recognize these UIDs as subscription requests
     */
    void SetSubscribeUids(std::set<uint32_t> uids);

    /**
     * Get the known set of subscribe function UIDs
     *
     * @return Set of function UIDs that are recognized as subscribe operations
     */
    std::set<uint32_t> GetSubscribeUids() const;

    /**
     * Set the known set of unsubscribe function UIDs
     *
     * @param[in] uids Set of function UIDs that represent unsubscribe operations
     * @post The protocol will recognize these UIDs as unsubscription requests
     */
    void SetUnsubscribeUids(std::set<uint32_t> uids);

    /**
     * Get the known set of unsubscribe function UIDs
     *
     * @return Set of function UIDs that are recognized as unsubscribe operations
     */
    std::set<uint32_t> GetUnsubscribeUids() const;

    /**
     * Check if a specific notification is currently subscribed to by any session
     *
     * @param[in] service_id Service identifier of the notification
     * @param[in] function_id Function identifier of the notification
     * @return True if at least one session is subscribed to this notification, false otherwise
     */
    bool IsSubscribedToNotification(uint16_t service_id, uint16_t function_id) const;


private:

    /**
     * Manage protocol requirements before sending a message through the Transport
     * This will 'sniff' outcoming messages and detect the frame type. Then it will
     * enforce the proper requirements (e.g.: subscribing/unsubscribing to topics)
     *
     * @param[in] msg The outcoming message
     */
    void OnSend(Kinova::Api::Frame const& msg);


    /**
     * Manage protocol requirements after receiving a message through the Transport
     * This will 'sniff' incoming messages and detect the frame type. Then it will
     * enforce the proper requirements (e.g.: subscribing/unsubscribing to topics)
     *
     * @param[in] msg The incoming message
     */
    void OnReceive(Kinova::Api::Frame const& msg);

    // Helper function to subscribe to notification MQTT topic
    void SubscribeToNotificationTopic(uint16_t service_id, uint16_t function_id);

    // Helper function to unsubscribe to notification MQTT topic
    void UnsubscribeToNotificationTopic(uint16_t service_id, uint16_t function_id);
};


} // namespace Api
} // namespace Kinova

#endif//_KORTEXAPI_MQTTPROTOCOL_H_