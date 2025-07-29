#ifndef _KORTEXAPI_FAKEMQTTBROKER_H_
#define _KORTEXAPI_FAKEMQTTBROKER_H_

#include "Frame.pb.h"
#include "HeaderInfo.h"
#include <string>
#include <sstream>
#include <regex>
#include <mutex>

namespace mqtt
{
    class client;
    class message;
}

namespace Kinova
{
namespace Api
{
// Forward
class FakeTransportMQTT;

// Fake MQTT broker class to avoid having to connect to a real one
class FakeMQTTBroker
{
    // Mutex for accessing/editing the client map
    std::mutex m_clientMutex;

    // Mutex for accessing/editing the subscription map
    std::mutex m_subMutex;

    // Map of all connected clients
    std::map<std::string, FakeTransportMQTT*> m_clients;
    
    // Map of all subscriptions <topic, vector of clientId subscribed to it>
    std::map<std::string, std::vector<std::string>> m_subscriptions;

public:

    // Default Constructor
    FakeMQTTBroker() = default;

    // Default Destructor
    virtual ~FakeMQTTBroker() noexcept = default;

    /**
     * Connect a transport to the broker
     *
     * @param[in] client_ptr Pointer to the FakeTransportMQTT object to connect
     * @param[in] clientId The unique client identifier
     * @return true if a new connection was made
     */
    bool connect(FakeTransportMQTT* client_ptr, const std::string& clientId);

    /**
     * Disconnect a transport from the broker and remove its subscriptions
     *
     * @param[in] clientId The unique client identifier to disconnect
     * @return true if a connection was removed
     */
    bool disconnect(const std::string& clientId);

    /**
     * Disconnect all transports from the broker and remove their subscriptions
     *
     * @return how many transports were disconnected
     */
    int disconnectAllTransports();

    /**
     * Publish a message
     *
     * @param[in] message The message frame to send
     * @return how many clients the message was sent to
     */
    int publish(const mqtt::message& message);

    /**
     * Subscribe a client to a topic
     *
     * @param[in] topicPattern The topic to subscribe the client to
     * @param[in] clientId The unique client identifier
     * @return true if a new subscription was created
     */
    bool subscribe(const std::string& topicPattern, const std::string& clientId);

    /**
     * Unsubscribe a client to a topic
     *
     * @param[in] topicPattern The topic to unsubscribe the client from
     * @param[in] clientId The unique client identifier
     * @return true if a subscription was removed
     */
    bool unsubscribe(const std::string& topicPattern, const std::string& clientId);

    /**
     * Returns the map of clients
     *
     * @return the map containing all connected clients
     */
    std::map<std::string, FakeTransportMQTT*> getClients() const;

    /**
     * Returns the map of subscriptions
     *
     * @return the map containing all topics and all clients subscribed to them
     */
    std::map<std::string, std::vector<std::string>> getSubscriptions() const;

    /**
     * Returns the total amount of subscriptions in the map
     *
     * @return the total number of subscriptions
     */
    int getSubscriptionAmount() const;

    /**
     * Helper function to determine if a topicPattern has a valid format
     *
     * @param[in] topicPattern The topic pattern to validate
     * @param[in] allowWildcards Determines if the topic containing a wildcard makes it invalid or not
     */
    static bool validatePatternFormat(const std::string& topicPattern, bool allowWildcards);

    /**
     * Helper function to determine if a topicPattern fits with the publish topic
     *
     * @param[in] topicPattern The topic pattern to validate against
     * @param[in] pubTopic The topic that is being published to
     */
    static bool checkIfTopicFitsPattern(const std::string& topicPattern, const std::string& pubTopic);

    /**
     * Helper function to split a topic by levels
     *
     * @param[in] topic The topic string to split
     */
    static std::vector<std::string> splitTopic(const std::string& topic);
};

} // namespace Api
} // namespace Kinova

#endif // _KORTEXAPI_FAKEMQTTBROKER_H_