#ifndef _KORTEXAPI_TRANSPORTMQTT_H_
#define _KORTEXAPI_TRANSPORTMQTT_H_

#include "ITransportMQTT.h"

#include "HeaderInfo.h"

#include "Frame.pb.h"

#include "CoreBenchmarker.h"

#include <memory>
#include <string>
#include <set>
#include <map>
#include <unordered_set>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <chrono>

namespace mqtt
{
    class client;
    class message;
}


namespace Kinova
{
namespace Api
{

// Transport class for talking with a mqtt broker
class TransportMQTT : public ITransportMQTT
{

private:

    // Mutex for accessing <m_processSpinThread>
    std::mutex                                  m_spinThreadMutex;
    // Thread for automatic incoming message processing
    std::thread                                 m_processSpinThread;
    // Flag to stop <m_processSpinThread>
    std::atomic_flag                            m_threadRunning;
    
    // URL to connect to
    std::string m_url;

    // Mutex for synchronous access to client
    std::mutex m_clientMutex;

    // Mqtt client
    std::unique_ptr<mqtt::client> m_client;

    // Client id with Mqtt broker
    std::string m_clientId;

    // Message callback
    std::function<CallbackType> m_callback;

    // Benchmarker object used to benchmark API calls.
    std::shared_ptr<Kinova::Api::Util::Benchmarking::CoreBenchmarker> m_benchmarker;

public:

    /**
     * Default constructor
     */
    TransportMQTT() = default;

    /**
     * Constructor with broker URL and optional client ID
     *
     * @param[in] url MQTT broker URL (e.g., "tcp://localhost:1883")
     * @param[in] clientId Optional client identifier (if empty, broker assigns one)
     */
    TransportMQTT(
        std::string const& url,
        std::string const& clientId = std::string{});

    /**
     * Full constructor with all MQTT transport parameters
     *
     * @param[in] url MQTT broker URL
     * @param[in] client Unique pointer to the MQTT client instance
     * @param[in] clientId Client identifier for the MQTT connection
     * @param[in] callback Callback function to handle incoming messages
     * @param[in] responseTopicsToReceive Set of response topics to subscribe to
     * @param[in] responseTopicsToSend Map of response identifiers to their send topics
     */
    TransportMQTT(
        std::string const& url,
        std::unique_ptr<mqtt::client> client,
        std::string const& clientId,
        std::function<CallbackType> callback,
        std::set<std::string> responseTopicsToReceive,
        std::map<ResponseIdentifier, std::string> responseTopicsToSend
    );

    /**
     * Copy constructor - deleted (copying is not allowed)
     */
    TransportMQTT(TransportMQTT const&) = delete;

    /**
     * Copy assignment operator - deleted (copying is not allowed)
     */
    TransportMQTT& operator=(TransportMQTT const&) = delete;

    /**
     * Move constructor
     */
    TransportMQTT(TransportMQTT &&) noexcept = default;

    /**
     * Move assignment operator
     */
    TransportMQTT& operator=(TransportMQTT &&) noexcept = default;

    /**
     * Destructor - disconnects and cleans up MQTT resources
     */
    ~TransportMQTT() noexcept override;

    // @see ITransportMQTT::SpinProcess
    void SpinProcess(const std::chrono::milliseconds& delay = DEFAULT_MQTT_SPIN_PROCESS_DELAY);

    // @see ITransportMQTT::UnspinProcess
    void UnspinProcess();

    // @see ITransportMQTT::Connect
    void Connect() override;

    // @see ITransportMQTT::Disconnect
    void Disconnect() noexcept override;

    // @see ITransportMQTT::Reconnect
    void Reconnect(const Kinova::Api::Frame& lastWillTestamentFrame) override;

    // @see ITransportMQTT::GetClientId
    const std::string GetClientId() const noexcept override;

    // @see ITransportMQTT::IsConnected
    bool IsConnected() const override;

    // @see ITransportMQTT::SendAll
    void SendAll(Kinova::Api::Frame const& msg, const std::string& ns = "") override;

    // @see ITransportMQTT::SendTo
    void SendTo(Kinova::Api::Frame const& msg, const std::string& topic) override;

    // @see ITransportMQTT::SendTo
    void SendTo(std::string const& msg, const std::string& topic) override;

    // @see ITransportMQTT::OnMessage
    void OnMessage(std::function<CallbackType> callback) override;

    // @see ITransportMQTT::ProcessNextMessage
    unsigned int ProcessNextMessages() override;

    // @see ITransportMQTT::Subscribe
    void Subscribe(std::string const& topicPattern) override;

    // @see ITransportMQTT::Unsubscribe
    void Unsubscribe(std::string const& topicPattern) override;

    // @see ITransportMQTT::ValidateCustomTopic
    bool ValidateCustomTopic(const std::string& topic) const override;
    
    // @see ITransportMQTT::GenerateTopic
    std::string GenerateTopic(Kinova::Api::HeaderInfo const& info, const std::string& ns = "") const override;

    // @see ITransportMQTT::ActivateBenchmarker
    void ActivateBenchmarker(bool activate) override;


private:
    // Prohibited topic names to publish on the network with a custom publisher
    const std::unordered_set<std::string> m_prohibitedNames{
        "dev",
        "req",
        "res",
        "notif",
        "ping",
        "pong"
    };

    /**
     * Process a message
     *
     * @param[in] msg The message to process
     * @return True if another message can be processed in the same round or false if we break the message processing loop
     */
    bool ProcessMessage(mqtt::message const& msg);

    /**
     * Helper function to get namespace from whole notif topic
     *
     * @param[in] topic The notification topic to extract the namespace from
     * @return The prepended namespace
     */
    std::string getNamespaceFromNotificationTopic(const std::string& topic) const;
};

} // namespace Api
} // namespace Kinova

#endif // _KORTEXAPI_TRANSPORTMQTT_H_