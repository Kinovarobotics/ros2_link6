#ifndef _KORTEXAPI_FAKETRANSPORTMQTT_H_
#define _KORTEXAPI_FAKETRANSPORTMQTT_H_

#include "ITransportMQTT.h"

#include "HeaderInfo.h"

#include "Frame.pb.h"

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
#include <deque>

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
// Forward declare the fake broker class
class FakeMQTTBroker;

// Fake transport class for talking with a fake broker
class FakeTransportMQTT : public ITransportMQTT
{
    // Mutex for accessing <m_processSpinThread>
    std::mutex                                  m_spinThreadMutex;
    // Thread for automatic incoming message processing
    std::thread                                 m_processSpinThread;
    // Flag to stop <m_processSpinThread>
    std::atomic_flag                            m_threadRunning;

    // Mutex for accessing/editing the message queue
    std::mutex                                  m_queueMutex;

    // Client id with Mqtt broker
    std::string m_clientId;

    //Fake MQTT broker
    FakeMQTTBroker* m_broker;

    // Message callback
    std::function<CallbackType> m_callback;

    // Message queue
    std::deque<mqtt::message> m_messageQueue;

public:

    // Constructor
    FakeTransportMQTT(
        FakeMQTTBroker* broker,
        std::string const& clientId = std::string{});

    FakeTransportMQTT(
        FakeMQTTBroker* broker,
        std::function<CallbackType> callback,
        std::string const& clientId = std::string{});

    // Destructor
    ~FakeTransportMQTT() noexcept override;

    // @see ITransportMQTT::SpinProcess
    void SpinProcess(const std::chrono::milliseconds& delay = DEFAULT_MQTT_SPIN_PROCESS_DELAY) override;

    // @see ITransportMQTT::UnspinProcess
    void UnspinProcess() override;

    // @see ITransportMQTT::Connect
    void Connect() override;

    // @see ITransportMQTT::Disconnect
    void Disconnect() noexcept override {}

    // @see ITransportMQTT::Reconnect
    void Reconnect(const Kinova::Api::Frame& lastWillTestamentFrame) override {}

    // @see ITransportMQTT::GetClientId
    const std::string GetClientId() const noexcept override
    {
        return m_clientId;
    }

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

    /**
     * @see ITransportMQTT::ActivateBenchmarker
     * @note purposely not implemented for now
     */
    void ActivateBenchmarker(bool activate) override {}

    /**
     * Receive a message from the fake broker
     *
     * @param[in] msg The message to process
     */
    void ReceiveNewMessage(mqtt::message const& msg);

    /**
     * Returns a constant image of the message queue
     *
     * @return The message queue
     */
    const std::deque<mqtt::message> getMessageQueue();

    // Empties the message queue
    void emptyMessageQueue();

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

    // Utility function for generating namespaces to use on the network so there is no clash with other instances
    const std::string GenerateRandomString();
};

} // namespace Api
} // namespace Kinova

#endif // _KORTEXAPI_FAKETRANSPORTMQTT_H_