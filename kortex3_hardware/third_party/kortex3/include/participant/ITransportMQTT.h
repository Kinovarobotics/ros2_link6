#ifndef _KORTEXAPI_ITRANSPORTMQTT_H_
#define _KORTEXAPI_ITRANSPORTMQTT_H_

#include "Frame.pb.h"

#include "HeaderInfo.h"

#include <functional>

#include <string>
#include <chrono>

namespace Kinova
{
namespace Api
{

static const std::chrono::milliseconds DEFAULT_MQTT_SPIN_PROCESS_DELAY = std::chrono::milliseconds(10);


// Interface Transport class for a Mqtt network
class ITransportMQTT
{
public:


    /**
     * Friendship declaration for access to 
     * GenerateTopic(...) function for the
     * Kortex Api Protocol class
     *
     * @see KortexApiProtocol::GenerateTopic
     */
    friend class KortexApiProtocol;

    // Type for message callbacks
    using CallbackType = void(Kinova::Api::Frame const&, const std::string&, const std::string&);

    // Default Constructor
    ITransportMQTT() = default;

    // Default Destructor
    virtual ~ITransportMQTT() noexcept = default;

    /**
     * Starts the incoming message processing thread for this router
     * This will start a thread that will continuously try to consume the
     * next received mqtt message and call RouterMQTT::ProcessReceive 
     * with it
     *
     * @param[in] delay between each processing of any incoming messages
     * @post the processing thread will be set to a joinable thread
     *       if the thread is already joinable, nothing
     *       happens
     */
    virtual void SpinProcess(const std::chrono::milliseconds& delay = DEFAULT_MQTT_SPIN_PROCESS_DELAY) = 0;

    /**
     * Stops the incoming message processing thread for this router, if it was running.
     *
     * @post the processing thread will be joined. If the thread is already 
     *       stopped, nothing happens.
     */
    virtual void UnspinProcess() = 0;

    // Connect to the Mqtt network broker
    virtual void Connect() = 0;

    // Disconnect from the Mqtt network broker
    virtual void Disconnect() noexcept = 0;

    /**
     * Reconnect to the Mqtt network broker
     *
     * @pre The transport must be connected
     * @param[in] lastWillTestament The Kinova::Api::Frame message to use as MQTT Last Will Testament (if default Frame, then no LWT is set)
     * @post The transport is reconnected with specified last will, with no last will if not specified
     */
    virtual void Reconnect(const Kinova::Api::Frame& lastWillTestamentFrame) = 0;

    /**
     * Get the client id within the Mqtt network broker
     *
     * @return The client id used for communicating with
     *         the Mqtt network
     */
    virtual const std::string GetClientId() const noexcept = 0;

    /**
     * Check if the transport is currently connected
     *
     * @return True is connection is established
     *         False otherwise
     */
    virtual bool IsConnected() const = 0;

    /**
     * Send a message Frame to the Mqtt network
     *
     * @pre The transport must be connected
     * @param[in] msg The message frame to send
     * @param[in] ns The namespace to prepend the MQTT topic for the message
     * @post The message has been sent to the mqtt network
     */
    virtual void SendAll(Kinova::Api::Frame const& msg, const std::string& ns = "") = 0;

    /**
     * Send a message Frame to a specific client on the MQTT network (used for message type RESPONSE and RAW)
     *
     * @pre The transport must be connected
     * @param[in] msg The message frame to send
     * @param[in] msg The specific topic
     * @post The message has been sent to the mqtt network on the topic
     */
    virtual void SendTo(Kinova::Api::Frame const& msg, const std::string& topic) = 0;

    /**
     * Send a message Frame completely raw on the MQTT network (used for MQTT RAW messages)
     *
     * @pre The transport must be connected
     * @param[in] msg The payload to send
     * @param[in] msg The specific topic
     * @post The message has been sent to the mqtt network on the topic
     */
    virtual void SendTo(std::string const& msg, const std::string& topic) = 0;

    /**
     * Setup a callback on incoming messages
     *
     * @param[in] callback Function to call when processing messages
     */
    virtual void OnMessage(std::function<CallbackType> callback) = 0;

    /**
     * Process the next incoming message
     *
     * @return Number of messages processed during the call
     */
    virtual unsigned int ProcessNextMessages() = 0;

    /**
     * Subscribe to a topic pattern
     *
     * @pre topicPattern should have been validated if it is a custom user topic
     * @param[in] topicPattern The topic pattern to subscribe to
     */
    virtual void Subscribe(std::string const& topicPattern) = 0;

    /**
     * Unsubscribe to a topic pattern
     *
     * @param[in] topicPattern The topic pattern to unsubscribe from
     */
    virtual void Unsubscribe(std::string const& topicPattern) = 0;

    /**
     * Validate if a custom topic is suitable to publish on or subscribe to
     *
     * @param[in] topic The topic
     * @return A boolean value indicating if the topic passes or fails validation
     */
    virtual bool ValidateCustomTopic(const std::string& topic) const = 0;

    /**
     * Generate a topic based on a frame header information
     *
     * @param[in] info The frame header information
     * @param[in] ns The namespace to prepend the MQTT topic for the message
     * @return A topic
     */
    virtual std::string GenerateTopic(Kinova::Api::HeaderInfo const& info, const std::string& ns = "") const = 0;

    /**
     * Toggles benchmarking on the Transport.
     *
     * @param[in] activate True to activate it, false to deactivate it.
     */
    virtual void ActivateBenchmarker(bool activate) = 0;
};

} // namespace Api
} // namespace Kinova

#endif // _KORTEXAPI_ITRANSPORTMQTT_H_