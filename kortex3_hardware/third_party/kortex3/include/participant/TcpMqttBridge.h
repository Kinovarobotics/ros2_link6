#include "ITransportServer.h"
#include "ITransportMQTT.h"

#include <unordered_map>
#include <memory>
#include <functional>

namespace Kinova
{
namespace Api
{

/**
 * Bridge class between Tcp protocol and Mqtt protocol for the Kortex Api
 * This enables a single Tcp transport layer to speak to many Mqtt transport 
 * layers (1 per client)
 *
 * @invariant When a message passes from the Tcp interface with a new 
 *            ClientConnectionHandle, a new Mqtt transport protocol is created 
 *            by calling <m_constructorFunction> once, and is added to 
 *            <m_mqttTransportList>, incrementing its size.
 *
 * @invariant When a message passes from the Tcp interface with a Client Id 
 *            present in <m_mqttTransportList>, the message is passed on to the
 *            mqtt transport corresponding to the Client id
 *
 * @invariant When a message passes from any of the mqtt transports in 
 *            <m_mqttTransportList>, it is passed on to the tcp transport along
 *            with the Client Id of that mqttTransport
 */
class TcpMqttBridge
{
private:
    // Tcp Tranport
    std::unique_ptr<ITransportServer> m_tcpTransport;

    /**
     * Mqtt Transports for each client of the Tcp transport
     *
     * @note The key is a client identifier
     */
    std::unordered_map<uint32_t, std::unique_ptr<ITransportMQTT>> m_mqttTransportList;

    /**
     * Construction Function used to create a new Mqtt Transport with a 
     * specified client identifier
     */
    std::function<std::unique_ptr<ITransportMQTT>(ClientConnectionHandle const&)> m_constructorFunction;

public:

    /**
     * Full constructor
     *
     * @param[in] tcpTransport The Tcp Transport
     * @param[in] mqttTransportList The Mqtt Transport for clients
     * @param[in] constructorFunction The construction function
     * @post all invariants are established
     */
    TcpMqttBridge(
        std::unique_ptr<ITransportServer> tcpTransport,
        std::unordered_map<uint32_t, std::unique_ptr<ITransportMQTT>> mqttTransportList,
        std::function<std::unique_ptr<ITransportMQTT>(ClientConnectionHandle const&)> constructorFunction
    );

    /**
     * Copy constructor - deleted (copying is not allowed)
     */
    TcpMqttBridge(const TcpMqttBridge&) = delete;

    /**
     * Copy assignment operator - deleted (copying is not allowed)
     */
    TcpMqttBridge& operator=(const TcpMqttBridge&) = delete;

    /**
     * Move constructor
     *
     * @param[in] other TcpMqttBridge to move from
     * @post The other bridge is left in a valid but unspecified state
     */
    TcpMqttBridge(TcpMqttBridge&&) noexcept;

    /**
     * Move assignment operator
     *
     * @param[in] other TcpMqttBridge to move from
     * @return Reference to this object
     * @post The other bridge is left in a valid but unspecified state
     */
    TcpMqttBridge& operator=(TcpMqttBridge&&) noexcept;

    /**
     * Destructor - cleans up all transport resources
     */
    ~TcpMqttBridge() noexcept;

    /**
     * Process incoming messages from all transports (TCP and MQTT)
     *
     * @post All pending messages from TCP and MQTT transports are processed
     * @note This should be called regularly to maintain message flow
     */
    void ProcessAllTransports();
};

} // namespace Api
} // namespace Kinova