#ifndef _I_TRANSPORT_CLIENT_H_
#define _I_TRANSPORT_CLIENT_H_


#include <string>
#include <functional>
#include <exception>

#include <cmath>

namespace Kinova
{
namespace Api
{
    /**
     * Transport layer connection state enumeration
     * Represents the current state of the transport connection
     */
    enum TransportReadyStateEnum
    {
        CONNECTING = 0,     // Currently establishing connection
        OPEN = 1,           // Connection established and ready
        CLOSING = 2,        // Connection is being closed
        CLOSED = 3,         // Connection is closed
        UNINITIALIZED = 4,  // Transport not yet initialized
        RECONNECTING = 5,   // Attempting to reconnect
    };

    /**
     * Transport client interface for network communication
     *
     * This interface defines the contract for all transport implementations
     * (TCP, UDP) used to communicate with Kortex devices. Implementations
     * handle the low-level network operations.
     *
     * @note This is an internal interface - users should not implement this directly
     */
    class ITransportClient
    {
    public:
        // Virtual destructor
        virtual ~ITransportClient() {}

        /**
         * Connect to a remote host
         *
         * @param[in] host Hostname or IP address
         * @param[in] port Port number
         * @return True if connection succeeded, false otherwise
         */
        virtual bool connect(std::string host, uint32_t port) = 0;

        // Disconnect from the remote host
        virtual void disconnect() = 0;

        /**
         * Send data through the transport
         *
         * @param[in] txBuffer Buffer containing data to send
         * @param[in] txSize Size of data in bytes
         */
        virtual void send(const char* txBuffer, uint32_t txSize) = 0;

        /**
         * Register callback for incoming messages
         *
         * @param[in] callback Function to call when data is received
         *                     Parameters: (data buffer, size in bytes)
         */
        virtual void onMessage(std::function<void (const char*, uint32_t)> callback) = 0;

        /**
         * Get a transmission buffer for preparing data
         *
         * @param[in] allocation_size Size of buffer needed
         * @return Pointer to transmission buffer
         */
        virtual char* getTxBuffer(uint32_t const& allocation_size) = 0;

        /**
         * Get maximum transmission buffer size
         *
         * @return Maximum size in bytes that can be sent in one transmission
         */
        virtual size_t getMaxTxBufferSize() = 0;

        /**
         * Get current host address and port
         *
         * @param[out] host Hostname or IP address
         * @param[out] port Port number
         */
        virtual void getHostAddress(std::string &host, uint32_t &port) = 0;

        /**
         * Get current connection state
         *
         * @return Current ready state of the transport
         */
        virtual TransportReadyStateEnum getReadyState() const = 0;

    protected:
        // Connection state - accessible to derived classes
        TransportReadyStateEnum readyState;
    };


} // namespace Api
} // namespace Kinova

#endif // _I_TRANSPORT_CLIENT_H_
