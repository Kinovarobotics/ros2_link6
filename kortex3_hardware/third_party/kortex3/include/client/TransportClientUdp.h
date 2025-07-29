/* ***************************************************************************
 * Kinova inc.
 * Project :
 *
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __TRANSPORT_CLIENT_UDP_H__
#define __TRANSPORT_CLIENT_UDP_H__


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>      // host struct
#include <sys/select.h> // use select() for multiplexing
#include <sys/fcntl.h>  // for non-blocking

#include <iostream>
#include <unistd.h>
#include <ctime>
#include <stdio.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <atomic>
#include <thread>
#include <mutex>

#include <string>
#include <functional>
#include <exception>

#include <iostream>
#include <chrono>

#include "ITransportClient.h"

namespace Kinova
{
namespace Api
{
    /**
     * UDP transport implementation for Kortex API client
     *
     * Provides connectionless, datagram-based communication with Kortex devices
     * over UDP/IP. Suitable for low-latency scenarios where occasional packet
     * loss is acceptable. Handles automatic receive threading and buffer management.
     *
     * @note This is an internal implementation class - use through ITransportClient interface
     */
    class TransportClientUdp : public ITransportClient
    {
    private:
        // Default API port for Kortex devices
        static constexpr uint32_t kApiPort = 10000;

        bool                   	m_isInitialized;
        struct sockaddr_in      m_socketAddr;
        socklen_t               m_socketAddrSize;
        int32_t                 m_socketFd;

        bool                    m_isUsingRcvThread;
        std::atomic<bool>       m_isRunning { true };
        std::mutex              m_sendMutex;

        fd_set          m_original_rx;
        fd_set          m_readfds;

        int             numfd;
        struct hostent  *m_host;
        struct timeval  m_tv;

        char* m_txBuffer;
        char* m_rxBuffer;

        std::function<void (const char*, uint32_t) > m_onMessageCallback;

    public:
        /**
         * Constructor
         *
         * @param[in] isUsingRcvThread If true, spawns a dedicated receive thread (default: true)
         */
        TransportClientUdp(bool isUsingRcvThread = true);

        // Destructor - ensures proper cleanup of sockets and threads
        virtual ~TransportClientUdp();

        /**
         * Connect to a UDP endpoint
         *
         * @param[in] host Hostname or IP address (default: "127.0.0.1")
         * @param[in] port Port number (default: kApiPort)
         * @return True if connection succeeded, false otherwise
         */
        virtual bool connect(std::string host = "127.0.0.1", uint32_t port = kApiPort) override;

        // Disconnect from the UDP endpoint
        virtual void disconnect() override;

        /**
         * Send data through UDP
         *
         * @param[in] txBuffer Buffer containing data to send
         * @param[in] txSize Size of data in bytes
         */
        virtual void send(const char* txBuffer, uint32_t txSize) override;

        /**
         * Register callback for incoming UDP messages
         *
         * @param[in] callback Function to call when data is received
         */
        virtual void onMessage(std::function<void (const char*, uint32_t)> callback) override;

        /**
         * Get a transmission buffer for preparing data
         *
         * @param[in] allocation_size Size of buffer needed
         * @return Pointer to transmission buffer
         */
        virtual char* getTxBuffer(uint32_t const& allocation_size) override { return m_txBuffer; }

        /**
         * Get maximum transmission buffer size
         *
         * @return Maximum size in bytes (65507 bytes for UDP)
         */
        virtual size_t getMaxTxBufferSize() override { return 65507; }

        /**
         * Get current host address and port
         *
         * @param[out] host Hostname or IP address
         * @param[out] port Port number
         */
        virtual void getHostAddress(std::string &host, uint32_t &port) override {
            host = mHostAddress;
            port = mHostPort;
        }

        /**
         * Get current connection state
         *
         * @return Current ready state of the transport
         */
        virtual TransportReadyStateEnum getReadyState() const override {
            return readyState;
        }

        /**
         * Process incoming data with timeout (microseconds) - for testing
         *
         * @param[in] rcvTimeout_usec Receive timeout in microseconds
         * @return Number of bytes received, or error code if negative
         */
        int processReceive(long rcvTimeout_usec);

        /**
         * Process incoming data with timeout (timeval struct) - for testing
         *
         * @param[in] rcvTimeout_tv Receive timeout as timeval
         * @return Number of bytes received, or error code if negative
         */
        int processReceive(struct timeval rcvTimeout_tv);

    private:
        // Host address (IP or hostname)
        std::string mHostAddress;

        // Host port number
        uint32_t mHostPort;

        // Receive thread for handling incoming UDP data
        std::thread m_receiveThread;

        /**
         * Receive thread main loop
         *
         * @param[in] program_is_running Atomic flag to control thread execution
         */
        void receiveThread(std::atomic<bool> &program_is_running);

        /**
         * Perform the actual receive operation from socket
         *
         * @return <0 means error (-errorCode); =0 means timeout, nothing received; >0 means number of handles ready to recvFrom
         */
        int callReceiveFrom();
    };

} // namespace Api
} // namespace Kinova

#endif // __TRANSPORT_CLIENT_H__
