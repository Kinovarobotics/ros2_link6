/* ***************************************************************************
 * Kinova inc.
 *
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __TRANSPORT_CLIENT_TCP_H__
#define __TRANSPORT_CLIENT_TCP_H__

// --- linux ---
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/fcntl.h>

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
#include "KinovaTcpUtilities.h"

namespace Kinova
{
namespace Api
{
    /**
     * TCP transport implementation for Kortex API client
     *
     * Provides reliable, connection-oriented communication with Kortex devices
     * over TCP/IP. Handles automatic receive threading and buffer management.
     *
     * @note This is an internal implementation class - use through ITransportClient interface
     */
    class TransportClientTcp : public ITransportClient
    {
    private:
        bool                    m_isInitialized;
        struct sockaddr_in      m_socketAddr{};
        socklen_t               m_socketAddrSize{};
        int32_t                 m_socketFd{};

        bool                    m_isUsingRcvThread;
        std::atomic<bool>       m_isRunning { true };
        std::mutex              m_sendMutex;

        fd_set          m_original_rx{};
        fd_set          m_readfds{};

        int             numfd{};
        struct hostent  *m_host{};
        struct timeval  m_tv{};

        bool                 m_bIsReceiving { false };
        uint32_t             m_nTotalBytesRead {0};
        uint32_t             m_nTotalBytesToRead {0};
        uint8_t*             m_tx_buffer;
        uint8_t*             m_rx_buffer;

        uint32_t             m_current_buffer_size_rx;
        uint32_t             m_current_buffer_size_tx;


        KinovaTcpUtilities m_utilities_object;

        std::function<void (const char*, uint32_t) > m_onMessageCallback;


    public:
        /**
         * Constructor
         *
         * @param[in] isUsingRcvThread If true, spawns a dedicated receive thread (default: true)
         */
        explicit TransportClientTcp(bool isUsingRcvThread = true);

        // Destructor - ensures proper cleanup of sockets and threads
        ~TransportClientTcp() override;

        /**
         * Connect to a TCP server
         *
         * @param[in] host Hostname or IP address (default: "127.0.0.1")
         * @param[in] port Port number (default: 10000)
         * @return True if connection succeeded, false otherwise
         */
        bool connect(std::string host = "127.0.0.1", uint32_t port = 10000) override;

        // Disconnect from the TCP server
        void disconnect() override;

        /**
         * Send data through TCP connection
         *
         * @param[in] txBuffer Buffer containing data to send
         * @param[in] txSize Size of data in bytes
         */
        void send(const char* txBuffer, uint32_t txSize) override;

        /**
         * Register callback for incoming TCP messages
         *
         * @param[in] callback Function to call when data is received
         */
        void onMessage(std::function<void (const char*, uint32_t)> callback) override;

        /**
         * Get a transmission buffer for preparing data
         *
         * @param[in] allocation_size Size of buffer needed
         * @return Pointer to transmission buffer
         */
        char* getTxBuffer(uint32_t const& allocation_size) override;

        /**
         * Get maximum transmission buffer size
         *
         * @return Maximum size in bytes (16777216 bytes = ~16MB)
         */
        size_t getMaxTxBufferSize() override { return 16777216; }

        /**
         * Get current host address and port
         *
         * @param[out] host Hostname or IP address
         * @param[out] port Port number
         */
        void getHostAddress(std::string &host, uint32_t &port) override {
            host = mHostAddress;
            port = mHostPort;
        }

        /**
         * Get current connection state
         *
         * @return Current ready state of the transport
         */
        TransportReadyStateEnum getReadyState() const override {
            return readyState;
        }

    private:
        // Host address (IP or hostname)
        std::string mHostAddress;

        // Host port number
        uint32_t mHostPort;

        // Receive thread for handling incoming TCP data
        std::thread m_receiveThread;

        /**
         * Receive thread main loop
         *
         * @param[in] program_is_running Atomic flag to control thread execution
         */
        void receiveThread(std::atomic<bool> &program_is_running);

        /**
         * Process incoming data with timeout (microseconds)
         *
         * @param[in] rcvTimeout_usec Receive timeout in microseconds
         * @return Number of bytes received, or error code if negative
         */
        int processReceive(long rcvTimeout_usec);

        /**
         * Process incoming data with timeout (timeval struct)
         *
         * @param[in] rcvTimeout_tv Receive timeout as timeval
         * @return Number of bytes received, or error code if negative
         */
        int processReceive(struct timeval rcvTimeout_tv);

        /**
         * Perform the actual receive operation from socket
         *
         * @return Number of bytes received, or error code if negative
         */
        int callReceiveFrom();
    };

} // namespace Api
} // namespace Kinova

#endif // __TRANSPORT_CLIENT_H__
