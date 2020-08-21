// Copyright 2020 The Defold Foundation
// Licensed under the Defold License version 1.0 (the "License"); you may not use
// this file except in compliance with the License.
// 
// You may obtain a copy of the License, together with FAQs at
// https://www.defold.com/license
// 
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#ifndef DM_CONNECTION_POOL
#define DM_CONNECTION_POOL

#include <stdint.h>
#include <dlib/socket.h>
#include <dlib/dns.h>

/**
 * Connection pooling
 */
namespace dmConnectionPool
{
    /**
     * Connection pool handle
     */
    typedef struct ConnectionPool* HPool;

    /**
     * Connection handle
     */
    typedef uint32_t HConnection;

    /**
     * Result codes
     */
    enum Result
    {
        RESULT_OK = 0,               //!< RESULT_OK
        RESULT_OUT_OF_RESOURCES = -1,//!< RESULT_OUT_OF_RESOURCES
        RESULT_SOCKET_ERROR = -2,    //!< RESULT_SOCKET_ERROR
        RESULT_HANDSHAKE_FAILED = -3,//!< RESULT_HANDSHAKE_FAILED
        RESULT_SHUT_DOWN = -4,       //<! RESULT_SHUT_DOWN
    };

    /**
     * Stats
     */
    struct Stats
    {
        uint32_t m_Free;
        uint32_t m_Connected;
        uint32_t m_InUse;
    };

    /**
     * Parameters
     */
    struct Params
    {
        Params()
        {
            m_MaxConnections = 64;
            m_MaxKeepAlive = 10;
        }

        /// Max connection in pool
        uint32_t m_MaxConnections;
        /// Default max-keep-alive time in seconds
        uint32_t m_MaxKeepAlive;
    };

    /**
     * Create a new connection pool
     * @param params
     * @param pool
     * @return RESULT_OK on success
     */
    Result New(const Params* params, HPool* pool);

    /**
     * Delete connnection pool
     * @param pool
     * @return RESULT_OK on success
     */
    Result Delete(HPool pool);

    /**
     * Set max keep-alive for sockets in seconds. Sockets older than max_keep_alive
     * are not reused and hence closed
     * @param pool
     * @param max_keep_alive
     */
    void SetMaxKeepAlive(HPool pool, uint32_t max_keep_alive);

    /**
     * Get statistics
     * @param pool
     * @param stats
     */
    void GetStats(HPool pool, Stats* stats);

    /**
     * Connection to a host/port
     * @param pool pool
     * @param host host
     * @param port port
     * @param dns_channel The DNS channel that will be used for translating the host to an address
     * @param ssl true for ssl connection
     * @param timeout The timeout (micro seconds) for the connection and ssl handshake
     * @param connection connection (out)
     * @param sock_res socket-result code on failure
     * @return RESULT_OK on success
     */
    Result Dial(HPool pool, const char* host, uint16_t port, dmDNS::HChannel dns_channel, bool ssl, int timeout, HConnection* connection, dmSocket::Result* sock_res);

    /**
     * Return connection to pool
     * @param pool
     * @param connection
     */
    void Return(HPool pool, HConnection connection);

    /**
     * Close connection. Use this function whenever an error occur in eg http.
     * @param pool
     * @param connection
     */
    void Close(HPool pool, HConnection connection);

    /**
     * Get socket for connection
     * @param pool
     * @param connection
     * @return RESULT_OK on success
     */
    dmSocket::Socket GetSocket(HPool pool, HConnection connection);

    /**
     * Get ssl-handle. The returned value is an mbedtls_ssl_context* (mbedtls)
     * @param pool
     * @param connection
     * @return An mbedtls_ssl_context* pointer on success
     */
    void* GetSSLConnection(HPool pool, HConnection connection);

    /**
     * Get reuse count for a connection
     * @param pool
     * @param connection
     * @return reuse count
     */
    uint32_t GetReuseCount(HPool pool, HConnection connection);

    /**
     * Shuts down all open sockets in the pool and block new connection attempts. The function can be
     * called repeatedly on the same pool until it returns no more connections in use.
     *
     * @param pool pool
     * @param how shutdown type to pass to socket shutdown function
     * @return current number of connections in use
     */
    uint32_t Shutdown(HPool pool, dmSocket::ShutdownType how);

    /**
     * Reopen the pool from a Shutdown call so it allows Dialing again. This function is here so the pool can be reset
     * during testing, or subsequent tests will break when the pool has been put in shutdown mode.
     */
    void Reopen(HPool pool);

}

#endif // #ifndef DM_CONNECTION_POOL
