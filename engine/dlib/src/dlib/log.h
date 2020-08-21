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

#ifndef DM_LOG_H
#define DM_LOG_H

#include <dmsdk/dlib/log.h>
#include <dlib/message.h>

/**
 * @file
 * Logging functions. If DLIB_LOG_DOMAIN is defined the value of the defined is printed
 * after severity. Otherwise DEFAULT will be printed.
 *
 * Network protocol:
 * When connected a message with the following syntax is sent to the client
 * code <space> msg\n
 * eg 0 OK\n
 *
 * code > 0 indicates an error and the connections is closed by remote peer
 *
 * After connection is established log messages are streamed over the socket.
 * No other messages with semantic meaning is sent.
 */

struct dmLogMessage
{
    enum Type
    {
        MESSAGE = 0,
        SHUTDOWN = 1,
    };

    uint8_t m_Type;
    char    m_Message[0];
};

const uint32_t DM_LOG_MAX_STRING_SIZE = dmMessage::DM_MESSAGE_MAX_DATA_SIZE - sizeof(dmLogMessage);
struct dmLogParams
{
    dmLogParams()
    {
    }
};

/**
 * Initialize logging system. Running this function is only required in order to start the log-server.
 * The function will never fail even if the log-server can't be started. Any errors will be reported to stderr though
 * @param params log parameters
 */
void dmLogInitialize(const dmLogParams* params);


/**
 * Finalize logging system
 */
void dmLogFinalize();

/**
 * Get log server port
 * @return server port. 0 if the server isn't started.
 */
uint16_t dmLogGetPort();


/**
 * Set log level
 * @param severity Log severity
 */
void dmLogSetlevel(dmLogSeverity severity);

/**
 * Set log file. The file will be created and truncated.
 * Subsequent invocations to this function will close previous opened file.
 * If the file can't be created a message will be logged to the "console"
 * @param path log path
 */
void dmSetLogFile(const char* path);

/**
 * Callback declaration for dmSetCustomLogCallback
 */
typedef void (*dmCustomLogCallback)(void* user_data, const char* s);

/**
 * Sets a custom callback for log output, if this function is set output
 * will only be sent to this callback.
 * Useful for testing purposes to validate logging output from a test
 * Calling dmSetCustomLogCallback with (0x0, 0x0) will restore normal operation
 * @param callback the callback to call with output, once per logging call
 * @param user_data user data pointer that is provided as context in the callback
 */
void dmSetCustomLogCallback(dmCustomLogCallback callback, void* user_data);

/**
 * iOS specific print function that wraps NSLog to be able to
 * output logging to the device/XCode log.
 *
 * Declared here to be accessible from log.cpp, defined in log_ios.mm
 * since it needs to be compiled as Objective-C.
 *
 * @param severity Log severity
 * @param str_buf String buffer to print
 */
void __ios_log_print(dmLogSeverity severity, const char* str_buf);


#endif // DM_LOG_H
