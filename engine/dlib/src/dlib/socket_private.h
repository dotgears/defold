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

#ifndef DM_SOCKET_PRIVATE_H
#define DM_SOCKET_PRIVATE_H

namespace dmSocket
{
#if defined(__linux__) || defined(__MACH__) || defined(__EMSCRIPTEN__) || defined(__NX__)
    #define DM_SOCKET_ERRNO errno
    #define DM_SOCKET_HERRNO h_errno
#else
    #define DM_SOCKET_ERRNO WSAGetLastError()
    #define DM_SOCKET_HERRNO WSAGetLastError()
#endif

    Result PlatformInitialize();
    Result PlatformFinalize();

#if defined(__NX__)
    int gethostname(char*, int);
#endif

}

#endif // #ifndef DM_SOCKET_PRIVATE_H

