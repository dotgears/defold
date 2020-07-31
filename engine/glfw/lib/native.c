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

#include "../include/GL/glfw.h"
#include "../include/GL/glfw_native.h"

#define GLFW_EXPOSE_NATIVE_STUB(return_type, func_name) GLFWAPI return_type glfwGet##func_name() { return NULL; }

//========================================================================
// Expose stubs returning NULL for handles not available on current platform.
//========================================================================
#if !defined(__MACH__)
    GLFW_EXPOSE_NATIVE_STUB(id, iOSUIWindow);
    GLFW_EXPOSE_NATIVE_STUB(id, iOSUIView);
    GLFW_EXPOSE_NATIVE_STUB(id, iOSEAGLContext);
    GLFW_EXPOSE_NATIVE_STUB(id, OSXNSWindow);
    GLFW_EXPOSE_NATIVE_STUB(id, OSXNSView);
    GLFW_EXPOSE_NATIVE_STUB(id, OSXNSOpenGLContext);
#else
    #if defined(__arm__) || defined(__arm64__) || defined(IOS_SIMULATOR)
    GLFW_EXPOSE_NATIVE_STUB(id, OSXNSWindow);
    GLFW_EXPOSE_NATIVE_STUB(id, OSXNSView);
    GLFW_EXPOSE_NATIVE_STUB(id, OSXNSOpenGLContext);
    #else
    GLFW_EXPOSE_NATIVE_STUB(id, iOSUIWindow);
    GLFW_EXPOSE_NATIVE_STUB(id, iOSUIView);
    GLFW_EXPOSE_NATIVE_STUB(id, iOSEAGLContext);
    #endif
#endif

#if !defined(_WIN32)
    GLFW_EXPOSE_NATIVE_STUB(HWND, WindowsHWND);
    GLFW_EXPOSE_NATIVE_STUB(HGLRC, WindowsHGLRC);
#endif

#if !defined(ANDROID)
    GLFW_EXPOSE_NATIVE_STUB(EGLContext, AndroidEGLContext);
    GLFW_EXPOSE_NATIVE_STUB(EGLSurface, AndroidEGLSurface);
    GLFW_EXPOSE_NATIVE_STUB(JavaVM*, AndroidJavaVM);
    GLFW_EXPOSE_NATIVE_STUB(jobject, AndroidActivity);
    GLFW_EXPOSE_NATIVE_STUB(android_app*, AndroidApp);
#endif

#if !defined(__linux__) || defined(__EMSCRIPTEN__) || defined(ANDROID)
    GLFW_EXPOSE_NATIVE_STUB(Window, X11Window);
    GLFW_EXPOSE_NATIVE_STUB(GLXContext, X11GLXContext);
#endif

#undef GLFW_EXPOSE_NATIVE_STUB

