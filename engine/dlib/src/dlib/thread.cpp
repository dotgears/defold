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

#include <assert.h>
#include <dmsdk/dlib/thread.h>

#if defined(_WIN32)
#include <stdlib.h>
#include <wchar.h>
#endif

namespace dmThread
{
#if defined(__linux__) || defined(__MACH__) || defined(__EMSCRIPTEN__) || defined(__NX__)
    struct ThreadData
    {
        ThreadStart m_Start;
        const char* m_Name;
        void*       m_Arg;
    };

    static void ThreadStartProxy(void* arg)
    {
        ThreadData* data = (ThreadData*) arg;
#if defined(__MACH__)
        int ret = pthread_setname_np(data->m_Name);
        assert(ret == 0);
#elif defined(__EMSCRIPTEN__)
#else
        int ret = pthread_setname_np(pthread_self(), data->m_Name);
        assert(ret == 0);
#endif
        data->m_Start(data->m_Arg);
        delete data;
    }

    Thread New(ThreadStart thread_start, uint32_t stack_size, void* arg, const char* name)
    {
        pthread_attr_t attr;
#if defined(__NX__)
        long page_size = -1;
#else
        long page_size = sysconf(_SC_PAGESIZE);
#endif
        int ret = pthread_attr_init(&attr);
        assert(ret == 0);

        if (page_size == -1)
            page_size = 4096;

        if (PTHREAD_STACK_MIN > stack_size)
            stack_size = PTHREAD_STACK_MIN;

        // NOTE: At least on OSX the stack-size must be a multiple of page size
        stack_size /= page_size;
        stack_size += 1;
        stack_size *= page_size;

        ret = pthread_attr_setstacksize(&attr, stack_size);
        assert(ret == 0);

        pthread_t thread;

        ThreadData* thread_data = new ThreadData;
        thread_data->m_Start = thread_start;
        thread_data->m_Name = name;
        thread_data->m_Arg = arg;

        ret = pthread_create(&thread, &attr, (void* (*)(void*)) ThreadStartProxy, thread_data);
        assert(ret == 0);
        ret = pthread_attr_destroy(&attr);
        assert(ret == 0);


        return thread;
    }

    void Join(Thread thread)
    {
        int ret = pthread_join(thread, 0);
        assert(ret == 0);
    }

    TlsKey AllocTls()
    {
        pthread_key_t key;
        int ret = pthread_key_create(&key, 0);
        assert(ret == 0);
        return key;
    }

    void FreeTls(TlsKey key)
    {
        int ret = pthread_key_delete(key);
        assert(ret == 0);
    }

    void SetTlsValue(TlsKey key, void* value)
    {
        int ret = pthread_setspecific(key, value);
        assert(ret == 0);
    }

    void* GetTlsValue(TlsKey key)
    {
        return pthread_getspecific(key);
    }

    Thread GetCurrentThread()
    {
        return pthread_self();
    }

    void SetThreadName(Thread thread, const char* name)
    {
#if defined(__MACH__)
        (void)thread;
        pthread_setname_np(name);
#elif defined(__EMSCRIPTEN__)
#else
        pthread_setname_np(thread, name);
#endif
    }

#elif defined(_WIN32)

    static void* GetFunctionPtr(const char* dllname, const char* fnname)
    {
        return (void*)GetProcAddress(GetModuleHandleA(dllname), fnname);
    }

    typedef HRESULT (*PfnSetThreadDescription)(HANDLE,PCWSTR);

    void SetThreadName(Thread thread, const char* name)
    {
        (void)thread;
        (void)name;
    // Currently, this crashed mysteriously on Win32, so we'll keep it only for Win64 until we've figured it out
    #if defined(_WIN64)
        static PfnSetThreadDescription pfn = (PfnSetThreadDescription)GetFunctionPtr("kernel32.dll", "SetThreadDescription");
        if (pfn) {
            size_t wn = mbsrtowcs(NULL, &name, 0, NULL);
            wchar_t* buf = (wchar_t*)malloc((wn + 1) * sizeof(wchar_t));
            wn = mbsrtowcs(buf, &name, wn + 1, NULL);

            pfn(thread, buf);

            free(buf);
        }
    #endif
    }

    Thread New(ThreadStart thread_start, uint32_t stack_size, void* arg, const char* name)
    {
        DWORD thread_id;
        HANDLE thread = CreateThread(NULL, stack_size,
                                     (LPTHREAD_START_ROUTINE) thread_start,
                                     arg, 0, &thread_id);
        assert(thread);

        SetThreadName((Thread)thread, name);

        return thread;
    }

    void Join(Thread thread)
    {
        uint32_t ret = WaitForSingleObject(thread, INFINITE);
        assert(ret == WAIT_OBJECT_0);
    }

    TlsKey AllocTls()
    {
        return TlsAlloc();
    }

    void FreeTls(TlsKey key)
    {
        BOOL ret = TlsFree(key);
        assert(ret);
    }

    void SetTlsValue(TlsKey key, void* value)
    {
        BOOL ret = TlsSetValue(key, value);
        assert(ret);
    }

    void* GetTlsValue(TlsKey key)
    {
        return TlsGetValue(key);
    }

    Thread GetCurrentThread()
    {
        return ::GetCurrentThread();
    }

#else
#error "Unsupported platform"
#endif

}



