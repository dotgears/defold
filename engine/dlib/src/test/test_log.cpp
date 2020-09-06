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

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <map>
#include "../dlib/array.h"
#include "../dlib/dlib.h"
#include "../dlib/hash.h"
#include "../dlib/log.h"
#include "../dlib/dstrings.h"
#include "../dlib/socket.h"
#include "../dlib/thread.h"
#include "../dlib/time.h"
#include "../dlib/path.h"
#include "../dlib/sys.h"
#define JC_TEST_IMPLEMENTATION
#include <jc_test/jc_test.h>

TEST(dmLog, Init)
{
    dmLogParams params;
    dmLogInitialize(&params);
    dmLogFinalize();
}

static void LogThread(void* arg)
{
    dmLogWarning("a warning %d", 123);
    dmLogError("an error %d", 456);

    int n = 1024 * 1024;
    char* s = new char[n];
    for (int i = 0; i < n; ++i) {
        s[i] = '.';
    }
    const char* msg = "Very large message";
    memcpy(s, msg, strlen(msg));
    s[n-1] = '\0';
    dmLogInfo("%s", s);
    delete[] s;
}

#if !(defined(__EMSCRIPTEN__) || defined(__NX__))
TEST(dmLog, Client)
{
    char buf[256];
    dmLogParams params;
    dmLogInitialize(&params);
    uint16_t port = dmLogGetPort();
    ASSERT_GT(port, 0);
    dmSnPrintf(buf, sizeof(buf), "python src/test/test_log.py %d", port);
#ifdef _WIN32
    FILE* f = _popen(buf, "rb");
#else
    FILE* f = popen(buf, "r");
#endif
    ASSERT_NE((void*) 0, f);
    // Wait for test_log.py to be ready, ie connection established
    int c = fgetc(f);
    ASSERT_EQ(255, c);
    ASSERT_NE((void*) 0, f);
    dmThread::Thread log_thread = dmThread::New(LogThread, 0x80000, 0, "test");

    size_t n;

    do
    {
        n = fread(buf, 1, 1, f);
        for (size_t i = 0; i < n; ++i)
            printf("%c", buf[i]);
    } while (n > 0);

#ifdef _WIN32
    _pclose(f);
#else
    pclose(f);
#endif
    dmThread::Join(log_thread);
    dmLogFinalize();
}
#endif

TEST(dmLog, LogFile)
{
    if (!dLib::FeaturesSupported(DM_FEATURE_BIT_SOCKET_SERVER_TCP))
    {
        printf("Test disabled due to platform not supporting TCP");
        return;
    }

    char path[DMPATH_MAX_PATH];
    dmSys::GetLogPath(path, sizeof(path));
    dmStrlCat(path, "log.txt", sizeof(path));

    dmLogParams params;
    dmLogInitialize(&params);
    dmSetLogFile(path);
    dmLogInfo("TESTING_LOG");
    dmLogFinalize();

    char tmp[1024];
    FILE* f = fopen(path, "rb");
    ASSERT_NE((FILE*) 0, f);
    if (f) {
        fread(tmp, 1, sizeof(tmp), f);
        ASSERT_TRUE(strstr(tmp, "TESTING_LOG") != 0);
        fclose(f);
    }
    dmSys::Unlink(path);
}

static void TestLogCaptureCallback(void* user_data, const char* log)
{
    dmArray<char>* log_output = (dmArray<char>*)user_data;
    uint32_t len = (uint32_t)strlen(log);
    log_output->SetCapacity(log_output->Size() + len + 1);
    log_output->PushArray(log, len);
}

TEST(dmLog, TestCapture)
{
    dmArray<char> log_output;
    dmSetCustomLogCallback(TestLogCaptureCallback, &log_output);
    dmLogDebug("This is a debug message");
    dmLogInfo("This is a info message");
    dmLogWarning("This is a warning message");
    dmLogError("This is a error message");
    dmLogFatal("This is a fata message");

    log_output.Push(0);

    const char* ExpectedOutput =
                "INFO:DLIB: This is a info message\n"
                "WARNING:DLIB: This is a warning message\n"
                "ERROR:DLIB: This is a error message\n"
                "FATAL:DLIB: This is a fata message\n";

    ASSERT_STREQ(ExpectedOutput,
                log_output.Begin());
    dmSetCustomLogCallback(0x0, 0x0);
}

int main(int argc, char **argv)
{
    dmSocket::Initialize();
    jc_test_init(&argc, argv);
    int ret = jc_test_run_all();
    dmSocket::Finalize();
    return ret;
}

