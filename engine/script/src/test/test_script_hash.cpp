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

#define JC_TEST_IMPLEMENTATION
#include <jc_test/jc_test.h>

#include "script.h"
#include "script_hash.h"

#include <dlib/dstrings.h>
#include <dlib/hash.h>
#include <dlib/log.h>

extern "C"
{
#include <lua/lauxlib.h>
#include <lua/lualib.h>
}

#define PATH_FORMAT "build/default/src/test/%s"

#if defined(__NX__)
    #define MOUNTFS "host:/"
#else
    #define MOUNTFS
#endif

class ScriptHashTest : public jc_test_base_class
{
protected:
protected:
    virtual void SetUp()
    {
        m_Context = dmScript::NewContext(0, 0, true);
        dmScript::Initialize(m_Context);

        L = dmScript::GetLuaState(m_Context);
    }

    virtual void TearDown()
    {
        dmScript::Finalize(m_Context);
        dmScript::DeleteContext(m_Context);
    }

    dmScript::HContext m_Context;
    lua_State* L;
};

bool RunFile(lua_State* L, const char* filename)
{
    char path[64];
    dmSnPrintf(path, 64, MOUNTFS PATH_FORMAT, filename);
    if (luaL_dofile(L, path) != 0)
    {
        dmLogError("%s", lua_tolstring(L, -1, 0));
        return false;
    }
    return true;
}

bool RunString(lua_State* L, const char* script)
{
    if (luaL_dostring(L, script) != 0)
    {
        dmLogError("%s", lua_tolstring(L, -1, 0));
        return false;
    }
    return true;
}

TEST_F(ScriptHashTest, TestHash)
{
    int top = lua_gettop(L);

    char hash_hex[64];
    const char* s = "test_value";
    dmhash_t hash = dmHashString64(s);
    dmSnPrintf(hash_hex, sizeof(hash_hex), "%016llx", (unsigned long long)hash);
    dmScript::PushHash(L, hash);
    ASSERT_EQ(hash, dmScript::CheckHash(L, -1));
    lua_pop(L, 1);

    ASSERT_EQ(top, lua_gettop(L));

    ASSERT_TRUE(RunFile(L, "test_hash.luac"));

    lua_getglobal(L, "functions");
    ASSERT_EQ(LUA_TTABLE, lua_type(L, -1));
    lua_getfield(L, -1, "test_hash");
    ASSERT_EQ(LUA_TFUNCTION, lua_type(L, -1));
    dmScript::PushHash(L, hash);
    lua_pushstring(L, hash_hex);
    int result = dmScript::PCall(L, 2, LUA_MULTRET);
    if (result == LUA_ERRRUN)
    {
        ASSERT_TRUE(false);
    }
    else
    {
        ASSERT_EQ(0, result);
    }
    lua_pop(L, 1);

    dmScript::PushHash(L, dmHashString64("test"));
    ASSERT_TRUE(dmScript::IsHash(L, -1));
    lua_pop(L, 1);
    ASSERT_FALSE(dmScript::IsHash(L, -1));

    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptHashTest, TestHashUnknown)
{
    int top = lua_gettop(L);
    (void)top;

    dmhash_t hash = 1234;
    dmScript::PushHash(L, hash);
    lua_setglobal(L, "test_hash");
    const char* script =
        "print(\"tostring: \" .. tostring(test_hash))\n"
        "print(\"concat: \" .. test_hash)\n"
        "print(test_hash .. \" :concat\")\n"
        "print(test_hash .. test_hash)\n";
    ASSERT_TRUE(RunString(L, script));

    ASSERT_EQ(top, lua_gettop(L));
}


TEST_F(ScriptHashTest, TestGetStringFromHashOrString)
{
    int top = lua_gettop(L);
    (void)top;

    dmHashEnableReverseHash(true);

    dmhash_t hash = dmHashString64("Hello");
    dmScript::PushHash(L, hash);

    char buffer[128];
    const char* s;

    s = dmScript::GetStringFromHashOrString(L, -1, buffer, sizeof(buffer));
    ASSERT_NE(0U, (uintptr_t)s);
    ASSERT_TRUE(strcmp("Hello", s) == 0);

    dmHashEnableReverseHash(false);

    s = dmScript::GetStringFromHashOrString(L, -1, buffer, sizeof(buffer));
    ASSERT_NE(0U, (uintptr_t)s);
    ASSERT_TRUE(strcmp("8244253450232885714", s) == 0);

    lua_pop(L, 1);
    lua_pushstring(L, "Lua Hello");

    s = dmScript::GetStringFromHashOrString(L, -1, buffer, sizeof(buffer));
    ASSERT_NE(0U, (uintptr_t)s);
    ASSERT_TRUE(strcmp("Lua Hello", s) == 0);

    lua_pop(L, 1);
    lua_pushnumber(L, 42.0);

    s = dmScript::GetStringFromHashOrString(L, -1, buffer, sizeof(buffer));
    ASSERT_NE(0U, (uintptr_t)s);
    ASSERT_TRUE(strcmp("<unknown>", s) == 0);

    lua_pop(L, 1);
    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptHashTest, TestHashTString) // def2821 - Making sure that the strings before/after the fix get hashed the same
{
    lua_pushstring(L, "Hello World!");
    const char* str = lua_tostring(L, -1);

    dmhash_t hash_tostring = dmHashString64(str);

    size_t len = 0;
    str = lua_tolstring(L, -1, &len);
    dmhash_t hash_tolstring = dmHashBuffer64(str, len);

    ASSERT_EQ(strlen(str), len);
    ASSERT_EQ(hash_tostring, hash_tolstring);
}

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);

    int ret = jc_test_run_all();
    return ret;
}
