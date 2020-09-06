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
#include "script_private.h"

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

#include <script/lua_source_ddf.h>

class ScriptModuleTest : public jc_test_base_class
{
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

// NOTE: we don't generate actual bytecode for this test-data, so
// just pass in regular lua source instead.
static dmLuaDDF::LuaSource* LuaSourceFromText(const char *text)
{
    static dmLuaDDF::LuaSource tmp;
    memset(&tmp, 0x00, sizeof(tmp));
    tmp.m_Script.m_Data = (uint8_t*)text;
    tmp.m_Script.m_Count = strlen(text);
    tmp.m_Bytecode.m_Data = (uint8_t*)text;
    tmp.m_Bytecode.m_Count = strlen(text);
    tmp.m_Bytecode64.m_Data = (uint8_t*)text;
    tmp.m_Bytecode64.m_Count = strlen(text);
    tmp.m_Filename = "dummy";
    return &tmp;
}

bool RunFile(lua_State* L, const char* filename)
{
    char path[64];
    dmSnPrintf(path, 64, MOUNTFS PATH_FORMAT, filename);
    if (luaL_dofile(L, path) != 0)
    {
        const char* str = lua_tostring(L, -1);
        dmLogError("%s", str);
        lua_pop(L, 1);
        return false;
    }
    return true;
}

TEST_F(ScriptModuleTest, TestModule)
{
    int top = lua_gettop(L);
    const char* script = "module(..., package.seeall)\n function f1()\n return 123\n end\n";
    const char* script_file_name = "x.test_mod";
    ASSERT_FALSE(dmScript::ModuleLoaded(m_Context, script_file_name));
    dmScript::Result ret = dmScript::AddModule(m_Context, LuaSourceFromText(script), script_file_name, 0, dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_OK, ret);
    ASSERT_TRUE(dmScript::ModuleLoaded(m_Context, script_file_name));
    ASSERT_TRUE(RunFile(L, "test_module.luac"));
    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptModuleTest, TestReload)
{
    int top = lua_gettop(L);
    const char* script = "module(..., package.seeall)\n function f1()\n return 123\n end\n";
    const char* script_reload = "module(..., package.seeall)\n reloaded = 1010\n function f1()\n return 456\n end\n";
    const char* script_file_name = "x.test_mod";
    ASSERT_FALSE(dmScript::ModuleLoaded(m_Context, script_file_name));
    dmScript::Result ret = dmScript::AddModule(m_Context, LuaSourceFromText(script), script_file_name, 0, dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_OK, ret);
    ASSERT_TRUE(dmScript::ModuleLoaded(m_Context, script_file_name));
    ASSERT_TRUE(RunFile(L, "test_module.luac"));

    ret = dmScript::ReloadModule(m_Context, LuaSourceFromText(script_reload), dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_OK, ret);
    lua_getfield(L, LUA_GLOBALSINDEX, "x");
    lua_getfield(L, -1, "test_mod");
    lua_getfield(L, -1, "reloaded");
    int reloaded = luaL_checkinteger(L, -1);
    ASSERT_EQ(1010, reloaded);
    lua_pop(L, 3);

    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptModuleTest, TestReloadReturn)
{
    int top = lua_gettop(L);
    const char* script = "local M = {}\nreturn M\n";
    const char* script_file_name = "x.test_mod";
    ASSERT_FALSE(dmScript::ModuleLoaded(m_Context, script_file_name));
    dmScript::Result ret = dmScript::AddModule(m_Context, LuaSourceFromText(script), script_file_name, 0, dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_OK, ret);
    ASSERT_TRUE(dmScript::ModuleLoaded(m_Context, script_file_name));

    ret = dmScript::ReloadModule(m_Context, LuaSourceFromText(script), dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_OK, ret);

    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptModuleTest, TestReloadFail)
{
    int top = lua_gettop(L);
    const char* script = "module(..., package.seeall)\n reloaded = 1010\n function f1()\n return 123\n end\n";
    const char* script_reload = "module(..., package.seeall)\n reloaded = -1\n function f1()\n return 123\n en\n"; // NOTE: en instead of end
    const char* script_file_name = "x.test_mod";
    ASSERT_FALSE(dmScript::ModuleLoaded(m_Context, script_file_name));
    dmScript::Result ret = dmScript::AddModule(m_Context, LuaSourceFromText(script), script_file_name, 0, dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_OK, ret);
    ASSERT_TRUE(dmScript::ModuleLoaded(m_Context, script_file_name));
    ASSERT_TRUE(RunFile(L, "test_module.luac"));

    ret = dmScript::ReloadModule(m_Context, LuaSourceFromText(script_reload), dmHashString64(script_file_name));
    ASSERT_EQ(dmScript::RESULT_LUA_ERROR, ret);
    lua_getfield(L, LUA_GLOBALSINDEX, "x");
    lua_getfield(L, -1, "test_mod");
    lua_getfield(L, -1, "reloaded");
    int reloaded = luaL_checkinteger(L, -1);
    ASSERT_EQ(1010, reloaded);
    lua_pop(L, 3);

    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptModuleTest, TestModuleMissing)
{
    int top = lua_gettop(L);
    ASSERT_FALSE(RunFile(L, "test_module_missing.luac"));
    ASSERT_EQ(top, lua_gettop(L));
}

TEST_F(ScriptModuleTest, TestReloadNotLoaded)
{
    int top = lua_gettop(L);
    dmScript::Result ret = dmScript::ReloadModule(m_Context, LuaSourceFromText(""), dmHashString64("not_loaded"));
    ASSERT_EQ(dmScript::RESULT_MODULE_NOT_LOADED, ret);
    ASSERT_EQ(top, lua_gettop(L));
}

struct ChunknameParam
{
    const char* m_Input;
    const char* m_Expected;
};

class ChunknameTests : public jc_test_params_class<ChunknameParam>
{
protected:
    void SetUp() {};
    void TearDown() {};
};

// Verify that Lua chunknames are prefixed with '=' and the last parts
// of the paths are taken, not from the start.
TEST_P(ChunknameTests, Chunkname)
{
    const ChunknameParam& param = GetParam();
    char tmp[61];
    dmScript::PrefixFilename(dmScript::FindSuitableChunkname(param.m_Input), '=', tmp, sizeof(tmp));
    ASSERT_EQ('=', tmp[0]);
    ASSERT_STREQ(param.m_Expected, tmp);
}

ChunknameParam chunkname_tests[] = {
    {"", "="},
    {"a.script", "=a.script"},
    {"abbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.script", "=abbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.script"},
    {"abbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.script", "=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.script"},
    {"aabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.script", "=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb.script"},
};
INSTANTIATE_TEST_CASE_P(Test, ChunknameTests, jc_test_values_in(chunkname_tests));

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);

    int ret = jc_test_run_all();
    return ret;
}
