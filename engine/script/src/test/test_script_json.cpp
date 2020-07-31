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

#include <dlib/dstrings.h>
#include <dlib/hash.h>
#include <dlib/log.h>
#include <dlib/configfile.h>
#include <dmsdk/dlib/json.h>

extern "C"
{
#include <lua/lauxlib.h>
#include <lua/lualib.h>
}

#define PATH_FORMAT "build/default/src/test/%s"

class ScriptJsonTest : public jc_test_base_class
{
protected:
    virtual void SetUp()
    {
        dmConfigFile::Result r = dmConfigFile::Load("src/test/test.config", 0, 0, &m_ConfigFile);
        ASSERT_EQ(dmConfigFile::RESULT_OK, r);

        m_Context = dmScript::NewContext(m_ConfigFile, 0, true);
        dmScript::Initialize(m_Context);
        L = dmScript::GetLuaState(m_Context);
    }

    virtual void TearDown()
    {
        dmConfigFile::Delete(m_ConfigFile);
        dmScript::Finalize(m_Context);
        dmScript::DeleteContext(m_Context);
    }

    dmScript::HContext m_Context;
    dmConfigFile::HConfig m_ConfigFile;
    lua_State* L;
};

bool RunFile(lua_State* L, const char* filename)
{
    char path[64];
    dmSnPrintf(path, 64, PATH_FORMAT, filename);
    if (luaL_dofile(L, path) != 0)
    {
        dmLogError("%s", lua_tolstring(L, -1, 0));
        return false;
    }
    return true;
}

TEST_F(ScriptJsonTest, TestJson)
{
    int top = lua_gettop(L);

    ASSERT_TRUE(RunFile(L, "test_json.luac"));

    lua_getglobal(L, "functions");
    ASSERT_EQ(LUA_TTABLE, lua_type(L, -1));
    lua_getfield(L, -1, "test_json");
    ASSERT_EQ(LUA_TFUNCTION, lua_type(L, -1));
    int result = dmScript::PCall(L, 0, LUA_MULTRET);
    if (result == LUA_ERRRUN)
    {
        ASSERT_TRUE(false);
    }
    else
    {
        ASSERT_EQ(0, result);
    }
    lua_pop(L, 1);


    ASSERT_EQ(top, lua_gettop(L));
}

struct JsonToLuaParams
{
    const char* m_JsonStr;
    bool m_ExpectedParseOK;
    bool m_ExpectedConvertOK;
};

class JsonToLuaTest : public jc_test_params_class<JsonToLuaParams>
{
protected:
    virtual void SetUp()
    {
        dmConfigFile::Result r = dmConfigFile::Load("src/test/test.config", 0, 0, &m_ConfigFile);
        ASSERT_EQ(dmConfigFile::RESULT_OK, r);

        m_Context = dmScript::NewContext(m_ConfigFile, 0, true);
        dmScript::Initialize(m_Context);
        L = dmScript::GetLuaState(m_Context);
    }

    virtual void TearDown()
    {
        dmConfigFile::Delete(m_ConfigFile);
        dmScript::Finalize(m_Context);
        dmScript::DeleteContext(m_Context);
    }

    dmScript::HContext m_Context;
    dmConfigFile::HConfig m_ConfigFile;
    lua_State* L;
};

TEST_P(JsonToLuaTest, TestJsonToLua)
{
    int top = lua_gettop(L);

    const JsonToLuaParams& p = GetParam();
    dmLogInfo("Expected %s: %s", p.m_ExpectedParseOK && p.m_ExpectedConvertOK ? "valid" : "invalid", p.m_JsonStr);

    dmJson::Document doc;
    dmJson::Result r = dmJson::Parse(p.m_JsonStr, &doc);

    if (p.m_ExpectedParseOK) {
        ASSERT_EQ(r, dmJson::RESULT_OK);
        ASSERT_TRUE(doc.m_NodeCount > 0);

        char err_str[128];
        int top_before_call = lua_gettop(L);
        int convert_r = dmScript::JsonToLua(L, &doc, 0, err_str, sizeof(err_str));

        if (p.m_ExpectedConvertOK) {
            ASSERT_NE(-1, convert_r);
            lua_pop(L, 1);
        } else {
            ASSERT_EQ(-1, convert_r);
            ASSERT_EQ(top_before_call, lua_gettop(L));
        }

        dmJson::Free(&doc);
    } else {
        ASSERT_NE(r, dmJson::RESULT_OK);
    }

    ASSERT_EQ(top, lua_gettop(L));
}

const JsonToLuaParams json_to_lua_setups[] = {
    // VALID
    {"null", true, true},
    {"true", true, true},
    {"false", true, true},
    {"10", true, true},
    {"010", true, true},
    {"-10", true, true},
    {"-010", true, true},
    {"0", true, true},
    {"-0", true, true},
    {"10.05", true, true},
    {"10.0", true, true},
    {"10.00", true, true},
    {"010.0", true, true},
    {"-10.05", true, true},
    {"-10.0", true, true},
    {"-10.00", true, true},
    {"-010.0", true, true},
    {"0.0", true, true},
    {"-0.0", true, true},
    {"00.0", true, true},
    {"{ \"response\" : 123 }", true, true},
    {"{ \"data\": \"asd\"}", true, true}, // DEF-3707

    // INVALID
    {"{", false, false},
    {"Null", true, false},
    {"NULL", true, false},
    {"True", true, false},
    {"TRUE", true, false},
    {"False", true, false},
    {"FALSE", true, false},
    {"defold", true, false},
    {"0.d3", true, false},
    {"{1 2 3}", true, false},
    {"{1: 2, 3}", true, false},
    {"{ response = \"ok\" }", true, false},
    {"{ 'data': 'asd' }", true, false}, // DEF-3707
};

INSTANTIATE_TEST_CASE_P(JsonToLuaTestSequence, JsonToLuaTest, jc_test_values_in(json_to_lua_setups));

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);

    int ret = jc_test_run_all();
    return ret;
}
