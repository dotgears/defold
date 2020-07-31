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

#include <dmsdk/extension/extension.h>

namespace dmWebView
{

static int WebView_ThrowError(lua_State* L)
{
    return luaL_error(L, "webview has been removed from core, please read /builtins/docs/webview.md for more information.");
}

static const luaL_reg WebView_methods[] =
{
    {"create", WebView_ThrowError},
    {"destroy", WebView_ThrowError},
    {"open", WebView_ThrowError},
    {"open_raw", WebView_ThrowError},
    {"eval", WebView_ThrowError},
    {"set_visible", WebView_ThrowError},
    {"set_position", WebView_ThrowError},
    {"is_visible", WebView_ThrowError},
    {0, 0}
};

static void LuaInit(lua_State* L)
{
    int top = lua_gettop(L);
    lua_getglobal(L, "webview");
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        luaL_register(L, "webview", WebView_methods);
    }
    lua_pop(L, 1);
    assert(top == lua_gettop(L));
}

static dmExtension::Result WebView_AppInitialize(dmExtension::AppParams* params)
{
    return dmExtension::RESULT_OK;
}

static dmExtension::Result WebView_AppFinalize(dmExtension::AppParams* params)
{
    return dmExtension::RESULT_OK;
}

static dmExtension::Result WebView_Initialize(dmExtension::Params* params)
{
    LuaInit(params->m_L);
    return dmExtension::RESULT_OK;
}

DM_DECLARE_EXTENSION(WebViewExt, "WebView", WebView_AppInitialize, WebView_AppFinalize, WebView_Initialize, 0, 0, 0)

} // namespace
