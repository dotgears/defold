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

#include "gamesys.h"

#include <dlib/log.h>
#include <physics/physics.h>

#include "gamesys_ddf.h"
#include "gamesys.h"
#include "gamesys_private.h"

#include "scripts/script_label.h"
#include "scripts/script_particlefx.h"
#include "scripts/script_tilemap.h"
#include "scripts/script_physics.h"
#include "scripts/script_sound.h"
#include "scripts/script_sprite.h"
#include "scripts/script_factory.h"
#include "scripts/script_collection_factory.h"
#include "scripts/script_spine_model.h"
#include "scripts/script_resource.h"
#include "scripts/script_model.h"
#include "scripts/script_window.h"
#include "scripts/script_collectionproxy.h"
#include "scripts/script_buffer.h"
#include <liveupdate/liveupdate.h>

extern "C"
{
#include <lua/lauxlib.h>
#include <lua/lualib.h>
}

namespace dmGameSystem
{

    ScriptLibContext::ScriptLibContext()
    {
        memset(this, 0, sizeof(*this));
    }

    bool InitializeScriptLibs(const ScriptLibContext& context)
    {
        lua_State* L = context.m_LuaState;

        int top = lua_gettop(L);
        (void)top;

        bool result = true;

        ScriptBufferRegister(context);
        ScriptLabelRegister(context);
        ScriptParticleFXRegister(context);
        ScriptTileMapRegister(context);
        ScriptPhysicsRegister(context);
        ScriptFactoryRegister(context);
        ScriptCollectionFactoryRegister(context);
        ScriptSpriteRegister(context);
        ScriptSoundRegister(context);
        ScriptSpineModelRegister(context);
        ScriptResourceRegister(context);
        ScriptModelRegister(context);
        ScriptWindowRegister(context);
        ScriptCollectionProxyRegister(context);

        assert(top == lua_gettop(L));
        return result;
    }

    void FinalizeScriptLibs(const ScriptLibContext& context)
    {
        ScriptCollectionProxyFinalize(context);
        ScriptLabelFinalize(context);
        ScriptPhysicsFinalize(context);
        ScriptResourceFinalize(context);
        ScriptWindowFinalize(context);
    }

    dmGameObject::HInstance CheckGoInstance(lua_State* L) {
        dmGameObject::HInstance instance = dmGameObject::GetInstanceFromLua(L);
        if (instance == 0) {
            dmGui::HScene scene = dmGui::GetSceneFromLua(L);
            if (scene != 0) {
                instance = (dmGameObject::HInstance)GuiGetUserDataCallback(scene);
            }
        }
        // No instance for render scripts, ignored
        if (instance == 0) {
            luaL_error(L, "no instance could be found in the current script environment");
        }
        return instance;
    }


    void OnWindowFocus(bool focus)
    {
        ScriptWindowOnWindowFocus(focus);
        // We need to call ScriptWindowOnWindowFocus before ScriptSoundOnWindowFocus to
        // allow the is_music_playing() script function to return the correct result.
        // When the window activation is received the application sound is not yet playing
        // any sounds and the Android platform function will return the correct result.
        // Once ScriptSoundOnWindowFocus is called when focus is gained we always say
        // that background music is off if the game is playing music and the app has focus
        ScriptSoundOnWindowFocus(focus);
    }

    void OnWindowIconify(bool iconify)
    {
        ScriptWindowOnWindowIconify(iconify);
    }

    void OnWindowResized(int width, int height)
    {
        ScriptWindowOnWindowResized(width, height);
    }

    void OnWindowCreated(int width, int height)
    {
        ScriptWindowOnWindowCreated(width, height);
    }
}
