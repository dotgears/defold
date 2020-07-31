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

#ifndef DM_ENGINE_PRIVATE_H
#define DM_ENGINE_PRIVATE_H

#include <stdint.h>

#include <dlib/configfile.h>
#include <dlib/hashtable.h>
#include <dlib/message.h>

#include <resource/resource.h>

#include <render/render.h>
#include <render/font_renderer.h>
#include <rig/rig.h>

#include <hid/hid.h>
#include <input/input.h>

#include <gameobject/gameobject.h>

#include <gui/gui.h>

#include <gamesys/gamesys.h>

#include <record/record.h>

#include "engine.h"
#include "engine_service.h"
#include "engine_ddf.h"

namespace dmEngine
{
    const uint32_t MAX_RUN_RESULT_ARGS = 32;
    struct RunResult
    {
        RunResult()
        {
            memset(this, 0, sizeof(*this));
        }

        //`RunResult::EXIT` value (as `run_action` in `AppDelegate.m` `ShutdownEngine()`)
        // compares with GLFW_APP_RUN_EXIT, that's why `RunResult` should have the same values as `glfwAppRunAction`
        enum Action
        {
            NONE,
            EXIT = -1,
            REBOOT = 1,
        };

        void Free()
        {
            for (uint32_t i = 0; i < MAX_RUN_RESULT_ARGS; ++i)
            {
                if (m_Argv[i])
                {
                    free(m_Argv[i]);
                }
            }
        }

        int     m_Argc;
        char*   m_Argv[MAX_RUN_RESULT_ARGS];
        int32_t m_ExitCode;
        Action  m_Action;
    };

    struct Stats
    {
        Stats();

        uint32_t m_FrameCount;
    };

    struct RecordData
    {
        RecordData()
        {
            memset(this, 0, sizeof(*this));
        }

        dmRecord::HRecorder m_Recorder;
        char*               m_Buffer;
        uint32_t            m_FrameCount;
        uint32_t            m_FramePeriod;
        uint32_t            m_Fps;
    };

    enum Vsync
    {
        VSYNC_SOFTWARE = 0,
        VSYNC_HARDWARE = 1,

    };

    struct Engine
    {
        Engine(dmEngineService::HEngineService engine_service);
        dmEngineService::HEngineService             m_EngineService;
        dmConfigFile::HConfig                       m_Config;

        PreRun                                      m_PreRun;
        PostRun                                     m_PostRun;
        void*                                       m_PrePostRunContext;
        RunResult                                   m_RunResult;
        bool                                        m_Alive;

        dmGameObject::HRegister                     m_Register;
        dmGameObject::HCollection                   m_MainCollection;
        dmArray<dmGameObject::InputAction>          m_InputBuffer;

        uint32_t                                    m_LastReloadMTime;

        float                                       m_MouseSensitivity;

        dmGraphics::HContext                        m_GraphicsContext;
        dmRender::HRenderContext                    m_RenderContext;
        dmGameSystem::PhysicsContext                m_PhysicsContext;
        dmGameSystem::ParticleFXContext             m_ParticleFXContext;
        /// If the shared context is set, the three environment specific contexts below will point to the same context
        dmScript::HContext                          m_SharedScriptContext;
        dmScript::HContext                          m_GOScriptContext;
        dmScript::HContext                          m_RenderScriptContext;
        dmScript::HContext                          m_GuiScriptContext;
        dmResource::HFactory                        m_Factory;
        dmGameSystem::GuiContext                    m_GuiContext;
        dmMessage::HSocket                          m_SystemSocket;
        dmGameSystem::SpriteContext                 m_SpriteContext;
        dmGameSystem::CollectionProxyContext        m_CollectionProxyContext;
        dmGameSystem::FactoryContext                m_FactoryContext;
        dmGameSystem::CollectionFactoryContext      m_CollectionFactoryContext;
        dmGameSystem::SpineModelContext             m_SpineModelContext;
        dmGameSystem::ModelContext                  m_ModelContext;
        dmGameSystem::MeshContext                   m_MeshContext;
        dmGameSystem::LabelContext                  m_LabelContext;
        dmGameSystem::TilemapContext                m_TilemapContext;
        dmGameSystem::SoundContext                  m_SoundContext;
        dmGameObject::ModuleContext                 m_ModuleContext;

        dmRender::HFontMap                          m_SystemFontMap;
        dmHID::HContext                             m_HidContext;
        dmInput::HContext                           m_InputContext;
        dmInput::HBinding                           m_GameInputBinding;
        dmRender::HDisplayProfiles                  m_DisplayProfiles;

        dmGameSystem::RenderScriptPrototype*        m_RenderScriptPrototype;

        Stats                                       m_Stats;

        bool                                        m_UseSwVsync;
        bool                                        m_UseVariableDt;
        bool                                        m_WasIconified;
        bool                                        m_QuitOnEsc;
        bool                                        m_ConnectionAppMode;        //!< If the app was started on a device, listening for connections
        bool                                        m_RunWhileIconified;
        uint64_t                                    m_PreviousFrameTime;
        uint64_t                                    m_PreviousRenderTime;
        uint64_t                                    m_FlipTime;
        uint32_t                                    m_UpdateFrequency;
        uint32_t                                    m_Width;
        uint32_t                                    m_Height;
        uint32_t                                    m_ClearColor;
        float                                       m_InvPhysicalWidth;
        float                                       m_InvPhysicalHeight;
        Vsync                                       m_VsyncMode;

        RecordData                                  m_RecordData;
    };


    HEngine New(dmEngineService::HEngineService engine_service);
    void Delete(HEngine engine);
    bool Init(HEngine engine, int argc, char *argv[]);
    RunResult InitRun(dmEngineService::HEngineService engine_service, int argc, char *argv[], PreRun pre_run, PostRun post_run, void* context);
    void Step(HEngine engine);

    void ReloadResources(HEngine engine, const char* extension);
    bool LoadBootstrapContent(HEngine engine, dmConfigFile::HConfig config);
    void UnloadBootstrapContent(HEngine engine);
}

#endif // DM_ENGINE_PRIVATE_H
