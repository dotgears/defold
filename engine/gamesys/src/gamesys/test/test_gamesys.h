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

#include <resource/resource.h>

#include <dlib/buffer.h>
#include <hid/hid.h>

#include <sound/sound.h>
#include <gameobject/gameobject.h>
#include <rig/rig.h>

#include "gamesys/gamesys.h"
#include "gamesys/scripts/script_buffer.h"
#include "../components/comp_gui.h"

#define JC_TEST_IMPLEMENTATION
#include <jc_test/jc_test.h>

struct Params
{
    const char* m_ValidResource;
    const char* m_InvalidResource;
    const char* m_TempResource;
};

template<typename T>
class GamesysTest : public jc_test_params_class<T>
{
protected:
    virtual void SetUp();
    virtual void TearDown();

    dmGameObject::UpdateContext m_UpdateContext;
    dmGameObject::HRegister m_Register;
    dmGameObject::HCollection m_Collection;
    dmResource::HFactory m_Factory;

    dmScript::HContext m_ScriptContext;
    dmGraphics::HContext m_GraphicsContext;
    dmRender::HRenderContext m_RenderContext;
    dmGameSystem::PhysicsContext m_PhysicsContext;
    dmGameSystem::ParticleFXContext m_ParticleFXContext;
    dmGameSystem::GuiContext m_GuiContext;
    dmHID::HContext m_HidContext;
    dmInput::HContext m_InputContext;
    dmInputDDF::GamepadMaps* m_GamepadMapsDDF;
    dmGameSystem::SpriteContext m_SpriteContext;
    dmGameSystem::CollectionProxyContext m_CollectionProxyContext;
    dmGameSystem::FactoryContext m_FactoryContext;
    dmGameSystem::CollectionFactoryContext m_CollectionFactoryContext;
    dmGameSystem::ModelContext m_ModelContext;
    dmGameSystem::MeshContext m_MeshContext;
    dmGameSystem::SpineModelContext m_SpineModelContext;
    dmGameSystem::LabelContext m_LabelContext;
    dmGameSystem::TilemapContext m_TilemapContext;
    dmGameSystem::SoundContext m_SoundContext;
    dmRig::HRigContext m_RigContext;
    dmGameObject::ModuleContext m_ModuleContext;
};

class ResourceTest : public GamesysTest<const char*>
{
public:
    virtual ~ResourceTest() {}
};

struct ResourceReloadParams
{
    const char* m_FilenameEnding;
    const char* m_InitialResource;
    const char* m_SecondResource;
};

class ResourceReloadTest : public GamesysTest<ResourceReloadParams>
{
public:
    virtual ~ResourceReloadTest() {}
};

struct ResourceFailParams
{
    const char* m_ValidResource;
    const char* m_InvalidResource;
};

class ResourceFailTest : public GamesysTest<ResourceFailParams>
{
public:
    virtual ~ResourceFailTest() {}
};

class InvalidVertexSpaceTest : public GamesysTest<const char*>
{
public:
    virtual ~InvalidVertexSpaceTest() {}
};

class ComponentTest : public GamesysTest<const char*>
{
public:
    virtual ~ComponentTest() {}
};

class ComponentFailTest : public GamesysTest<const char*>
{
public:
    virtual ~ComponentFailTest() {}
};

struct FactoryTestParams
{
    const char* m_GOPath;
    bool m_IsDynamic;
    bool m_IsPreloaded;
};

class FactoryTest : public GamesysTest<FactoryTestParams>
{
public:
    virtual ~FactoryTest() {}
};

struct CollectionFactoryTestParams
{
    const char* m_GOPath;
    bool m_IsDynamic;
    bool m_IsPreloaded;
};

class CollectionFactoryTest : public GamesysTest<CollectionFactoryTestParams>
{
public:
    virtual ~CollectionFactoryTest() {}
};

class SpriteAnimTest : public GamesysTest<const char*>
{
public:
    virtual ~SpriteAnimTest() {}
};

class WindowEventTest : public GamesysTest<const char*>
{
public:
    virtual ~WindowEventTest() {}
};

struct DrawCountParams
{
    const char* m_GOPath;
    uint64_t m_ExpectedDrawCount;
};

class DrawCountTest : public GamesysTest<DrawCountParams>
{
public:
    virtual ~DrawCountTest() {}
};

struct BoxRenderParams
{
    const static uint8_t MAX_VERTICES_IN_9_SLICED_QUAD = 16;
    const static uint8_t MAX_INDICES_IN_9_SLICED_QUAD = 3 * 2 * 9;

    const char* m_GOPath;
    dmGameSystem::BoxVertex m_ExpectedVertices[MAX_VERTICES_IN_9_SLICED_QUAD];
    uint8_t m_ExpectedVerticesCount;
    int m_ExpectedIndices[MAX_INDICES_IN_9_SLICED_QUAD];
};

class BoxRenderTest : public GamesysTest<BoxRenderParams>
{
public:
    virtual ~BoxRenderTest() {}
};

class GamepadConnectedTest : public GamesysTest<const char*>
{
public:
    virtual ~GamepadConnectedTest() {}
};

struct ResourcePropParams {
    const char* m_PropertyName;
    const char* m_ResourcePath;
    const char* m_ResourcePathNotFound;
    const char* m_ResourcePathInvExt;
    const char* m_Component0;
    const char* m_Component1;
    const char* m_Component2;
    const char* m_Component3;
    const char* m_Component4;
    const char* m_Component5;
};

class ResourcePropTest : public GamesysTest<ResourcePropParams>
{
protected:
    void SetUp()
    {
        GamesysTest::SetUp();
    }
public:
    virtual ~ResourcePropTest() {}
};

class FlipbookTest : public GamesysTest<const char*>
{
public:
    virtual ~FlipbookTest() {}
};

struct CursorTestParams
{
    const char* m_AnimationId;
    float m_CursorStart;
    float m_PlaybackRate;
    float m_Expected[16];
    uint8_t m_ExpectedCount;
};

class CursorTest : public GamesysTest<CursorTestParams>
{
public:
    virtual ~CursorTest() {}
};

bool CopyResource(const char* src, const char* dst);
bool UnlinkResource(const char* name);

template<typename T>
void GamesysTest<T>::SetUp()
{
    dmSound::Initialize(0x0, 0x0);

    m_UpdateContext.m_DT = 1.0f / 60.0f;

    dmResource::NewFactoryParams params;
    params.m_MaxResources = 64;
    params.m_Flags = RESOURCE_FACTORY_FLAGS_RELOAD_SUPPORT;
    m_Factory = dmResource::NewFactory(&params, "build/default/src/gamesys/test");
    m_ScriptContext = dmScript::NewContext(0, m_Factory, true);
    dmScript::Initialize(m_ScriptContext);
    m_Register = dmGameObject::NewRegister();
    dmGameObject::Initialize(m_Register, m_ScriptContext);
    dmGameObject::RegisterResourceTypes(m_Factory, m_Register, m_ScriptContext, &m_ModuleContext);
    dmGameObject::RegisterComponentTypes(m_Factory, m_Register, m_ScriptContext);

    dmGraphics::Initialize();
    m_GraphicsContext = dmGraphics::NewContext(dmGraphics::ContextParams());
    dmRender::RenderContextParams render_params;
    render_params.m_MaxRenderTypes = 10;
    render_params.m_MaxInstances = 1000;
    render_params.m_MaxRenderTargets = 10;
    render_params.m_ScriptContext = m_ScriptContext;
    render_params.m_MaxCharacters = 256;
    m_RenderContext = dmRender::NewRenderContext(m_GraphicsContext, render_params);
    m_GuiContext.m_RenderContext = m_RenderContext;
    m_GuiContext.m_ScriptContext = m_ScriptContext;
    dmGui::NewContextParams gui_params;
    gui_params.m_ScriptContext = m_ScriptContext;
    gui_params.m_GetURLCallback = dmGameSystem::GuiGetURLCallback;
    gui_params.m_GetUserDataCallback = dmGameSystem::GuiGetUserDataCallback;
    gui_params.m_ResolvePathCallback = dmGameSystem::GuiResolvePathCallback;
    m_GuiContext.m_GuiContext = dmGui::NewContext(&gui_params);
    m_GuiContext.m_MaxParticleFXCount = 64;
    m_GuiContext.m_MaxParticleCount = 1024;
    m_GuiContext.m_MaxSpineCount = 8;

    m_HidContext = dmHID::NewContext(dmHID::NewContextParams());
    dmHID::Init(m_HidContext);
    dmInput::NewContextParams input_params;
    input_params.m_HidContext = m_HidContext;
    input_params.m_RepeatDelay = 0.3f;
    input_params.m_RepeatInterval = 0.1f;
    m_InputContext = dmInput::NewContext(input_params);

    memset(&m_PhysicsContext, 0, sizeof(m_PhysicsContext));
    m_PhysicsContext.m_3D = false;
    m_PhysicsContext.m_Context2D = dmPhysics::NewContext2D(dmPhysics::NewContextParams());

    m_ParticleFXContext.m_Factory = m_Factory;
    m_ParticleFXContext.m_RenderContext = m_RenderContext;
    m_ParticleFXContext.m_MaxParticleFXCount = 64;
    m_ParticleFXContext.m_MaxParticleCount = 256;

    m_SpriteContext.m_RenderContext = m_RenderContext;
    m_SpriteContext.m_MaxSpriteCount = 32;

    m_CollectionProxyContext.m_Factory = m_Factory;
    m_CollectionProxyContext.m_MaxCollectionProxyCount = 8;

    m_FactoryContext.m_MaxFactoryCount = 128;
    m_FactoryContext.m_ScriptContext = m_ScriptContext;
    m_CollectionFactoryContext.m_MaxCollectionFactoryCount = 128;
    m_CollectionFactoryContext.m_ScriptContext = m_ScriptContext;

    m_SpineModelContext.m_RenderContext = m_RenderContext;
    m_SpineModelContext.m_Factory = m_Factory;
    m_SpineModelContext.m_MaxSpineModelCount = 32;

    m_LabelContext.m_RenderContext = m_RenderContext;
    m_LabelContext.m_MaxLabelCount = 32;
    m_LabelContext.m_Subpixels     = 0;

    m_TilemapContext.m_RenderContext = m_RenderContext;
    m_TilemapContext.m_MaxTilemapCount = 16;
    m_TilemapContext.m_MaxTileCount = 512;

    m_ModelContext.m_RenderContext = m_RenderContext;
    m_ModelContext.m_Factory = m_Factory;
    m_ModelContext.m_MaxModelCount = 128;

    m_MeshContext.m_RenderContext = m_RenderContext;
    m_MeshContext.m_Factory       = m_Factory;
    m_MeshContext.m_MaxMeshCount  = 128;

    dmBuffer::NewContext();

    m_SoundContext.m_MaxComponentCount = 32;

    dmResource::Result r = dmGameSystem::RegisterResourceTypes(m_Factory, m_RenderContext, &m_GuiContext, m_InputContext, &m_PhysicsContext);
    assert(dmResource::RESULT_OK == r);

    dmResource::Get(m_Factory, "/input/valid.gamepadsc", (void**)&m_GamepadMapsDDF);
    assert(m_GamepadMapsDDF);
    dmInput::RegisterGamepads(m_InputContext, m_GamepadMapsDDF);

    assert(dmGameObject::RESULT_OK == dmGameSystem::RegisterComponentTypes(m_Factory, m_Register, m_RenderContext, &m_PhysicsContext, &m_ParticleFXContext, &m_GuiContext, &m_SpriteContext,
                                                                                                    &m_CollectionProxyContext, &m_FactoryContext, &m_CollectionFactoryContext, &m_SpineModelContext,
                                                                                                    &m_ModelContext, &m_MeshContext, &m_LabelContext, &m_TilemapContext, &m_SoundContext));

    m_Collection = dmGameObject::NewCollection("collection", m_Factory, m_Register, 1024);
}

template<typename T>
void GamesysTest<T>::TearDown()
{
    dmGameObject::DeleteCollection(m_Collection);
    dmGameObject::PostUpdate(m_Register);
    dmResource::Release(m_Factory, m_GamepadMapsDDF);
    dmGui::DeleteContext(m_GuiContext.m_GuiContext, m_ScriptContext);
    dmRender::DeleteRenderContext(m_RenderContext, m_ScriptContext);
    dmGraphics::DeleteContext(m_GraphicsContext);
    dmScript::Finalize(m_ScriptContext);
    dmScript::DeleteContext(m_ScriptContext);
    dmResource::DeleteFactory(m_Factory);
    dmGameObject::DeleteRegister(m_Register);
    dmSound::Finalize();
    dmInput::DeleteContext(m_InputContext);
    dmHID::Final(m_HidContext);
    dmHID::DeleteContext(m_HidContext);
    dmPhysics::DeleteContext2D(m_PhysicsContext.m_Context2D);
    dmBuffer::DeleteContext();
}

// Specific test class for testing dmBuffers in scripts
class ScriptBufferTest : public jc_test_base_class
{
protected:
    virtual void SetUp()
    {
        dmBuffer::NewContext();
        m_Context = dmScript::NewContext(0, 0, true);
        dmScript::Initialize(m_Context);

        m_ScriptLibContext.m_Factory = 0x0;
        m_ScriptLibContext.m_Register = 0x0;
        m_ScriptLibContext.m_LuaState = dmScript::GetLuaState(m_Context);
        dmGameSystem::InitializeScriptLibs(m_ScriptLibContext);

        L = dmScript::GetLuaState(m_Context);

        const dmBuffer::StreamDeclaration streams_decl[] = {
            {dmHashString64("rgb"), dmBuffer::VALUE_TYPE_UINT16, 3},
            {dmHashString64("a"), dmBuffer::VALUE_TYPE_FLOAT32, 1},
        };

        m_Count = 256;
        dmBuffer::Create(m_Count, streams_decl, 2, &m_Buffer);
    }

    virtual void TearDown()
    {
        if( m_Buffer )
            dmBuffer::Destroy(m_Buffer);

        dmGameSystem::FinalizeScriptLibs(m_ScriptLibContext);
        dmScript::Finalize(m_Context);
        dmScript::DeleteContext(m_Context);

        dmBuffer::DeleteContext();
    }

    dmGameSystem::ScriptLibContext m_ScriptLibContext;
    dmScript::HContext m_Context;
    lua_State* L;
    dmBuffer::HBuffer m_Buffer;
    uint32_t m_Count;
};

struct CopyBufferTestParams
{
    uint32_t m_Count;
    uint32_t m_DstOffset;
    uint32_t m_SrcOffset;
    uint32_t m_CopyCount;
    bool m_ExpectedOk;
};

class ScriptBufferCopyTest : public jc_test_params_class<CopyBufferTestParams>
{
protected:
    virtual void SetUp()
    {
        dmBuffer::NewContext();
        m_Context = dmScript::NewContext(0, 0, true);

        m_ScriptLibContext.m_Factory = 0x0;
        m_ScriptLibContext.m_Register = 0x0;
        m_ScriptLibContext.m_LuaState = dmScript::GetLuaState(m_Context);
        dmGameSystem::InitializeScriptLibs(m_ScriptLibContext);

        dmScript::Initialize(m_Context);
        L = dmScript::GetLuaState(m_Context);


        const dmBuffer::StreamDeclaration streams_decl[] = {
            {dmHashString64("rgb"), dmBuffer::VALUE_TYPE_UINT16, 3},
            {dmHashString64("a"), dmBuffer::VALUE_TYPE_FLOAT32, 1},
        };

        const CopyBufferTestParams& p = GetParam();
        dmBuffer::Create(p.m_Count, streams_decl, 2, &m_Buffer);
    }

    virtual void TearDown()
    {
        dmBuffer::Destroy(m_Buffer);

        dmGameSystem::FinalizeScriptLibs(m_ScriptLibContext);

        dmScript::Finalize(m_Context);
        dmScript::DeleteContext(m_Context);

        dmBuffer::DeleteContext();
    }

    dmGameSystem::ScriptLibContext m_ScriptLibContext;
    dmScript::HContext m_Context;
    lua_State* L;
    dmBuffer::HBuffer m_Buffer;
};

class LabelTest : public jc_test_base_class 
{
protected:
    virtual void SetUp()
    {
        m_Position = Vectormath::Aos::Point3(0.0);
        m_Size = Vectormath::Aos::Vector3(2.0, 2.0, 0.0);
        m_Scale = Vectormath::Aos::Vector3(1.0, 1.0, 0.0);

        m_BottomLeft = Vectormath::Aos::Point3(0.0, 0.0, 0.0);
        m_TopLeft = Vectormath::Aos::Point3(0.0, m_Size.getY(), 0.0);
        m_TopRight = Vectormath::Aos::Point3(m_Size.getX(), m_Size.getY(), 0.0);
        m_BottomRight = Vectormath::Aos::Point3(m_Size.getX(), 0.0, 0.0);

        m_Rotation = dmVMath::EulerToQuat(Vectormath::Aos::Vector3(0, 0, -180));
        m_Rotation = normalize(m_Rotation);
    }

    Vectormath::Aos::Quat m_Rotation;
    Vectormath::Aos::Point3 m_Position;
    Vectormath::Aos::Point3 m_BottomLeft;
    Vectormath::Aos::Point3 m_TopLeft;
    Vectormath::Aos::Point3 m_TopRight;
    Vectormath::Aos::Point3 m_BottomRight;
    Vectormath::Aos::Vector3 m_Size;
    Vectormath::Aos::Vector3 m_Scale;
};
