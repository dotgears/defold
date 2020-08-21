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

#ifndef DM_GAMESYS_COMP_GUI_H
#define DM_GAMESYS_COMP_GUI_H

#include <stdint.h>

#include <gui/gui.h>
#include <gameobject/gameobject.h>
#include <render/render.h>
#include <rig/rig.h>

namespace dmGameSystem
{
    extern dmRender::HRenderType g_GuiRenderType;

    struct GuiSceneResource;

    struct GuiComponent
    {
        GuiSceneResource*       m_Resource;
        dmGui::HScene           m_Scene;
        dmGameObject::HInstance m_Instance;
        dmRender::HMaterial     m_Material;
        uint16_t                m_ComponentIndex;
        uint8_t                 m_Enabled : 1;
        uint8_t                 m_AddedToUpdate : 1;
    };

    struct BoxVertex
    {
        inline BoxVertex() {}
        inline BoxVertex(const Vectormath::Aos::Vector4& p, float u, float v, const Vectormath::Aos::Vector4& color)
        {
            SetPosition(p);
            SetUV(u, v);
            SetColor(color);
        }

        inline void SetPosition(const Vectormath::Aos::Vector4& p)
        {
            m_Position[0] = p.getX();
            m_Position[1] = p.getY();
            m_Position[2] = p.getZ();
        }

        inline void SetUV(float u, float v)
        {
            m_UV[0] = u;
            m_UV[1] = v;
        }

        inline void SetColor(const Vectormath::Aos::Vector4& c)
        {
            m_Color[0] = c.getX();
            m_Color[1] = c.getY();
            m_Color[2] = c.getZ();
            m_Color[3] = c.getW();
        }

        float m_Position[3];
        float m_UV[2];
        float m_Color[4];
    };

    struct GuiRenderObject
    {
        dmRender::RenderObject m_RenderObject;
        uint32_t m_SortOrder;
    };

    struct GuiWorld
    {
        dmArray<GuiRenderObject>         m_GuiRenderObjects;
        dmArray<GuiComponent*>           m_Components;
        dmGraphics::HVertexDeclaration   m_VertexDeclaration;
        dmGraphics::HVertexBuffer        m_VertexBuffer;
        dmArray<BoxVertex>               m_ClientVertexBuffer;
        dmGraphics::HTexture             m_WhiteTexture;
        dmParticle::HParticleContext     m_ParticleContext;
        uint32_t                         m_MaxParticleFXCount;
        uint32_t                         m_MaxParticleCount;
        uint32_t                         m_RenderedParticlesSize;
        float                            m_DT;
        dmRig::HRigContext               m_RigContext;
        dmScript::ScriptWorld*           m_ScriptWorld;
    };

    typedef BoxVertex ParticleGuiVertex;

    dmGameObject::CreateResult CompGuiNewWorld(const dmGameObject::ComponentNewWorldParams& params);

    dmGameObject::CreateResult CompGuiDeleteWorld(const dmGameObject::ComponentDeleteWorldParams& params);

    dmGameObject::CreateResult CompGuiCreate(const dmGameObject::ComponentCreateParams& params);

    dmGameObject::CreateResult CompGuiDestroy(const dmGameObject::ComponentDestroyParams& params);

    dmGameObject::CreateResult CompGuiInit(const dmGameObject::ComponentInitParams& params);

    dmGameObject::CreateResult CompGuiFinal(const dmGameObject::ComponentFinalParams& params);

    dmGameObject::CreateResult CompGuiAddToUpdate(const dmGameObject::ComponentAddToUpdateParams& params);

    dmGameObject::UpdateResult CompGuiUpdate(const dmGameObject::ComponentsUpdateParams& params, dmGameObject::ComponentsUpdateResult& update_result);

    dmGameObject::UpdateResult CompGuiRender(const dmGameObject::ComponentsRenderParams& params);

    dmGameObject::UpdateResult CompGuiOnMessage(const dmGameObject::ComponentOnMessageParams& params);

    dmGameObject::InputResult CompGuiOnInput(const dmGameObject::ComponentOnInputParams& params);

    void CompGuiOnReload(const dmGameObject::ComponentOnReloadParams& params);

    dmGameObject::PropertyResult CompGuiGetProperty(const dmGameObject::ComponentGetPropertyParams& params, dmGameObject::PropertyDesc& out_value);

    dmGameObject::PropertyResult CompGuiSetProperty(const dmGameObject::ComponentSetPropertyParams& params);
}

#endif // DM_GAMESYS_COMP_GUI_H
