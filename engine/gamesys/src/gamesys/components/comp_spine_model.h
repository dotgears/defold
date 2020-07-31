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

#ifndef DM_GAMESYS_COMP_SPINE_MODEL_H
#define DM_GAMESYS_COMP_SPINE_MODEL_H

#include <stdint.h>
#include <dlib/object_pool.h>
#include <gameobject/gameobject.h>
#include <rig/rig.h>

#include "../resources/res_spine_model.h"
#include "comp_private.h"

namespace dmGameSystem
{
    using namespace Vectormath::Aos;
    using namespace dmGameSystemDDF;

    struct SpineModelComponent
    {
        dmGameObject::HInstance     m_Instance;
        dmTransform::Transform      m_Transform;
        Matrix4                     m_World;
        SpineModelResource*         m_Resource;
        dmRig::HRigInstance         m_RigInstance;
        uint32_t                    m_MixedHash;
        dmMessage::URL              m_Listener;
        CompRenderConstants         m_RenderConstants;
        dmRender::HMaterial         m_Material;
        /// Node instances corresponding to the bones
        dmArray<dmGameObject::HInstance> m_NodeInstances;
        uint16_t                    m_ComponentIndex;
        /// Component enablement
        uint8_t                     m_Enabled : 1;
        uint8_t                     m_DoRender : 1;
        /// Added to update or not
        uint8_t                     m_AddedToUpdate : 1;
        uint8_t                     m_ReHash : 1;
    };

    struct SpineModelWorld
    {
        dmObjectPool<SpineModelComponent*>  m_Components;
        dmArray<dmRender::RenderObject>     m_RenderObjects;
        dmGraphics::HVertexDeclaration      m_VertexDeclaration;
        dmGraphics::HVertexBuffer           m_VertexBuffer;
        dmArray<dmRig::RigSpineModelVertex> m_VertexBufferData;
        // Temporary scratch array for instances, only used during the creation phase of components
        dmArray<dmGameObject::HInstance>    m_ScratchInstances;
        dmRig::HRigContext                  m_RigContext;
    };

    dmGameObject::CreateResult CompSpineModelNewWorld(const dmGameObject::ComponentNewWorldParams& params);

    dmGameObject::CreateResult CompSpineModelDeleteWorld(const dmGameObject::ComponentDeleteWorldParams& params);

    dmGameObject::CreateResult CompSpineModelCreate(const dmGameObject::ComponentCreateParams& params);

    dmGameObject::CreateResult CompSpineModelDestroy(const dmGameObject::ComponentDestroyParams& params);

    dmGameObject::CreateResult CompSpineModelAddToUpdate(const dmGameObject::ComponentAddToUpdateParams& params);

    dmGameObject::UpdateResult CompSpineModelUpdate(const dmGameObject::ComponentsUpdateParams& params, dmGameObject::ComponentsUpdateResult& update_result);

    dmGameObject::UpdateResult CompSpineModelRender(const dmGameObject::ComponentsRenderParams& params);

    dmGameObject::UpdateResult CompSpineModelOnMessage(const dmGameObject::ComponentOnMessageParams& params);

    void CompSpineModelOnReload(const dmGameObject::ComponentOnReloadParams& params);

    dmGameObject::PropertyResult CompSpineModelGetProperty(const dmGameObject::ComponentGetPropertyParams& params, dmGameObject::PropertyDesc& out_value);

    dmGameObject::PropertyResult CompSpineModelSetProperty(const dmGameObject::ComponentSetPropertyParams& params);

    bool CompSpineModelSetIKTargetInstance(SpineModelComponent* component, dmhash_t constraint_id, float mix, dmhash_t instance_id);
    bool CompSpineModelSetIKTargetPosition(SpineModelComponent* component, dmhash_t constraint_id, float mix, Point3 position);
    bool CompSpineModelResetIKTarget(SpineModelComponent* component, dmhash_t constraint_id);

    bool CompSpineModelSetSkin(SpineModelComponent* component, dmhash_t skin_id);
    bool CompSpineModelSetSkinSlot(SpineModelComponent* component, dmhash_t skin_id, dmhash_t slot_id);

}

#endif // DM_GAMESYS_COMP_SPINE_MODEL_H
