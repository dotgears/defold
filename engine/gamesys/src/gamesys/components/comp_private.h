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

#ifndef DM_GAMESYS_COMP_PRIVATE_H
#define DM_GAMESYS_COMP_PRIVATE_H

#include <dlib/hash.h>
#include <dlib/math.h>
#include <gameobject/gameobject.h>
#include <render/render.h>
#include <dmsdk/vectormath/cpp/vectormath_aos.h>

namespace dmGameSystem
{
    const uint32_t MAX_COMP_RENDER_CONSTANTS = 16;

    struct PropVector3
    {
        dmhash_t m_Vector;
        dmhash_t m_X;
        dmhash_t m_Y;
        dmhash_t m_Z;
        bool m_ReadOnly;

        PropVector3(dmhash_t v, dmhash_t x, dmhash_t y, dmhash_t z, bool readOnly)
        {
            m_Vector = v;
            m_X = x;
            m_Y = y;
            m_Z = z;
            m_ReadOnly = readOnly;
        }
    };

    struct PropVector4
    {
        dmhash_t m_Vector;
        dmhash_t m_X;
        dmhash_t m_Y;
        dmhash_t m_Z;
        dmhash_t m_W;
        bool m_ReadOnly;

        PropVector4(dmhash_t v, dmhash_t x, dmhash_t y, dmhash_t z, dmhash_t w, bool readOnly)
        {
            m_Vector = v;
            m_X = x;
            m_Y = y;
            m_Z = z;
            m_W = w;
            m_ReadOnly = readOnly;
        }
    };

    inline bool IsReferencingProperty(const PropVector3& property, dmhash_t query)
    {
        return property.m_Vector == query || property.m_X == query || property.m_Y == query || property.m_Z == query;
    }

    inline bool IsReferencingProperty(const PropVector4& property, dmhash_t query)
    {
        return property.m_Vector == query || property.m_X == query || property.m_Y == query || property.m_Z == query || property.m_W == query;
    }

    struct CompRenderConstants
    {
        CompRenderConstants();
        dmRender::Constant          m_RenderConstants[MAX_COMP_RENDER_CONSTANTS];
        Vectormath::Aos::Vector4    m_PrevRenderConstants[MAX_COMP_RENDER_CONSTANTS];
        uint32_t                    m_ConstantCount;
    };

    dmGameObject::PropertyResult GetProperty(dmGameObject::PropertyDesc& out_value, dmhash_t get_property, const Vectormath::Aos::Vector3& ref_value, const PropVector3& property);
    dmGameObject::PropertyResult SetProperty(dmhash_t set_property, const dmGameObject::PropertyVar& in_value, Vectormath::Aos::Vector3& set_value, const PropVector3& property);

    dmGameObject::PropertyResult GetProperty(dmGameObject::PropertyDesc& out_value, dmhash_t get_property, const Vectormath::Aos::Vector4& ref_value, const PropVector4& property);
    dmGameObject::PropertyResult SetProperty(dmhash_t set_property, const dmGameObject::PropertyVar& in_value, Vectormath::Aos::Vector4& set_value, const PropVector4& property);

    dmGameObject::PropertyResult GetResourceProperty(dmResource::HFactory factory, void* resource, dmGameObject::PropertyDesc& out_value);
    dmGameObject::PropertyResult SetResourceProperty(dmResource::HFactory factory, const dmGameObject::PropertyVar& value, dmhash_t ext, void** out_resource);
    dmGameObject::PropertyResult SetResourceProperty(dmResource::HFactory factory, const dmGameObject::PropertyVar& value, dmhash_t* exts, uint32_t ext_count, void** out_resource);

    bool GetRenderConstant(CompRenderConstants* constants, dmhash_t name_hash, dmRender::Constant** out_constant);
    void SetRenderConstant(CompRenderConstants* constants, dmRender::HMaterial material, dmhash_t name_hash, uint32_t* element_index, const dmGameObject::PropertyVar& var);
    int  ClearRenderConstant(CompRenderConstants* constants, dmhash_t name_hash);
    void ReHashRenderConstants(CompRenderConstants* constants, HashState32* state);
    int  AreRenderConstantsUpdated(CompRenderConstants* constants);

#define DM_GAMESYS_PROP_VECTOR3(var_name, prop_name, readOnly)\
    static const dmGameSystem::PropVector3 var_name(dmHashString64(#prop_name),\
            dmHashString64(#prop_name ".x"),\
            dmHashString64(#prop_name ".y"),\
            dmHashString64(#prop_name ".z"),\
            readOnly);

#define DM_GAMESYS_PROP_VECTOR4(var_name, prop_name, readOnly)\
    static const dmGameSystem::PropVector4 var_name(dmHashString64(#prop_name),\
            dmHashString64(#prop_name ".x"),\
            dmHashString64(#prop_name ".y"),\
            dmHashString64(#prop_name ".z"),\
            dmHashString64(#prop_name ".w"),\
            readOnly);

}

#endif // DM_GAMESYS_COMP_PRIVATE_H
