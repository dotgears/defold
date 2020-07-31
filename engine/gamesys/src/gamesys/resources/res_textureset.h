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

#ifndef DMGAMESYSTEM_RES_TEXTURESET_H
#define DMGAMESYSTEM_RES_TEXTURESET_H

#include <stdint.h>

#include <dlib/hashtable.h>

#include <resource/resource.h>

#include <render/render.h>

#include <physics/physics.h>

#include "texture_set_ddf.h"

namespace dmGameSystem
{
    struct TextureSetResource
    {
        inline TextureSetResource()
        {
            m_Texture = 0;
            m_TextureSet = 0;
            m_HullSet = 0;
        }

        dmArray<dmhash_t>                   m_HullCollisionGroups;
        dmHashTable<dmhash_t, uint32_t>     m_AnimationIds;
        dmGraphics::HTexture                m_Texture;
        dmhash_t                            m_TexturePath;
        dmGameSystemDDF::TextureSet*        m_TextureSet;
        dmPhysics::HHullSet2D               m_HullSet;
    };

    dmResource::Result ResTextureSetPreload(const dmResource::ResourcePreloadParams& params);

    dmResource::Result ResTextureSetCreate(const dmResource::ResourceCreateParams& params);

    dmResource::Result ResTextureSetDestroy(const dmResource::ResourceDestroyParams& params);

    dmResource::Result ResTextureSetRecreate(const dmResource::ResourceRecreateParams& params);
}

#endif // DMGAMESYSTEM_RES_TEXTURESET_H
