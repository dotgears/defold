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

#include "res_spine_model.h"

#include <dlib/log.h>

namespace dmGameSystem
{
    dmResource::Result AcquireResources(dmResource::HFactory factory, SpineModelResource* resource, const char* filename)
    {
        dmResource::Result result = dmResource::Get(factory, resource->m_Model->m_SpineScene, (void**) &resource->m_RigScene);
        if (result != dmResource::RESULT_OK)
            return result;
        result = dmResource::Get(factory, resource->m_Model->m_Material, (void**) &resource->m_Material);
        if (result != dmResource::RESULT_OK)
        {
            return result;
        }
        if(dmRender::GetMaterialVertexSpace(resource->m_Material) != dmRenderDDF::MaterialDesc::VERTEX_SPACE_WORLD)
        {
            dmLogError("Failed to create Spine Model component. This component only supports materials with the Vertex Space property set to 'vertex-space-world'");
            return dmResource::RESULT_NOT_SUPPORTED;
        }
        return dmResource::RESULT_OK;
    }

    static void ReleaseResources(dmResource::HFactory factory, SpineModelResource* resource)
    {
        if (resource->m_Model != 0x0)
            dmDDF::FreeMessage(resource->m_Model);
        if (resource->m_RigScene != 0x0)
            dmResource::Release(factory, resource->m_RigScene);
        if (resource->m_Material != 0x0)
            dmResource::Release(factory, resource->m_Material);
    }

    dmResource::Result ResSpineModelPreload(const dmResource::ResourcePreloadParams& params)
    {
        dmGameSystemDDF::SpineModelDesc* ddf;
        dmDDF::Result e = dmDDF::LoadMessage(params.m_Buffer, params.m_BufferSize, &dmGameSystemDDF_SpineModelDesc_DESCRIPTOR, (void**) &ddf);
        if (e != dmDDF::RESULT_OK)
        {
            return dmResource::RESULT_DDF_ERROR;
        }

        dmResource::PreloadHint(params.m_HintInfo, ddf->m_SpineScene);
        dmResource::PreloadHint(params.m_HintInfo, ddf->m_Material);

        *params.m_PreloadData = ddf;
        return dmResource::RESULT_OK;
    }

    dmResource::Result ResSpineModelCreate(const dmResource::ResourceCreateParams& params)
    {
        SpineModelResource* model_resource = new SpineModelResource();
        model_resource->m_Model = (dmGameSystemDDF::SpineModelDesc*) params.m_PreloadData;
        dmResource::Result r = AcquireResources(params.m_Factory, model_resource, params.m_Filename);
        if (r == dmResource::RESULT_OK)
        {
            params.m_Resource->m_Resource = (void*) model_resource;
        }
        else
        {
            ReleaseResources(params.m_Factory, model_resource);
            delete model_resource;
        }
        return r;
    }

    dmResource::Result ResSpineModelDestroy(const dmResource::ResourceDestroyParams& params)
    {
        SpineModelResource* model_resource = (SpineModelResource*)params.m_Resource->m_Resource;
        ReleaseResources(params.m_Factory, model_resource);
        delete model_resource;
        return dmResource::RESULT_OK;
    }

    dmResource::Result ResSpineModelRecreate(const dmResource::ResourceRecreateParams& params)
    {
        dmGameSystemDDF::SpineModelDesc* ddf;
        dmDDF::Result e = dmDDF::LoadMessage(params.m_Buffer, params.m_BufferSize, &dmGameSystemDDF_SpineModelDesc_DESCRIPTOR, (void**) &ddf);
        if (e != dmDDF::RESULT_OK)
        {
            return dmResource::RESULT_DDF_ERROR;
        }
        SpineModelResource* model_resource = (SpineModelResource*)params.m_Resource->m_Resource;
        ReleaseResources(params.m_Factory, model_resource);
        model_resource->m_Model = ddf;
        return AcquireResources(params.m_Factory, model_resource, params.m_Filename);
    }
}
