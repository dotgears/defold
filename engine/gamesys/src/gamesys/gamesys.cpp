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

#include "gamesys_private.h"

#include <dlib/dstrings.h>
#include <dlib/hash.h>
#include <dlib/log.h>
#include <dlib/message.h>

#include "resources/res_collection_proxy.h"
#include "resources/res_collision_object.h"
#include "resources/res_convex_shape.h"
#include "resources/res_emitter.h"
#include "resources/res_particlefx.h"
#include "resources/res_texture.h"
#include "resources/res_vertex_program.h"
#include "resources/res_fragment_program.h"
#include "resources/res_font_map.h"
#include "resources/res_model.h"
#include "resources/res_buffer.h"
#include "resources/res_mesh.h"
#include "resources/res_material.h"
#include "resources/res_gui.h"
#include "resources/res_sound_data.h"
#include "resources/res_sound.h"
#include "resources/res_camera.h"
#include "resources/res_input_binding.h"
#include "resources/res_gamepad_map.h"
#include "resources/res_factory.h"
#include "resources/res_collection_factory.h"
#include "resources/res_light.h"
#include "resources/res_render_script.h"
#include "resources/res_render_prototype.h"
#include "resources/res_sprite.h"
#include "resources/res_textureset.h"
#include "resources/res_tilegrid.h"
#include "resources/res_animationset.h"
#include "resources/res_meshset.h"
#include "resources/res_skeleton.h"
#include "resources/res_rig_scene.h"
#include "resources/res_spine_model.h"
#include "resources/res_display_profiles.h"
#include "resources/res_label.h"

#include "components/comp_private.h"
#include "components/comp_collection_proxy.h"
#include "components/comp_collision_object.h"
#include "components/comp_emitter.h"
#include "components/comp_particlefx.h"
#include "components/comp_model.h"
#include "components/comp_mesh.h"
#include "components/comp_gui.h"
#include "components/comp_sound.h"
#include "components/comp_camera.h"
#include "components/comp_factory.h"
#include "components/comp_collection_factory.h"
#include "components/comp_light.h"
#include "components/comp_sprite.h"
#include "components/comp_tilegrid.h"
#include "components/comp_spine_model.h"
#include "components/comp_label.h"

#include "camera_ddf.h"
#include "physics_ddf.h"
#include "tile_ddf.h"
#include "sprite_ddf.h"

namespace dmGameSystem
{
    GuiContext::GuiContext()
    : m_Worlds()
    , m_RenderContext(0)
    , m_GuiContext(0)
    , m_ScriptContext(0)
    , m_MaxGuiComponents(64)
    {
        m_Worlds.SetCapacity(128);
    }

    dmResource::Result RegisterResourceTypes(dmResource::HFactory factory, dmRender::HRenderContext render_context, GuiContext* gui_context, dmInput::HContext input_context, PhysicsContext* physics_context)
    {
        dmResource::Result e;

        DM_STATIC_ASSERT( dmGameSystem::MAX_COMP_RENDER_CONSTANTS == dmRender::RenderObject::MAX_CONSTANT_COUNT, Constant_Count_Must_Be_Equal );

#define REGISTER_RESOURCE_TYPE(extension, context, preload_func, create_func, post_create_func, destroy_func, recreate_func)\
    e = dmResource::RegisterType(factory, extension, context, preload_func, create_func, post_create_func, destroy_func, recreate_func);\
    if( e != dmResource::RESULT_OK )\
    {\
        dmLogFatal("Unable to register resource type: %s", extension);\
        return e;\
    }\

        dmGraphics::HContext graphics_context = dmRender::GetGraphicsContext(render_context);

        REGISTER_RESOURCE_TYPE("collectionproxyc", 0, 0, ResCollectionProxyCreate, 0, ResCollectionProxyDestroy, ResCollectionProxyRecreate);
        REGISTER_RESOURCE_TYPE("collisionobjectc", physics_context, 0, ResCollisionObjectCreate, 0, ResCollisionObjectDestroy, ResCollisionObjectRecreate);
        REGISTER_RESOURCE_TYPE("convexshapec", physics_context, 0, ResConvexShapeCreate, 0, ResConvexShapeDestroy, ResConvexShapeRecreate);
        REGISTER_RESOURCE_TYPE("emitterc", 0, 0, ResEmitterCreate, 0,ResEmitterDestroy, ResEmitterRecreate);
        REGISTER_RESOURCE_TYPE("particlefxc", 0, ResParticleFXPreload, ResParticleFXCreate, 0, ResParticleFXDestroy, ResParticleFXRecreate);
        REGISTER_RESOURCE_TYPE("texturec", graphics_context, ResTexturePreload, ResTextureCreate, ResTexturePostCreate, ResTextureDestroy, ResTextureRecreate);
        REGISTER_RESOURCE_TYPE("vpc", graphics_context, ResVertexProgramPreload, ResVertexProgramCreate, 0, ResVertexProgramDestroy, ResVertexProgramRecreate);
        REGISTER_RESOURCE_TYPE("fpc", graphics_context, ResFragmentProgramPreload, ResFragmentProgramCreate, 0, ResFragmentProgramDestroy, ResFragmentProgramRecreate);
        REGISTER_RESOURCE_TYPE("fontc", render_context, ResFontMapPreload, ResFontMapCreate, 0, ResFontMapDestroy, ResFontMapRecreate);
        REGISTER_RESOURCE_TYPE("bufferc", graphics_context, ResBufferPreload, ResBufferCreate, 0, ResBufferDestroy, ResBufferRecreate);
        REGISTER_RESOURCE_TYPE("meshc", graphics_context, ResMeshPreload, ResMeshCreate, 0, ResMeshDestroy, ResMeshRecreate);
        REGISTER_RESOURCE_TYPE("modelc", graphics_context, ResModelPreload, ResModelCreate, 0, ResModelDestroy, ResModelRecreate);
        REGISTER_RESOURCE_TYPE("materialc", render_context, ResMaterialPreload, ResMaterialCreate, 0, ResMaterialDestroy, ResMaterialRecreate);
        REGISTER_RESOURCE_TYPE("guic", gui_context, ResPreloadSceneDesc, ResCreateSceneDesc, 0, ResDestroySceneDesc, ResRecreateSceneDesc);
        REGISTER_RESOURCE_TYPE("gui_scriptc", gui_context, ResPreloadGuiScript, ResCreateGuiScript, 0, ResDestroyGuiScript, ResRecreateGuiScript);
        REGISTER_RESOURCE_TYPE("wavc", 0, 0, ResSoundDataCreate, 0, ResSoundDataDestroy, ResSoundDataRecreate);
        REGISTER_RESOURCE_TYPE("oggc", 0, 0, ResSoundDataCreate, 0, ResSoundDataDestroy, ResSoundDataRecreate);
        REGISTER_RESOURCE_TYPE("soundc", 0, ResSoundPreload, ResSoundCreate, 0, ResSoundDestroy, ResSoundRecreate);
        REGISTER_RESOURCE_TYPE("camerac", 0, 0, ResCameraCreate, 0, ResCameraDestroy, ResCameraRecreate);
        REGISTER_RESOURCE_TYPE("input_bindingc", input_context, 0, ResInputBindingCreate, 0, ResInputBindingDestroy, ResInputBindingRecreate);
        REGISTER_RESOURCE_TYPE("gamepadsc", 0, 0, ResGamepadMapCreate, 0, ResGamepadMapDestroy, ResGamepadMapRecreate);
        REGISTER_RESOURCE_TYPE("factoryc", 0, ResFactoryPreload, ResFactoryCreate, 0, ResFactoryDestroy, ResFactoryRecreate);
        REGISTER_RESOURCE_TYPE("collectionfactoryc", 0, ResCollectionFactoryPreload, ResCollectionFactoryCreate, 0, ResCollectionFactoryDestroy, ResCollectionFactoryRecreate);
        REGISTER_RESOURCE_TYPE("labelc", 0, ResLabelPreload, ResLabelCreate, 0, ResLabelDestroy, ResLabelRecreate);
        REGISTER_RESOURCE_TYPE("lightc", 0, 0, ResLightCreate, 0, ResLightDestroy, ResLightRecreate);
        REGISTER_RESOURCE_TYPE("render_scriptc", render_context, 0, ResRenderScriptCreate, 0, ResRenderScriptDestroy, ResRenderScriptRecreate);
        REGISTER_RESOURCE_TYPE("renderc", render_context, 0, ResRenderPrototypeCreate, 0, ResRenderPrototypeDestroy, ResRenderPrototypeRecreate);
        REGISTER_RESOURCE_TYPE("spritec", 0, ResSpritePreload, ResSpriteCreate, 0, ResSpriteDestroy, ResSpriteRecreate);
        REGISTER_RESOURCE_TYPE("texturesetc", physics_context, ResTextureSetPreload, ResTextureSetCreate, 0, ResTextureSetDestroy, ResTextureSetRecreate);
        REGISTER_RESOURCE_TYPE(TILE_MAP_EXT, physics_context, ResTileGridPreload, ResTileGridCreate, 0, ResTileGridDestroy, ResTileGridRecreate);
        REGISTER_RESOURCE_TYPE("animationsetc", 0, ResAnimationSetPreload, ResAnimationSetCreate, 0, ResAnimationSetDestroy, ResAnimationSetRecreate);
        REGISTER_RESOURCE_TYPE("meshsetc", 0, ResMeshSetPreload, ResMeshSetCreate, 0, ResMeshSetDestroy, ResMeshSetRecreate);
        REGISTER_RESOURCE_TYPE("skeletonc", 0, ResSkeletonPreload, ResSkeletonCreate, 0, ResSkeletonDestroy, ResSkeletonRecreate);
        REGISTER_RESOURCE_TYPE("rigscenec", 0, ResRigScenePreload, ResRigSceneCreate, 0, ResRigSceneDestroy, ResRigSceneRecreate);
        REGISTER_RESOURCE_TYPE(SPINE_MODEL_EXT, 0, ResSpineModelPreload, ResSpineModelCreate, 0, ResSpineModelDestroy, ResSpineModelRecreate);
        REGISTER_RESOURCE_TYPE("display_profilesc", render_context, 0, ResDisplayProfilesCreate, 0, ResDisplayProfilesDestroy, ResDisplayProfilesRecreate);

#undef REGISTER_RESOURCE_TYPE

        return e;
    }

    dmGameObject::Result RegisterComponentTypes(dmResource::HFactory factory,
                                                dmGameObject::HRegister regist,
                                                dmRender::RenderContext* render_context,
                                                PhysicsContext* physics_context,
                                                ParticleFXContext* particlefx_context,
                                                GuiContext* gui_context,
                                                SpriteContext* sprite_context,
                                                CollectionProxyContext* collection_proxy_context,
                                                FactoryContext* factory_context,
                                                CollectionFactoryContext *collectionfactory_context,
                                                SpineModelContext* spine_model_context,
                                                ModelContext* model_context,
                                                MeshContext* mesh_context,
                                                LabelContext* label_context,
                                                TilemapContext* tilemap_context,
                                                SoundContext* sound_context)
    {
        dmResource::ResourceType type;
        dmGameObject::ComponentType component_type;
        dmResource::Result factory_result;
        dmGameObject::Result go_result;

#define REGISTER_COMPONENT_TYPE(extension, prio, context, new_world_func, delete_world_func, \
                                create_func, destroy_func, init_func, final_func, add_to_update_func, get_func, \
                                update_func, render_func, post_update_func, on_message_func, on_input_func, \
                                on_reload_func, get_property_func, set_property_func, set_reads_transforms)\
    factory_result = dmResource::GetTypeFromExtension(factory, extension, &type);\
    if (factory_result != dmResource::RESULT_OK)\
    {\
        dmLogWarning("Unable to get resource type for '%s' (%d)", extension, factory_result);\
        return dmGameObject::RESULT_UNKNOWN_ERROR;\
    }\
    component_type = dmGameObject::ComponentType();\
    component_type.m_Name = extension;\
    component_type.m_ResourceType = type;\
    component_type.m_Context = context;\
    component_type.m_NewWorldFunction = new_world_func;\
    component_type.m_DeleteWorldFunction = delete_world_func;\
    component_type.m_CreateFunction = create_func;\
    component_type.m_DestroyFunction = destroy_func;\
    component_type.m_InitFunction = init_func;\
    component_type.m_FinalFunction = final_func;\
    component_type.m_AddToUpdateFunction = add_to_update_func;\
    component_type.m_GetFunction = get_func;\
    component_type.m_RenderFunction = render_func;\
    component_type.m_UpdateFunction = update_func;\
    component_type.m_PostUpdateFunction = post_update_func;\
    component_type.m_OnMessageFunction = on_message_func;\
    component_type.m_OnInputFunction = on_input_func;\
    component_type.m_OnReloadFunction = on_reload_func;\
    component_type.m_GetPropertyFunction = get_property_func;\
    component_type.m_SetPropertyFunction = set_property_func;\
    component_type.m_ReadsTransforms = set_reads_transforms;\
    component_type.m_InstanceHasUserData = (uint32_t)true;\
    component_type.m_UpdateOrderPrio = prio;\
    go_result = dmGameObject::RegisterComponentType(regist, component_type);\
    if (go_result != dmGameObject::RESULT_OK)\
        return go_result;

        /*
         * About update priority. Component types below have priority evenly spaced with increments by 100
         *
         */

        // Modified by dotGears / TheTrung : from 100 
        REGISTER_COMPONENT_TYPE("collectionproxyc", 100, collection_proxy_context,
                &CompCollectionProxyNewWorld, &CompCollectionProxyDeleteWorld,
                &CompCollectionProxyCreate, &CompCollectionProxyDestroy, 0, &CompCollectionProxyFinal, &CompCollectionProxyAddToUpdate, 0,
                &CompCollectionProxyUpdate, &CompCollectionProxyRender, &CompCollectionProxyPostUpdate, &CompCollectionProxyOnMessage, &CompCollectionProxyOnInput, 0, 0, 0,
                0);

        // Modified by dotGears / TheTrung : from 400 -> 200
        REGISTER_COMPONENT_TYPE("collisionobjectc", 200, physics_context, &CompCollisionObjectNewWorld, &CompCollisionObjectDeleteWorld, &CompCollisionObjectCreate, &CompCollisionObjectDestroy, 0, &CompCollisionObjectFinal, &CompCollisionObjectAddToUpdate, 0, &CompCollisionObjectUpdate, 0, &CompCollisionObjectPostUpdate, &CompCollisionObjectOnMessage, 0, &CompCollisionObjectOnReload, CompCollisionObjectGetProperty, CompCollisionObjectSetProperty, 1);
        
        // See gameobject_comp.cpp for these two component types:
        // Priority 200 is reserved for scriptc (read+write transforms) -> 250
        // Priority 250 is reserved for animc (read+write transforms) -> 300

        // Modified by dotGears / TheTrung : from 300 -> 400
        REGISTER_COMPONENT_TYPE("guic", 400, gui_context,
                CompGuiNewWorld, CompGuiDeleteWorld,
                CompGuiCreate, CompGuiDestroy, CompGuiInit, CompGuiFinal, CompGuiAddToUpdate, 0,
                CompGuiUpdate, CompGuiRender, 0, CompGuiOnMessage, CompGuiOnInput, CompGuiOnReload, CompGuiGetProperty, CompGuiSetProperty,
                0);


        REGISTER_COMPONENT_TYPE("camerac", 500, render_context,
                &CompCameraNewWorld, &CompCameraDeleteWorld,
                &CompCameraCreate, &CompCameraDestroy, 0, 0, &CompCameraAddToUpdate, 0,
                &CompCameraUpdate, 0, 0, &CompCameraOnMessage, 0, &CompCameraOnReload, 0, 0,
                1);

        REGISTER_COMPONENT_TYPE("soundc", 600, sound_context,
                CompSoundNewWorld, CompSoundDeleteWorld,
                CompSoundCreate, CompSoundDestroy, 0, 0, CompSoundAddToUpdate, 0,
                CompSoundUpdate, 0, 0, CompSoundOnMessage, 0, 0, CompSoundGetProperty, CompSoundSetProperty,
                0);

        REGISTER_COMPONENT_TYPE("modelc", 700, model_context,
                CompModelNewWorld, CompModelDeleteWorld,
                CompModelCreate, CompModelDestroy, 0, 0, CompModelAddToUpdate, 0,
                CompModelUpdate, CompModelRender, 0, CompModelOnMessage, 0, 0, CompModelGetProperty, CompModelSetProperty,
                0);

        REGISTER_COMPONENT_TYPE("meshc", 725, mesh_context,
                CompMeshNewWorld, CompMeshDeleteWorld,
                CompMeshCreate, CompMeshDestroy, 0, 0, CompMeshAddToUpdate, 0,
                CompMeshUpdate, CompMeshRender, 0, CompMeshOnMessage, 0, 0, CompMeshGetProperty, CompMeshSetProperty,
                0);

        REGISTER_COMPONENT_TYPE("emitterc", 750, 0x0,
                &CompEmitterNewWorld, &CompEmitterDeleteWorld,
                &CompEmitterCreate, &CompEmitterDestroy, 0, 0, 0, 0,
                0, 0, 0, CompEmitterOnMessage, 0, 0, 0, 0,
                0);

        REGISTER_COMPONENT_TYPE("particlefxc", 800, particlefx_context,
                &CompParticleFXNewWorld, &CompParticleFXDeleteWorld,
                &CompParticleFXCreate, &CompParticleFXDestroy, 0, 0, &CompParticleFXAddToUpdate, 0,
                &CompParticleFXUpdate, &CompParticleFXRender, 0, &CompParticleFXOnMessage, 0, &CompParticleFXOnReload, 0, 0,
                1);

        REGISTER_COMPONENT_TYPE("factoryc", 900, factory_context,
                CompFactoryNewWorld, CompFactoryDeleteWorld,
                CompFactoryCreate, CompFactoryDestroy, 0, 0, CompFactoryAddToUpdate, 0,
                CompFactoryUpdate, 0, 0, CompFactoryOnMessage, 0, 0, 0, 0,
                0);

        REGISTER_COMPONENT_TYPE("collectionfactoryc", 950, collectionfactory_context,
                CompCollectionFactoryNewWorld, CompCollectionFactoryDeleteWorld,
                CompCollectionFactoryCreate, CompCollectionFactoryDestroy, 0, 0, CompCollectionFactoryAddToUpdate, 0,
                CompCollectionFactoryUpdate, 0, 0, 0, 0, 0, 0, 0,
                0);

        REGISTER_COMPONENT_TYPE("lightc", 1000, render_context,
                CompLightNewWorld, CompLightDeleteWorld,
                CompLightCreate, CompLightDestroy, 0, 0, CompLightAddToUpdate, 0,
                CompLightUpdate, 0, 0, CompLightOnMessage, 0, 0, 0, 0,
                1);

        REGISTER_COMPONENT_TYPE("spritec", 1100, sprite_context,
                CompSpriteNewWorld, CompSpriteDeleteWorld,
                CompSpriteCreate, CompSpriteDestroy, 0, 0, CompSpriteAddToUpdate, 0,
                CompSpriteUpdate, CompSpriteRender, 0, CompSpriteOnMessage, 0, CompSpriteOnReload, CompSpriteGetProperty, CompSpriteSetProperty,
                1);

        REGISTER_COMPONENT_TYPE(TILE_MAP_EXT, 1200, tilemap_context,
                CompTileGridNewWorld, CompTileGridDeleteWorld,
                CompTileGridCreate, CompTileGridDestroy, 0, 0, CompTileGridAddToUpdate, 0,
                CompTileGridUpdate, CompTileGridRender, 0, CompTileGridOnMessage, 0, CompTileGridOnReload, CompTileGridGetProperty, CompTileGridSetProperty,
                1);

        REGISTER_COMPONENT_TYPE(SPINE_MODEL_EXT, 1300, spine_model_context,
                CompSpineModelNewWorld, CompSpineModelDeleteWorld,
                CompSpineModelCreate, CompSpineModelDestroy, 0, 0, CompSpineModelAddToUpdate, 0,
                CompSpineModelUpdate, CompSpineModelRender, 0, CompSpineModelOnMessage, 0, CompSpineModelOnReload, CompSpineModelGetProperty, CompSpineModelSetProperty,
                0);

        REGISTER_COMPONENT_TYPE("labelc", 1400, label_context,
                CompLabelNewWorld, CompLabelDeleteWorld,
                CompLabelCreate, CompLabelDestroy, 0, 0, CompLabelAddToUpdate, CompLabelGetComponent,
                CompLabelUpdate, CompLabelRender, 0, CompLabelOnMessage, 0, CompLabelOnReload, CompLabelGetProperty, CompLabelSetProperty,
                1);

        #undef REGISTER_COMPONENT_TYPE

        return go_result;
    }
}
