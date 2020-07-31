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

#include "comp_collision_object.h"

#include <dlib/dlib.h>
#include <dlib/dstrings.h>
#include <dlib/hash.h>
#include <dlib/log.h>
#include <dlib/math.h>

#include <physics/physics.h>

#include <gameobject/gameobject.h>
#include <gameobject/gameobject_ddf.h>

#include "gamesys.h"
#include "../resources/res_collision_object.h"

#include "../proto/physics_ddf.h"

namespace dmGameSystem
{
    using namespace Vectormath::Aos;

    /// Config key to use for tweaking maximum number of collisions reported
    const char* PHYSICS_MAX_COLLISIONS_KEY  = "physics.max_collisions";
    /// Config key to use for tweaking maximum number of contacts reported
    const char* PHYSICS_MAX_CONTACTS_KEY    = "physics.max_contacts";

    static const dmhash_t PROP_LINEAR_DAMPING = dmHashString64("linear_damping");
    static const dmhash_t PROP_ANGULAR_DAMPING = dmHashString64("angular_damping");
    static const dmhash_t PROP_LINEAR_VELOCITY  = dmHashString64("linear_velocity");
    static const dmhash_t PROP_ANGULAR_VELOCITY = dmHashString64("angular_velocity");
    static const dmhash_t PROP_MASS = dmHashString64("mass");
    //TrungB begin: Add more stuffs here
    static const dmhash_t PROP_BODY_ANGLE    = dmHashString64("body_angle");
    static const dmhash_t PROP_BODY_POSITION = dmHashString64("body_position");
    //TrungB end

    struct CollisionComponent;
    struct JointEndPoint;

    /// Joint entry that will keep track of joint connections from collision components.
    struct JointEntry
    {
        dmhash_t m_Id;
        dmPhysics::JointType m_Type;
        dmPhysics::HJoint m_Joint;
        JointEntry* m_Next;
        JointEndPoint* m_EndPoint;
        JointEntry(dmhash_t id, dmPhysics::HJoint joint, JointEntry* next)
        {
            m_Id = id;
            m_Joint = joint;
            m_Next = next;
        }
    };

    /// Joint end point to keep track of joint connections to collision components.
    struct JointEndPoint
    {
        JointEndPoint* m_Next;
        CollisionComponent* m_Owner;
        JointEntry* m_JointEntry;
    };

    struct CollisionComponent
    {
        CollisionObjectResource* m_Resource;
        dmGameObject::HInstance m_Instance;

        union
        {
            dmPhysics::HCollisionObject3D m_Object3D;
            dmPhysics::HCollisionObject2D m_Object2D;
        };

        /// Linked list of joints FROM this component.
        JointEntry* m_Joints;

        /// Linked list of joints TO this component.
        JointEndPoint* m_JointEndPoints;

        uint16_t m_Mask;
        uint16_t m_ComponentIndex;
        // True if the physics is 3D
        // This is used to determine physics engine kind and to preserve
        // z for the 2d-case
        // A bit awkward to have a flag for this but we don't have access to
        // to PhysicsContext in the SetWorldTransform callback. This
        // could perhaps be improved.
        uint8_t m_3D : 1;

        // Tracking initial state.
        uint8_t m_AddedToUpdate : 1;
        uint8_t m_StartAsEnabled : 1;
        uint8_t m_FlippedX : 1; // set if it's been flipped
        uint8_t m_FlippedY : 1;
    };

    struct CollisionWorld
    {
        uint64_t m_Groups[16];
        union
        {
            dmPhysics::HWorld2D m_World2D;
            dmPhysics::HWorld3D m_World3D;
        };
        float m_LastDT; // Used to calculate joint reaction force and torque.
        uint8_t m_ComponentIndex;
        uint8_t m_3D : 1;
        dmArray<CollisionComponent*> m_Components;
    };

    // Forward declarations
    static void DeleteJoint(CollisionWorld* world, dmPhysics::HJoint joint);
    static void DeleteJoint(CollisionWorld* world, JointEntry* joint_entry);

    static void GetWorldTransform(void* user_data, dmTransform::Transform& world_transform)
    {
        if (!user_data)
            return;
        CollisionComponent* component = (CollisionComponent*)user_data;
        dmGameObject::HInstance instance = component->m_Instance;
        world_transform = dmGameObject::GetWorldTransform(instance);
    }

    // TODO: Allow the SetWorldTransform to have a physics context which we can check instead!!
    static int g_NumPhysicsTransformsUpdated = 0;

    static void SetWorldTransform(void* user_data, const Vectormath::Aos::Point3& position, const Vectormath::Aos::Quat& rotation)
    {
        if (!user_data)
            return;
        CollisionComponent* component = (CollisionComponent*)user_data;
        dmGameObject::HInstance instance = component->m_Instance;
        if (component->m_3D)
        {
            dmGameObject::SetPosition(instance, position);
        }
        else
        {
            // Preserve z for 2D physics
            Vectormath::Aos::Point3 p = dmGameObject::GetPosition(instance);
            p.setX(position.getX());
            p.setY(position.getY());
            dmGameObject::SetPosition(instance, p);
        }
        dmGameObject::SetRotation(instance, rotation);
        ++g_NumPhysicsTransformsUpdated;
    }

    dmGameObject::CreateResult CompCollisionObjectNewWorld(const dmGameObject::ComponentNewWorldParams& params)
    {
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        dmPhysics::NewWorldParams world_params;
        world_params.m_GetWorldTransformCallback = GetWorldTransform;
        world_params.m_SetWorldTransformCallback = SetWorldTransform;

        dmPhysics::HWorld2D world2D;
        dmPhysics::HWorld3D world3D;
        if (physics_context->m_3D)
        {
            world3D = dmPhysics::NewWorld3D(physics_context->m_Context3D, world_params);
            if (world3D == 0x0)
            {
                *params.m_World = 0x0;
                return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
            }
        }
        else
        {
            world2D = dmPhysics::NewWorld2D(physics_context->m_Context2D, world_params);
            if (world2D == 0x0)
            {
                *params.m_World = 0x0;
                return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
            }
        }

        CollisionWorld* world = new CollisionWorld();
        memset(world, 0, sizeof(CollisionWorld));
        if (physics_context->m_3D)
        {
            world->m_World3D = world3D;
        }
        else
        {
            world->m_World2D = world2D;
        }
        world->m_ComponentIndex = params.m_ComponentIndex;
        world->m_3D = physics_context->m_3D;
        world->m_Components.SetCapacity(32);
        *params.m_World = world;
        return dmGameObject::CREATE_RESULT_OK;
    }

    dmGameObject::CreateResult CompCollisionObjectDeleteWorld(const dmGameObject::ComponentDeleteWorldParams& params)
    {
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        CollisionWorld* world = (CollisionWorld*)params.m_World;
        if (world == 0x0)
        {
            return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
        }
        if (physics_context->m_3D)
            dmPhysics::DeleteWorld3D(physics_context->m_Context3D, world->m_World3D);
        else
            dmPhysics::DeleteWorld2D(physics_context->m_Context2D, world->m_World2D);
        delete world;
        return dmGameObject::CREATE_RESULT_OK;
    }

    static uint16_t GetGroupBitIndex(CollisionWorld* world, uint64_t group_hash)
    {
        if (group_hash != 0)
        {
            for (uint32_t i = 0; i < 16; ++i)
            {
                if (world->m_Groups[i] != 0)
                {
                    if (world->m_Groups[i] == group_hash)
                    {
                        return 1 << i;
                    }
                }
                else
                {
                    world->m_Groups[i] = group_hash;
                    return 1 << i;
                }
            }

            // When we get here, there are no more group bits available
            dmLogWarning("The collision group '%s' could not be used since the maximum group count has been reached (16).", dmHashReverseSafe64(group_hash));
        }
        return 0;
    }

    uint64_t GetLSBGroupHash(void* _world, uint16_t mask)
    {
        if (mask > 0)
        {
            uint32_t index = 0;
            while ((mask & 1) == 0)
            {
                mask >>= 1;
                ++index;
            }
            CollisionWorld* world = (CollisionWorld*)_world;
            return world->m_Groups[index];
        }
        return 0;
    }

    static void SetupEmptyTileGrid(CollisionWorld* world, CollisionComponent* component)
    {
        if (component->m_Resource->m_TileGrid)
        {
            dmPhysics::ClearGridShapeHulls(component->m_Object2D);
        }
    }

    static void SetupTileGrid(CollisionWorld* world, CollisionComponent* component) {
        CollisionObjectResource* resource = component->m_Resource;
        if (resource->m_TileGrid)
        {
            TileGridResource* tile_grid_resource = resource->m_TileGridResource;
            dmGameSystemDDF::TileGrid* tile_grid = tile_grid_resource->m_TileGrid;
            dmArray<dmPhysics::HCollisionShape2D>& shapes = resource->m_TileGridResource->m_GridShapes;
            uint32_t shape_count = shapes.Size();
            dmPhysics::HullFlags flags;

            TextureSetResource* texture_set_resource = tile_grid_resource->m_TextureSet;
            dmGameSystemDDF::TextureSet* tile_set = texture_set_resource->m_TextureSet;

            for (uint32_t i = 0; i < shape_count; ++i)
            {
                dmGameSystemDDF::TileLayer* layer = &tile_grid->m_Layers[i];

                // Set non-empty tiles
                uint32_t cell_count = layer->m_Cell.m_Count;
                for (uint32_t j = 0; j < cell_count; ++j)
                {
                    dmGameSystemDDF::TileCell* cell = &layer->m_Cell[j];
                    uint32_t tile = cell->m_Tile;

                    if (tile < tile_set->m_ConvexHulls.m_Count && tile_set->m_ConvexHulls[tile].m_Count > 0)
                    {
                        uint32_t cell_x = cell->m_X - tile_grid_resource->m_MinCellX;
                        uint32_t cell_y = cell->m_Y - tile_grid_resource->m_MinCellY;
                        dmPhysics::SetGridShapeHull(component->m_Object2D, i, cell_y, cell_x, tile, flags);
                        uint32_t child = cell_x + tile_grid_resource->m_ColumnCount * cell_y;
                        uint16_t group = GetGroupBitIndex(world, texture_set_resource->m_HullCollisionGroups[tile]);
                        dmPhysics::SetCollisionObjectFilter(component->m_Object2D, i, child, group, component->m_Mask);
                    }
                }

                dmPhysics::SetGridShapeEnable(component->m_Object2D, i, layer->m_IsVisible);
            }
        }
    }

    static void SetCollisionObjectData(CollisionWorld* world, CollisionComponent* component, CollisionObjectResource* resource, dmPhysicsDDF::CollisionObjectDesc* ddf, bool enabled, dmPhysics::CollisionObjectData &out_data)
    {
        out_data.m_UserData = component;
        out_data.m_Type = (dmPhysics::CollisionObjectType)ddf->m_Type;
        out_data.m_Mass = ddf->m_Mass;
        out_data.m_Friction = ddf->m_Friction;
        out_data.m_Restitution = ddf->m_Restitution;
        out_data.m_Group = GetGroupBitIndex(world, resource->m_Group);
        out_data.m_Mask = 0;
        out_data.m_LinearDamping = ddf->m_LinearDamping;
        out_data.m_AngularDamping = ddf->m_AngularDamping;
        out_data.m_LockedRotation = ddf->m_LockedRotation;
        out_data.m_Enabled = enabled;
        for (uint32_t i = 0; i < 16 && resource->m_Mask[i] != 0; ++i)
        {
            out_data.m_Mask |= GetGroupBitIndex(world, resource->m_Mask[i]);
        }
    }

    static bool CreateCollisionObject(PhysicsContext* physics_context, CollisionWorld* world, dmGameObject::HInstance instance, CollisionComponent* component, bool enabled)
    {
        if (world == 0x0)
        {
            return false;
        }
        CollisionObjectResource* resource = component->m_Resource;
        dmPhysicsDDF::CollisionObjectDesc* ddf = resource->m_DDF;
        dmPhysics::CollisionObjectData data;
        SetCollisionObjectData(world, component, resource, ddf, enabled, data);
        component->m_Mask = data.m_Mask;
        if (physics_context->m_3D)
        {
            if (resource->m_TileGrid)
            {
                dmLogError("Collision objects in 3D can not have a tile grid as shape.");
                return false;
            }
            dmPhysics::HWorld3D physics_world = world->m_World3D;
            dmPhysics::HCollisionObject3D collision_object =
                    dmPhysics::NewCollisionObject3D(physics_world, data,
                                                    resource->m_Shapes3D,
                                                    resource->m_ShapeTranslation,
                                                    resource->m_ShapeRotation,
                                                    resource->m_ShapeCount);

            if (collision_object != 0x0)
            {
                if (component->m_Object3D != 0x0)
                    dmPhysics::DeleteCollisionObject3D(physics_world, component->m_Object3D);
                component->m_Object3D = collision_object;
            }
            else
            {
                return false;
            }
        }
        else
        {
            dmPhysics::HWorld2D physics_world = world->m_World2D;
            dmPhysics::HCollisionObject2D collision_object = 0;
            collision_object = dmPhysics::NewCollisionObject2D(physics_world, data,
                                                               resource->m_Shapes2D,
                                                               resource->m_ShapeTranslation,
                                                               resource->m_ShapeRotation,
                                                               resource->m_ShapeCount);

            if (collision_object != 0x0)
            {
                if (component->m_Object2D != 0x0)
                    dmPhysics::DeleteCollisionObject2D(physics_world, component->m_Object2D);
                component->m_Object2D = collision_object;

                SetupEmptyTileGrid(world,component);

                if (enabled) {
                    SetupTileGrid(world, component);
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    }

    dmGameObject::CreateResult CompCollisionObjectCreate(const dmGameObject::ComponentCreateParams& params)
    {
        CollisionObjectResource* co_res = (CollisionObjectResource*)params.m_Resource;
        if (co_res == 0x0 || co_res->m_DDF == 0x0)
            return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
        if ((co_res->m_DDF->m_Mass == 0.0f && co_res->m_DDF->m_Type == dmPhysicsDDF::COLLISION_OBJECT_TYPE_DYNAMIC)
            || (co_res->m_DDF->m_Mass > 0.0f && co_res->m_DDF->m_Type != dmPhysicsDDF::COLLISION_OBJECT_TYPE_DYNAMIC))
        {
            dmLogError("Invalid mass %f for shape type %d", co_res->m_DDF->m_Mass, co_res->m_DDF->m_Type);
            return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
        }
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        CollisionComponent* component = new CollisionComponent();
        component->m_3D = (uint8_t) physics_context->m_3D;
        component->m_Resource = (CollisionObjectResource*)params.m_Resource;
        component->m_Instance = params.m_Instance;
        component->m_Object2D = 0;
        component->m_ComponentIndex = params.m_ComponentIndex;
        component->m_AddedToUpdate = false;
        component->m_StartAsEnabled = true;
        component->m_Joints = 0x0;
        component->m_JointEndPoints = 0x0;
        component->m_FlippedX = 0;
        component->m_FlippedY = 0;

        CollisionWorld* world = (CollisionWorld*)params.m_World;
        if (!CreateCollisionObject(physics_context, world, params.m_Instance, component, false))
        {
            delete component;
            return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
        }
        *params.m_UserData = (uintptr_t)component;
        return dmGameObject::CREATE_RESULT_OK;
    }

    dmGameObject::CreateResult CompCollisionObjectFinal(const dmGameObject::ComponentFinalParams& params)
    {
        CollisionComponent* component = (CollisionComponent*)*params.m_UserData;
        component->m_AddedToUpdate = false;
        component->m_StartAsEnabled = true;
        return dmGameObject::CREATE_RESULT_OK;
    }

    dmGameObject::CreateResult CompCollisionObjectDestroy(const dmGameObject::ComponentDestroyParams& params)
    {
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        CollisionComponent* component = (CollisionComponent*)*params.m_UserData;
        CollisionWorld* world = (CollisionWorld*)params.m_World;

        // Destroy joint ends
        JointEndPoint* joint_end = component->m_JointEndPoints;
        while (joint_end) {
            JointEntry* joint_entry = joint_end->m_JointEntry;
            DeleteJoint(world, joint_entry->m_Joint);
            joint_entry->m_Joint = 0x0;

            JointEndPoint* next = joint_end->m_Next;
            delete joint_end;
            joint_end = next;
        }
        component->m_JointEndPoints = 0x0;

        // Destroy joints
        JointEntry* joint_entry = component->m_Joints;
        while (joint_entry)
        {
            if (joint_entry->m_Joint)
            {
                DeleteJoint(world, joint_entry);
            }
            JointEntry* next = joint_entry->m_Next;
            delete joint_entry;
            joint_entry = next;
        }
        component->m_Joints = 0x0;

        if (physics_context->m_3D)
        {
            if (component->m_Object3D != 0)
            {
                dmPhysics::HWorld3D physics_world = world->m_World3D;
                dmPhysics::DeleteCollisionObject3D(physics_world, component->m_Object3D);
                component->m_Object3D = 0;
            }
        }
        else
        {
            if (component->m_Object2D != 0)
            {
                dmPhysics::HWorld2D physics_world = world->m_World2D;
                dmPhysics::DeleteCollisionObject2D(physics_world, component->m_Object2D);
                component->m_Object2D = 0;
            }
        }

        uint32_t num_components = world->m_Components.Size();
        for (uint32_t i = 0; i < num_components; ++i)
        {
            CollisionComponent* c = world->m_Components[i];
            if (c == component)
            {
                world->m_Components.EraseSwap(i);
                break;
            }
        }

        delete component;
        return dmGameObject::CREATE_RESULT_OK;
    }

    struct CollisionUserData
    {
        CollisionWorld* m_World;
        PhysicsContext* m_Context;
        uint32_t m_Count;
    };

    template <class DDFMessage>
    static void BroadCast(DDFMessage* ddf, dmGameObject::HInstance instance, dmhash_t instance_id, uint16_t component_index)
    {
        dmhash_t message_id = DDFMessage::m_DDFDescriptor->m_NameHash;
        uintptr_t descriptor = (uintptr_t)DDFMessage::m_DDFDescriptor;
        uint32_t data_size = sizeof(DDFMessage);
        dmMessage::URL sender;
        dmMessage::ResetURL(sender);
        dmMessage::URL receiver;
        dmMessage::ResetURL(receiver);
        receiver.m_Socket = dmGameObject::GetMessageSocket(dmGameObject::GetCollection(instance));
        receiver.m_Path = instance_id;
        // sender is the same as receiver, but with the specific collision object as fragment
        sender = receiver;
        dmGameObject::Result r = dmGameObject::GetComponentId(instance, component_index, &sender.m_Fragment);
        if (r != dmGameObject::RESULT_OK)
        {
            dmLogError("Could not retrieve sender component when reporting %s: %d", DDFMessage::m_DDFDescriptor->m_Name, r);
        }
        dmMessage::Result result = dmMessage::Post(&sender, &receiver, message_id, 0, descriptor, ddf, data_size, 0);
        if (result != dmMessage::RESULT_OK)
        {
            dmLogError("Could not send %s to component: %d", DDFMessage::m_DDFDescriptor->m_Name, result);
        }
    }

    bool CollisionCallback(void* user_data_a, uint16_t group_a, void* user_data_b, uint16_t group_b, void* user_data)
    {
        CollisionUserData* cud = (CollisionUserData*)user_data;
        if (cud->m_Count < cud->m_Context->m_MaxCollisionCount)
        {
            cud->m_Count += 1;

            CollisionComponent* component_a = (CollisionComponent*)user_data_a;
            CollisionComponent* component_b = (CollisionComponent*)user_data_b;
            dmGameObject::HInstance instance_a = component_a->m_Instance;
            dmGameObject::HInstance instance_b = component_b->m_Instance;
            dmhash_t instance_a_id = dmGameObject::GetIdentifier(instance_a);
            dmhash_t instance_b_id = dmGameObject::GetIdentifier(instance_b);

            dmPhysicsDDF::CollisionResponse ddf;

            uint64_t group_hash_a = GetLSBGroupHash(cud->m_World, group_a);
            uint64_t group_hash_b = GetLSBGroupHash(cud->m_World, group_b);

            // Broadcast to A components
            ddf.m_OwnGroup = group_hash_a;
            ddf.m_OtherGroup = group_hash_b;
            ddf.m_Group = group_hash_b;
            ddf.m_OtherId = instance_b_id;
            ddf.m_OtherPosition = dmGameObject::GetWorldPosition(instance_b);
            BroadCast(&ddf, instance_a, instance_a_id, component_a->m_ComponentIndex);

            // Broadcast to B components
            ddf.m_OwnGroup = group_hash_b;
            ddf.m_OtherGroup = group_hash_a;
            ddf.m_Group = group_hash_a;
            ddf.m_OtherId = instance_a_id;
            ddf.m_OtherPosition = dmGameObject::GetWorldPosition(instance_a);
            BroadCast(&ddf, instance_b, instance_b_id, component_b->m_ComponentIndex);

            return true;
        }
        else
        {
            return false;
        }
    }

    bool ContactPointCallback(const dmPhysics::ContactPoint& contact_point, void* user_data)
    {
        CollisionUserData* cud = (CollisionUserData*)user_data;
        if (cud->m_Count < cud->m_Context->m_MaxContactPointCount)
        {
            cud->m_Count += 1;

            CollisionComponent* component_a = (CollisionComponent*)contact_point.m_UserDataA;
            CollisionComponent* component_b = (CollisionComponent*)contact_point.m_UserDataB;
            dmGameObject::HInstance instance_a = component_a->m_Instance;
            dmGameObject::HInstance instance_b = component_b->m_Instance;
            dmhash_t instance_a_id = dmGameObject::GetIdentifier(instance_a);
            dmhash_t instance_b_id = dmGameObject::GetIdentifier(instance_b);

            dmPhysicsDDF::ContactPointResponse ddf;
            float mass_a = dmMath::Select(-contact_point.m_MassA, 0.0f, contact_point.m_MassA);
            float mass_b = dmMath::Select(-contact_point.m_MassB, 0.0f, contact_point.m_MassB);

            uint64_t group_hash_a = GetLSBGroupHash(cud->m_World, contact_point.m_GroupA);
            uint64_t group_hash_b = GetLSBGroupHash(cud->m_World, contact_point.m_GroupB);

            // Broadcast to A components
            ddf.m_Position = contact_point.m_PositionA;
            ddf.m_Normal = -contact_point.m_Normal;
            ddf.m_RelativeVelocity = -contact_point.m_RelativeVelocity;
            ddf.m_Distance = contact_point.m_Distance;
            ddf.m_AppliedImpulse = contact_point.m_AppliedImpulse;
            ddf.m_Mass = mass_a;
            ddf.m_OtherMass = mass_b;
            ddf.m_OtherId = instance_b_id;
            ddf.m_OtherPosition = dmGameObject::GetWorldPosition(instance_b);
            ddf.m_Group = group_hash_b;
            ddf.m_OwnGroup = group_hash_a;
            ddf.m_OtherGroup = group_hash_b;
            ddf.m_LifeTime = 0;
            BroadCast(&ddf, instance_a, instance_a_id, component_a->m_ComponentIndex);

            // Broadcast to B components
            ddf.m_Position = contact_point.m_PositionB;
            ddf.m_Normal = contact_point.m_Normal;
            ddf.m_RelativeVelocity = contact_point.m_RelativeVelocity;
            ddf.m_Distance = contact_point.m_Distance;
            ddf.m_AppliedImpulse = contact_point.m_AppliedImpulse;
            ddf.m_Mass = mass_b;
            ddf.m_OtherMass = mass_a;
            ddf.m_OtherId = instance_a_id;
            ddf.m_OtherPosition = dmGameObject::GetWorldPosition(instance_a);
            ddf.m_Group = group_hash_a;
            ddf.m_OwnGroup = group_hash_b;
            ddf.m_OtherGroup = group_hash_a;
            ddf.m_LifeTime = 0;
            BroadCast(&ddf, instance_b, instance_b_id, component_b->m_ComponentIndex);

            return true;
        }
        else
        {
            return false;
        }
    }

    static bool g_CollisionOverflowWarning = false;
    static bool g_ContactOverflowWarning = false;

    void TriggerEnteredCallback(const dmPhysics::TriggerEnter& trigger_enter, void* user_data)
    {
        CollisionWorld* world = (CollisionWorld*)user_data;
        CollisionComponent* component_a = (CollisionComponent*)trigger_enter.m_UserDataA;
        CollisionComponent* component_b = (CollisionComponent*)trigger_enter.m_UserDataB;
        dmGameObject::HInstance instance_a = component_a->m_Instance;
        dmGameObject::HInstance instance_b = component_b->m_Instance;
        dmhash_t instance_a_id = dmGameObject::GetIdentifier(instance_a);
        dmhash_t instance_b_id = dmGameObject::GetIdentifier(instance_b);

        dmPhysicsDDF::TriggerResponse ddf;
        ddf.m_Enter = 1;

        uint64_t group_hash_a = GetLSBGroupHash(world, trigger_enter.m_GroupA);
        uint64_t group_hash_b = GetLSBGroupHash(world, trigger_enter.m_GroupB);

        // Broadcast to A components
        ddf.m_OtherId = instance_b_id;
        ddf.m_Group = group_hash_b;
        ddf.m_OwnGroup = group_hash_a;
        ddf.m_OtherGroup = group_hash_b;
        BroadCast(&ddf, instance_a, instance_a_id, component_a->m_ComponentIndex);

        // Broadcast to B components
        ddf.m_OtherId = instance_a_id;
        ddf.m_Group = group_hash_a;
        ddf.m_OwnGroup = group_hash_b;
        ddf.m_OtherGroup = group_hash_a;
        BroadCast(&ddf, instance_b, instance_b_id, component_b->m_ComponentIndex);
    }

    void TriggerExitedCallback(const dmPhysics::TriggerExit& trigger_exit, void* user_data)
    {
        CollisionWorld* world = (CollisionWorld*)user_data;
        CollisionComponent* component_a = (CollisionComponent*)trigger_exit.m_UserDataA;
        CollisionComponent* component_b = (CollisionComponent*)trigger_exit.m_UserDataB;
        dmGameObject::HInstance instance_a = component_a->m_Instance;
        dmGameObject::HInstance instance_b = component_b->m_Instance;
        dmhash_t instance_a_id = dmGameObject::GetIdentifier(instance_a);
        dmhash_t instance_b_id = dmGameObject::GetIdentifier(instance_b);

        dmPhysicsDDF::TriggerResponse ddf;
        ddf.m_Enter = 0;

        uint64_t group_hash_a = GetLSBGroupHash(world, trigger_exit.m_GroupA);
        uint64_t group_hash_b = GetLSBGroupHash(world, trigger_exit.m_GroupB);

        // Broadcast to A components
        ddf.m_OtherId = instance_b_id;
        ddf.m_Group = group_hash_b;
        ddf.m_OwnGroup = group_hash_a;
        ddf.m_OtherGroup = group_hash_b;
        BroadCast(&ddf, instance_a, instance_a_id, component_a->m_ComponentIndex);

        // Broadcast to B components
        ddf.m_OtherId = instance_a_id;
        ddf.m_Group = group_hash_a;
        ddf.m_OwnGroup = group_hash_b;
        ddf.m_OtherGroup = group_hash_a;
        BroadCast(&ddf, instance_b, instance_b_id, component_b->m_ComponentIndex);
    }

    struct RayCastUserData
    {
        dmGameObject::HInstance m_Instance;
        CollisionWorld* m_World;
    };

    static void RayCastCallback(const dmPhysics::RayCastResponse& response, const dmPhysics::RayCastRequest& request, void* user_data)
    {
        dmhash_t message_id;
        uintptr_t descriptor;
        uint32_t data_size;
        void* message_data;
        dmPhysicsDDF::RayCastResponse responsDDF;
        dmPhysicsDDF::RayCastMissed missedDDF;
        if (response.m_Hit)
        {
            CollisionWorld* world = (CollisionWorld*)user_data;
            CollisionComponent* component = (CollisionComponent*)response.m_CollisionObjectUserData;

            responsDDF.m_Fraction = response.m_Fraction;
            responsDDF.m_Id = dmGameObject::GetIdentifier(component->m_Instance);
            responsDDF.m_Group = GetLSBGroupHash(world, response.m_CollisionObjectGroup);
            responsDDF.m_Position = response.m_Position;
            responsDDF.m_Normal = response.m_Normal;
            responsDDF.m_RequestId = request.m_UserId & 0xff;

            message_id = dmPhysicsDDF::RayCastResponse::m_DDFDescriptor->m_NameHash;
            descriptor = (uintptr_t)dmPhysicsDDF::RayCastResponse::m_DDFDescriptor;
            data_size = sizeof(dmPhysicsDDF::RayCastResponse);
            message_data = &responsDDF;
        } else {
            missedDDF.m_RequestId = request.m_UserId & 0xff;

            message_id = dmPhysicsDDF::RayCastMissed::m_DDFDescriptor->m_NameHash;
            descriptor = (uintptr_t)dmPhysicsDDF::RayCastMissed::m_DDFDescriptor;
            data_size = sizeof(dmPhysicsDDF::RayCastMissed);
            message_data = &missedDDF;
        }

        dmGameObject::HInstance instance = (dmGameObject::HInstance)request.m_UserData;
        dmMessage::URL receiver;
        receiver.m_Socket = dmGameObject::GetMessageSocket(dmGameObject::GetCollection(instance));
        receiver.m_Path = dmGameObject::GetIdentifier(instance);
        uint16_t component_index = request.m_UserId >> 16;
        dmGameObject::Result result = dmGameObject::GetComponentId(instance, component_index, &receiver.m_Fragment);
        if (result != dmGameObject::RESULT_OK)
        {
            dmLogError("Error when sending ray cast response: %d", result);
        }
        else
        {
            dmMessage::Result message_result = dmMessage::Post(0x0, &receiver, message_id, 0, descriptor, message_data, data_size, 0);
            if (message_result != dmMessage::RESULT_OK)
            {
                dmLogError("Error when sending ray cast response: %d", message_result);
            }
        }
    }

    struct DispatchContext
    {
        PhysicsContext* m_PhysicsContext;
        bool m_Success;
        dmGameObject::HCollection m_Collection;
        CollisionWorld* m_World;
    };

    void DispatchCallback(dmMessage::Message *message, void* user_ptr)
    {
        DispatchContext* context = (DispatchContext*)user_ptr;
        if (message->m_Descriptor != 0)
        {
            dmDDF::Descriptor* descriptor = (dmDDF::Descriptor*)message->m_Descriptor;
            if (descriptor == dmPhysicsDDF::RequestRayCast::m_DDFDescriptor)
            {
                dmPhysicsDDF::RequestRayCast* ddf = (dmPhysicsDDF::RequestRayCast*)message->m_Data;
                dmGameObject::HInstance sender_instance = (dmGameObject::HInstance)message->m_UserData;
                uint16_t component_index;
                dmGameObject::Result go_result = dmGameObject::GetComponentIndex(sender_instance, message->m_Sender.m_Fragment, &component_index);
                if (go_result != dmGameObject::RESULT_OK)
                {
                    dmLogError("Component index could not be retrieved when handling '%s': %d.", dmPhysicsDDF::RequestRayCast::m_DDFDescriptor->m_Name, go_result);
                    context->m_Success = false;
                }
                else
                {
                    // Target collection which can be different than we are updating for.
                    dmGameObject::HCollection collection = dmGameObject::GetCollection(sender_instance);

                    // NOTE! The collision world for the target collection is looked up using this worlds component index
                    //       which is assumed to be the same as in the target collection.
                    CollisionWorld* world = (CollisionWorld*) dmGameObject::GetWorld(collection, context->m_World->m_ComponentIndex);

                    // Give that the assumption above holds, this assert will hold too.
                    assert(world->m_ComponentIndex == context->m_World->m_ComponentIndex);

                    dmPhysics::RayCastRequest request;
                    request.m_From = ddf->m_From;
                    request.m_To = ddf->m_To;
                    request.m_IgnoredUserData = sender_instance;
                    request.m_Mask = ddf->m_Mask;
                    request.m_UserId = component_index << 16 | (ddf->m_RequestId & 0xff);
                    request.m_UserData = (void*)sender_instance;

                    if (world->m_3D)
                    {
                        dmPhysics::RequestRayCast3D(world->m_World3D, request);
                    }
                    else
                    {
                        dmPhysics::RequestRayCast2D(world->m_World2D, request);
                    }
                }
            }
        }
    }

    dmGameObject::CreateResult CompCollisionObjectAddToUpdate(const dmGameObject::ComponentAddToUpdateParams& params) {
        CollisionWorld* world = (CollisionWorld*)params.m_World;
        if (world == 0x0)
        {
            return dmGameObject::CREATE_RESULT_UNKNOWN_ERROR;
        }
        CollisionComponent* component = (CollisionComponent*)*params.m_UserData;
        assert(!component->m_AddedToUpdate);

        if (component->m_3D) {
            dmPhysics::SetEnabled3D(world->m_World3D, component->m_Object3D, component->m_StartAsEnabled);
        } else {
            dmPhysics::SetEnabled2D(world->m_World2D, component->m_Object2D, component->m_StartAsEnabled);
            SetupTileGrid(world, component);
        }
        component->m_AddedToUpdate = true;

        if (world->m_Components.Full())
            world->m_Components.OffsetCapacity(32);
        world->m_Components.Push(component);
        return dmGameObject::CREATE_RESULT_OK;
    }

    // This function will dispatch on the (global) physics socket, and will potentially handle messages belonging to other collections
    // than the current one being updated.
    //
    // TODO: Make a nicer solution for this, perhaps a per-collection physics socket
    bool CompCollisionObjectDispatchPhysicsMessages(PhysicsContext *physics_context, CollisionWorld *world, dmGameObject::HCollection collection)
    {
        DispatchContext dispatch_context;
        dispatch_context.m_PhysicsContext = physics_context;
        dispatch_context.m_Success = true;
        dispatch_context.m_World = world;
        dispatch_context.m_Collection = collection;
        dmMessage::HSocket physics_socket;
        if (physics_context->m_3D)
        {
            physics_socket = dmPhysics::GetSocket3D(physics_context->m_Context3D);
        }
        else
        {
            physics_socket = dmPhysics::GetSocket2D(physics_context->m_Context2D);

        }
        dmMessage::Dispatch(physics_socket, DispatchCallback, (void*)&dispatch_context);
        return dispatch_context.m_Success;
    }

    dmGameObject::UpdateResult CompCollisionObjectUpdate(const dmGameObject::ComponentsUpdateParams& params, dmGameObject::ComponentsUpdateResult& update_result)
    {
        if (params.m_World == 0x0)
            return dmGameObject::UPDATE_RESULT_OK;
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;

        dmGameObject::UpdateResult result = dmGameObject::UPDATE_RESULT_OK;
        CollisionWorld* world = (CollisionWorld*)params.m_World;

        if (!CompCollisionObjectDispatchPhysicsMessages(physics_context, world, params.m_Collection))
            result = dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;

        // Hot-reload is not available in release, so lets not iterate collision components in that case.
        if (dLib::IsDebugMode())
        {
            uint32_t num_components = world->m_Components.Size();
            for (uint32_t i = 0; i < num_components; ++i)
            {
                CollisionComponent* c = world->m_Components[i];
                TileGridResource* tile_grid_res = c->m_Resource->m_TileGridResource;
                if (tile_grid_res != 0x0 && tile_grid_res->m_Dirty)
                {
                    CollisionObjectResource* resource = c->m_Resource;
                    dmPhysicsDDF::CollisionObjectDesc* ddf = resource->m_DDF;
                    dmPhysics::CollisionObjectData data;
                    SetCollisionObjectData(world, c, c->m_Resource, ddf, true, data);
                    c->m_Mask = data.m_Mask;

                    dmPhysics::DeleteCollisionObject2D(world->m_World2D, c->m_Object2D);
                    dmArray<dmPhysics::HCollisionShape2D>& shapes = resource->m_TileGridResource->m_GridShapes;
                    c->m_Object2D = dmPhysics::NewCollisionObject2D(world->m_World2D, data, &shapes.Front(), shapes.Size());

                    SetupEmptyTileGrid(world, c);
                    SetupTileGrid(world, c);
                    tile_grid_res->m_Dirty = 0;
                }
            }
        }

        CollisionUserData collision_user_data;
        collision_user_data.m_World = world;
        collision_user_data.m_Context = physics_context;
        collision_user_data.m_Count = 0;
        CollisionUserData contact_user_data;
        contact_user_data.m_World = world;
        contact_user_data.m_Context = physics_context;
        contact_user_data.m_Count = 0;

        dmPhysics::StepWorldContext step_world_context;
        step_world_context.m_DT = params.m_UpdateContext->m_DT;
        step_world_context.m_CollisionCallback = CollisionCallback;
        step_world_context.m_CollisionUserData = &collision_user_data;
        step_world_context.m_ContactPointCallback = ContactPointCallback;
        step_world_context.m_ContactPointUserData = &contact_user_data;
        step_world_context.m_TriggerEnteredCallback = TriggerEnteredCallback;
        step_world_context.m_TriggerEnteredUserData = world;
        step_world_context.m_TriggerExitedCallback = TriggerExitedCallback;
        step_world_context.m_TriggerExitedUserData = world;
        step_world_context.m_RayCastCallback = RayCastCallback;
        step_world_context.m_RayCastUserData = world;

        world->m_LastDT = params.m_UpdateContext->m_DT;

        g_NumPhysicsTransformsUpdated = 0;

        if (physics_context->m_3D)
        {
            dmPhysics::StepWorld3D(world->m_World3D, step_world_context);
        }
        else
        {
            dmPhysics::StepWorld2D(world->m_World2D, step_world_context);
        }

        update_result.m_TransformsUpdated = g_NumPhysicsTransformsUpdated > 0;

        if (collision_user_data.m_Count >= physics_context->m_MaxCollisionCount)
        {
            if (!g_CollisionOverflowWarning)
            {
                dmLogWarning("Maximum number of collisions (%d) reached, messages have been lost. Tweak \"%s\" in the config file.", physics_context->m_MaxCollisionCount, PHYSICS_MAX_COLLISIONS_KEY);
                g_CollisionOverflowWarning = true;
            }
        }
        else
        {
            g_CollisionOverflowWarning = false;
        }
        if (contact_user_data.m_Count >= physics_context->m_MaxContactPointCount)
        {
            if (!g_ContactOverflowWarning)
            {
                dmLogWarning("Maximum number of contacts (%d) reached, messages have been lost. Tweak \"%s\" in the config file.", physics_context->m_MaxContactPointCount, PHYSICS_MAX_CONTACTS_KEY);
                g_ContactOverflowWarning = true;
            }
        }
        else
        {
            g_ContactOverflowWarning = false;
        }
        if (physics_context->m_3D)
            dmPhysics::SetDrawDebug3D(world->m_World3D, physics_context->m_Debug);
        else
            dmPhysics::SetDrawDebug2D(world->m_World2D, physics_context->m_Debug);
        return result;
    }

    dmGameObject::UpdateResult CompCollisionObjectPostUpdate(const dmGameObject::ComponentsPostUpdateParams& params)
    {
        if (params.m_World == 0x0)
            return dmGameObject::UPDATE_RESULT_OK;

        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        CollisionWorld* world = (CollisionWorld*)params.m_World;

        // Dispatch also in post-messages since messages might have been posting from script components, or init
        // functions in factories, and they should not linger around to next frame (which might not come around)
        if (!CompCollisionObjectDispatchPhysicsMessages(physics_context, world, params.m_Collection))
            return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;

        return dmGameObject::UPDATE_RESULT_OK;
    }

    dmGameObject::UpdateResult CompCollisionObjectOnMessage(const dmGameObject::ComponentOnMessageParams& params)
    {
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        CollisionComponent* component = (CollisionComponent*) *params.m_UserData;

        if (params.m_Message->m_Id == dmGameObjectDDF::Enable::m_DDFDescriptor->m_NameHash
                || params.m_Message->m_Id == dmGameObjectDDF::Disable::m_DDFDescriptor->m_NameHash)
        {
            bool enable = false;
            if (params.m_Message->m_Id == dmGameObjectDDF::Enable::m_DDFDescriptor->m_NameHash)
            {
                enable = true;
            }
            CollisionWorld* world = (CollisionWorld*)params.m_World;

            if (component->m_AddedToUpdate)
            {
                if (physics_context->m_3D)
                {
                    dmPhysics::SetEnabled3D(world->m_World3D, component->m_Object3D, enable);
                }
                else
                {
                    dmPhysics::SetEnabled2D(world->m_World2D, component->m_Object2D, enable);
                }
            }
            else
            {
                // Deferred controlling the enabled state. Objects are force disabled until
                // they are added to update.
                component->m_StartAsEnabled = enable;
            }
        }
        else if (params.m_Message->m_Id == dmPhysicsDDF::ApplyForce::m_DDFDescriptor->m_NameHash)
        {
            dmPhysicsDDF::ApplyForce* af = (dmPhysicsDDF::ApplyForce*) params.m_Message->m_Data;
            if (physics_context->m_3D)
            {
                dmPhysics::ApplyForce3D(physics_context->m_Context3D, component->m_Object3D, af->m_Force, af->m_Position);
            }
            else
            {
                dmPhysics::ApplyForce2D(physics_context->m_Context2D, component->m_Object2D, af->m_Force, af->m_Position);
            }
        }
        else if (params.m_Message->m_Id == dmPhysicsDDF::ApplyForceImpulse::m_DDFDescriptor->m_NameHash)
        {
            dmPhysicsDDF::ApplyForceImpulse* af = (dmPhysicsDDF::ApplyForceImpulse*) params.m_Message->m_Data;
            if (physics_context->m_3D)
            {
                dmPhysics::ApplyForce3DImpulse(physics_context->m_Context3D, component->m_Object3D, af->m_Force, af->m_Position);
            }
            else
            {
                dmPhysics::ApplyForce2DImpulse(physics_context->m_Context2D, component->m_Object2D, af->m_Force, af->m_Position);
            }
        }
        else if (params.m_Message->m_Id == dmPhysicsDDF::RequestVelocity::m_DDFDescriptor->m_NameHash)
        {
            dmPhysicsDDF::VelocityResponse response;
            if (physics_context->m_3D)
            {
                response.m_LinearVelocity = dmPhysics::GetLinearVelocity3D(physics_context->m_Context3D, component->m_Object3D);
                response.m_AngularVelocity = dmPhysics::GetAngularVelocity3D(physics_context->m_Context3D, component->m_Object3D);
            }
            else
            {
                response.m_LinearVelocity = dmPhysics::GetLinearVelocity2D(physics_context->m_Context2D, component->m_Object2D);
                response.m_AngularVelocity = dmPhysics::GetAngularVelocity2D(physics_context->m_Context2D, component->m_Object2D);
            }
            dmhash_t message_id = dmPhysicsDDF::VelocityResponse::m_DDFDescriptor->m_NameHash;
            uintptr_t descriptor = (uintptr_t)dmPhysicsDDF::VelocityResponse::m_DDFDescriptor;
            uint32_t data_size = sizeof(dmPhysicsDDF::VelocityResponse);
            dmMessage::Result result = dmMessage::Post(&params.m_Message->m_Receiver, &params.m_Message->m_Sender, message_id, 0, descriptor, &response, data_size, 0);
            if (result != dmMessage::RESULT_OK)
            {
                dmLogError("Could not send %s to component, result: %d.", dmPhysicsDDF::VelocityResponse::m_DDFDescriptor->m_Name, result);
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
        }
        else if (params.m_Message->m_Id == dmPhysicsDDF::RequestBodyPosition::m_DDFDescriptor->m_NameHash)
        {
            dmPhysicsDDF::BodyPositionResponse response;
            if (physics_context->m_3D)
            {
                response.m_BodyPosition = Vector3(dmPhysics::GetWorldPosition3D(
                    physics_context->m_Context3D, component->m_Object3D));
            }
            else
            {
                response.m_BodyPosition = Vector3(dmPhysics::GetWorldPosition2D(
                    physics_context->m_Context2D, component->m_Object2D));
            }
            dmhash_t message_id      = dmPhysicsDDF::BodyPositionResponse::m_DDFDescriptor->m_NameHash;
            uintptr_t descriptor     = (uintptr_t)dmPhysicsDDF::BodyPositionResponse::m_DDFDescriptor;
            uint32_t data_size       = sizeof(dmPhysicsDDF::BodyPositionResponse);
            dmMessage::Result result = dmMessage::Post(&params.m_Message->m_Receiver, &params.m_Message->m_Sender, message_id, 0, descriptor, &response, data_size, 0);
            if (result != dmMessage::RESULT_OK)
            {
                dmLogError("Could not send %s to component, result: %d.",   dmPhysicsDDF::BodyPositionResponse::m_DDFDescriptor->m_Name, result);
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
        }
        else if (params.m_Message->m_Id == dmPhysicsDDF::RequestBodyAngle::m_DDFDescriptor->m_NameHash)
        {
            dmPhysicsDDF::BodyAngleResponse response;
            if (physics_context->m_3D)
            {
                // Undefined
            }
            else
            {
                response.m_BodyAngle = dmPhysics::GetBodyAngle2D(component->m_Object2D);
            }
            dmhash_t message_id      = dmPhysicsDDF::BodyAngleResponse::m_DDFDescriptor->m_NameHash;
            uintptr_t descriptor     = (uintptr_t)dmPhysicsDDF::BodyAngleResponse::m_DDFDescriptor;
            uint32_t data_size       = sizeof(dmPhysicsDDF::BodyAngleResponse);
            dmMessage::Result result = dmMessage::Post(&params.m_Message->m_Receiver, &params.m_Message->m_Sender, message_id, 0, descriptor, &response, data_size, 0);
            if (result != dmMessage::RESULT_OK)
            {
                dmLogError("Could not send %s to component, result: %d.", dmPhysicsDDF::BodyAngleResponse::m_DDFDescriptor->m_Name, result);
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
        }
        else if (params.m_Message->m_Id == dmPhysicsDDF::SetGridShapeHull::m_DDFDescriptor->m_NameHash)
        {
            if (physics_context->m_3D)
            {
                dmLogError("Grid shape hulls can only be set for 2D physics.");
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
            if (component->m_Resource->m_TileGrid == 0)
            {
                dmLogError("Hulls can only be set for collision objects with tile grids as shape.");
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
            dmPhysicsDDF::SetGridShapeHull* ddf = (dmPhysicsDDF::SetGridShapeHull*) params.m_Message->m_Data;
            uint32_t column = ddf->m_Column;
            uint32_t row = ddf->m_Row;
            uint32_t hull = ddf->m_Hull;

            TileGridResource* tile_grid_resource = component->m_Resource->m_TileGridResource;

            if (row >= tile_grid_resource->m_RowCount || column >= tile_grid_resource->m_ColumnCount)
            {
                dmLogError("SetGridShapeHull: <row,column> out of bounds");
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
            if (hull != ~0u && hull >= tile_grid_resource->m_TextureSet->m_HullCollisionGroups.Size())
            {
                dmLogError("SetGridShapHull: specified hull index is out of bounds.");
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }

            dmPhysics::HullFlags flags;
            flags.m_FlipHorizontal = ddf->m_FlipHorizontal;
            flags.m_FlipVertical = ddf->m_FlipVertical;
            dmPhysics::SetGridShapeHull(component->m_Object2D, ddf->m_Shape, row, column, hull, flags);
            uint16_t child = column + tile_grid_resource->m_ColumnCount * row;
            uint16_t group = 0;
            uint16_t mask = 0;
            // Hull-index of 0xffffffff is empty cell
            if (hull != ~0u)
            {
                group = GetGroupBitIndex((CollisionWorld*)params.m_World, tile_grid_resource->m_TextureSet->m_HullCollisionGroups[hull]);
                mask = component->m_Mask;
            }
            dmPhysics::SetCollisionObjectFilter(component->m_Object2D, ddf->m_Shape, child, group, mask);
        }
        else if(params.m_Message->m_Id == dmPhysicsDDF::EnableGridShapeLayer::m_DDFDescriptor->m_NameHash)
        {
            assert(!physics_context->m_3D);
            if (component->m_Resource->m_TileGrid == 0)
            {
                dmLogError("Layer visibility can only be set on tile grids");
                return dmGameObject::UPDATE_RESULT_UNKNOWN_ERROR;
            }
            dmPhysicsDDF::EnableGridShapeLayer* ddf = (dmPhysicsDDF::EnableGridShapeLayer*) params.m_Message->m_Data;
            dmPhysics::SetGridShapeEnable(component->m_Object2D, ddf->m_Shape, ddf->m_Enable);
        }
        return dmGameObject::UPDATE_RESULT_OK;
    }

    void CompCollisionObjectOnReload(const dmGameObject::ComponentOnReloadParams& params)
    {
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        CollisionWorld* world = (CollisionWorld*)params.m_World;
        CollisionComponent* component = (CollisionComponent*)*params.m_UserData;
        component->m_Resource = (CollisionObjectResource*)params.m_Resource;
        component->m_AddedToUpdate = false;
        component->m_StartAsEnabled = true;
        if (!CreateCollisionObject(physics_context, world, params.m_Instance, component, true))
        {
            dmLogError("%s", "Could not recreate collision object component, not reloaded.");
        }
    }

    dmGameObject::PropertyResult CompCollisionObjectGetProperty(const dmGameObject::ComponentGetPropertyParams& params, dmGameObject::PropertyDesc& out_value) {
        CollisionComponent* component = (CollisionComponent*)*params.m_UserData;
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;
        if (params.m_PropertyId == PROP_LINEAR_VELOCITY) {
            if (physics_context->m_3D) {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetLinearVelocity3D(physics_context->m_Context3D, component->m_Object3D));
            } else {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetLinearVelocity2D(physics_context->m_Context2D, component->m_Object2D));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_ANGULAR_VELOCITY) {
            if (physics_context->m_3D) {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetAngularVelocity3D(physics_context->m_Context3D, component->m_Object3D));
            } else {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetAngularVelocity2D(physics_context->m_Context2D, component->m_Object2D));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_MASS) {
            if (physics_context->m_3D) {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetMass3D(component->m_Object3D));
            } else {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetMass2D(component->m_Object2D));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_LINEAR_DAMPING) {
            if (physics_context->m_3D) {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetLinearDamping3D(component->m_Object3D));
            } else {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetLinearDamping2D(component->m_Object2D));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_ANGULAR_DAMPING) {
            if (physics_context->m_3D) {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetAngularDamping3D(component->m_Object3D));
            } else {
                out_value.m_Variant = dmGameObject::PropertyVar(dmPhysics::GetAngularDamping2D(component->m_Object2D));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        }
        else if (params.m_PropertyId == PROP_BODY_POSITION)
        {
            if (physics_context->m_3D)
            {
                Vectormath::Aos::Vector3 vec3 = Vectormath::Aos::Vector3(dmPhysics::GetWorldPosition3D(physics_context->m_Context3D, component->m_Object3D));
                out_value.m_Variant           = dmGameObject::PropertyVar(vec3);
            }
            else
            {
                Vectormath::Aos::Vector3 vec3 = Vectormath::Aos::Vector3(dmPhysics::GetWorldPosition2D(physics_context->m_Context2D, component->m_Object2D));
                out_value.m_Variant           = dmGameObject::PropertyVar(vec3);
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        }
        else if (params.m_PropertyId == PROP_BODY_ANGLE)
        {
            if (physics_context->m_3D)
            {
                float angle         = 0.0f;//dmPhysics::GetBodyAngle2D(physics_context);
                out_value.m_Variant = dmGameObject::PropertyVar(angle);
            }
            else
            {
                float angle         = dmPhysics::GetBodyAngle2D(physics_context);
                out_value.m_Variant = dmGameObject::PropertyVar(angle);
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        }
        else
        {
            return dmGameObject::PROPERTY_RESULT_NOT_FOUND;
        }
    }

    dmGameObject::PropertyResult CompCollisionObjectSetProperty(const dmGameObject::ComponentSetPropertyParams& params) {
        CollisionComponent* component = (CollisionComponent*)*params.m_UserData;
        PhysicsContext* physics_context = (PhysicsContext*)params.m_Context;

        if (params.m_PropertyId == PROP_LINEAR_VELOCITY) {
            if (params.m_Value.m_Type != dmGameObject::PROPERTY_TYPE_VECTOR3)
                return dmGameObject::PROPERTY_RESULT_TYPE_MISMATCH;
            if (physics_context->m_3D) {
                dmPhysics::SetLinearVelocity3D(physics_context->m_Context3D, component->m_Object3D, Vectormath::Aos::Vector3(params.m_Value.m_V4[0], params.m_Value.m_V4[1], params.m_Value.m_V4[2]));
            } else {
                dmPhysics::SetLinearVelocity2D(physics_context->m_Context2D, component->m_Object2D, Vectormath::Aos::Vector3(params.m_Value.m_V4[0], params.m_Value.m_V4[1], params.m_Value.m_V4[2]));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_ANGULAR_VELOCITY) {
            if (params.m_Value.m_Type != dmGameObject::PROPERTY_TYPE_VECTOR3)
                return dmGameObject::PROPERTY_RESULT_TYPE_MISMATCH;
            if (physics_context->m_3D) {
                dmPhysics::SetAngularVelocity3D(physics_context->m_Context3D, component->m_Object3D, Vectormath::Aos::Vector3(params.m_Value.m_V4[0], params.m_Value.m_V4[1], params.m_Value.m_V4[2]));
            } else {
                dmPhysics::SetAngularVelocity2D(physics_context->m_Context2D, component->m_Object2D, Vectormath::Aos::Vector3(params.m_Value.m_V4[0], params.m_Value.m_V4[1], params.m_Value.m_V4[2]));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_LINEAR_DAMPING) {
            if (params.m_Value.m_Type != dmGameObject::PROPERTY_TYPE_NUMBER)
                return dmGameObject::PROPERTY_RESULT_TYPE_MISMATCH;
            if (physics_context->m_3D) {
                dmPhysics::SetLinearDamping3D(component->m_Object3D, params.m_Value.m_Number);
            } else {
                dmPhysics::SetLinearDamping2D(component->m_Object2D, params.m_Value.m_Number);
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        } else if (params.m_PropertyId == PROP_ANGULAR_DAMPING) {
            if (params.m_Value.m_Type != dmGameObject::PROPERTY_TYPE_NUMBER)
                return dmGameObject::PROPERTY_RESULT_TYPE_MISMATCH;
            if (physics_context->m_3D) {
                dmPhysics::SetAngularDamping3D(component->m_Object3D, params.m_Value.m_Number);
            } else {
                dmPhysics::SetAngularDamping2D(component->m_Object2D, params.m_Value.m_Number);
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        }
        else if (params.m_PropertyId == PROP_BODY_POSITION)
        {
            if (params.m_Value.m_Type != dmGameObject::PROPERTY_TYPE_VECTOR3)
                return dmGameObject::PROPERTY_RESULT_TYPE_MISMATCH;
            if (physics_context->m_3D)
            {
                //TODO: Undefined
            }
            else
            {
                dmPhysics::SetWorldPosition2D(physics_context->m_Context2D,component->m_Object2D,
                                              Vectormath::Aos::Vector3(
                                                  params.m_Value.m_V4[0],
                                                  params.m_Value.m_V4[1],
                                                  params.m_Value.m_V4[2]));
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        }
        else if (params.m_PropertyId == PROP_BODY_ANGLE)
        {
            if (params.m_Value.m_Type != dmGameObject::PROPERTY_TYPE_NUMBER)
                return dmGameObject::PROPERTY_RESULT_TYPE_MISMATCH;
            if (physics_context->m_3D)
            {
                //TODO: Undefined
            }
            else
            {
                dmPhysics::SetBodyAngle2D(component->m_Object2D, params.m_Value.m_Number);
            }
            return dmGameObject::PROPERTY_RESULT_OK;
        }
        else
        {
            return dmGameObject::PROPERTY_RESULT_NOT_FOUND;
        }
    }

    uint16_t CompCollisionGetGroupBitIndex(void* world, uint64_t group_hash)
    {
        return GetGroupBitIndex((CollisionWorld*)world, group_hash);
    }

    void RayCast(void* _world, const dmPhysics::RayCastRequest& request, dmArray<dmPhysics::RayCastResponse>& results)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (world->m_3D)
        {
            dmPhysics::RayCast3D(world->m_World3D, request, results);
        }
        else
        {
            dmPhysics::RayCast2D(world->m_World2D, request, results);
        }
    }

    // Find a JointEntry in the linked list of a collision component based on the joint id.
    static JointEntry* FindJointEntry(CollisionWorld* world, CollisionComponent* component, dmhash_t id)
    {
        JointEntry* joint_entry = component->m_Joints;
        while (joint_entry)
        {
            if (joint_entry->m_Id == id)
            {
                return joint_entry;
            }
            joint_entry = joint_entry->m_Next;
        }

        return 0x0;
    }

    inline bool IsJointsSupported(CollisionWorld* world)
    {
        if (world->m_3D)
        {
            dmLogError("joints are currently only available in 2D physics");
            return false;
        }
        return true;
    }

    // Connects a joint between two components, a JointEntry with the id must exist for this to succeed.
    dmPhysics::JointResult CreateJoint(void* _world, void* _component_a, dmhash_t id, const Vectormath::Aos::Point3& apos, void* _component_b, const Vectormath::Aos::Point3& bpos, dmPhysics::JointType type, const dmPhysics::ConnectJointParams& joint_params)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component_a = (CollisionComponent*)_component_a;
        CollisionComponent* component_b = (CollisionComponent*)_component_b;

        // Check if there is already a joint with this id
        JointEntry* joint_entry = FindJointEntry(world, component_a, id);
        if (joint_entry) {
            return dmPhysics::RESULT_ID_EXISTS;
        }

        dmPhysics::HJoint joint_handle = dmPhysics::CreateJoint2D(world->m_World2D, component_a->m_Object2D, apos, component_b->m_Object2D, bpos, type, joint_params);

        // NOTE: For the future, we might think about preallocating these structs
        // in batches (when needed) and store them in an object pool.
        // - so that when deleting a collision world, it's easy to delete everything quickly (as opposed to traversing each component).
        // - so we can avoid the frequent new/delete.

        // Create new joint entry to hold the generic joint handle
        joint_entry = new JointEntry(id, joint_handle, component_a->m_Joints);
        component_a->m_Joints = joint_entry;
        joint_entry->m_Type = type;

        // Setup a joint end point for component B.
        JointEndPoint* new_end_point = new JointEndPoint();
        new_end_point->m_Next = component_b->m_JointEndPoints;
        new_end_point->m_JointEntry = joint_entry;
        new_end_point->m_Owner = component_b;
        component_b->m_JointEndPoints = new_end_point;

        joint_entry->m_EndPoint = new_end_point;

        return dmPhysics::RESULT_OK;
    }

    dmPhysics::JointResult DestroyJoint(void* _world, void* _component, dmhash_t id)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component = (CollisionComponent*)_component;
        JointEntry* joint_entry = FindJointEntry(world, component, id);
        if (!joint_entry) {
            return dmPhysics::RESULT_ID_NOT_FOUND;
        }

        if (!joint_entry->m_Joint) {
            return dmPhysics::RESULT_NOT_CONNECTED;
        }

        DeleteJoint(world, joint_entry);

        // Remove joint entry from list on component instance
        if (component->m_Joints == joint_entry) {
            component->m_Joints = joint_entry->m_Next;
        } else {
            JointEntry* j = component->m_Joints;
            while (j) {
                if (j->m_Next == joint_entry) {
                    j->m_Next = joint_entry->m_Next;
                    break;
                }
                j = j->m_Next;
            }
        }

        delete joint_entry;

        return dmPhysics::RESULT_OK;
    }

    //Added by dotGears / TrungVu
    void SetMasterBody(void * comp, void * master_body)
    {
      CollisionComponent* component = (CollisionComponent*)comp;
      CollisionComponent* master = (CollisionComponent*)master_body;
      dmPhysics::SetMasterBody(component->m_Object2D, master->m_Object2D);
    }
    
    // Added by dotGears/TheTrung
    void SetVelocityLimit(void* comp, float minX, float minY, float maxX, float maxY)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetVelocityLimit(component->m_Object2D, minX, minY, maxX, maxY);
    }
    void CopyState(void* comp, uint16_t state)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::CopyState(component->m_Object2D, state);
    }
    void SetCopyRatio(void* comp, float ratio)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetCopyRatio(component->m_Object2D, ratio);
    }
    void SetCopyDisable(void* comp)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetCopyDisable(component->m_Object2D);
    }

    void SetControllable(void* comp, bool flag)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetControllable(component->m_Object2D, flag);
    }

    void SetSleepingAllowed(void* comp, bool flag)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetSleepingAllowed(component->m_Object2D, flag);
    }

    void SetBullet(void* comp, bool flag)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetBullet(component->m_Object2D, flag);
    }

    void SetDeltaValue(void* comp, float alphaX, float alphaY, float alphaZ)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetDeltaValue(component->m_Object2D, alphaX, alphaY, alphaZ);
    }
    void SetGravityScale(void* comp, float gravityScale)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetGravityScale(component->m_Object2D, gravityScale);
    }
    void SetAllowSleep(void* comp, bool allow_sleep)
    {
        CollisionComponent* component = (CollisionComponent*)comp;
        dmPhysics::SetAllowSleep(component->m_Object2D, allow_sleep);
    } 
    void SetWorld2DStepIteration(void* _world, int stepIteration, int velocityIteration, int positionIteration)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (world->m_3D)
        {
        }
        else
        {
            dmPhysics::SetWorld2DStepIteration(world->m_World2D, stepIteration, velocityIteration, positionIteration);
        }
    }

    // End of Passion

    dmPhysics::JointResult GetJointParams(void* _world, void* _component, dmhash_t id, dmPhysics::JointType& joint_type, dmPhysics::ConnectJointParams& joint_params)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component = (CollisionComponent*)_component;
        JointEntry* joint_entry = FindJointEntry(world, component, id);

        if (!joint_entry) {
            return dmPhysics::RESULT_ID_NOT_FOUND;
        }

        if (!joint_entry->m_Joint) {
            return dmPhysics::RESULT_NOT_CONNECTED;
        }

        joint_type = joint_entry->m_Type;
        bool r = GetJointParams2D(world->m_World2D, joint_entry->m_Joint, joint_entry->m_Type, joint_params);
        return (r ? dmPhysics::RESULT_OK : dmPhysics::RESULT_UNKNOWN_ERROR);
    }

    dmPhysics::JointResult GetJointType(void* _world, void* _component, dmhash_t id, dmPhysics::JointType& joint_type)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component = (CollisionComponent*)_component;
        JointEntry* joint_entry = FindJointEntry(world, component, id);

        if (!joint_entry) {
            return dmPhysics::RESULT_ID_NOT_FOUND;
        }

        if (!joint_entry->m_Joint) {
            return dmPhysics::RESULT_NOT_CONNECTED;
        }

        joint_type = joint_entry->m_Type;

        return dmPhysics::RESULT_OK;
    }

    dmPhysics::JointResult SetJointParams(void* _world, void* _component, dmhash_t id, const dmPhysics::ConnectJointParams& joint_params)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component = (CollisionComponent*)_component;
        JointEntry* joint_entry = FindJointEntry(world, component, id);

        if (!joint_entry) {
            return dmPhysics::RESULT_ID_NOT_FOUND;
        }

        if (!joint_entry->m_Joint) {
            return dmPhysics::RESULT_NOT_CONNECTED;
        }

        bool r = SetJointParams2D(world->m_World2D, joint_entry->m_Joint, joint_entry->m_Type, joint_params);
        return (r ? dmPhysics::RESULT_OK : dmPhysics::RESULT_UNKNOWN_ERROR);
    }

    static void DeleteJoint(CollisionWorld* world, JointEntry* joint_entry)
    {
        assert(joint_entry);

        DeleteJoint(world, joint_entry->m_Joint);
        joint_entry->m_Joint = 0x0;

        // Remove end point
        assert(joint_entry->m_EndPoint);
        JointEndPoint* end_point = joint_entry->m_EndPoint;
        CollisionComponent* owner_component = end_point->m_Owner;

        // Find and remove end point from previous component_b
        bool removed = false;
        JointEndPoint* end_point_prev = 0x0;
        JointEndPoint* end_point_next = owner_component->m_JointEndPoints;
        while (end_point_next) {
            if (end_point_next == end_point)
            {
                if (end_point_prev) {
                    end_point_prev->m_Next = end_point_next->m_Next;
                } else {
                    owner_component->m_JointEndPoints = end_point_next->m_Next;
                }

                removed = true;
                break;
            }
            end_point_prev = end_point_next;
            end_point_next = end_point_next->m_Next;
        }

        assert(removed);
        delete end_point;
    }

    static void DeleteJoint(CollisionWorld* world, dmPhysics::HJoint joint)
    {
        assert(joint);
        if (!world->m_3D)
        {
            dmPhysics::DeleteJoint2D(world->m_World2D, joint);
        }
    }

    dmPhysics::JointResult GetJointReactionForce(void* _world, void* _component, dmhash_t id, Vectormath::Aos::Vector3& force)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component = (CollisionComponent*)_component;
        JointEntry* joint_entry = FindJointEntry(world, component, id);

        if (!joint_entry) {
            return dmPhysics::RESULT_ID_NOT_FOUND;
        }

        if (!joint_entry->m_Joint) {
            return dmPhysics::RESULT_NOT_CONNECTED;
        }

        bool r = GetJointReactionForce2D(world->m_World2D, joint_entry->m_Joint, force, 1.0f / world->m_LastDT);
        return (r ? dmPhysics::RESULT_OK : dmPhysics::RESULT_UNKNOWN_ERROR);
    }

    dmPhysics::JointResult GetJointReactionTorque(void* _world, void* _component, dmhash_t id, float& torque)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (!IsJointsSupported(world)) {
            return dmPhysics::RESULT_NOT_SUPPORTED;
        }

        CollisionComponent* component = (CollisionComponent*)_component;
        JointEntry* joint_entry = FindJointEntry(world, component, id);

        if (!joint_entry) {
            return dmPhysics::RESULT_ID_NOT_FOUND;
        }

        if (!joint_entry->m_Joint) {
            return dmPhysics::RESULT_NOT_CONNECTED;
        }

        bool r = GetJointReactionTorque2D(world->m_World2D, joint_entry->m_Joint, torque, 1.0f / world->m_LastDT);
        return (r ? dmPhysics::RESULT_OK : dmPhysics::RESULT_UNKNOWN_ERROR);
    }

    void SetGravity(void* _world, const Vectormath::Aos::Vector3& gravity)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (world->m_3D)
        {
            dmPhysics::SetGravity3D(world->m_World3D, gravity);
        }
        else
        {
            dmPhysics::SetGravity2D(world->m_World2D, gravity);
        }
    }

    Vectormath::Aos::Vector3 GetGravity(void* _world)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        if (world->m_3D)
        {
            return dmPhysics::GetGravity3D(world->m_World3D);
        }
        else
        {
            return dmPhysics::GetGravity2D(world->m_World2D);
        }
    }

    dmhash_t CompCollisionObjectGetIdentifier(void* _component)
    {
        CollisionComponent* component = (CollisionComponent*)_component;
        return dmGameObject::GetIdentifier(component->m_Instance);
    }

    bool IsCollision2D(void* _world)
    {
        CollisionWorld* world = (CollisionWorld*)_world;
        return !world->m_3D;
    }

    void SetCollisionFlipH(void* _component, bool flip)
    {
        CollisionComponent* component = (CollisionComponent*)_component;
        if (component->m_FlippedX != flip)
            dmPhysics::FlipH2D(component->m_Object2D);
        component->m_FlippedX = flip;
    }

    void SetCollisionFlipV(void* _component, bool flip)
    {
        CollisionComponent* component = (CollisionComponent*)_component;
        if (component->m_FlippedY != flip)
            dmPhysics::FlipV2D(component->m_Object2D);
        component->m_FlippedY = flip;
    }

}
