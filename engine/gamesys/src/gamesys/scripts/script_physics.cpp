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

#include <float.h>
#include <stdio.h>
#include <assert.h>

#include <dlib/hash.h>
#include <dlib/log.h>
#include <dlib/math.h>

#include "gamesys.h"
#include "gamesys_ddf.h"
#include "physics_ddf.h"
#include "../gamesys_private.h"

#include "../../script/src/script.h"
#include "components/comp_collision_object.h"

#include "script_physics.h"
#include <physics/physics.h>

extern "C"
{
#include <lua/lauxlib.h>
#include <lua/lualib.h>
}

static const char PHYSICS_CONTEXT_NAME[] = "__PhysicsContext";
static uint32_t PHYSICS_CONTEXT_HASH = 0;

namespace dmGameSystem
{
    /*# Collision object physics API documentation
     *
     * Functions and messages for collision object physics interaction
     * with other objects (collisions and ray-casting) and control of
     * physical behaviors.
     *
     * @document
     * @name Collision object
     * @namespace physics
     */

    /*# Physics.COPY_POSITION_X
     *
     * @name physics.COPY_POSITION_X
     * @variable
     */

    /*# Physics.COPY_POSITION_Y
     *
     * @name physics.COPY_POSITION_Y
     * @variable
     */

    /*# Physics.COPY_ROTATION_Z
     *
     * @name physics.COPY_ROTATION_Z
     * @variable
     */

    /*# Physics.COPY_LINEAR_VEC
     *
     * @name physics.COPY_LINEAR_VEC
     * @variable
     */

    /*# Physics.COPY_ANGULAR_VEC
     *
     * @name physics.COPY_ANGULAR_VEC
     * @variable
     */

    /*# spring joint type
     *
     * The following properties are available when connecting a joint of `JOINT_TYPE_SPRING` type:
     * @param length [type:number] The natural length between the anchor points.
     * @param frequency [type:number] The mass-spring-damper frequency in Hertz. A value of 0 disables softness.
     * @param damping [type:number] The damping ratio. 0 = no damping, 1 = critical damping.
     *
     * @name physics.JOINT_TYPE_SPRING
     * @variable
     */

    /*# fixed joint type
     *
     * The following properties are available when connecting a joint of `JOINT_TYPE_FIXED` type:
     * @param max_length [type:number] The maximum length of the rope.
     *
     * @name physics.JOINT_TYPE_FIXED
     * @variable
     */

    /*# hinge joint type
     *
     * The following properties are available when connecting a joint of `JOINT_TYPE_HINGE` type:
     * @param reference_angle [type:number] The bodyB angle minus bodyA angle in the reference state (radians).
     * @param lower_angle [type:number] The lower angle for the joint limit (radians).
     * @param upper_angle [type:number] The upper angle for the joint limit (radians).
     * @param max_motor_torque [type:number] The maximum motor torque used to achieve the desired motor speed. Usually in N-m.
     * @param motor_speed [type:number] The desired motor speed. Usually in radians per second.
     * @param enable_limit [type:boolean] A flag to enable joint limits.
     * @param enable_motor [type:boolean] A flag to enable the joint motor.
     * @param joint_angle [type:number] [mark:READ ONLY]Current joint angle in radians.
     * (Read only field, available from `physics.get_joint_properties()`)
     * @param joint_speed [type:number] [mark:READ ONLY]Current joint angle speed in radians per second.
     * (Read only field, available from `physics.get_joint_properties()`)
     *
     * @name physics.JOINT_TYPE_HINGE
     * @variable
     */

    /*# slider joint type
     *
     * The following properties are available when connecting a joint of `JOINT_TYPE_SLIDER` type:
     * @param local_axis_a [type:vector3] The local translation unit axis in bodyA.
     * @param reference_angle [type:number] The constrained angle between the bodies: bodyB_angle - bodyA_angle.
     * @param enable_limit [type:boolean] Enable/disable the joint limit.
     * @param lower_translation [type:number] The lower translation limit, usually in meters.
     * @param upper_translation [type:number] The upper translation limit, usually in meters.
     * @param enable_motor [type:boolean] Enable/disable the joint motor.
     * @param max_motor_force [type:number] The maximum motor torque, usually in N-m.
     * @param motor_speed [type:number] The desired motor speed in radians per second.
     * @param joint_translation [type:number] [mark:READ ONLY]Current joint translation, usually in meters.
     * (Read only field, available from `physics.get_joint_properties()`)
     * @param joint_speed [type:number] [mark:READ ONLY]Current joint translation speed, usually in meters per second.
     * (Read only field, available from `physics.get_joint_properties()`)
     *
     * @name physics.JOINT_TYPE_SLIDER
     * @variable
     */

    struct PhysicsScriptContext
    {
        dmMessage::HSocket m_Socket;
        uint32_t m_ComponentIndex;
    };

    /*# [type:number] collision object mass
     *
     * [mark:READ ONLY] Returns the defined physical mass of the collision object component as a number.
     *
     * @name mass
     * @property
     *
     * @examples
     *
     * How to query a collision object component's mass:
     *
     * ```lua
     * -- get mass from collision object component "boulder"
     * local mass = go.get("#boulder", "mass")
     * -- do something useful
     * assert(mass > 1)
     * ```
     */

    /*# [type:vector3] collision object linear velocity
     *
     * [mark:READ ONLY] Returns the current linear velocity of the collision object component as a vector3.
     * The velocity is measured in units/s (pixels/s).
     *
     * @name linear_velocity
     * @replaces request_velocity and velocity_response
     * @property
     *
     * @examples
     *
     * How to query a collision object component's linear velocity:
     *
     * ```lua
     * -- get linear velocity from collision object "collisionobject" in gameobject "ship"
     * local source = "ship#collisionobject"
     * local velocity = go.get(source, "linear_velocity")
     * -- apply the velocity on target game object "boulder"'s collision object as a force
     * local target = "boulder#collisionobject"
     * local pos = go.get_position(target)
     * msg.post(target, "apply_force", { force = velocity, position = pos })
     * ```
     */

    /*# [type:vector3] collision object angular velocity
     *
     * [mark:READ ONLY] Returns the current angular velocity of the collision object component as a [type:vector3].
     * The velocity is measured as a rotation around the vector with a speed equivalent to the vector length
     * in radians/s.
     *
     * @name angular_velocity
     * @replaces request_velocity and velocity_response
     * @property
     * @examples
     *
     * How to query a collision object component's angular velocity:
     *
     * ```lua
     * -- get angular velocity from collision object "collisionobject" in gameobject "boulder"
     * -- this is a 2d game so rotation around z is the only one available.
     * local velocity = go.get("boulder#collisionobject", "angular_velocity.z")
     * -- do something interesting
     * if velocity < 0 then
     *     -- clockwise rotation
     *     ...
     * else
     *     -- counter clockwise rotation
     *     ...
     * end
     * ```
     */

    /*# [type:number] collision object linear damping
     *
     * The linear damping value for the collision object. Setting this value alters the damping of
     * linear motion of the object. Valid values are between 0 (no damping) and 1 (full damping).
     *
     * @name linear_damping
     * @property
     * @examples
     *
     * How to increase a collision object component's linear damping:
     *
     * ```lua
     * -- get linear damping from collision object "collisionobject" in gameobject "floater"
     * local target = "floater#collisionobject"
     * local damping = go.get(target, "linear_damping")
     * -- increase it by 10% if it's below 0.9
     * if damping <= 0.9 then
     *     go.set(target, "linear_damping", damping * 1.1)
     * end
     * ```
     */

    /*# [type:number] collision object angular damping
     *
     * The angular damping value for the collision object. Setting this value alters the damping of
     * angular motion of the object (rotation). Valid values are between 0 (no damping) and 1 (full damping).
     *
     * @name angular_damping
     * @property
     * @examples
     *
     * How to decrease a collision object component's angular damping:
     *
     * ```lua
     * -- get angular damping from collision object "collisionobject" in gameobject "floater"
     * local target = "floater#collisionobject"
     * local damping = go.get(target, "angular_damping")
     * -- decrease it by 10%
     * go.set(target, "angular_damping", damping * 0.9)
     * ```
     */

    /*# requests a ray cast to be performed
     *
     * Ray casts are used to test for intersections against collision objects in the physics world.
     * Collision objects of types kinematic, dynamic and static are tested against. Trigger objects
     * do not intersect with ray casts.
     * Which collision objects to hit is filtered by their collision groups and can be configured
     * through `groups`.
     * The actual ray cast will be performed during the physics-update.
     *
     * - If an object is hit, the result will be reported via a `ray_cast_response` message.
     * - If there is no object hit, the result will be reported via a `ray_cast_missed` message.
     *
     * @name physics.raycast_async
     * @param from [type:vector3] the world position of the start of the ray
     * @param to [type:vector3] the world position of the end of the ray
     * @param groups [type:table] a lua table containing the hashed groups for which to test collisions against
     * @param [request_id] [type:number] a number between [0,-255]. It will be sent back in the response for identification, 0 by default
     * @examples
     *
     * How to perform a ray cast asynchronously:
     *
     * ```lua
     * function init(self)
     *     self.my_groups = {hash("my_group1"), hash("my_group2")}
     * end
     *
     * function update(self, dt)
     *     -- request ray cast
     *     physics.raycast_async(my_start, my_end, self.my_groups)
     * end
     *
     * function on_message(self, message_id, message, sender)
     *     -- check for the response
     *     if message_id == hash("ray_cast_response") then
     *         -- act on the hit
     *     elseif message_id == hash("ray_cast_missed") then
     *         -- act on the miss
     *     end
     * end
     * ```
     */
    int Physics_RayCastAsync(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);
        int top = lua_gettop(L);

        dmMessage::URL sender;
        if (!dmScript::GetURL(L, &sender)) {
            return luaL_error(L, "could not find a requesting instance for physics.raycast_async");
        }

        dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
        PhysicsScriptContext* context = (PhysicsScriptContext*)lua_touserdata(L, -1);
        lua_pop(L, 1);

        dmGameObject::HInstance sender_instance = CheckGoInstance(L);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(sender_instance);
        void* world = dmGameObject::GetWorld(collection, context->m_ComponentIndex);

        Vectormath::Aos::Point3 from( *dmScript::CheckVector3(L, 1) );
        Vectormath::Aos::Point3 to( *dmScript::CheckVector3(L, 2) );

        uint32_t mask = 0;
        luaL_checktype(L, 3, LUA_TTABLE);
        lua_pushnil(L);
        while (lua_next(L, 3) != 0)
        {
            mask |= CompCollisionGetGroupBitIndex(world, dmScript::CheckHash(L, -1));
            lua_pop(L, 1);
        }

        int request_id = 0;
        if (top > 3)
        {
            request_id = luaL_checkinteger(L, 4);
            if (request_id < 0 || request_id > 255)
            {
                return luaL_error(L, "request_id must be between 0-255");
            }
        }

        dmPhysicsDDF::RequestRayCast request;
        request.m_From = from;
        request.m_To = to;
        request.m_Mask = mask;
        request.m_RequestId = request_id;

        dmMessage::URL receiver;
        dmMessage::ResetURL(receiver);
        receiver.m_Socket = context->m_Socket;
        dmMessage::Post(&sender, &receiver, dmPhysicsDDF::RequestRayCast::m_DDFDescriptor->m_NameHash, (uintptr_t)sender_instance, (uintptr_t)dmPhysicsDDF::RequestRayCast::m_DDFDescriptor, &request, sizeof(dmPhysicsDDF::RequestRayCast), 0);
        return 0;
    }

    static void PushRayCastResponse(lua_State* L, void* world, const dmPhysics::RayCastResponse& response)
    {
        lua_pushnumber(L, response.m_Fraction);
        lua_setfield(L, -2, "fraction");
        dmScript::PushVector3(L, Vectormath::Aos::Vector3(response.m_Position));
        lua_setfield(L, -2, "position");
        dmScript::PushVector3(L, response.m_Normal);
        lua_setfield(L, -2, "normal");

        dmhash_t group = dmGameSystem::GetLSBGroupHash(world, response.m_CollisionObjectGroup);
        dmScript::PushHash(L, group);
        lua_setfield(L, -2, "group");

        dmhash_t id = dmGameSystem::CompCollisionObjectGetIdentifier(response.m_CollisionObjectUserData);
        dmScript::PushHash(L, id);
        lua_setfield(L, -2, "id");
    }

    /*# requests a ray cast to be performed
     *
     * Ray casts are used to test for intersections against collision objects in the physics world.
     * Collision objects of types kinematic, dynamic and static are tested against. Trigger objects
     * do not intersect with ray casts.
     * Which collision objects to hit is filtered by their collision groups and can be configured
     * through `groups`.
     *
     * @name physics.raycast
     * @param from [type:vector3] the world position of the start of the ray
     * @param to [type:vector3] the world position of the end of the ray
     * @param groups [type:table] a lua table containing the hashed groups for which to test collisions against
     * @param options [type:table] a lua table containing options for the raycast.
     *
     * `all`
     * : [type:boolean] Set to `true` to return all ray cast hits. If `false`, it will only return the closest hit.
     *
     * @return result [type:table] It returns a list. If missed it returns nil. See `ray_cast_response` for details on the returned values.
     * @examples
     *
     * How to perform a ray cast synchronously:
     *
     * ```lua
     * function init(self)
     *     self.groups = {hash("world"), hash("enemy")}
     * end
     *
     * function update(self, dt)
     *     -- request ray cast
     *     local result = physics.raycast(from, to, self.groups, {all=true})
     *     if result ~= nil then
     *         -- act on the hit (see 'ray_cast_response')
     *         for _,result in ipairs(results) do
     *             handle_result(result)
     *         end
     *     end
     * end
     * ```
     */
    int Physics_RayCast(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 1);

        dmMessage::URL sender;
        if (!dmScript::GetURL(L, &sender)) {
            return luaL_error(L, "could not find a requesting instance for physics.raycast");
        }

        dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
        PhysicsScriptContext* context = (PhysicsScriptContext*)lua_touserdata(L, -1);
        lua_pop(L, 1);

        dmGameObject::HInstance sender_instance = CheckGoInstance(L);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(sender_instance);
        void* world = dmGameObject::GetWorld(collection, context->m_ComponentIndex);

        Vectormath::Aos::Point3 from( *dmScript::CheckVector3(L, 1) );
        Vectormath::Aos::Point3 to( *dmScript::CheckVector3(L, 2) );

        uint32_t mask = 0;
        luaL_checktype(L, 3, LUA_TTABLE);
        lua_pushnil(L);
        while (lua_next(L, 3) != 0)
        {
            mask |= CompCollisionGetGroupBitIndex(world, dmScript::CheckHash(L, -1));
            lua_pop(L, 1);
        }

        bool list_format = false;
        bool return_all_results = false;
        if (lua_istable(L, 4))
        {
            lua_pushvalue(L, 4);

            lua_getfield(L, -1, "all");
            return_all_results = lua_isnil(L, -1) ? false : lua_toboolean(L, -1);
            lua_pop(L, 1);

            lua_pop(L, 1);

            list_format = true;
        }

        dmArray<dmPhysics::RayCastResponse> hits;
        hits.SetCapacity(32);

        dmPhysics::RayCastRequest request;
        request.m_From = from;
        request.m_To = to;
        request.m_Mask = mask;
        request.m_ReturnAllResults = return_all_results ? 1 : 0;

        dmGameSystem::RayCast(world, request, hits);

        if (hits.Empty())
        {
            lua_pushnil(L);
            return 1;
        }

        uint32_t count = hits.Size();

        if (!return_all_results)
            count = 1;

        lua_newtable(L);

        for (uint32_t i = 0; i < count; ++i)
        {
            if(list_format)
            {
                lua_newtable(L);
            }

            PushRayCastResponse(L, world, hits[i]);

            if (list_format)
            {
                lua_rawseti(L, -2, i+1);
            }
        }

        return 1;
    }

    // Matches JointResult in physics.h
    static const char* PhysicsResultString[] = {
        "result ok",
        "not supported",
        "a joint with that id already exist",
        "joint id not found",
        "joint not connected",
        "unknown error",
    };

    // Helper to get collisionobject component and world.
    static void GetCollisionObject(lua_State* L, int indx, dmGameObject::HCollection collection, void** comp, void** comp_world)
    {
        dmMessage::URL receiver;
        dmGameObject::GetComponentUserDataFromLua(L, indx, collection, COLLISION_OBJECT_EXT, (uintptr_t*)comp, &receiver, comp_world);
    }

    static int GetTableField(lua_State* L, int table_index, const char* table_field, int expected_type)
    {
        lua_getfield(L, table_index, table_field);
        int type = lua_type(L, -1);

        // return if the field was not found
        if (type == LUA_TNIL || type == LUA_TNONE) {
            lua_pop(L, 1);
            return 0;
        } else if (type != expected_type) {
            return luaL_error(L, "joint property table field %s must be of %s type.", table_field, lua_typename(L, expected_type));
        }

        return 1;
    }

    static void UnpackFloatParam(lua_State* L, int table_index, const char* table_field, float& float_out)
    {
        if (GetTableField(L, table_index, table_field, LUA_TNUMBER))
        {
            float_out = lua_tonumber(L, -1);
            lua_pop(L, 1);
        }
    }

    static void UnpackVec3Param(lua_State* L, int table_index, const char* table_field, float float_out[3])
    {
        if (GetTableField(L, table_index, table_field, LUA_TUSERDATA))
        {
            Vectormath::Aos::Vector3* v3 = dmScript::ToVector3(L, -1);
            if (!v3) {
                lua_pop(L, 1);
                luaL_error(L, "joint property table field %s must be of vmath.vector3 type.", table_field);
                return;
            }

            float_out[0] = v3->getX();
            float_out[1] = v3->getY();
            float_out[2] = v3->getZ();
            lua_pop(L, 1);
        }
    }

    static void UnpackBoolParam(lua_State* L, int table_index, const char* table_field, bool& bool_out)
    {
        if (GetTableField(L, table_index, table_field, LUA_TBOOLEAN))
        {
            bool_out = lua_toboolean(L, -1);
            lua_pop(L, 1);
        }
    }

    static void UnpackConnectJointParams(lua_State* L, dmPhysics::JointType type, int table_index, dmPhysics::ConnectJointParams& params)
    {
        DM_LUA_STACK_CHECK(L, 0);

        // Fill with default values
        params = dmPhysics::ConnectJointParams(type);

        int table_index_type = lua_type(L, table_index);
        if (table_index_type == LUA_TNIL || table_index_type == LUA_TNONE) {
            // Early exit if table was nil (just returns default values from above).
            return;
        } else if (table_index_type != LUA_TTABLE) {
            DM_LUA_ERROR("argument %d to physics.connect_joint must be either nil or table.", table_index)
            return;
        }

        // Common fields for all joints:
        UnpackBoolParam(L, table_index, "collide_connected", params.m_CollideConnected);

        switch (type)
        {
            case dmPhysics::JOINT_TYPE_SPRING:
                UnpackFloatParam(L, table_index, "length", params.m_SpringJointParams.m_Length);
                UnpackFloatParam(L, table_index, "frequency", params.m_SpringJointParams.m_FrequencyHz);
                UnpackFloatParam(L, table_index, "damping", params.m_SpringJointParams.m_DampingRatio);
                break;

            case dmPhysics::JOINT_TYPE_FIXED:
                UnpackFloatParam(L, table_index, "max_length", params.m_FixedJointParams.m_MaxLength);
                break;

            case dmPhysics::JOINT_TYPE_HINGE:
                UnpackFloatParam(L, table_index, "reference_angle", params.m_HingeJointParams.m_ReferenceAngle);
                UnpackFloatParam(L, table_index, "lower_angle", params.m_HingeJointParams.m_LowerAngle);
                UnpackFloatParam(L, table_index, "upper_angle", params.m_HingeJointParams.m_UpperAngle);
                UnpackFloatParam(L, table_index, "max_motor_torque", params.m_HingeJointParams.m_MaxMotorTorque);
                UnpackFloatParam(L, table_index, "motor_speed", params.m_HingeJointParams.m_MotorSpeed);
                UnpackBoolParam(L, table_index, "enable_limit", params.m_HingeJointParams.m_EnableLimit);
                UnpackBoolParam(L, table_index, "enable_motor", params.m_HingeJointParams.m_EnableMotor);

                // We need to catch this as soon as possible, if it trickles down to Box2D it could cause an assert.
                // The default values are both zero so they will not cause this error.
                // (Same check below in JOINT_TYPE_SLIDER.)
                if (params.m_HingeJointParams.m_LowerAngle > params.m_HingeJointParams.m_UpperAngle) {
                    luaL_error(L, "property field 'lower_angle' must be lower or equal to 'upper_angle'");
                    return;
                }
                break;

            case dmPhysics::JOINT_TYPE_SLIDER:
                UnpackVec3Param(L, table_index, "local_axis_a", params.m_SliderJointParams.m_LocalAxisA);
                UnpackFloatParam(L, table_index, "reference_angle", params.m_SliderJointParams.m_ReferenceAngle);
                UnpackBoolParam(L, table_index, "enable_limit", params.m_SliderJointParams.m_EnableLimit);
                UnpackFloatParam(L, table_index, "lower_translation", params.m_SliderJointParams.m_LowerTranslation);
                UnpackFloatParam(L, table_index, "upper_translation", params.m_SliderJointParams.m_UpperTranslation);
                UnpackBoolParam(L, table_index, "enable_motor", params.m_SliderJointParams.m_EnableMotor);
                UnpackFloatParam(L, table_index, "max_motor_force", params.m_SliderJointParams.m_MaxMotorForce);
                UnpackFloatParam(L, table_index, "motor_speed", params.m_SliderJointParams.m_MotorSpeed);

                if (params.m_SliderJointParams.m_LowerTranslation > params.m_SliderJointParams.m_UpperTranslation) {
                    luaL_error(L, "property field 'lower_translation' must be lower or equal to 'upper_translation'");
                    return;
                }
                break;

            default:
                DM_LUA_ERROR("property table not implemented for joint type %d", type)
                return;
        }

        return;
    }

    /*# create a physics joint
     *
     * Create a physics joint between two collision object components.
     *
     * Note: Currently only supported in 2D physics.
     *
     * @name physics.create_joint
     * @param joint_type [type:number] the joint type
     * @param collisionobject_a [type:string|hash|url] first collision object
     * @param joint_id [type:string|hash] id of the joint
     * @param position_a [type:vector3] local position where to attach the joint on the first collision object
     * @param collisionobject_b [type:string|hash|url] second collision object
     * @param position_b [type:vector3] local position where to attach the joint on the second collision object
     * @param [properties] [type:table] optional joint specific properties table
     *
     * See each joint type for possible properties field. The one field that is accepted for all joint types is:
     * - [type:boolean] `collide_connected`: Set this flag to true if the attached bodies should collide.
     *
     */
    static int Physics_CreateJoint(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmPhysics::JointType type = (dmPhysics::JointType)luaL_checkinteger(L, 1);
        if (type >= dmPhysics::JOINT_TYPE_COUNT)
        {
            return DM_LUA_ERROR("unknown joint type: %d", type);
        }

        dmhash_t joint_id = dmScript::CheckHashOrString(L, 3);
        Vectormath::Aos::Point3 pos_a = Vectormath::Aos::Point3(*dmScript::CheckVector3(L, 4));
        Vectormath::Aos::Point3 pos_b = Vectormath::Aos::Point3(*dmScript::CheckVector3(L, 6));

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));

        void* comp_a = 0x0;
        void* comp_world_a = 0x0;
        GetCollisionObject(L, 2, collection, &comp_a, &comp_world_a);
        void* comp_b = 0x0;
        void* comp_world_b = 0x0;
        GetCollisionObject(L, 5, collection, &comp_b, &comp_world_b);

        if (comp_world_a != comp_world_b) {
            return DM_LUA_ERROR("joints can only be connected to collision objects within the same physics world");
        }

        dmPhysics::ConnectJointParams params(type);
        UnpackConnectJointParams(L, type, 7, params);
        dmPhysics::JointResult r = dmGameSystem::CreateJoint(comp_world_a, comp_a, joint_id, pos_a, comp_b, pos_b, type, params);
        if (r != dmPhysics::RESULT_OK) {
            return DM_LUA_ERROR("could not create joint: %s (%d)", PhysicsResultString[r], r);
        }

        return 0;

    }

    /*# destroy a physics joint
     *
     * Destroy an already physics joint. The joint has to be created before a
     * destroy can be issued.
     *
     * Note: Currently only supported in 2D physics.
     *
     * @name physics.destroy_joint
     * @param collisionobject [type:string|hash|url] collision object where the joint exist
     * @param joint_id [type:string|hash] id of the joint
     *
     */
    static int Physics_DestroyJoint(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmhash_t joint_id = dmScript::CheckHashOrString(L, 2);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));

        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        // Unpack type specific joint connection paramaters
        dmPhysics::JointResult r = dmGameSystem::DestroyJoint(comp_world, comp, joint_id);
        if (r != dmPhysics::RESULT_OK) {
            return DM_LUA_ERROR("could not disconnect joint: %s (%d)", PhysicsResultString[r], r);
        }

        return 0;
    }

    /*# get properties for a joint
     *
     * Get a table for properties for a connected joint. The joint has to be created before
     * properties can be retrieved.
     *
     * Note: Currently only supported in 2D physics.
     *
     * @name physics.get_joint_properties
     * @param collisionobject [type:string|hash|url] collision object where the joint exist
     * @param joint_id [type:string|hash] id of the joint
     * @return [type:table] properties table. See the joint types for what fields are available, the only field available for all types is:
     *
     * - [type:boolean] `collide_connected`: Set this flag to true if the attached bodies should collide.
     *
     */
    static int Physics_GetJointProperties(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 1);

        dmhash_t joint_id = dmScript::CheckHashOrString(L, 2);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));

        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        dmPhysics::JointType joint_type;
        dmPhysics::ConnectJointParams joint_params;
        dmPhysics::JointResult r = GetJointParams(comp_world, comp, joint_id, joint_type, joint_params);
        if (r != dmPhysics::RESULT_OK)
        {
            return DM_LUA_ERROR("unable to get joint properties for %s: %s (%d)", dmHashReverseSafe64(joint_id), PhysicsResultString[r], r);
        }

        lua_newtable(L);
        lua_pushboolean(L, joint_params.m_CollideConnected); lua_setfield(L, -2, "collide_connected");

        switch (joint_type)
        {
            case dmPhysics::JOINT_TYPE_SPRING:
                {
                    lua_pushnumber(L, joint_params.m_SpringJointParams.m_Length); lua_setfield(L, -2, "length");
                    lua_pushnumber(L, joint_params.m_SpringJointParams.m_FrequencyHz); lua_setfield(L, -2, "frequency");
                    lua_pushnumber(L, joint_params.m_SpringJointParams.m_DampingRatio); lua_setfield(L, -2, "damping");
                }
                break;
            case dmPhysics::JOINT_TYPE_FIXED:
                {
                    lua_pushnumber(L, joint_params.m_FixedJointParams.m_MaxLength); lua_setfield(L, -2, "max_length");
                }
                break;
            case dmPhysics::JOINT_TYPE_HINGE:
                {
                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_ReferenceAngle); lua_setfield(L, -2, "reference_angle");
                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_LowerAngle); lua_setfield(L, -2, "lower_angle");
                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_UpperAngle); lua_setfield(L, -2, "upper_angle");
                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_MaxMotorTorque); lua_setfield(L, -2, "max_motor_torque");
                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_MotorSpeed); lua_setfield(L, -2, "motor_speed");
                    lua_pushboolean(L, joint_params.m_HingeJointParams.m_EnableLimit); lua_setfield(L, -2, "enable_limit");
                    lua_pushboolean(L, joint_params.m_HingeJointParams.m_EnableMotor); lua_setfield(L, -2, "enable_motor");

                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_JointAngle); lua_setfield(L, -2, "joint_angle");
                    lua_pushnumber(L, joint_params.m_HingeJointParams.m_JointSpeed); lua_setfield(L, -2, "joint_speed");

                }
                break;
            case dmPhysics::JOINT_TYPE_SLIDER:
                {
                    Vectormath::Aos::Vector3 v(joint_params.m_SliderJointParams.m_LocalAxisA[0], joint_params.m_SliderJointParams.m_LocalAxisA[1], joint_params.m_SliderJointParams.m_LocalAxisA[2]);
                    dmScript::PushVector3(L, v);
                    lua_setfield(L, -2, "local_axis_a");
                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_ReferenceAngle); lua_setfield(L, -2, "reference_angle");
                    lua_pushboolean(L, joint_params.m_SliderJointParams.m_EnableLimit); lua_setfield(L, -2, "enable_limit");
                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_LowerTranslation); lua_setfield(L, -2, "lower_translation");
                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_UpperTranslation); lua_setfield(L, -2, "upper_translation");
                    lua_pushboolean(L, joint_params.m_SliderJointParams.m_EnableMotor); lua_setfield(L, -2, "enable_motor");
                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_MaxMotorForce); lua_setfield(L, -2, "max_motor_force");
                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_MotorSpeed); lua_setfield(L, -2, "motor_speed");

                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_JointTranslation); lua_setfield(L, -2, "joint_translation");
                    lua_pushnumber(L, joint_params.m_SliderJointParams.m_JointSpeed); lua_setfield(L, -2, "joint_speed");
                }
                break;
            default:
                return false;
        }

        return 1;
    }

    /*# set properties for a joint
     *
     * Updates the properties for an already connected joint. The joint has to be created before
     * properties can be changed.
     *
     * Note: Currently only supported in 2D physics.
     *
     * @name physics.set_joint_properties
     * @param collisionobject [type:string|hash|url] collision object where the joint exist
     * @param joint_id [type:string|hash] id of the joint
     * @param properties [type:table] joint specific properties table
     *
     * Note: The `collide_connected` field cannot be updated/changed after a connection has been made.
     *
     */
    static int Physics_SetJointProperties(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmhash_t joint_id = dmScript::CheckHashOrString(L, 2);
        dmGameObject::HInstance instance = CheckGoInstance(L);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(instance);

        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        dmPhysics::JointType joint_type;
        dmPhysics::JointResult r = GetJointType(comp_world, comp, joint_id, joint_type);
        if (r != dmPhysics::RESULT_OK) {
            return DM_LUA_ERROR("unable to set joint properties, could not get joint type: %s (%d)", PhysicsResultString[r], r);
        }

        dmPhysics::ConnectJointParams joint_params(joint_type);
        UnpackConnectJointParams(L, joint_type, 3, joint_params);

        r = SetJointParams(comp_world, comp, joint_id, joint_params);
        if (r != dmPhysics::RESULT_OK) {
            return DM_LUA_ERROR("unable to set joint properties: %s (%d)", PhysicsResultString[r], r);
        }

        return 0;
    }

    /*# get the reaction force for a joint
     *
     * Get the reaction force for a joint. The joint has to be created before
     * the reaction force can be calculated.
     *
     * Note: Currently only supported in 2D physics.
     *
     * @name physics.get_joint_reaction_force
     * @param collisionobject [type:string|hash|url] collision object where the joint exist
     * @param joint_id [type:string|hash] id of the joint
     * @return force [type:vector3] reaction force for the joint
     *
     */
    static int Physics_GetJointReactionForce(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 1);

        dmhash_t joint_id = dmScript::CheckHashOrString(L, 2);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));

        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        Vectormath::Aos::Vector3 reaction_force(0.0f);
        dmPhysics::JointResult r = GetJointReactionForce(comp_world, comp, joint_id, reaction_force);
        if (r != dmPhysics::RESULT_OK)
        {
            return DM_LUA_ERROR("unable to get joint reaction force for %s: %s (%d)", dmHashReverseSafe64(joint_id), PhysicsResultString[r], r);
        }

        dmScript::PushVector3(L, reaction_force);

        return 1;
    }

    /*# get the reaction torque for a joint
     *
     * Get the reaction torque for a joint. The joint has to be created before
     * the reaction torque can be calculated.
     *
     * Note: Currently only supported in 2D physics.
     *
     * @name physics.get_joint_reaction_torque
     * @param collisionobject [type:string|hash|url] collision object where the joint exist
     * @param joint_id [type:string|hash] id of the joint
     * @return torque [type:float] the reaction torque on bodyB in N*m.
     *
     */
    static int Physics_GetJointReactionTorque(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 1);

        dmhash_t joint_id = dmScript::CheckHashOrString(L, 2);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));

        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        float reaction_torque = 0.0f;
        dmPhysics::JointResult r = GetJointReactionTorque(comp_world, comp, joint_id, reaction_torque);
        if (r != dmPhysics::RESULT_OK)
        {
            return DM_LUA_ERROR("unable to get joint reaction torque for %s: %s (%d)", dmHashReverseSafe64(joint_id), PhysicsResultString[r], r);
        }

        lua_pushnumber(L, reaction_torque);

        return 1;
    }

    /*# set the gravity for collection
     *
     * Set the gravity in runtime. The gravity change is not global, it will only affect
     * the collection that the function is called from.
     *
     * Note: For 2D physics the z component of the gravity vector will be ignored.
     *
     * @name physics.set_gravity
     * @param gravity [type:vector3] the new gravity vector
     * @examples
     *
     * ```lua
     * function init(self)
     *     -- Set "upside down" gravity for this collection.
     *     physics.set_gravity(vmath.vector3(0, 10.0, 0))
     * end
     * ```
     */
    static int Physics_SetGravity(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmMessage::URL sender;
        if (!dmScript::GetURL(L, &sender)) {
            return DM_LUA_ERROR("could not find a requesting instance for physics.set_gravity");
        }

        dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
        PhysicsScriptContext* context = (PhysicsScriptContext*)lua_touserdata(L, -1);
        lua_pop(L, 1);

        dmGameObject::HInstance sender_instance = CheckGoInstance(L);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(sender_instance);
        void* world = dmGameObject::GetWorld(collection, context->m_ComponentIndex);

        Vectormath::Aos::Vector3 new_gravity( *dmScript::CheckVector3(L, 1) );

        dmGameSystem::SetGravity(world, new_gravity);

        return 0;
    }

    /*# get the gravity for collection
     *
     * Get the gravity in runtime. The gravity returned is not global, it will return
     * the gravity for the collection that the function is called from.
     *
     * Note: For 2D physics the z component will always be zero.
     *
     * @name physics.get_gravity
     * @return [type:vector3] gravity vector of collection
     * @examples
     *
     * ```lua
     * function init(self)
     *     local gravity = physics.get_gravity()
     *     -- Inverse gravity!
     *     gravity = -gravity
     *     physics.set_gravity(gravity)
     * end
     * ```
     */
    static int Physics_GetGravity(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 1);

        dmMessage::URL sender;
        if (!dmScript::GetURL(L, &sender)) {
            return DM_LUA_ERROR("could not find a requesting instance for physics.get_gravity");
        }

        dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
        PhysicsScriptContext* context = (PhysicsScriptContext*)lua_touserdata(L, -1);
        lua_pop(L, 1);

        dmGameObject::HInstance sender_instance = CheckGoInstance(L);
        dmGameObject::HCollection collection = dmGameObject::GetCollection(sender_instance);
        void* world = dmGameObject::GetWorld(collection, context->m_ComponentIndex);

        Vectormath::Aos::Vector3 gravity = dmGameSystem::GetGravity(world);
        dmScript::PushVector3(L, gravity);

        return 1;
    }

    /*# set the gravity scale for collection
     *
     * Set the gravity in runtime. The gravity change is not global, it will only affect
     * the collection that the function is called from.
     *
     * Note: For 2D physics the z component of the gravity vector will be ignored.
     *
     * @name physics.set_gravity_scale
     * @param body [type:string|hash|url] collision-object-id
     * @param gravity [type:vector3] the new gravity vector
     * @examples
     *
     * ```lua
     * function init(self)
     *     -- Set "upside down" gravity scale for this collection.
     *     physics.set_gravity_scale("#body", 1.5)
     * end
     * ```
     */
    static int Physics_SetGravityScale(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }
        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        float gravityScale = lua_tonumber(L, 2);

        dmGameSystem::SetGravityScale(comp, gravityScale);

        return 0;
    }

    /*# Set sleep allow for collision object.
     * Added by dotGears/TrungA
     *
     * @name physics.set_allow_sleep
     * @param  collisionobject [type:string|hash|url] target body.
     * @param  flag [type:boolean] mark a body to allow sleeping or not.
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_allow_sleep("#body", true)
     * end
     * ```
     */
    static int Physics_SetAllowSleep(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }
        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        bool allow_sleep = lua_tonumber(L, 2);

        dmGameSystem::SetAllowSleep(comp, allow_sleep);

        return 0;
    }

    /*# set physics step per frame.
     * Set the amount of steps for physics 2D to update inside Step() function.
     * Added by dotGears/TrungB.
     *
     * , velocityIteration, positionIteration
     * @name physics.set_step_per_frame
     * @param  stepIteration [type:integer] steps per frame that Physics2D will update
     * @param  velocityIteration [type:integer] iteration of velocity per step
     * @param  positionIteration [type:integer] iteration of position per step
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     local gravity = physics.set_step_per_frame(8,16,8)
     * end
     * ```
     */
    static int
    Physics_SetStepPerFrame(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmMessage::URL sender;
        if (!dmScript::GetURL(L, &sender))
        {
            return DM_LUA_ERROR("could not find a requesting instance for physics.step");
        }

        dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
        PhysicsScriptContext* context = (PhysicsScriptContext*)lua_touserdata(L, -1);
        lua_pop(L, 1);

        dmGameObject::HInstance sender_instance = CheckGoInstance(L);
        dmGameObject::HCollection collection    = dmGameObject::GetCollection(sender_instance);
        void* world                             = dmGameObject::GetWorld(collection, context->m_ComponentIndex);

        int stepIteration     = lua_tointeger(L, 1);
        int velocityIteration = lua_tointeger(L, 2);
        int positionIteration = lua_tointeger(L, 3);

        dmGameSystem::SetWorld2DStepIteration(world, stepIteration, velocityIteration, positionIteration);

        return 0;
    }

    /*# Set alpha tag to a body, which is by then will be updated more per frame
     * along with the world step.
     * Added by dotGears/TrungB
     *
     * @name physics.set_controllable
     * @param  collisionobject [type:string|hash|url] mark a body with alpha tag.
     * @param  flag [type:boolean] mark a body with alpha tag or disable it.
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_controllable("#body", true)
     * end
     * ```
     */
    static int Physics_SetControllable(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world)) {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp) {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        bool flag = lua_toboolean(L, 2);

        dmGameSystem::SetControllable(comp, flag);

        return 0;
    }

    /*# Set bullet allow for collision object.
     * Added by dotGears/TrungB
     *
     * @name physics.set_bullet
     * @param  collisionobject [type:string|hash|url] target body.
     * @param  flag [type:boolean] mark a body to allow sleeping or not.
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_bullet("#body", true)
     * end
     * ```
     */
    static int Physics_SetBullet(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        bool flag = lua_toboolean(L, 2);

        dmGameSystem::SetBullet(comp, flag);

        return 0;
    }

    /*# Set active for collision object.
     * Added by dotGears/TrungB
     *
     * @name physics.set_active
     * @param  collisionobject [type:string|hash|url] target body.
     * @param  flag [type:boolean] mark a body to active or not.
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_bullet("#body", true)
     * end
     * ```
     */
    static int
    Physics_SetActive(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        bool flag = lua_toboolean(L, 2);

        dmGameSystem::SetActive(comp, flag);

        return 0;
    }
    /*# Return if world is locked or not.
     *
     *
     * @name physics.is_world_locked
     * @return [type:boolean]
     * @examples
     *
     * ```lua
     * function init(self)
     *     if physics.is_world_locked() then
     *          physics.set_active(body, true)
     *     end
     * end
     * ```
     */
    static int Physics_IsWorldLocked(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 1);

        dmMessage::URL sender;
        if (!dmScript::GetURL(L, &sender))
        {
            return DM_LUA_ERROR("could not find a requesting instance for physics.is_world_locked");
        }

        dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
        PhysicsScriptContext* context = (PhysicsScriptContext*)lua_touserdata(L, -1);
        lua_pop(L, 1);

        dmGameObject::HInstance sender_instance = CheckGoInstance(L);
        dmGameObject::HCollection collection    = dmGameObject::GetCollection(sender_instance);
        void* world                             = dmGameObject::GetWorld(collection, context->m_ComponentIndex);

        bool is_world_locked = dmGameSystem::IsWorldLocked(world);
        lua_pushboolean(L, is_world_locked);

        return 1;
    }
    /*# Set Master Body for an collision object
     * Added by dotGears/TrungVu
     *
     * @name physics.set_master
     * @param  collision_object [type:string|hash|url] current body
     * @param  master_body [type:string|hash|url]  target body to be copied.
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_master("#body_slave", "#body_master")
     * end
     * ```
     */
    static int
    Physics_SetMasterBody(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world)) {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp) {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        void * master = 0x0;
        GetCollisionObject(L, 2, collection, &master, &comp_world);

        dmGameSystem::SetMasterBody(comp, master);

        return 0;
    }
    /*# Set Min/Max Velocity for an collision object
     * Added by dotGears/TrungB
     *
     * @name physics.set_velocity_limit
     * @param  collision_object [type:string|hash|url] current body
     * @param  enable [type:boolean] enable or not
     * @param  minX   [type:number]  min velocity X
     * @param  minY   [type:number]  min velocity Y
     * @param  maxX   [type:number]  max velocity X
     * @param  maxY   [type:number]  max velocity Y
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_velocity_limit("#body", true, -8.0, -8.0, 8.0, 8.0)
     *     physics.set_velocity_limit("#body", false)
     * end
     * ```
     */
    static int Physics_SetVelocityLimit(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        bool flag = lua_toboolean(L, 2);

        if(flag)
        {
            float minX = lua_tonumber(L, 3);
            float minY = lua_tonumber(L, 4);
            float maxX = lua_tonumber(L, 5);
            float maxY = lua_tonumber(L, 6);
            // dmGameSystem::SetVelocityLimit(comp, minX, minY, maxX, maxY);
        }
        // else dmGameSystem::DisableVelocityLimit(comp);

        return 0;
    }

    /*# Add copy State to body
     * Added by dotGears/TrungVu
     *
     * @name physics.copy
     * @param  collision_object [type:string|hash|url] slave body.
     * @param  state [type:constant]
     * @param  ratio [type: float]
     * @param  offset [type: float]
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.copy("#body_slave", physics.COPY_POSITION_X, 1.0, 0.0)
     * end
     * ```
     */
    static int Physics_CopyState(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        uint16_t state = luaL_checknumber(L, 2);
        float ratio = luaL_checknumber(L, 3);
        float offset = luaL_checknumber(L, 4);

        dmGameSystem::CopyState(comp, state, ratio, offset);

        return 0;
    }

    /*# Set limits for slave body
     * Added by dotGears/TrungVu
     *
     * @name physics.set_limit
     * @param  collision_object [type:string|hash|url] slave body.
     * @param  state [type:constant]
     * @param  min [type: float]
     * @param  max [type: float]
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_limit("#body_slave", physics.COPY_POSITION_X, 0.0, 2.0)
     * end
     * ```
     */
    static int Physics_SetStateLimit(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp                           = 0x0;
        void* comp_world                     = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world))
        {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp)
        {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        uint16_t state = luaL_checknumber(L, 2);
        float min = luaL_checknumber(L, 3);
        float max = luaL_checknumber(L, 4);

        dmGameSystem::SetStateLimit(comp, state, min, max);

        return 0;
    }

    /*# Set delta tag to a body, which is by then will be updated more per frame
     * along with the world step.
     * Added by dotGears/TrungB
     *
     * @name physics.set_delta_value
     * @param  collisionobject [type:string|hash|url] string, hash or url of the collision-object
     * @param  deltaX [type:float] delta value of body position
     * @param  deltaY [type:float] delta value of body position
     * @param  deltaZ [type:float] delta value of body position
     *
     * @examples
     *
     * ```lua
     * function init(self)
     *     physics.set_delta_value("#body", 1.0 ,2.0 , 0.0)
     * end
     * ```
     */
    static int Physics_SetDeltaValue(lua_State* L)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));
        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world)) {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp) {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        float alphaX = lua_tonumber(L, 2);
        float alphaY = lua_tonumber(L, 3);
        float alphaZ = lua_tonumber(L, 4);
        ///
        /// Need correct place to cast down pointer :
        ///
        dmGameSystem::SetDeltaValue(comp, alphaX, alphaY, alphaZ);

        return 0;
    }

    static int Physics_SetFlipInternal(lua_State* L, bool horizontal)
    {
        DM_LUA_STACK_CHECK(L, 0);

        dmGameObject::HCollection collection = dmGameObject::GetCollection(CheckGoInstance(L));

        void* comp = 0x0;
        void* comp_world = 0x0;
        GetCollisionObject(L, 1, collection, &comp, &comp_world);

        if (!IsCollision2D(comp_world)) {
            return DM_LUA_ERROR("function only available in 2D physics");
        }

        if (!comp) {
            return DM_LUA_ERROR("couldn't find collision object"); // todo: add url
        }

        bool flip = lua_toboolean(L, 2);

        if (horizontal)
            SetCollisionFlipH(comp, flip);
        else
            SetCollisionFlipV(comp, flip);

        return 0;
    }

    /*# flip the geometry horizontally for a collision object
     *
     * Flips the collision shapes horizontally for a collision object
     *
     * @name physics.set_hflip
     * @param url [type:string|hash|url] the collision object that should flip its shapes
     * @param flip [type:boolean] `true` if the collision object should flip its shapes, `false` if not
     * @examples
     *
     * ```lua
     * function init(self)
     *     self.fliph = true -- set on some condition
     *     physics.set_hflip("#collisionobject", self.fliph)
     * end
     * ```
     */
    static int Physics_SetFlipH(lua_State* L)
    {
        return Physics_SetFlipInternal(L, true);
    }

    /*# flip the geometry vertically for a collision object
     *
     * Flips the collision shapes vertically for a collision object
     *
     * @name physics.set_vflip
     * @param url [type:string|hash|url] the collision object that should flip its shapes
     * @param flip [type:boolean] `true` if the collision object should flip its shapes, `false` if not
     * @examples
     *
     * ```lua
     * function init(self)
     *     self.flipv = true -- set on some condition
     *     physics.set_vflip("#collisionobject", self.flipv)
     * end
     * ```
     */
    static int Physics_SetFlipV(lua_State* L)
    {
        return Physics_SetFlipInternal(L, false);
    }

    static const luaL_reg PHYSICS_FUNCTIONS[] = {
        { "ray_cast", Physics_RayCastAsync }, // Deprecated
        { "raycast_async", Physics_RayCastAsync },
        { "raycast", Physics_RayCast },

        { "create_joint", Physics_CreateJoint },
        { "destroy_joint", Physics_DestroyJoint },
        { "get_joint_properties", Physics_GetJointProperties },
        { "set_joint_properties", Physics_SetJointProperties },
        { "get_joint_reaction_force", Physics_GetJointReactionForce },
        { "get_joint_reaction_torque", Physics_GetJointReactionTorque },

        { "set_gravity", Physics_SetGravity },
        { "get_gravity", Physics_GetGravity },

        // Copying B2Body States
        { "set_master", Physics_SetMasterBody },
        { "copy", Physics_CopyState },
        { "set_limit", Physics_SetStateLimit },

        // Set delta value during physics step
        { "set_controllable", Physics_SetControllable },
        { "set_delta_value", Physics_SetDeltaValue },
        { "set_bullet", Physics_SetBullet },
        { "set_active", Physics_SetActive },
        { "is_world_locked", Physics_IsWorldLocked },

        // Config Body/World
        { "set_allow_sleep", Physics_SetAllowSleep },
        { "set_gravity_scale", Physics_SetGravityScale },
        { "set_step_per_frame", Physics_SetStepPerFrame },

        // Config Velocity Limitation
        { "set_velocity_limit", Physics_SetVelocityLimit },

        { "set_hflip", Physics_SetFlipH },
        { "set_vflip", Physics_SetFlipV },
        { 0, 0 }
    };

    void ScriptPhysicsRegister(const ScriptLibContext& context)
    {
        lua_State* L = context.m_LuaState;
        luaL_register(L, "physics", PHYSICS_FUNCTIONS);

#define SETCONSTANT(name) \
    lua_pushnumber(L, (lua_Number) dmPhysics::name); \
    lua_setfield(L, -2, #name);\

        SETCONSTANT(JOINT_TYPE_SPRING)
        SETCONSTANT(JOINT_TYPE_FIXED)
        SETCONSTANT(JOINT_TYPE_HINGE)
        SETCONSTANT(JOINT_TYPE_SLIDER)

        // Added by dotGears / TheTrung
        // This is for CopyState enum as physics.COPY_
        SETCONSTANT(COPY_POSITION_X)
        SETCONSTANT(COPY_POSITION_Y)
        SETCONSTANT(COPY_ROTATION_Z)
        SETCONSTANT(COPY_LINEAR_VEC)
        SETCONSTANT(COPY_ANGULAR_VEC)

#undef SETCONSTANT

        lua_pop(L, 1);

        bool result = true;

        PhysicsScriptContext* physics_context = new PhysicsScriptContext();
        dmMessage::Result socket_result = dmMessage::GetSocket(dmPhysics::PHYSICS_SOCKET_NAME, &physics_context->m_Socket);
        if (socket_result != dmMessage::RESULT_OK)
        {
            result = false;
        }
        dmResource::ResourceType co_resource_type;
        if (result)
        {
            dmResource::Result fact_result = dmResource::GetTypeFromExtension(context.m_Factory, COLLISION_OBJECT_EXT, &co_resource_type);
            if (fact_result != dmResource::RESULT_OK)
            {
                dmLogError("Unable to get resource type for '%s': %d.", COLLISION_OBJECT_EXT, fact_result);
                result = false;
            }
        }
        if (result)
        {
            dmGameObject::ComponentType* co_component_type = dmGameObject::FindComponentType(context.m_Register, co_resource_type, &physics_context->m_ComponentIndex);
            if (co_component_type == 0x0)
            {
                dmLogError("Could not find component type '%s'.", COLLISION_OBJECT_EXT);
                result = false;
            }
        }
        if (result)
        {
            lua_pushlightuserdata(L, physics_context);
            PHYSICS_CONTEXT_HASH = dmScript::SetGlobal(L, PHYSICS_CONTEXT_NAME);
        }
        else
        {
            delete physics_context;
        }
    }

    void ScriptPhysicsFinalize(const ScriptLibContext& context)
    {
        lua_State* L = context.m_LuaState;
        if (L != 0x0)
        {
            int top = lua_gettop(L);
            (void)top;

            dmScript::GetGlobal(L, PHYSICS_CONTEXT_HASH);
            PhysicsScriptContext* physics_context = (PhysicsScriptContext*)lua_touserdata(L, -1);
            lua_pop(L, 1);
            if (physics_context != 0x0)
            {
                delete physics_context;
            }

            assert(top == lua_gettop(L));
        }
    }

}
