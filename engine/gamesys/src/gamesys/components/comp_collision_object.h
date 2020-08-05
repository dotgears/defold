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

#ifndef DM_GAMESYS_COMP_COLLISION_OBJECT_H
#define DM_GAMESYS_COMP_COLLISION_OBJECT_H

#include <stdint.h>

#include <gameobject/gameobject.h>
#include <physics/physics.h>

template <typename T> class dmArray;

namespace dmGameSystem
{
    dmGameObject::CreateResult CompCollisionObjectNewWorld(const dmGameObject::ComponentNewWorldParams& params);

    dmGameObject::CreateResult CompCollisionObjectDeleteWorld(const dmGameObject::ComponentDeleteWorldParams& params);

    dmGameObject::CreateResult CompCollisionObjectCreate(const dmGameObject::ComponentCreateParams& params);

    dmGameObject::CreateResult CompCollisionObjectDestroy(const dmGameObject::ComponentDestroyParams& params);

    dmGameObject::CreateResult CompCollisionObjectFinal(const dmGameObject::ComponentFinalParams& params);

    dmGameObject::CreateResult CompCollisionObjectAddToUpdate(const dmGameObject::ComponentAddToUpdateParams& params);

    dmGameObject::UpdateResult CompCollisionObjectUpdate(const dmGameObject::ComponentsUpdateParams& params, dmGameObject::ComponentsUpdateResult& update_result);

    dmGameObject::UpdateResult CompCollisionObjectPostUpdate(const dmGameObject::ComponentsPostUpdateParams& params);

    dmGameObject::UpdateResult CompCollisionObjectOnMessage(const dmGameObject::ComponentOnMessageParams& params);

    void CompCollisionObjectOnReload(const dmGameObject::ComponentOnReloadParams& params);

    dmGameObject::PropertyResult CompCollisionObjectGetProperty(const dmGameObject::ComponentGetPropertyParams& params, dmGameObject::PropertyDesc& out_value);

    dmGameObject::PropertyResult CompCollisionObjectSetProperty(const dmGameObject::ComponentSetPropertyParams& params);

    uint16_t CompCollisionGetGroupBitIndex(void* world, uint64_t group_hash);

    // For script_physics.cpp
    void RayCast(void* world, const dmPhysics::RayCastRequest& request, dmArray<dmPhysics::RayCastResponse>& results);
    uint64_t GetLSBGroupHash(void* world, uint16_t mask);
    dmhash_t CompCollisionObjectGetIdentifier(void* component);

    dmPhysics::JointResult CreateJoint(void* _world, void* _component_a, dmhash_t id, const Vectormath::Aos::Point3& apos, void* _component_b, const Vectormath::Aos::Point3& bpos, dmPhysics::JointType type, const dmPhysics::ConnectJointParams& joint_params);
    dmPhysics::JointResult DestroyJoint(void* _world, void* _component, dmhash_t id);
    dmPhysics::JointResult GetJointParams(void* _world, void* _component, dmhash_t id, dmPhysics::JointType& joint_type, dmPhysics::ConnectJointParams& joint_params);
    dmPhysics::JointResult GetJointType(void* _world, void* _component, dmhash_t id, dmPhysics::JointType& joint_type);
    dmPhysics::JointResult SetJointParams(void* _world, void* _component, dmhash_t id, const dmPhysics::ConnectJointParams& joint_params);
    dmPhysics::JointResult GetJointReactionForce(void* _world, void* _component, dmhash_t id, Vectormath::Aos::Vector3& force);
    dmPhysics::JointResult GetJointReactionTorque(void* _world, void* _component, dmhash_t id, float& torque);

    void SetGravity(void* world, const Vectormath::Aos::Vector3& gravity);
    Vectormath::Aos::Vector3 GetGravity(void* _world);
    /// Added by dotGears/TrungB
    void SetWorld2DStepIteration(void* _world, int stepIteration, int velocityIteration, int positionIteration);
    void SetDeltaValue(void* comp, float alphaX, float alphaY, float alphaZ);
    void SetControllable(void* comp, bool flag);
    void SetSleepingAllowed(void* comp, bool flag);
    void SetBullet(void* comp, bool flag);
    void SetActive(void* comp, bool flag);
    bool IsWorldLocked(void* _world);

    void
    SetGravityScale(void* comp, float gravityScale);
    void SetAllowSleep(void* comp, bool allow_sleep);

    void CopyState(void* comp, uint16_t state);
    void SetCopyRatio(void* comp, float ratio);
    void SetCopyDisable(void* comp);

    void DisableVelocityLimit(void* comp);
    void SetVelocityLimit(void* comp, float minX, float minY, float maxX, float maxY);
    
    //Added by dotGears / TrungA
    void SetMasterBody(void* comp, void* master_body);
    
    /// End of Passion

    bool IsCollision2D(void* _world);
    void SetCollisionFlipH(void* _component, bool flip);
    void SetCollisionFlipV(void* _component, bool flip);
}

#endif // DM_GAMESYS_COMP_COLLISION_OBJECT_H
