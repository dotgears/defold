// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "../box2d/b2_body.h"
#include "../box2d/b2_contact.h"
#include "../box2d/b2_fixture.h"
#include "../box2d/b2_joint.h"
#include "../box2d/b2_world.h"

b2Body::b2Body(const b2BodyDef* bd, b2World* world)
{
	b2Assert(bd->position.IsValid());
	b2Assert(bd->linearVelocity.IsValid());
	b2Assert(b2IsValid(bd->angle));
	b2Assert(b2IsValid(bd->angularVelocity));
	b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
	b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);

	m_flags = 0;

	if (bd->bullet)
	{
		m_flags |= e_bulletFlag;
	}
	if (bd->fixedRotation)
	{
		m_flags |= e_fixedRotationFlag;
	}
	if (bd->allowSleep)
	{
		m_flags |= e_autoSleepFlag;
	}
	if (bd->awake)
	{
		m_flags |= e_awakeFlag;
	}
	if (bd->active)
	{
		m_flags |= e_activeFlag;
	}

	m_world = world;

	m_xf.p = bd->position;
	m_xf.q.Set(bd->angle);

	m_sweep.localCenter.SetZero();
	m_sweep.c0 = m_xf.p;
	m_sweep.c = m_xf.p;
	m_sweep.a0 = bd->angle;
	m_sweep.a = bd->angle;
	m_sweep.alpha0 = 0.0f;

	m_jointList = nullptr;
	m_contactList = nullptr;
	m_prev = nullptr;
	m_next = nullptr;

	m_linearVelocity = bd->linearVelocity;
	m_angularVelocity = bd->angularVelocity;

	m_linearDamping = bd->linearDamping;
	m_angularDamping = bd->angularDamping;
	m_gravityScale = bd->gravityScale;

	m_force.SetZero();
	m_torque = 0.0f;

	m_sleepTime = 0.0f;

	m_type = bd->type;

	if (m_type == b2_dynamicBody)
	{
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}
	else
	{
		m_mass = 0.0f;
		m_invMass = 0.0f;
	}

	m_I = 0.0f;
	m_invI = 0.0f;

	m_userData = bd->userData;

	m_fixtureList = nullptr;
	m_fixtureCount = 0;

	//Added by dotGears - Trung Vu
	m_copy_flags = 0;
	m_limit_flags = 0;

	m_ratio_pos_x = 1.0f;
	m_ratio_pos_y = 1.0f;
	m_ratio_rotation = 1.0f;
	m_ratio_linear_velo = 1.0f;
	m_ratio_angular_velo = 1.0f;

	m_offset_pos_x = 0.0f;
	m_offset_pos_y = 0.0f;
	m_offset_rotation = 0.0f;
	m_offset_linear_velo = 0.0f;
	m_offset_angular_velo = 0.0f;

	m_min_pos_x = 0.0f;
	m_min_pos_y = 0.0f;
	m_min_rotation = 0.0f;
	m_min_linear_velo = 0.0f;
	m_min_angular_velo = 0.0f;

	m_max_pos_x = 0.0f;
	m_max_pos_y = 0.0f;
	m_max_rotation = 0.0f;
	m_max_linear_velo = 0.0f;
	m_max_angular_velo = 0.0f;

}

b2Body::~b2Body()
{
	// shapes and joints are destroyed in b2World::Destroy
}

void b2Body::SetType(b2BodyType type)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == b2_staticBody)
	{
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_sweep.a0 = m_sweep.a;
		m_sweep.c0 = m_sweep.c;
		SynchronizeFixtures();
	}

	SetAwake(true);

	m_force.SetZero();
	m_torque = 0.0f;

	// Delete the attached contacts.
	b2ContactEdge* ce = m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_world->m_contactManager.Destroy(ce0->contact);
	}
	m_contactList = nullptr;

	// Touch the proxies so that new contacts will be created (when appropriate)
	b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		int32 proxyCount = f->m_proxyCount;
		for (int32 i = 0; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

b2Fixture* b2Body::CreateFixture(const b2FixtureDef* def)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return nullptr;
	}

	b2BlockAllocator* allocator = &m_world->m_blockAllocator;

	void* memory = allocator->Allocate(sizeof(b2Fixture));
	b2Fixture* fixture = new (memory) b2Fixture;
	fixture->Create(allocator, this, def);

	if (m_flags & e_activeFlag)
	{
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->CreateProxies(broadPhase, m_xf);
	}

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;
	++m_fixtureCount;

	fixture->m_body = this;

	// Adjust mass properties if needed.
	if (fixture->m_density > 0.0f)
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->m_flags |= b2World::e_newFixture;

	return fixture;
}

b2Fixture* b2Body::CreateFixture(const b2Shape* shape, float density)
{
	b2FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(&def);
}

void b2Body::DestroyFixture(b2Fixture* fixture)
{
	if (fixture == NULL)
	{
		return;
	}

	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	b2Assert(fixture->m_body == this);

	// Remove the fixture from this body's singly linked list.
	b2Assert(m_fixtureCount > 0);
	b2Fixture** node = &m_fixtureList;
	bool found = false;
	while (*node != nullptr)
	{
		if (*node == fixture)
		{
			*node = fixture->m_next;
			found = true;
			break;
		}

		node = &(*node)->m_next;
	}

	// You tried to remove a shape that is not attached to this body.
	b2Assert(found);

	// Destroy any contacts associated with the fixture.
	b2ContactEdge* edge = m_contactList;
	while (edge)
	{
		b2Contact* c = edge->contact;
		edge = edge->next;

		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();

		if (fixture == fixtureA || fixture == fixtureB)
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			m_world->m_contactManager.Destroy(c);
		}
	}

	b2BlockAllocator* allocator = &m_world->m_blockAllocator;

	if (m_flags & e_activeFlag)
	{
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->DestroyProxies(broadPhase);
	}

	fixture->m_body = nullptr;
	fixture->m_next = nullptr;
	fixture->Destroy(allocator);
	fixture->~b2Fixture();
	allocator->Free(fixture, sizeof(b2Fixture));

	--m_fixtureCount;

	// Reset the mass data.
	ResetMassData();
}

void b2Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.
	m_mass = 0.0f;
	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;
	m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (m_type == b2_staticBody || m_type == b2_kinematicBody)
	{
		m_sweep.c0 = m_xf.p;
		m_sweep.c = m_xf.p;
		m_sweep.a0 = m_sweep.a;
		return;
	}

	b2Assert(m_type == b2_dynamicBody);

	// Accumulate mass over all fixtures.
	b2Vec2 localCenter = b2Vec2_zero;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == 0.0f)
		{
			continue;
		}

		b2MassData massData;
		f->GetMassData(&massData);
		m_mass += massData.mass;
		localCenter += massData.mass * massData.center;
		m_I += massData.I;
	}

	// Compute center of mass.
	if (m_mass > 0.0f)
	{
		m_invMass = 1.0f / m_mass;
		localCenter *= m_invMass;
	}
	else
	{
		// Force all dynamic bodies to have a positive mass.
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}

	if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
	{
		// Center the inertia about the center of mass.
		m_I -= m_mass * b2Dot(localCenter, localCenter);
		b2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;

	}
	else
	{
		m_I = 0.0f;
		m_invI = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter = localCenter;
	m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

// DEFOLD
void b2Body::PurgeContacts(b2Fixture* fixture)
{
    // Destroy any contacts associated with the fixture.
    b2ContactEdge* edge = m_contactList;
    while (edge)
    {
        b2Contact* c = edge->contact;
        edge         = edge->next;

        b2Fixture* fixtureA = c->GetFixtureA();
        b2Fixture* fixtureB = c->GetFixtureB();

        if (fixture == fixtureA || fixture == fixtureB)
        {
            // This destroys the contact and removes it from
            // this body's contact list.
            m_world->m_contactManager.Destroy(c);
        }
    }
}

void b2Body::SetMassData(const b2MassData* massData)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type != b2_dynamicBody)
	{
		return;
	}

	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;

	m_mass = massData->mass;
	if (m_mass <= 0.0f)
	{
		m_mass = 1.0f;
	}

	m_invMass = 1.0f / m_mass;

	if (massData->I > 0.0f && (m_flags & b2Body::e_fixedRotationFlag) == 0)
	{
		m_I = massData->I - m_mass * b2Dot(massData->center, massData->center);
		b2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;
	}

	// Move center of mass.
	b2Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter =  massData->center;
	m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

bool b2Body::ShouldCollide(const b2Body* other) const
{
	// At least one body should be dynamic.
	if (m_type != b2_dynamicBody && other->m_type != b2_dynamicBody)
	{
		return false;
	}

	// Does a joint prevent collision?
	for (b2JointEdge* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
		{
			if (jn->joint->m_collideConnected == false)
			{
				return false;
			}
		}
	}

	return true;
}

void b2Body::SetTransform(const b2Vec2& position, float angle)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	m_xf.q.Set(angle);
	m_xf.p = position;

	m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
	m_sweep.a = angle;

	m_sweep.c0 = m_sweep.c;
	m_sweep.a0 = angle;

	b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}
}

void b2Body::SynchronizeFixtures()
{
	b2Transform xf1;
	xf1.q.Set(m_sweep.a0);
	xf1.p = m_sweep.c0 - b2Mul(xf1.q, m_sweep.localCenter);

	b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, xf1, m_xf);
	}
}
// Defold Modification
void b2Body::SynchronizeSingle(b2Shape* shape, int32 index)
{
    // Defold fix: Shapes call this function blindly not knowing if proxies have been created or not.
    // b2Body only has proxied created when active, so discard calls when not active so shapes can be
    // updated without crash on inactive objects.
    if (!IsActive())
    {
        return;
    }

    b2Transform xf1;
    xf1.q.Set(m_sweep.a0);
    xf1.p = m_sweep.c0 - b2Mul(xf1.q, m_sweep.localCenter);

    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        if (f->GetShape() == shape)
        {
            f->SynchronizeSingle(broadPhase, index, xf1, m_xf);
        }
    }
}

void b2Body::SetActive(bool flag)
{
	b2Assert(m_world->IsLocked() == false);

	if (flag == IsActive())
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_activeFlag;

		// Create all proxies.
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->CreateProxies(broadPhase, m_xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~e_activeFlag;

		// Destroy all proxies.
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->DestroyProxies(broadPhase);
		}

		// Destroy the attached contacts.
		b2ContactEdge* ce = m_contactList;
		while (ce)
		{
			b2ContactEdge* ce0 = ce;
			ce = ce->next;
			m_world->m_contactManager.Destroy(ce0->contact);
		}
		m_contactList = nullptr;
	}
}

void b2Body::SetFixedRotation(bool flag)
{
	bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_fixedRotationFlag;
	}
	else
	{
		m_flags &= ~e_fixedRotationFlag;
	}

	m_angularVelocity = 0.0f;

	ResetMassData();
}

//Added by Dong Nguyen
void b2Body::SetCustomProperties(const char *propertyName, int intValue)
{
    if (m_customProperties_Int.count(propertyName) > 0)
    {
        m_customProperties_Int.erase(propertyName);
    }

    m_customProperties_Int.insert(std::pair<std::string, int>(std::string(propertyName), intValue));
}

void b2Body::SetCustomProperties(const char *propertyName, float floatValue)
{
    if (m_customProperties_Float.count(propertyName) > 0)
    {
        m_customProperties_Float.erase(propertyName);
    }
    
    m_customProperties_Float.insert(std::pair<std::string, float>(std::string(propertyName), floatValue));
}

void b2Body::SetCustomProperties(const char *propertyName, const char * stringValue)
{
    if (m_customProperties_String.count(propertyName) > 0)
    {
        m_customProperties_String.erase(propertyName);
    }

    m_customProperties_String.insert(std::pair<std::string, std::string>(std::string(propertyName), std::string(stringValue)));
}

//void b2Body::SetCustomProperties(const char *propertyName, b2Vec2 value)
//{
//    m_customProperties_b2Vec2.insert(std::pair<std::string, b2Vec2>(std::string(propertyName), value));
//}

//void b2Body::SetCustomProperties(const char *propertyName, b2Color color)
//{
//}

void b2Body::SetCustomProperties(const char *propertyName, bool value)
{
    if (m_customProperties_Bool.count(propertyName) > 0)
    {
        m_customProperties_Bool.erase(propertyName);
    }

    m_customProperties_Bool.insert(std::pair<std::string, bool>(std::string(propertyName), value));
}

int b2Body::GetCustomPropertiesInt(const char * propertyName)
{
    std::string key = std::string(propertyName);
    if (m_customProperties_Int.count(key) > 0)
    {
        return m_customProperties_Int.at(key);
    }
    else
        return 0;
}

float b2Body::GetCustomPropertiesFloat(const char * propertyName)
{
    std::string key = std::string(propertyName);
    if (m_customProperties_Float.count(key) > 0)
    {
        return m_customProperties_Float.at(key);
    }
    else
        return 0;
}

const char * b2Body::GetCustomPropertiesString(const char * propertyName)
{
    std::string key = std::string(propertyName);
    if (m_customProperties_String.count(key) > 0)
    {
        return m_customProperties_String.at(key).c_str();
    }
    else
        return "";
}

bool b2Body::GetCustomPropertiesBool(const char * propertyName)
{
    std::string key = std::string(propertyName);
    if (m_customProperties_Bool.count(key) > 0)
    {
        return m_customProperties_Bool.at(key);
    }
    else
        return false;
}

void b2Body::Dump()
{
	int32 bodyIndex = m_islandIndex;

	b2Log("{\n");
	b2Log("  b2BodyDef bd;\n");
	b2Log("  bd.type = b2BodyType(%d);\n", m_type);
	b2Log("  bd.position.Set(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
	b2Log("  bd.angle = %.15lef;\n", m_sweep.a);
	b2Log("  bd.linearVelocity.Set(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
	b2Log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
	b2Log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
	b2Log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
	b2Log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
	b2Log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
	b2Log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
	b2Log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
	b2Log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
	b2Log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
	b2Log("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
	b2Log("\n");
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		b2Log("  {\n");
		f->Dump(bodyIndex);
		b2Log("  }\n");
	}
	b2Log("}\n");
}

void b2Body::UpdateStateFromMasterBody()
{
	if (m_masterBody == NULL || m_copy_flags == 0)
	{
		return;
	}

	b2Vec2 position = this->GetPosition();
	
	if (( m_copy_flags & e_position_x) == e_position_x)
	{
		position.x = m_masterBody->GetPosition().x * m_ratio_pos_x + m_offset_pos_x;

		if ((m_limit_flags & e_position_x) == e_position_x)
		{
			position.x = position.x > m_max_pos_x ? m_max_pos_x : position.x < m_min_pos_x ? m_min_pos_x : position.x;
		}
	}

	if (( m_copy_flags & e_position_y) == e_position_y)
	{
		position.y = m_masterBody->GetPosition().y * m_ratio_pos_y + m_offset_pos_y;

		if ((m_limit_flags & e_position_y) == e_position_y)
		{
			position.y = position.y > m_max_pos_y ? m_max_pos_y : position.y < m_min_pos_y ? m_min_pos_y : position.y;
		}
		
	}

	float angle = this->GetAngle();

	if ((m_copy_flags & e_rotation) == e_rotation)
	{
		angle = m_masterBody->GetAngle() * m_ratio_rotation + m_offset_rotation;

		if ((m_limit_flags & e_rotation) == e_rotation)
		{
			angle = angle > m_max_rotation ? m_max_rotation : angle < m_min_rotation ? m_min_rotation : angle;
		}
		
	}

	b2Vec2 linear_velocity = this->GetLinearVelocity();

	if ((m_copy_flags & e_linear_velo) == e_linear_velo) 
	{
		linear_velocity.x = m_masterBody->GetLinearVelocity().x * m_ratio_linear_velo + m_offset_linear_velo;
		linear_velocity.y = m_masterBody->GetLinearVelocity().y * m_ratio_linear_velo + m_offset_linear_velo;

		if ((m_limit_flags & e_linear_velo) == e_linear_velo)
		{
			linear_velocity.x = linear_velocity.x > m_max_linear_velo ? m_max_linear_velo : linear_velocity.x < m_min_linear_velo ? m_min_linear_velo : linear_velocity.x;
			linear_velocity.y = linear_velocity.y > m_max_linear_velo ? m_max_linear_velo : linear_velocity.y < m_min_linear_velo ? m_min_linear_velo : linear_velocity.y;
		}
	}

	float angular_velo = this->GetAngularVelocity();
	if ((m_copy_flags & e_angular_velo) == e_angular_velo)
	{
		angular_velo = m_masterBody->GetAngularVelocity() * m_ratio_angular_velo + m_offset_angular_velo;

		if ((m_limit_flags & e_angular_velo) == e_angular_velo)
		{
			angular_velo = angular_velo > m_max_angular_velo ? m_max_angular_velo : angular_velo < m_min_angular_velo ? m_min_angular_velo : angular_velo;
		}
	}

	this->SetTransform(position, angle);
	this->SetLinearVelocity(linear_velocity);
	this->SetAngularVelocity(angular_velo);
}

b2Body * b2Body::CopyTo(b2World * world)
{
    b2BodyDef bd;
    bd.type = this->m_type;
    bd.position.Set(m_xf.p.x, m_xf.p.y);
    bd.angle = m_sweep.a;
    bd.linearVelocity.Set(m_linearVelocity.x, m_linearVelocity.y);
    bd.angularVelocity = m_angularVelocity;
    bd.linearDamping = m_linearDamping;
    bd.angularDamping = m_angularDamping;
    bd.allowSleep = m_flags & e_autoSleepFlag;
    bd.awake = m_flags & e_awakeFlag;
    bd.fixedRotation = m_flags & e_fixedRotationFlag;
    bd.bullet = m_flags & e_bulletFlag;
    bd.active = m_flags & e_activeFlag;
    bd.gravityScale = m_gravityScale;
    
    b2Body * newBody = world->CreateBody(&bd);
    newBody->m_islandIndex = m_islandIndex;
    newBody->m_id = m_id;
    newBody->SetName(m_name);
    
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        f->CopyTo(newBody);
    }
    
    return newBody;
}

void b2Body::Scale(float scale_factor)
{
    b2FixtureDef def[100];
    int defCount = 0;

    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        b2Shape * shape = f->GetShape();
        b2FixtureDef fixtureDef = f->GenerateScaledCopyDefinition(scale_factor);
        
        def[defCount] = fixtureDef;
        defCount++;
    }
    
    b2Fixture * fixture = m_fixtureList;
    while (fixture)
    {
        b2Fixture * next = fixture->GetNext();
        DestroyFixture(fixture);
        fixture = next;
    }
    
    for (int i = 0; i < defCount; i++)
    {
        CreateFixture(&def[i]);
        delete def[i].shape;
        def[i].shape = NULL;
    }
    
    b2MassData massData;
    this->GetMassData(&massData);
    massData.mass = massData.mass * scale_factor;
    this->SetMassData(&massData);
    
    this->SetCustomProperties("body_scale", scale_factor);
}
