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

#include "../box2d/b2_fixture.h"
#include "../box2d/b2_block_allocator.h"
#include "../box2d/b2_broad_phase.h"
#include "../box2d/b2_chain_shape.h"
#include "../box2d/b2_circle_shape.h"
#include "../box2d/b2_collision.h"
#include "../box2d/b2_contact.h"
#include "../box2d/b2_edge_shape.h"
#include "../box2d/b2_polygon_shape.h"
#include "../box2d/b2_world.h"

b2Fixture::b2Fixture()
{
	m_userData = nullptr;
	m_body = nullptr;
	m_next = nullptr;
	m_proxies = nullptr;
	m_proxyCount = 0;
	m_shape = nullptr;
	m_density = 0.0f;
}

void b2Fixture::Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;

	m_body = body;
	m_next = nullptr;

	m_filter = def->filter;

	m_isSensor = def->isSensor;

	m_shape = def->shape->Clone(allocator);

	// Reserve proxy space
	int32 childCount = m_shape->GetChildCount();
	m_proxies = (b2FixtureProxy*)allocator->Allocate(childCount * sizeof(b2FixtureProxy));
	for (int32 i = 0; i < childCount; ++i)
	{
		m_proxies[i].fixture = nullptr;
		m_proxies[i].proxyId = b2BroadPhase::e_nullProxy;
	}
	m_proxyCount = 0;

	m_density = def->density;
}

void b2Fixture::Destroy(b2BlockAllocator* allocator)
{
	// The proxies must be destroyed before calling this.
	b2Assert(m_proxyCount == 0);

	// Free the proxy array.
	int32 childCount = m_shape->GetChildCount();
	allocator->Free(m_proxies, childCount * sizeof(b2FixtureProxy));
	m_proxies = nullptr;

	// Free the child shape.
	switch (m_shape->m_type)
	{
	case b2Shape::e_circle:
		{
			b2CircleShape* s = (b2CircleShape*)m_shape;
			s->~b2CircleShape();
			allocator->Free(s, sizeof(b2CircleShape));
		}
		break;

	case b2Shape::e_edge:
		{
			b2EdgeShape* s = (b2EdgeShape*)m_shape;
			s->~b2EdgeShape();
			allocator->Free(s, sizeof(b2EdgeShape));
		}
		break;

	case b2Shape::e_polygon:
		{
			b2PolygonShape* s = (b2PolygonShape*)m_shape;
			s->~b2PolygonShape();
			allocator->Free(s, sizeof(b2PolygonShape));
		}
		break;

	case b2Shape::e_chain:
		{
			b2ChainShape* s = (b2ChainShape*)m_shape;
			s->~b2ChainShape();
			allocator->Free(s, sizeof(b2ChainShape));
		}
		break;

	default:
		b2Assert(false);
		break;
	}

	m_shape = nullptr;
}

void b2Fixture::CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf)
{
	b2Assert(m_proxyCount == 0);

	// Create proxies in the broad-phase.
	m_proxyCount = m_shape->GetChildCount();

	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b2FixtureProxy* proxy = m_proxies + i;
		m_shape->ComputeAABB(&proxy->aabb, xf, i);
		proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy);
		proxy->fixture = this;
		proxy->childIndex = i;
	}
}

void b2Fixture::DestroyProxies(b2BroadPhase* broadPhase)
{
	// Destroy proxies in the broad-phase.
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b2FixtureProxy* proxy = m_proxies + i;
		broadPhase->DestroyProxy(proxy->proxyId);
		proxy->proxyId = b2BroadPhase::e_nullProxy;
	}

	m_proxyCount = 0;
}

void b2Fixture::Synchronize(b2BroadPhase* broadPhase, const b2Transform& transform1, const b2Transform& transform2)
{
	if (m_proxyCount == 0)
	{	
		return;
	}

	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b2FixtureProxy* proxy = m_proxies + i;

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		b2AABB aabb1, aabb2;
		m_shape->ComputeAABB(&aabb1, transform1, proxy->childIndex);
		m_shape->ComputeAABB(&aabb2, transform2, proxy->childIndex);
	
		proxy->aabb.Combine(aabb1, aabb2);

		b2Vec2 displacement = transform2.p - transform1.p;

		broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
	}
}

void b2Fixture::SynchronizeSingle(b2BroadPhase* broadPhase, int32 index, const b2Transform& transform1, const b2Transform& transform2)
{
    b2Assert(index < m_proxyCount);

    b2FixtureProxy* proxy = m_proxies + index;

    b2AABB aabb1, aabb2;
    m_shape->ComputeAABB(&aabb1, transform1, proxy->childIndex);
    m_shape->ComputeAABB(&aabb2, transform2, proxy->childIndex);

    proxy->aabb.Combine(aabb1, aabb2);

    b2Vec2 displacement = transform2.p - transform1.p;

    broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
}

void b2Fixture::SetFilterData(const b2Filter& filter, int32 index)
{
    // Defold modifications. Added index
    m_filters[index * m_shape->m_filterPerChild] = filter;

    // Defold modifications. If the body is a grid,
    // we skip updating the proxy list since that will
    // potentially expand the movement buffer.
    // Instead, we just flag the entire body for
    // filtering, which is what the argument passed into
    // the function is for.
    Refilter(GetType() != b2Shape::e_grid);
}

void b2Fixture::Refilter()
{
	if (m_body == nullptr)
	{
		return;
	}

	// Flag associated contacts for filtering.
	b2ContactEdge* edge = m_body->GetContactList();
	while (edge)
	{
		b2Contact* contact = edge->contact;
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA == this || fixtureB == this)
		{
			contact->FlagForFiltering();
		}

		edge = edge->next;
	}

	b2World* world = m_body->GetWorld();

	if (world == nullptr)
	{
		return;
	}

	// Touch each proxy so that new pairs may be created
	b2BroadPhase* broadPhase = &world->m_contactManager.m_broadPhase;
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		broadPhase->TouchProxy(m_proxies[i].proxyId);
	}
}

void b2Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_body->SetAwake(true);
		m_isSensor = sensor;
	}
}

void b2Fixture::Dump(int32 bodyIndex)
{
	b2Log("    b2FixtureDef fd;\n");
	b2Log("    fd.friction = %.15lef;\n", m_friction);
	b2Log("    fd.restitution = %.15lef;\n", m_restitution);
	b2Log("    fd.density = %.15lef;\n", m_density);
	b2Log("    fd.isSensor = bool(%d);\n", m_isSensor);
	b2Log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
	b2Log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
	b2Log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

	switch (m_shape->m_type)
	{
	case b2Shape::e_circle:
		{
			b2CircleShape* s = (b2CircleShape*)m_shape;
			b2Log("    b2CircleShape shape;\n");
			b2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
			b2Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s->m_p.x, s->m_p.y);
		}
		break;

	case b2Shape::e_edge:
		{
			b2EdgeShape* s = (b2EdgeShape*)m_shape;
			b2Log("    b2EdgeShape shape;\n");
			b2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
			b2Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s->m_vertex0.x, s->m_vertex0.y);
			b2Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s->m_vertex1.x, s->m_vertex1.y);
			b2Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s->m_vertex2.x, s->m_vertex2.y);
			b2Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s->m_vertex3.x, s->m_vertex3.y);
			b2Log("    shape.m_hasVertex0 = bool(%d);\n", s->m_hasVertex0);
			b2Log("    shape.m_hasVertex3 = bool(%d);\n", s->m_hasVertex3);
		}
		break;

	case b2Shape::e_polygon:
		{
			b2PolygonShape* s = (b2PolygonShape*)m_shape;
			b2Log("    b2PolygonShape shape;\n");
			b2Log("    b2Vec2 vs[%d];\n", b2_maxPolygonVertices);
			for (int32 i = 0; i < s->m_count; ++i)
			{
				b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			b2Log("    shape.Set(vs, %d);\n", s->m_count);
		}
		break;

	case b2Shape::e_chain:
		{
			b2ChainShape* s = (b2ChainShape*)m_shape;
			b2Log("    b2ChainShape shape;\n");
			b2Log("    b2Vec2 vs[%d];\n", s->m_count);
			for (int32 i = 0; i < s->m_count; ++i)
			{
				b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			b2Log("    shape.CreateChain(vs, %d);\n", s->m_count);
			b2Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s->m_prevVertex.x, s->m_prevVertex.y);
			b2Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s->m_nextVertex.x, s->m_nextVertex.y);
			b2Log("    shape.m_hasPrevVertex = bool(%d);\n", s->m_hasPrevVertex);
			b2Log("    shape.m_hasNextVertex = bool(%d);\n", s->m_hasNextVertex);
		}
		break;

	default:
		return;
	}

	b2Log("\n");
	b2Log("    fd.shape = &shape;\n");
	b2Log("\n");
	b2Log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}

void b2Fixture::CopyTo(b2Body *anotherBody)
{
    b2FixtureDef fd;
    fd.friction = m_friction;
    fd.restitution = m_restitution;
    fd.density = m_density;
    fd.isSensor = m_isSensor;
    fd.filter.categoryBits = m_filter.categoryBits;
    fd.filter.maskBits = m_filter.maskBits;
    fd.filter.groupIndex = m_filter.groupIndex;
    
    switch (m_shape->m_type)
    {
    
        case b2Shape::e_circle:
        {
            b2CircleShape * s = (b2CircleShape *) m_shape;
            b2CircleShape shape;
            shape.m_radius = s->m_radius;
            shape.m_p.Set(s->m_p.x, s->m_p.y);
            fd.shape = &shape;
            break;
        }
        
        case b2Shape::e_edge:
        {
            b2EdgeShape * s = (b2EdgeShape *) m_shape;
            b2EdgeShape shape;
            shape.m_radius = s->m_radius;
            shape.m_vertex0.Set(s->m_vertex0.x, s->m_vertex0.y);
            shape.m_vertex1.Set(s->m_vertex1.x, s->m_vertex1.y);
            shape.m_vertex2.Set(s->m_vertex2.x, s->m_vertex2.y);
            shape.m_vertex3.Set(s->m_vertex3.x, s->m_vertex3.y);
            shape.m_hasVertex0 = s->m_hasVertex0;
            shape.m_hasVertex3 = s->m_hasVertex3;
            fd.shape = &shape;
            break;
        }
        
        case b2Shape::e_polygon:
        {
            b2PolygonShape * s = (b2PolygonShape *) m_shape;
            b2PolygonShape shape;
            b2Vec2 vs[b2_maxPolygonVertices];
            for (int32 i = 0; i < s->m_count; ++i)
            {
                vs[i].Set(s->m_vertices[i].x, s->m_vertices[i].y);
            }
            shape.Set(vs, s->m_count);
            fd.shape = &shape;
            break;
        }
        
        case b2Shape::e_chain:
        {
            b2ChainShape * s = (b2ChainShape *)m_shape;
            b2ChainShape shape;
            b2Vec2 vs[s->m_count];
            for (int32 i = 0; i < s->m_count; ++i)
            {
                vs[i].Set(s->m_vertices[i].x, s->m_vertices[i].y);
            }
            shape.CreateChain(vs, s->m_count);
            shape.m_prevVertex.Set(s->m_prevVertex.x, s->m_prevVertex.y);
            shape.m_nextVertex.Set(s->m_nextVertex.x, s->m_nextVertex.y);
            shape.m_hasNextVertex = s->m_hasPrevVertex;
            shape.m_hasNextVertex = s->m_hasNextVertex;
            fd.shape = &shape;
            anotherBody->CreateFixture(&fd);
            return;
        }
        
        default:
        {
            return;
        }
        
    }
    
    anotherBody->CreateFixture(&fd);
}

b2FixtureDef b2Fixture::GenerateScaledCopyDefinition(float scaleFactor)
{
    b2FixtureDef def;
    
    def.friction = m_friction;
    def.restitution = m_restitution;
    def.density = m_density;
    def.isSensor = m_isSensor;
    def.filter.categoryBits = m_filter.categoryBits;
    def.filter.maskBits = m_filter.maskBits;
    def.filter.groupIndex = m_filter.groupIndex;
    
    switch (m_shape->m_type)
    {
        case b2Shape::e_circle:
        {
            b2CircleShape * s = (b2CircleShape *) m_shape;
            b2CircleShape * shape = new b2CircleShape();
            shape->m_radius = s->m_radius * scaleFactor;
            shape->m_p.Set(s->m_p.x * scaleFactor, s->m_p.y * scaleFactor);
            def.shape = shape;
            break;
        }
        
        case b2Shape::e_edge:
        {
            b2EdgeShape * s = (b2EdgeShape *) m_shape;
            b2EdgeShape * shape = new b2EdgeShape();
            shape->m_radius = s->m_radius;
            shape->m_vertex0.Set(s->m_vertex0.x * scaleFactor, s->m_vertex0.y * scaleFactor);
            shape->m_vertex1.Set(s->m_vertex1.x * scaleFactor, s->m_vertex1.y * scaleFactor);
            shape->m_vertex2.Set(s->m_vertex2.x * scaleFactor, s->m_vertex2.y * scaleFactor);
            shape->m_vertex3.Set(s->m_vertex3.x * scaleFactor, s->m_vertex3.y * scaleFactor);
            shape->m_hasVertex0 = s->m_hasVertex0;
            shape->m_hasVertex3 = s->m_hasVertex3;
            def.shape = shape;
            break;
        }
        
        case b2Shape::e_polygon:
        {
            b2PolygonShape * s = (b2PolygonShape *) m_shape;
            b2PolygonShape * shape = new b2PolygonShape();
            b2Vec2 vs[b2_maxPolygonVertices];
            for (int32 i = 0; i < s->m_count; ++i)
            {
                vs[i].Set(s->m_vertices[i].x * scaleFactor, s->m_vertices[i].y * scaleFactor);
            }
            shape->Set(vs, s->m_count);
            def.shape = shape;
            break;
        }
        
        case b2Shape::e_chain:
        {
            b2ChainShape * s = (b2ChainShape *)m_shape;
            b2ChainShape * shape = new b2ChainShape();
            b2Vec2 vs[s->m_count];
            for (int32 i = 0; i < s->m_count; ++i)
            {
                vs[i].Set(s->m_vertices[i].x * scaleFactor, s->m_vertices[i].y * scaleFactor);
            }
            shape->CreateChain(vs, s->m_count);
            shape->m_prevVertex.Set(s->m_prevVertex.x * scaleFactor, s->m_prevVertex.y * scaleFactor);
            shape->m_nextVertex.Set(s->m_nextVertex.x * scaleFactor, s->m_nextVertex.y * scaleFactor);
            shape->m_hasNextVertex = s->m_hasPrevVertex;
            shape->m_hasNextVertex = s->m_hasNextVertex;
            def.shape = shape;
            break;
        }
        
    }
    
    return def;
}
