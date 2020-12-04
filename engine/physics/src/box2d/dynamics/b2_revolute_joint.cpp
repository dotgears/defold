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

#include "b2_body.h"
#include "b2_revolute_joint.h"
#include "b2_time_step.h"

//Added by Dong Nguyen
#include "b2_world.h"

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2RevoluteJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

b2RevoluteJoint::b2RevoluteJoint(const b2RevoluteJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_referenceAngle = def->referenceAngle;

	m_impulse.SetZero();
	m_motorImpulse = 0.0f;

	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;
	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableLimit = def->enableLimit;
	m_enableMotor = def->enableMotor;
	m_limitState = e_inactiveLimit;
}

void b2RevoluteJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	float aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;

	float aB = data.positions[m_indexB].a;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	b2Rot qA(aA), qB(aB);

	m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	bool fixedRotation = (iA + iB == 0.0f);

	m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB;
	m_mass.ex.y = m_mass.ey.x;
	m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
	m_mass.ex.z = m_mass.ez.x;
	m_mass.ey.z = m_mass.ez.y;
	m_mass.ez.z = iA + iB;

	m_motorMass = iA + iB;
	if (m_motorMass > 0.0f)
	{
		m_motorMass = 1.0f / m_motorMass;
	}

	if (m_enableMotor == false || fixedRotation)
	{
		m_motorImpulse = 0.0f;
	}

	if (m_enableLimit && fixedRotation == false)
	{
		float jointAngle = aB - aA - m_referenceAngle;
		if (b2Abs(m_upperAngle - m_lowerAngle) < 2.0f * b2_angularSlop)
		{
			m_limitState = e_equalLimits;
		}
		else if (jointAngle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_impulse.z = 0.0f;
			}
			m_limitState = e_atLowerLimit;
		}
		else if (jointAngle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_impulse.z = 0.0f;
			}
			m_limitState = e_atUpperLimit;
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_impulse.z = 0.0f;
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= data.step.dtRatio;
		m_motorImpulse *= data.step.dtRatio;

		b2Vec2 P(m_impulse.x, m_impulse.y);

		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + m_motorImpulse + m_impulse.z);

		vB += mB * P;
		wB += iB * (b2Cross(m_rB, P) + m_motorImpulse + m_impulse.z);
	}
	else
	{
		m_impulse.SetZero();
		m_motorImpulse = 0.0f;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2RevoluteJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	bool fixedRotation = (iA + iB == 0.0f);

	// Solve motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits && fixedRotation == false)
	{
		float Cdot = wB - wA - m_motorSpeed;
		float impulse = -m_motorMass * Cdot;
		float oldImpulse = m_motorImpulse;
		float maxImpulse = data.step.dt * m_maxMotorTorque;
		m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit && fixedRotation == false)
	{
		b2Vec2 Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
		float Cdot2 = wB - wA;
		b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

		b2Vec3 impulse = -m_mass.Solve33(Cdot);

		if (m_limitState == e_equalLimits)
		{
			m_impulse += impulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float newImpulse = m_impulse.z + impulse.z;
			if (newImpulse < 0.0f)
			{
				b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
				b2Vec2 reduced = m_mass.Solve22(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -m_impulse.z;
				m_impulse.x += reduced.x;
				m_impulse.y += reduced.y;
				m_impulse.z = 0.0f;
			}
			else
			{
				m_impulse += impulse;
			}
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float newImpulse = m_impulse.z + impulse.z;
			if (newImpulse > 0.0f)
			{
				b2Vec2 rhs = -Cdot1 + m_impulse.z * b2Vec2(m_mass.ez.x, m_mass.ez.y);
				b2Vec2 reduced = m_mass.Solve22(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -m_impulse.z;
				m_impulse.x += reduced.x;
				m_impulse.y += reduced.y;
				m_impulse.z = 0.0f;
			}
			else
			{
				m_impulse += impulse;
			}
		}

		b2Vec2 P(impulse.x, impulse.y);

		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (b2Cross(m_rB, P) + impulse.z);
	}
	else
	{
		// Solve point-to-point constraint
		b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
		b2Vec2 impulse = m_mass.Solve22(-Cdot);

		m_impulse.x += impulse.x;
		m_impulse.y += impulse.y;

		vA -= mA * impulse;
		wA -= iA * b2Cross(m_rA, impulse);

		vB += mB * impulse;
		wB += iB * b2Cross(m_rB, impulse);
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2RevoluteJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	b2Vec2 cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	b2Rot qA(aA), qB(aB);

	float angularError = 0.0f;
	float positionError = 0.0f;

	bool fixedRotation = (m_invIA + m_invIB == 0.0f);

	// Solve angular limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit && fixedRotation == false)
	{
		float angle = aB - aA - m_referenceAngle;
		float limitImpulse = 0.0f;

		if (m_limitState == e_equalLimits)
		{
			// Prevent large angular corrections
			float C = b2Clamp(angle - m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * C;
			angularError = b2Abs(C);
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float C = angle - m_lowerAngle;
			angularError = -C;

			// Prevent large angular corrections and allow some slop.
			C = b2Clamp(C + b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
			limitImpulse = -m_motorMass * C;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float C = angle - m_upperAngle;
			angularError = C;

			// Prevent large angular corrections and allow some slop.
			C = b2Clamp(C - b2_angularSlop, 0.0f, b2_maxAngularCorrection);
			limitImpulse = -m_motorMass * C;
		}

		aA -= m_invIA * limitImpulse;
		aB += m_invIB * limitImpulse;
	}

	// Solve point-to-point constraint.
	{
		qA.Set(aA);
		qB.Set(aB);
		b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
		b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

		b2Vec2 C = cB + rB - cA - rA;
		positionError = C.Length();

		float mA = m_invMassA, mB = m_invMassB;
		float iA = m_invIA, iB = m_invIB;

		b2Mat22 K;
		K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
		K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

		b2Vec2 impulse = -K.Solve(C);

		cA -= mA * impulse;
		aA -= iA * b2Cross(rA, impulse);

		cB += mB * impulse;
		aB += iB * b2Cross(rB, impulse);
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;
	
	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

b2Vec2 b2RevoluteJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2RevoluteJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2RevoluteJoint::GetReactionForce(float inv_dt) const
{
	b2Vec2 P(m_impulse.x, m_impulse.y);
	return inv_dt * P;
}

float b2RevoluteJoint::GetReactionTorque(float inv_dt) const
{
	return inv_dt * m_impulse.z;
}

float b2RevoluteJoint::GetJointAngle() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;
	return bB->m_sweep.a - bA->m_sweep.a - m_referenceAngle;
}

float b2RevoluteJoint::GetJointSpeed() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;
	return bB->m_angularVelocity - bA->m_angularVelocity;
}

bool b2RevoluteJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b2RevoluteJoint::EnableMotor(bool flag)
{
	if (flag != m_enableMotor)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_enableMotor = flag;
	}
}

float b2RevoluteJoint::GetMotorTorque(float inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void b2RevoluteJoint::SetMotorSpeed(float speed)
{
	if (speed != m_motorSpeed)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_motorSpeed = speed;
	}
}

void b2RevoluteJoint::SetMaxMotorTorque(float torque)
{
	if (torque != m_maxMotorTorque)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_maxMotorTorque = torque;
	}
}

bool b2RevoluteJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b2RevoluteJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_enableLimit = flag;
		m_impulse.z = 0.0f;
	}
}

float b2RevoluteJoint::GetLowerLimit() const
{
	return m_lowerAngle;
}

float b2RevoluteJoint::GetUpperLimit() const
{
	return m_upperAngle;
}

void b2RevoluteJoint::SetLimits(float lower, float upper)
{
	b2Assert(lower <= upper);
	
	if (lower != m_lowerAngle || upper != m_upperAngle)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_impulse.z = 0.0f;
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}

void b2RevoluteJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Log("  b2RevoluteJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
	b2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
	b2Log("  jd.lowerAngle = %.15lef;\n", m_lowerAngle);
	b2Log("  jd.upperAngle = %.15lef;\n", m_upperAngle);
	b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
	b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

b2Joint * b2RevoluteJoint::CopyTo(b2World *anotherWorld, b2Body **bodies, b2Joint ** joints)
{
    int32 indexA = m_bodyA->m_islandIndex;
    int32 indexB = m_bodyB->m_islandIndex;
    
    b2RevoluteJointDef jd;
    jd.bodyA = bodies[indexA];
    jd.bodyB = bodies[indexB];
    jd.collideConnected = m_collideConnected;
    jd.localAnchorA.Set(m_localAnchorA.x, m_localAnchorA.y);
    jd.localAnchorB.Set(m_localAnchorB.x, m_localAnchorB.y);
    jd.referenceAngle = m_referenceAngle;
    jd.enableLimit = m_enableLimit;
    jd.lowerAngle = m_lowerAngle;
    jd.upperAngle = m_upperAngle;
    jd.enableMotor = m_enableMotor;
    jd.motorSpeed = m_motorSpeed;
    jd.maxMotorTorque = m_maxMotorTorque;
   
    b2Joint * newJoint = anotherWorld->CreateJoint(&jd);
    return newJoint;
}