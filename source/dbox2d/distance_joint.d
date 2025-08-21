module dbox2d.distance_joint;

private template HasVersion(string versionId) {
	mixin("version("~versionId~") {enum HasVersion = true;} else {enum HasVersion = false;}");
}

static if (HasVersion!"_MSC_VER" && !HasVersion!"_CRT_SECURE_NO_WARNINGS") {
version = _CRT_SECURE_NO_WARNINGS;
}
import dbox2d.body;
import dbox2d.core;
import dbox2d.joint;
import dbox2d.solver;
import dbox2d.solver_set;
import dbox2d.physics_world;

// needed for dll export
import dbox2d.box2d;

mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));
mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2Body","b2Body"));

import core.stdc.stdio;

void b2DistanceJoint_SetLength(b2JointId jointId, float length)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;

	joint.length = clamp( length, B2_LINEAR_SLOP, B2_HUGE );
	joint.impulse = 0.0f;
	joint.lowerImpulse = 0.0f;
	joint.upperImpulse = 0.0f;
}

float b2DistanceJoint_GetLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;
	return joint.length;
}

void b2DistanceJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;
	joint.enableLimit = enableLimit;
}

bool b2DistanceJoint_IsLimitEnabled(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint.distanceJoint.enableLimit;
}

void b2DistanceJoint_SetLengthRange(b2JointId jointId, float minLength, float maxLength)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;

	minLength = clamp( minLength, B2_LINEAR_SLOP, B2_HUGE );
	maxLength = clamp( maxLength, B2_LINEAR_SLOP, B2_HUGE );
	joint.minLength = min( minLength, maxLength );
	joint.maxLength = max( minLength, maxLength );
	joint.impulse = 0.0f;
	joint.lowerImpulse = 0.0f;
	joint.upperImpulse = 0.0f;
}

float b2DistanceJoint_GetMinLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;
	return joint.minLength;
}

float b2DistanceJoint_GetMaxLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;
	return joint.maxLength;
}

float b2DistanceJoint_GetCurrentLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );

	b2World* world = b2GetWorld( jointId.world0 );
	B2_ASSERT( world.locked == false );
	if ( world.locked )
	{
		return 0.0f;
	}

	b2Transform transformA = b2GetBodyTransform( world, base.bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, base.bodyIdB );

	b2Vec2 pA = b2TransformPoint( transformA, base.localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, base.localFrameB.p );
	b2Vec2 d = pB - pA;
	float length = d.length();
	return length;
}

void b2DistanceJoint_EnableSpring(b2JointId jointId, bool enableSpring)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base.distanceJoint.enableSpring = enableSpring;
}

bool b2DistanceJoint_IsSpringEnabled(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return base.distanceJoint.enableSpring;
}

void b2DistanceJoint_SetSpringForceRange(b2JointId jointId, float lowerForce, float upperForce)
{
	B2_ASSERT( lowerForce <= upperForce );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base.distanceJoint.lowerSpringForce = lowerForce;
	base.distanceJoint.upperSpringForce = upperForce;
}

void b2DistanceJoint_GetSpringForceRange(b2JointId jointId, float* lowerForce, float* upperForce)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	*lowerForce = base.distanceJoint.lowerSpringForce;
	*upperForce = base.distanceJoint.upperSpringForce;
}

void b2DistanceJoint_SetSpringHertz(b2JointId jointId, float hertz)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base.distanceJoint.hertz = hertz;
}

void b2DistanceJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base.distanceJoint.dampingRatio = dampingRatio;
}

float b2DistanceJoint_GetSpringHertz(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;
	return joint.hertz;
}

float b2DistanceJoint_GetSpringDampingRatio(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base.distanceJoint;
	return joint.dampingRatio;
}

void b2DistanceJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	if ( enableMotor != joint.distanceJoint.enableMotor )
	{
		joint.distanceJoint.enableMotor = enableMotor;
		joint.distanceJoint.motorImpulse = 0.0f;
	}
}

bool b2DistanceJoint_IsMotorEnabled(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint.distanceJoint.enableMotor;
}

void b2DistanceJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	joint.distanceJoint.motorSpeed = motorSpeed;
}

float b2DistanceJoint_GetMotorSpeed(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint.distanceJoint.motorSpeed;
}

float b2DistanceJoint_GetMotorForce(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return world.inv_h * base.distanceJoint.motorImpulse;
}

void b2DistanceJoint_SetMaxMotorForce(b2JointId jointId, float force)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	joint.distanceJoint.maxMotorForce = force;
}

float b2DistanceJoint_GetMaxMotorForce(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint.distanceJoint.maxMotorForce;
}

b2Vec2 b2GetDistanceJointForce(b2World* world, b2JointSim* base)
{
	b2DistanceJoint* joint = &base.distanceJoint;

	b2Transform transformA = b2GetBodyTransform( world, base.bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, base.bodyIdB );

	b2Vec2 pA = b2TransformPoint( transformA, base.localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, base.localFrameB.p );
	b2Vec2 d = pB - pA;
	b2Vec2 axis = b2Normalize( d );
	float force = ( joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse ) * world.inv_h;
	return b2MulSV( force, axis );
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

void b2PrepareDistanceJoint(b2JointSim* base, b2StepContext* context)
{
	B2_ASSERT( base.type == b2_distanceJoint );

	// chase body id to the solver set where the body lives
	int idA = base.bodyIdA;
	int idB = base.bodyIdB;

	b2World* world = context.world;
	b2Body* bodyA = b2BodyArray_Get( world.bodies, idA );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, idB );

	B2_ASSERT( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet );

	b2SolverSet* setA = b2SolverSetArray_Get( world.solverSets, bodyA.setIndex );
	b2SolverSet* setB = b2SolverSetArray_Get( world.solverSets, bodyB.setIndex );

	int localIndexA = bodyA.localIndex;
	int localIndexB = bodyB.localIndex;

	b2BodySim* bodySimA = b2BodySimArray_Get( setA.bodySims, localIndexA );
	b2BodySim* bodySimB = b2BodySimArray_Get( setB.bodySims, localIndexB );

	float mA = bodySimA.invMass;
	float iA = bodySimA.invInertia;
	float mB = bodySimB.invMass;
	float iB = bodySimB.invInertia;

	base.invMassA = mA;
	base.invMassB = mB;
	base.invIA = iA;
	base.invIB = iB;

	b2DistanceJoint* joint = &base.distanceJoint;

	joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// initial anchors in world space
	joint.anchorA = b2RotateVector( bodySimA.transform.q, base.localFrameA.p - bodySimA.localCenter );
	joint.anchorB = b2RotateVector( bodySimB.transform.q, base.localFrameB.p - bodySimB.localCenter );
	joint.deltaCenter = bodySimB.center - bodySimA.center;

	b2Vec2 rA = joint.anchorA;
	b2Vec2 rB = joint.anchorB;
	b2Vec2 separation = rB - rA + joint.deltaCenter;
	b2Vec2 axis = b2Normalize( separation );

	// compute effective mass
	float crA = b2Cross( rA, axis );
	float crB = b2Cross( rB, axis );
	float k = mA + mB + iA * crA * crA + iB * crB * crB;
	joint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint.distanceSoftness = b2MakeSoft( joint.hertz, joint.dampingRatio, context.h );

	if ( context.enableWarmStarting == false )
	{
		joint.impulse = 0.0f;
		joint.lowerImpulse = 0.0f;
		joint.upperImpulse = 0.0f;
		joint.motorImpulse = 0.0f;
	}
}

void b2WarmStartDistanceJoint(b2JointSim* base, b2StepContext* context)
{
	B2_ASSERT( base.type == b2_distanceJoint );

	float mA = base.invMassA;
	float mB = base.invMassB;
	float iA = base.invIA;
	float iB = base.invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base.distanceJoint;
	b2BodyState* stateA = joint.indexA == B2_NULL_INDEX ? &dummyState : context.states + joint.indexA;
	b2BodyState* stateB = joint.indexB == B2_NULL_INDEX ? &dummyState : context.states + joint.indexB;

	b2Vec2 rA = b2RotateVector( stateA.deltaRotation, joint.anchorA );
	b2Vec2 rB = b2RotateVector( stateB.deltaRotation, joint.anchorB );

	b2Vec2 ds = ( stateB.deltaPosition - stateA.deltaPosition ) + ( rB - rA );
	b2Vec2 separation = joint.deltaCenter + ds;
	b2Vec2 axis = b2Normalize( separation );

	float axialImpulse = joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse;
	b2Vec2 P = b2MulSV( axialImpulse, axis );

	stateA.linearVelocity = b2MulSub( stateA.linearVelocity, mA, P );
	stateA.angularVelocity -= iA * b2Cross( rA, P );
	stateB.linearVelocity = b2MulAdd( stateB.linearVelocity, mB, P );
	stateB.angularVelocity += iB * b2Cross( rB, P );
}

void b2SolveDistanceJoint(b2JointSim* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT( base.type == b2_distanceJoint );

	float mA = base.invMassA;
	float mB = base.invMassB;
	float iA = base.invIA;
	float iB = base.invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base.distanceJoint;
	b2BodyState* stateA = joint.indexA == B2_NULL_INDEX ? &dummyState : context.states + joint.indexA;
	b2BodyState* stateB = joint.indexB == B2_NULL_INDEX ? &dummyState : context.states + joint.indexB;

	b2Vec2 vA = stateA.linearVelocity;
	float wA = stateA.angularVelocity;
	b2Vec2 vB = stateB.linearVelocity;
	float wB = stateB.angularVelocity;

	// current anchors
	b2Vec2 rA = b2RotateVector( stateA.deltaRotation, joint.anchorA );
	b2Vec2 rB = b2RotateVector( stateB.deltaRotation, joint.anchorB );

	// current separation
	b2Vec2 ds = ( stateB.deltaPosition - stateA.deltaPosition ) + ( rB - rA );
	b2Vec2 separation = joint.deltaCenter + ds;

	float length = separation.length();
	b2Vec2 axis = b2Normalize( separation );

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if ( joint.enableSpring && ( joint.minLength < joint.maxLength || joint.enableLimit == false ) )
	{
		// spring
		if ( joint.hertz > 0.0f )
		{
			// Cdot = dot(u, v + cross(w, r))
			b2Vec2 vr = ( vB - vA ) + ( b2CrossSV( wB, rB ) - b2CrossSV( wA, rA ) );
			float Cdot = b2Dot( axis, vr );
			float C = length - joint.length;
			float bias = joint.distanceSoftness.biasRate * C;

			float m = joint.distanceSoftness.massScale * joint.axialMass;
			float oldImpulse = joint.impulse;
			float impulse = -m * ( Cdot + bias ) - joint.distanceSoftness.impulseScale * oldImpulse;

			float h = context.h;
			joint.impulse = clamp( joint.impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h );
			impulse = joint.impulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axis );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		if ( joint.enableLimit )
		{
			// lower limit
			{
				b2Vec2 vr = ( vB - vA ) + ( b2CrossSV( wB, rB ) - b2CrossSV( wA, rA ) );
				float Cdot = b2Dot( axis, vr );

				float C = length - joint.minLength;

				float bias = 0.0f;
				float massCoeff = 1.0f;
				float impulseCoeff = 0.0f;
				if ( C > 0.0f )
				{
					// speculative
					bias = C * context.inv_h;
				}
				else if ( useBias )
				{
					bias = base.constraintSoftness.biasRate * C;
					massCoeff = base.constraintSoftness.massScale;
					impulseCoeff = base.constraintSoftness.impulseScale;
				}

				float impulse = -massCoeff * joint.axialMass * ( Cdot + bias ) - impulseCoeff * joint.lowerImpulse;
				float newImpulse = max( 0.0f, joint.lowerImpulse + impulse );
				impulse = newImpulse - joint.lowerImpulse;
				joint.lowerImpulse = newImpulse;

				b2Vec2 P = b2MulSV( impulse, axis );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}

			// upper
			{
				b2Vec2 vr = ( vA - vB ) + ( b2CrossSV( wA, rA ) - b2CrossSV( wB, rB ) );
				float Cdot = b2Dot( axis, vr );

				float C = joint.maxLength - length;

				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( C > 0.0f )
				{
					// speculative
					bias = C * context.inv_h;
				}
				else if ( useBias )
				{
					bias = base.constraintSoftness.biasRate * C;
					massScale = base.constraintSoftness.massScale;
					impulseScale = base.constraintSoftness.impulseScale;
				}

				float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.upperImpulse;
				float newImpulse = max( 0.0f, joint.upperImpulse + impulse );
				impulse = newImpulse - joint.upperImpulse;
				joint.upperImpulse = newImpulse;

				b2Vec2 P = b2MulSV( -impulse, axis );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}
		}

		if ( joint.enableMotor )
		{
			b2Vec2 vr = ( vB - vA ) + ( b2CrossSV( wB, rB ) - b2CrossSV( wA, rA ) );
			float Cdot = b2Dot( axis, vr );
			float impulse = joint.axialMass * ( joint.motorSpeed - Cdot );
			float oldImpulse = joint.motorImpulse;
			float maxImpulse = context.h * joint.maxMotorForce;
			joint.motorImpulse = clamp( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint.motorImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axis );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}
	}
	else
	{
		// rigid constraint
		b2Vec2 vr = ( vB - vA ) + ( b2CrossSV( wB, rB ) - b2CrossSV( wA, rA ) );
		float Cdot = b2Dot( axis, vr );

		float C = length - joint.length;

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			bias = base.constraintSoftness.biasRate * C;
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.impulse;
		joint.impulse += impulse;

		b2Vec2 P = b2MulSV( impulse, axis );
		vA = b2MulSub( vA, mA, P );
		wA -= iA * b2Cross( rA, P );
		vB = b2MulAdd( vB, mB, P );
		wB += iB * b2Cross( rB, P );
	}

	stateA.linearVelocity = vA;
	stateA.angularVelocity = wA;
	stateB.linearVelocity = vB;
	stateB.angularVelocity = wB;
}


// version (none) {
// void b2DistanceJoint:: Dump()
// {
// 	int indexA = m_bodyA.m_islandIndex;
// 	int indexB = m_bodyB.m_islandIndex;

// 	b2Dump("  b2DistanceJointDef jd;\n");
// 	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
// 	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
// 	b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
// 	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
// 	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
// 	b2Dump("  jd.length = %.9g;\n", m_length);
// 	b2Dump("  jd.minLength = %.9g;\n", m_minLength);
// 	b2Dump("  jd.maxLength = %.9g;\n", m_maxLength);
// 	b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
// 	b2Dump("  jd.damping = %.9g;\n", m_damping);
// 	b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
// }
// }

void b2DrawDistanceJoint(b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB)
{
	B2_ASSERT( base.type == b2_distanceJoint );

	b2DistanceJoint* joint = &base.distanceJoint;

	b2Vec2 pA = b2TransformPoint( transformA, base.localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, base.localFrameB.p );

	b2Vec2 axis = b2Normalize( pB - pA );

	if ( joint.minLength < joint.maxLength && joint.enableLimit )
	{
		b2Vec2 pMin = b2MulAdd( pA, joint.minLength, axis );
		b2Vec2 pMax = b2MulAdd( pA, joint.maxLength, axis );
		b2Vec2 offset = b2MulSV( 0.05f * b2_lengthUnitsPerMeter, axis.rightPerp() );

		if ( joint.minLength > B2_LINEAR_SLOP )
		{
			// draw->DrawPoint(pMin, 4.0f, c2, draw->context);
			draw.DrawSegmentFcn( pMin - offset, pMin + offset, b2_colorLightGreen, draw.context );
		}

		if ( joint.maxLength < B2_HUGE )
		{
			// draw->DrawPoint(pMax, 4.0f, c3, draw->context);
			draw.DrawSegmentFcn( pMax - offset, pMax + offset, b2_colorRed, draw.context );
		}

		if ( joint.minLength > B2_LINEAR_SLOP && joint.maxLength < B2_HUGE )
		{
			draw.DrawSegmentFcn( pMin, pMax, b2_colorGray, draw.context );
		}
	}

	draw.DrawSegmentFcn( pA, pB, b2_colorWhite, draw.context );
	draw.DrawPointFcn( pA, 4.0f, b2_colorWhite, draw.context );
	draw.DrawPointFcn( pB, 4.0f, b2_colorWhite, draw.context );

	if ( joint.hertz > 0.0f && joint.enableSpring )
	{
		b2Vec2 pRest = b2MulAdd( pA, joint.length, axis );
		draw.DrawPointFcn( pRest, 4.0f, b2_colorBlue, draw.context );
	}
}
