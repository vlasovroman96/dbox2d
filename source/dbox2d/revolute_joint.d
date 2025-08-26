module dbox2d.revolute_joint;

import std.format;

private template HasVersion(string versionId) {
	mixin("version("~versionId~") {enum HasVersion = true;} else {enum HasVersion = false;}");
}

static if (HasVersion!"_MSC_VER" && !HasVersion!"_CRT_SECURE_NO_WARNINGS") {
version = _CRT_SECURE_NO_WARNINGS;
}

mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2Body", "b2Body"));

import dbox2d.types;
import dbox2d.math;
import dbox2d.body;
import dbox2d.id;
import dbox2d.core;
import dbox2d.joint;
import dbox2d.physics_world;
import dbox2d.solver;
import dbox2d.solver_set;
import dbox2d.array;
// import dbox2d.box2d;
import dbox2d.base;

mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));

// Point-to-point constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// J = [-E -skew(rA) E skew(rB) ]

// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
// K = invIA + invIB

void b2RevoluteJoint_EnableSpring(b2JointId jointId, bool enableSpring)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableSpring != joint.revoluteJoint.enableSpring )
	{
		joint.revoluteJoint.enableSpring = enableSpring;
		joint.revoluteJoint.springImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsSpringEnabled(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.enableSpring;
}

void b2RevoluteJoint_SetSpringHertz(b2JointId jointId, float hertz)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint.revoluteJoint.hertz = hertz;
}

float b2RevoluteJoint_GetSpringHertz(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.hertz;
}

void b2RevoluteJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint.revoluteJoint.dampingRatio = dampingRatio;
}

float b2RevoluteJoint_GetSpringDampingRatio(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.dampingRatio;
}

void b2RevoluteJoint_SetTargetAngle(b2JointId jointId, float angle)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint.revoluteJoint.targetAngle = angle;
}

float b2RevoluteJoint_GetTargetAngle(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.targetAngle;
}

float b2RevoluteJoint_GetAngle(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	b2Transform transformA = b2GetBodyTransform( world, jointSim.bodyIdA );
	b2Transform transformB = b2GetBodyTransform( world, jointSim.bodyIdB );
	b2Rot qA = b2MulRot( transformA.q, jointSim.localFrameA.q );
	b2Rot qB = b2MulRot( transformB.q, jointSim.localFrameB.q );

	float angle = b2RelativeAngle( qA, qB );
	return angle;
}

void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableLimit != joint.revoluteJoint.enableLimit )
	{
		joint.revoluteJoint.enableLimit = enableLimit;
		joint.revoluteJoint.lowerImpulse = 0.0f;
		joint.revoluteJoint.upperImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsLimitEnabled(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.enableLimit;
}

float b2RevoluteJoint_GetLowerLimit(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.lowerAngle;
}

float b2RevoluteJoint_GetUpperLimit(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.upperAngle;
}

void b2RevoluteJoint_SetLimits(b2JointId jointId, float lower, float upper)
{
	B2_ASSERT( lower <= upper );
	B2_ASSERT( lower >= -0.99f * PI );
	B2_ASSERT( upper <= 0.99f * PI );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( lower != joint.revoluteJoint.lowerAngle || upper != joint.revoluteJoint.upperAngle )
	{
		joint.revoluteJoint.lowerAngle = min( lower, upper );
		joint.revoluteJoint.upperAngle = max( lower, upper );
		joint.revoluteJoint.lowerImpulse = 0.0f;
		joint.revoluteJoint.upperImpulse = 0.0f;
	}
}

void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableMotor != joint.revoluteJoint.enableMotor )
	{
		joint.revoluteJoint.enableMotor = enableMotor;
		joint.revoluteJoint.motorImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsMotorEnabled(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.enableMotor;
}

void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint.revoluteJoint.motorSpeed = motorSpeed;
}

float b2RevoluteJoint_GetMotorSpeed(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.motorSpeed;
}

float b2RevoluteJoint_GetMotorTorque(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return world.inv_h * joint.revoluteJoint.motorImpulse;
}

void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint.revoluteJoint.maxMotorTorque = torque;
}

float b2RevoluteJoint_GetMaxMotorTorque(b2JointId jointId)
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint.revoluteJoint.maxMotorTorque;
}

b2Vec2 b2GetRevoluteJointForce(b2World* world, b2JointSim* base)
{
	b2Vec2 force = b2MulSV( world.inv_h, base.revoluteJoint.linearImpulse );
	return force;
}

float b2GetRevoluteJointTorque(b2World* world, b2JointSim* base)
{
	const(b2RevoluteJoint)* revolute = &base.revoluteJoint;
	float torque = world.inv_h * ( revolute.motorImpulse + revolute.lowerImpulse - revolute.upperImpulse );
	return torque;
}

void b2PrepareRevoluteJoint(b2JointSim* base, b2StepContext* context)
{
	B2_ASSERT( base.type == b2_revoluteJoint );

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

	b2RevoluteJoint* joint = &base.revoluteJoint;

	joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass.
	// Avoid round-off here as much as possible.
	// b2Vec2 pf = (xf.p - c) + rot(xf.q, f.p)
	// pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
	// pf = rot(xf.q, f.p - lc)
	joint.frameA.q = b2MulRot( bodySimA.transform.q, base.localFrameA.q );
	joint.frameA.p = b2RotateVector( bodySimA.transform.q, base.localFrameA.p - bodySimA.localCenter );
	joint.frameB.q = b2MulRot( bodySimB.transform.q, base.localFrameB.q );
	joint.frameB.p = b2RotateVector( bodySimB.transform.q, base.localFrameB.p - bodySimB.localCenter );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = bodySimB.center - bodySimA.center;

	float k = iA + iB;
	joint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint.springSoftness = b2MakeSoft( joint.hertz, joint.dampingRatio, context.h );

	if ( context.enableWarmStarting == false )
	{
		joint.linearImpulse = b2Vec2.zero();
		joint.springImpulse = 0.0f;
		joint.motorImpulse = 0.0f;
		joint.lowerImpulse = 0.0f;
		joint.upperImpulse = 0.0f;
	}
}

void b2WarmStartRevoluteJoint(b2JointSim* base, b2StepContext* context)
{
	B2_ASSERT( base.type == b2_revoluteJoint );

	float mA = base.invMassA;
	float mB = base.invMassB;
	float iA = base.invIA;
	float iB = base.invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base.revoluteJoint;
	b2BodyState* stateA = joint.indexA == B2_NULL_INDEX ? &dummyState : context.states + joint.indexA;
	b2BodyState* stateB = joint.indexB == B2_NULL_INDEX ? &dummyState : context.states + joint.indexB;

	b2Vec2 rA = b2RotateVector( stateA.deltaRotation, joint.frameA.p );
	b2Vec2 rB = b2RotateVector( stateB.deltaRotation, joint.frameB.p );

	float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	stateA.linearVelocity = b2MulSub( stateA.linearVelocity, mA, joint.linearImpulse );
	stateA.angularVelocity -= iA * ( rA.cross( joint.linearImpulse ) + axialImpulse );

	stateB.linearVelocity = b2MulAdd( stateB.linearVelocity, mB, joint.linearImpulse );
	stateB.angularVelocity += iB * ( rB.cross( joint.linearImpulse ) + axialImpulse );
}

void b2SolveRevoluteJoint(b2JointSim* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT( base.type == b2_revoluteJoint );

	float mA = base.invMassA;
	float mB = base.invMassB;
	float iA = base.invIA;
	float iB = base.invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base.revoluteJoint;

	b2BodyState* stateA = joint.indexA == B2_NULL_INDEX ? &dummyState : context.states + joint.indexA;
	b2BodyState* stateB = joint.indexB == B2_NULL_INDEX ? &dummyState : context.states + joint.indexB;

	b2Vec2 vA = stateA.linearVelocity;
	float wA = stateA.angularVelocity;
	b2Vec2 vB = stateB.linearVelocity;
	float wB = stateB.angularVelocity;

	b2Rot qA = b2MulRot( stateA.deltaRotation, joint.frameA.q );
	b2Rot qB = b2MulRot( stateB.deltaRotation, joint.frameB.q );
	b2Rot relQ = b2InvMulRot( qA, qB );

	bool fixedRotation = ( iA + iB == 0.0f );

	// Solve spring.
	if ( joint.enableSpring && fixedRotation == false )
	{
		float jointAngle = b2Rot_GetAngle( relQ );
		float jointAngleDelta = b2UnwindAngle( jointAngle - joint.targetAngle );

		float C = jointAngleDelta;
		float bias = joint.springSoftness.biasRate * C;
		float massScale = joint.springSoftness.massScale;
		float impulseScale = joint.springSoftness.impulseScale;

		float Cdot = wB - wA;
		float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
		joint.springImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve motor constraint.
	if ( joint.enableMotor && fixedRotation == false )
	{
		float Cdot = wB - wA - joint.motorSpeed;
		float impulse = -joint.axialMass * Cdot;
		float oldImpulse = joint.motorImpulse;
		float maxImpulse = context.h * joint.maxMotorTorque;
		joint.motorImpulse = clamp( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint.motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if ( joint.enableLimit && fixedRotation == false )
	{
		float jointAngle = b2Rot_GetAngle( relQ );

		// Lower limit
		{
			float C = jointAngle - joint.lowerAngle;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( C > 0.0f )
			{
				// speculation
				bias = C * context.inv_h;
			}
			else if ( useBias )
			{
				bias = base.constraintSoftness.biasRate * C;
				massScale = base.constraintSoftness.massScale;
				impulseScale = base.constraintSoftness.impulseScale;
			}

			float Cdot = wB - wA;
			float oldImpulse = joint.lowerImpulse;
			float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * oldImpulse;
			joint.lowerImpulse = max( oldImpulse + impulse, 0.0f );
			impulse = joint.lowerImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			float C = joint.upperAngle - jointAngle;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( C > 0.0f )
			{
				// speculation
				bias = C * context.inv_h;
			}
			else if ( useBias )
			{
				bias = base.constraintSoftness.biasRate * C;
				massScale = base.constraintSoftness.massScale;
				impulseScale = base.constraintSoftness.impulseScale;
			}

			// sign flipped on Cdot
			float Cdot = wA - wB;
			float oldImpulse = joint.upperImpulse;
			float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * oldImpulse;
			joint.upperImpulse = max( oldImpulse + impulse, 0.0f );
			impulse = joint.upperImpulse - oldImpulse;

			// sign flipped on applied impulse
			wA += iA * impulse;
			wB -= iB * impulse;
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		// current anchors
		b2Vec2 rA = b2RotateVector( stateA.deltaRotation, joint.frameA.p );
		b2Vec2 rB = b2RotateVector( stateB.deltaRotation, joint.frameB.p );

		b2Vec2 Cdot = (vB + b2CrossSV( wB, rB )) - (vA + b2CrossSV( wA, rA));

		b2Vec2 bias = b2Vec2.zero();
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 dcA = stateA.deltaPosition;
			b2Vec2 dcB = stateB.deltaPosition;

			b2Vec2 separation = (( dcB - dcA ) + ( rB - rA ) ) + joint.deltaCenter;
			bias = b2MulSV( base.constraintSoftness.biasRate, separation );
			massScale = base.constraintSoftness.massScale;
			impulseScale = base.constraintSoftness.impulseScale;
		}

		b2Mat22 K = void;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22( K, Cdot + bias );

		b2Vec2 impulse = void;
		impulse.x = -massScale * b.x - impulseScale * joint.linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint.linearImpulse.y;
		joint.linearImpulse.x += impulse.x;
		joint.linearImpulse.y += impulse.y;

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * rA.cross( impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * rB.cross( impulse );
	}

	stateA.linearVelocity = vA;
	stateA.angularVelocity = wA;
	stateB.linearVelocity = vB;
	stateB.angularVelocity = wB;
}

// version (none) {
// void b2RevoluteJoint:: Dump()
// {
// 	int indexA = joint.bodyA.joint.islandIndex;
// 	int indexB = joint.bodyB.joint.islandIndex;

// 	b2Dump("  b2RevoluteJointDef jd;\n");
// 	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
// 	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
// 	b2Dump("  jd.collideConnected = bool(%d);\n", joint.collideConnected);
// 	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", joint.localAnchorA.x, joint.localAnchorA.y);
// 	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", joint.localAnchorB.x, joint.localAnchorB.y);
// 	b2Dump("  jd.referenceAngle = %.9g;\n", joint.referenceAngle);
// 	b2Dump("  jd.enableLimit = bool(%d);\n", joint.enableLimit);
// 	b2Dump("  jd.lowerAngle = %.9g;\n", joint.lowerAngle);
// 	b2Dump("  jd.upperAngle = %.9g;\n", joint.upperAngle);
// 	b2Dump("  jd.enableMotor = bool(%d);\n", joint.enableMotor);
// 	b2Dump("  jd.motorSpeed = %.9g;\n", joint.motorSpeed);
// 	b2Dump("  jd.maxMotorTorque = %.9g;\n", joint.maxMotorTorque);
// 	b2Dump("  joints[%d] = joint->world->CreateJoint(&jd);\n", joint.index);
// }
// }

void b2DrawRevoluteJoint(b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB, float drawSize)
{
	B2_ASSERT( base.type == b2_revoluteJoint );

	b2RevoluteJoint* joint = &base.revoluteJoint;

	b2Transform frameA = b2MulTransforms( transformA, base.localFrameA );
	b2Transform frameB = b2MulTransforms( transformB, base.localFrameB );

	const(float) radius = 0.25f * drawSize;
	draw.DrawCircleFcn( frameB.p, radius, b2_colorGray, draw.context );

	b2Vec2 rx = { radius, 0.0f };
	b2Vec2 r = b2RotateVector( frameA.q, rx );
	draw.DrawSegmentFcn( frameA.p, frameA.p + r, b2_colorGray, draw.context );

	r = b2RotateVector( frameB.q, rx );
	draw.DrawSegmentFcn( frameB.p, frameB.p + r, b2_colorBlue, draw.context );

	if ( draw.drawJointExtras )
	{
		float jointAngle = b2RelativeAngle( frameA.q, frameB.q );
		char[32] buffer = void;
		buffer.format(32 ~ " %.1f deg", 180.0f * jointAngle / PI );
		draw.DrawStringFcn( frameA.p + r, buffer.ptr, b2_colorWhite, draw.context );
	}

	float lowerAngle = joint.lowerAngle;
	float upperAngle = joint.upperAngle;

	if ( joint.enableLimit )
	{
		b2Rot rotLo = b2MulRot( frameA.q, b2Rot( lowerAngle ) );
		b2Vec2 rlo = b2RotateVector( rotLo, rx );

		b2Rot rotHi = b2MulRot( frameA.q, b2Rot( upperAngle ) );
		b2Vec2 rhi = b2RotateVector( rotHi, rx );

		draw.DrawSegmentFcn( frameB.p, frameB.p + rlo, b2_colorGreen, draw.context );
		draw.DrawSegmentFcn( frameB.p, frameB.p + rhi, b2_colorRed, draw.context );
	}

	if ( joint.enableSpring )
	{
		b2Rot q = b2MulRot( frameA.q, b2Rot( joint.targetAngle ) );
		b2Vec2 v = b2RotateVector( q, rx );
		draw.DrawSegmentFcn( frameB.p, frameB.p + v, b2_colorViolet, draw.context );
	}

	b2HexColor color = b2_colorGold;
	draw.DrawSegmentFcn( transformA.p, frameA.p, color, draw.context );
	draw.DrawSegmentFcn( frameA.p, frameB.p, color, draw.context );
	draw.DrawSegmentFcn( transformB.p, frameB.p, color, draw.context );

	// char buffer[32];
	// sprintf(buffer, "%.1f", b2Length(joint->impulse));
	// draw->DrawString(pA, buffer, draw->context);
}
