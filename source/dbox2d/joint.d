module dbox2d.joint;

public import dbox2d.array;
public import dbox2d.solver;
import dbox2d.body;

public import dbox2d.types;
import dbox2d.physics_world;
import dbox2d.contact;
import dbox2d.shape;
import dbox2d.island;
import dbox2d.solver_set;
import std.math;
import dbox2d.prismatic_joint;
import dbox2d.revolute_joint;
import dbox2d.weld_joint;
import dbox2d.wheel_joint;
import dbox2d.distance_joint;
import dbox2d.motor_joint;
import dbox2d.mouse_joint;

import core.stdc.stdio;

mixin(B2_ARRAY_SOURCE!("b2Body", "b2Body"));
mixin(B2_ARRAY_SOURCE!("b2Int", "int"));
mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2Shape", "b2Shape"));
mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2ContactHitEvent","b2ContactHitEvent"));
mixin(B2_ARRAY_SOURCE!("b2ContactBeginTouchEvent","b2ContactBeginTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2ContactEndTouchEvent","b2ContactEndTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2ContactSim","b2ContactSim"));
mixin(B2_ARRAY_SOURCE!("b2SensorBeginTouchEvent","b2SensorBeginTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2SensorEndTouchEvent","b2SensorEndTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2TaskContext","b2TaskContext"));
mixin(B2_ARRAY_SOURCE!("b2JointEvent","b2JointEvent"));
mixin(B2_ARRAY_SOURCE!("b2BodyMoveEvent","b2BodyMoveEvent"));
mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));
mixin(B2_ARRAY_SOURCE!("b2Joint","b2Joint"));
mixin(B2_ARRAY_SOURCE!("b2Contact","b2Contact"));

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
struct b2JointEdge {
	int bodyId;
	int prevKey;
	int nextKey;
}

// Map from b2JointId to b2Joint in the solver sets
struct b2Joint {
	void* userData;

	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
	// B2_NULL_INDEX when slot is free
	int colorIndex;

	// joint index within set or graph color
	// B2_NULL_INDEX when slot is free
	int localIndex;

	b2JointEdge[2] edges;

	int jointId;
	int islandId;
	int islandPrev;
	int islandNext;

	float drawScale = 0;

	b2JointType type;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2JointId
	ushort generation;

	bool isMarked;
	bool collideConnected;
}

struct b2DistanceJoint {
	float length = 0;
	float hertz = 0;
	float dampingRatio = 0;
	float lowerSpringForce = 0;
	float upperSpringForce = 0;
	float minLength = 0;
	float maxLength = 0;

	float maxMotorForce = 0;
	float motorSpeed = 0;

	float impulse = 0;
	float lowerImpulse = 0;
	float upperImpulse = 0;
	float motorImpulse = 0;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	b2Softness distanceSoftness;
	float axialMass = 0;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
}

struct b2MotorJoint {
	b2Vec2 linearVelocity;
	float maxVelocityForce = 0;
	float angularVelocity = 0;
	float maxVelocityTorque = 0;
	float linearHertz = 0;
	float linearDampingRatio = 0;
	float maxSpringForce = 0;
	float angularHertz = 0;
	float angularDampingRatio = 0;
	float maxSpringTorque = 0;

	b2Vec2 linearVelocityImpulse;
	float angularVelocityImpulse = 0;
	b2Vec2 linearSpringImpulse;
	float angularSpringImpulse = 0;

	b2Softness linearSpring;
	b2Softness angularSpring;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	b2Mat22 linearMass;
	float angularMass = 0;
}

struct b2MouseJoint {
	float hertz = 0;
	float dampingRatio = 0;
	float maxForce = 0;

	b2Vec2 linearImpulse;
	float angularImpulse = 0;

	b2Softness linearSoftness;
	b2Softness angularSoftness;
	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	b2Mat22 linearMass;
	float angularMass = 0;
}

struct b2PrismaticJoint {
	b2Vec2 impulse;
	float springImpulse = 0;
	float motorImpulse = 0;
	float lowerImpulse = 0;
	float upperImpulse = 0;
	float hertz = 0;
	float dampingRatio = 0;
	float targetTranslation = 0;
	float maxMotorForce = 0;
	float motorSpeed = 0;
	float lowerTranslation = 0;
	float upperTranslation = 0;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float axialMass = 0;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
}

struct b2RevoluteJoint {
	b2Vec2 linearImpulse;
	float springImpulse = 0;
	float motorImpulse = 0;
	float lowerImpulse = 0;
	float upperImpulse = 0;
	float hertz = 0;
	float dampingRatio = 0;
	float targetAngle = 0;
	float maxMotorTorque = 0;
	float motorSpeed = 0;
	float lowerAngle = 0;
	float upperAngle = 0;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float axialMass = 0;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
}

struct b2WeldJoint {
	float linearHertz = 0;
	float linearDampingRatio = 0;
	float angularHertz = 0;
	float angularDampingRatio = 0;

	b2Softness linearSpring;
	b2Softness angularSpring;
	b2Vec2 linearImpulse;
	float angularImpulse = 0;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float axialMass = 0;
}

struct b2WheelJoint {
	float perpImpulse = 0;
	float motorImpulse = 0;
	float springImpulse = 0;
	float lowerImpulse = 0;
	float upperImpulse = 0;
	float maxMotorTorque = 0;
	float motorSpeed = 0;
	float lowerTranslation = 0;
	float upperTranslation = 0;
	float hertz = 0;
	float dampingRatio = 0;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float perpMass = 0;
	float motorMass = 0;
	float axialMass = 0;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
struct b2JointSim {
	int jointId;

	int bodyIdA;
	int bodyIdB;

	b2JointType type;

	b2Transform localFrameA;
	b2Transform localFrameB;

	float invMassA = 0, invMassB = 0;
	float invIA = 0, invIB = 0;

	float constraintHertz = 0;
	float constraintDampingRatio = 0;

	b2Softness constraintSoftness;

	float forceThreshold = 0;
	float torqueThreshold = 0;

	union  {
		b2DistanceJoint distanceJoint;
		b2MotorJoint motorJoint;
		b2MouseJoint mouseJoint;
		b2RevoluteJoint revoluteJoint;
		b2PrismaticJoint prismaticJoint;
		b2WeldJoint weldJoint;
		b2WheelJoint wheelJoint;
	};
}

private b2JointDef b2DefaultJointDef()
{
	b2JointDef def;
	def.localFrameA.q = b2Rot.identity();
	def.localFrameB.q = b2Rot.identity();
	def.forceThreshold = float.max;
	def.torqueThreshold = float.max;
	def.constraintHertz = 60.0f;
	def.constraintDampingRatio = 0.0f;
	def.drawScale = 1.0f;
	return def;
}

b2DistanceJointDef b2DefaultDistanceJointDef()
{
	b2DistanceJointDef def;
	def.base = b2DefaultJointDef();
	def.lowerSpringForce = -float.max;
	def.upperSpringForce = float.max;
	def.length = 1.0f;
	def.maxLength = B2_HUGE;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2MotorJointDef b2DefaultMotorJointDef()
{
	b2MotorJointDef def;
	def.base = b2DefaultJointDef();
	def.relativeTransform.q = b2Rot.identity();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2MouseJointDef b2DefaultMouseJointDef()
{
	b2MouseJointDef def;
	def.base = b2DefaultJointDef();
	def.hertz = 4.0f;
	def.dampingRatio = 1.0f;
	def.maxForce = 1.0f;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2FilterJointDef b2DefaultFilterJointDef()
{
	b2FilterJointDef def;
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2PrismaticJointDef b2DefaultPrismaticJointDef()
{
	b2PrismaticJointDef def;
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2RevoluteJointDef b2DefaultRevoluteJointDef()
{
	b2RevoluteJointDef def;
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2WeldJointDef b2DefaultWeldJointDef()
{
	b2WeldJointDef def;
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2WheelJointDef b2DefaultWheelJointDef()
{
	b2WheelJointDef def;
	def.base = b2DefaultJointDef();
	def.enableSpring = true;
	def.hertz = 1.0f;
	def.dampingRatio = 0.7f;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2ExplosionDef b2DefaultExplosionDef()
{
	b2ExplosionDef def;
	def.maskBits = B2_DEFAULT_MASK_BITS;
	return def;
}

b2Joint* b2GetJointFullId(b2World* world, b2JointId jointId)
{
	int id = jointId.index1 - 1;
	b2Joint* joint = b2JointArray_Get( world.joints, id );
	B2_ASSERT( joint.jointId == id && joint.generation == jointId.generation );
	return joint;
}

b2JointSim* b2GetJointSim(b2World* world, b2Joint* joint)
{
	if ( joint.setIndex == b2_awakeSet )
	{
		B2_ASSERT( 0 <= joint.colorIndex && joint.colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = world.constraintGraph.colors.ptr + joint.colorIndex;
		return b2JointSimArray_Get( color.jointSims, joint.localIndex );
	}

	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, joint.setIndex );
	return b2JointSimArray_Get( set.jointSims, joint.localIndex );
}

b2JointSim* b2GetJointSimCheckType(b2JointId jointId, b2JointType type)
{
	// B2_UNUSED( type );

	b2World* world = b2GetWorld( jointId.world0 );
	B2_ASSERT( world.locked == false );
	if ( world.locked )
	{
		return null;
	}

	b2Joint* joint = b2GetJointFullId( world, jointId );
	B2_ASSERT( joint.type == type );
	b2JointSim* jointSim = b2GetJointSim( world, joint );
	B2_ASSERT( jointSim.type == type );
	return jointSim;
}

private void b2DestroyContactsBetweenBodies(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int contactKey = void;
	int otherBodyId = void;

	// use the smaller of the two contact lists
	if ( bodyA.contactCount < bodyB.contactCount )
	{
		contactKey = bodyA.headContactKey;
		otherBodyId = bodyB.id;
	}
	else
	{
		contactKey = bodyB.headContactKey;
		otherBodyId = bodyA.id;
	}

	// no need to wake bodies when a joint removes collision between them
	bool wakeBodies = false;

	// destroy the contacts
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );
		contactKey = contact.edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		if ( contact.edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			// Careful, this removes the contact from the current doubly linked list
			b2DestroyContact( world, contact, wakeBodies );
		}
	}

	b2ValidateSolverSets( world );
}

struct b2JointPair {
	b2Joint* joint;
	b2JointSim* jointSim;
}

private b2JointPair b2CreateJoint(b2World* world, const(b2JointDef)* def, b2JointType type)
{
	B2_ASSERT( b2IsValidTransform( def.localFrameA ) );
	B2_ASSERT( b2IsValidTransform( def.localFrameB ) );
	B2_ASSERT( world.worldId == def.bodyIdA.world0 );
	B2_ASSERT( world.worldId == def.bodyIdB.world0 );
	B2_ASSERT( B2_ID_EQUALS( def.bodyIdA, def.bodyIdB ) == false );

	b2Body* bodyA = b2GetBodyFullId( world, def.bodyIdA );
	b2Body* bodyB = b2GetBodyFullId( world, def.bodyIdB );

	int bodyIdA = bodyA.id;
	int bodyIdB = bodyB.id;
	int maxSetIndex = max( bodyA.setIndex, bodyB.setIndex );

	// Create joint id and joint
	int jointId = b2AllocId( &world.jointIdPool );
	if ( jointId == cast(int)world.joints.length )
	{
		b2JointArray_Push( world.joints, b2Joint() );
	}

	b2Joint* joint = b2JointArray_Get( world.joints, jointId );
	joint.jointId = jointId;
	joint.userData = cast(void*)def.userData;
	joint.generation += 1;
	joint.setIndex = B2_NULL_INDEX;
	joint.colorIndex = B2_NULL_INDEX;
	joint.localIndex = B2_NULL_INDEX;
	joint.islandId = B2_NULL_INDEX;
	joint.islandPrev = B2_NULL_INDEX;
	joint.islandNext = B2_NULL_INDEX;
	joint.drawScale = def.drawScale;
	joint.type = type;
	joint.collideConnected = def.collideConnected;
	joint.isMarked = false;

	// Doubly linked list on bodyA
	joint.edges[0].bodyId = bodyIdA;
	joint.edges[0].prevKey = B2_NULL_INDEX;
	joint.edges[0].nextKey = bodyA.headJointKey;

	int keyA = ( jointId << 1 ) | 0;
	if ( bodyA.headJointKey != B2_NULL_INDEX )
	{
		b2Joint* jointA = b2JointArray_Get( world.joints, bodyA.headJointKey >> 1 );
		b2JointEdge* edgeA = jointA.edges.ptr + ( bodyA.headJointKey & 1 );
		edgeA.prevKey = keyA;
	}
	bodyA.headJointKey = keyA;
	bodyA.jointCount += 1;

	// Doubly linked list on bodyB
	joint.edges[1].bodyId = bodyIdB;
	joint.edges[1].prevKey = B2_NULL_INDEX;
	joint.edges[1].nextKey = bodyB.headJointKey;

	int keyB = ( jointId << 1 ) | 1;
	if ( bodyB.headJointKey != B2_NULL_INDEX )
	{
		b2Joint* jointB = b2JointArray_Get( world.joints, bodyB.headJointKey >> 1 );
		b2JointEdge* edgeB = jointB.edges.ptr + ( bodyB.headJointKey & 1 );
		edgeB.prevKey = keyB;
	}
	bodyB.headJointKey = keyB;
	bodyB.jointCount += 1;

	b2JointSim* jointSim = void;

	if ( bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet )
	{
		// if either body is disabled, create in disabled set
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_disabledSet );
		joint.setIndex = b2_disabledSet;
		joint.localIndex = cast(int)set.jointSims.count;

		jointSim = b2JointSimArray_Add( set.jointSims );
		memset( jointSim, 0, b2JointSim.sizeof );

		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;
	}
	else if ( bodyA.setIndex == b2_staticSet && bodyB.setIndex == b2_staticSet )
	{
		// joint is connecting static bodies
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_staticSet );
		joint.setIndex = b2_staticSet;
		joint.localIndex = cast(int)set.jointSims.count;

		jointSim = b2JointSimArray_Add( set.jointSims );
		memset( jointSim, 0, b2JointSim.sizeof );

		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;
	}
	else if ( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet )
	{
		// if either body is sleeping, wake it
		if ( maxSetIndex >= b2_firstSleepingSet )
		{
			b2WakeSolverSet( world, maxSetIndex );
		}

		joint.setIndex = b2_awakeSet;

		jointSim = b2CreateJointInGraph( world, joint );
		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;
	}
	else
	{
		// joint connected between sleeping and/or static bodies
		B2_ASSERT( bodyA.setIndex >= b2_firstSleepingSet || bodyB.setIndex >= b2_firstSleepingSet );
		B2_ASSERT( bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet );

		// joint should go into the sleeping set (not static set)
		int setIndex = maxSetIndex;

		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setIndex );
		joint.setIndex = setIndex;
		joint.localIndex = cast(int)set.jointSims.count;

		jointSim = b2JointSimArray_Add( set.jointSims );
		memset( jointSim, 0, b2JointSim.sizeof );

		// These must be set to accommodate the merge below
		jointSim.jointId = jointId;
		jointSim.bodyIdA = bodyIdA;
		jointSim.bodyIdB = bodyIdB;

		if ( bodyA.setIndex != bodyB.setIndex && bodyA.setIndex >= b2_firstSleepingSet &&
			 bodyB.setIndex >= b2_firstSleepingSet )
		{
			// merge sleeping sets
			b2MergeSolverSets( world, bodyA.setIndex, bodyB.setIndex );
			B2_ASSERT( bodyA.setIndex == bodyB.setIndex );

			// fix potentially invalid set index
			setIndex = bodyA.setIndex;

			b2SolverSet* mergedSet = b2SolverSetArray_Get( world.solverSets, setIndex );

			// Careful! The joint sim pointer was orphaned by the set merge.
			jointSim = b2JointSimArray_Get( mergedSet.jointSims, joint.localIndex );
		}

		B2_ASSERT( joint.setIndex == setIndex );
	}

	jointSim.localFrameA = def.localFrameA;
	jointSim.localFrameB = def.localFrameB;
	jointSim.type = type;
	jointSim.constraintHertz = def.constraintHertz;
	jointSim.constraintDampingRatio = def.constraintDampingRatio;
	jointSim.constraintSoftness = b2Softness(
		biasRate: 0.0f,
		massScale: 1.0f,
		impulseScale: 0.0f,
	);

	B2_ASSERT( b2IsValidFloat( def.forceThreshold ) && def.forceThreshold >= 0.0f );
	B2_ASSERT( b2IsValidFloat( def.torqueThreshold ) && def.torqueThreshold >= 0.0f );

	jointSim.forceThreshold = def.forceThreshold;
	jointSim.torqueThreshold = def.torqueThreshold;

	B2_ASSERT( jointSim.jointId == jointId );
	B2_ASSERT( jointSim.bodyIdA == bodyIdA );
	B2_ASSERT( jointSim.bodyIdB == bodyIdB );

	if ( joint.setIndex > b2_disabledSet )
	{
		// Add edge to island graph
		b2LinkJoint( world, joint );
	}

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if ( def.collideConnected == false )
	{
		b2DestroyContactsBetweenBodies( world, bodyA, bodyB );
	}

	b2ValidateSolverSets( world );

	return b2JointPair( joint, jointSim );
}

b2JointId b2CreateDistanceJoint(b2WorldId worldId, const(b2DistanceJointDef)* def)
{
	B2_CHECK_DEF( def );
	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	B2_ASSERT( b2IsValidFloat( def.length ) && def.length > 0.0f );
	B2_ASSERT( def.lowerSpringForce <= def.upperSpringForce );

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_distanceJoint );

	b2JointSim* joint = pair.jointSim;

	b2DistanceJoint empty;
	joint.distanceJoint = empty;
	joint.distanceJoint.length = b2MaxFloat( def.length, B2_LINEAR_SLOP );
	joint.distanceJoint.hertz = def.hertz;
	joint.distanceJoint.dampingRatio = def.dampingRatio;
	joint.distanceJoint.minLength = b2MaxFloat( def.minLength, B2_LINEAR_SLOP );
	joint.distanceJoint.maxLength = b2MaxFloat( def.minLength, def.maxLength );
	joint.distanceJoint.maxMotorForce = def.maxMotorForce;
	joint.distanceJoint.motorSpeed = def.motorSpeed;
	joint.distanceJoint.enableSpring = def.enableSpring;
	joint.distanceJoint.lowerSpringForce = def.lowerSpringForce;
	joint.distanceJoint.upperSpringForce = def.upperSpringForce;
	joint.distanceJoint.enableLimit = def.enableLimit;
	joint.distanceJoint.enableMotor = def.enableMotor;
	joint.distanceJoint.impulse = 0.0f;
	joint.distanceJoint.lowerImpulse = 0.0f;
	joint.distanceJoint.upperImpulse = 0.0f;
	joint.distanceJoint.motorImpulse = 0.0f;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreateMotorJoint(b2WorldId worldId, const(b2MotorJointDef)* def)
{
	B2_CHECK_DEF( def );
	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_motorJoint );
	b2JointSim* joint = pair.jointSim;

	joint.motorJoint = b2MotorJoint();
	joint.motorJoint.linearVelocity = def.linearVelocity;
	joint.motorJoint.maxVelocityForce = def.maxVelocityForce;
	joint.motorJoint.angularVelocity = def.angularVelocity;
	joint.motorJoint.maxVelocityTorque = def.maxVelocityTorque;
	joint.motorJoint.linearHertz = def.linearHertz;
	joint.motorJoint.linearDampingRatio = def.linearDampingRatio;
	joint.motorJoint.maxSpringForce = def.maxSpringForce;
	joint.motorJoint.angularHertz = def.angularHertz;
	joint.motorJoint.angularDampingRatio = def.angularDampingRatio;
	joint.motorJoint.maxSpringTorque = def.maxSpringTorque;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreateMouseJoint(b2WorldId worldId, const(b2MouseJointDef)* def)
{
	B2_CHECK_DEF( def );
	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_mouseJoint );

	b2JointSim* joint = pair.jointSim;

	b2MouseJoint empty;
	joint.mouseJoint = empty;
	joint.mouseJoint.hertz = def.hertz;
	joint.mouseJoint.dampingRatio = def.dampingRatio;
	joint.mouseJoint.maxForce = def.maxForce;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreateFilterJoint(b2WorldId worldId, const(b2FilterJointDef)* def)
{
	B2_CHECK_DEF( def );
	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_filterJoint );

	b2JointSim* joint = pair.jointSim;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreatePrismaticJoint(b2WorldId worldId, const(b2PrismaticJointDef)* def)
{
	B2_CHECK_DEF( def );
	B2_ASSERT( def.lowerTranslation <= def.upperTranslation );

	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_prismaticJoint );

	b2JointSim* joint = pair.jointSim;


	joint.prismaticJoint = b2PrismaticJoint();
	joint.prismaticJoint.hertz = def.hertz;
	joint.prismaticJoint.dampingRatio = def.dampingRatio;
	joint.prismaticJoint.targetTranslation = def.targetTranslation;
	joint.prismaticJoint.lowerTranslation = def.lowerTranslation;
	joint.prismaticJoint.upperTranslation = def.upperTranslation;
	joint.prismaticJoint.maxMotorForce = def.maxMotorForce;
	joint.prismaticJoint.motorSpeed = def.motorSpeed;
	joint.prismaticJoint.enableSpring = def.enableSpring;
	joint.prismaticJoint.enableLimit = def.enableLimit;
	joint.prismaticJoint.enableMotor = def.enableMotor;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreateRevoluteJoint(b2WorldId worldId, const(b2RevoluteJointDef)* def)
{
	B2_CHECK_DEF( def );
	B2_ASSERT( def.lowerAngle <= def.upperAngle );
	B2_ASSERT( def.lowerAngle >= -0.99f * PI );
	B2_ASSERT( def.upperAngle <= 0.99f * PI );

	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_revoluteJoint );

	b2JointSim* joint = pair.jointSim;

	b2RevoluteJoint empty;
	joint.revoluteJoint = empty;

	joint.revoluteJoint.targetAngle = b2ClampFloat( def.targetAngle, -PI, PI );
	joint.revoluteJoint.hertz = def.hertz;
	joint.revoluteJoint.dampingRatio = def.dampingRatio;
	joint.revoluteJoint.lowerAngle = def.lowerAngle;
	joint.revoluteJoint.upperAngle = def.upperAngle;
	joint.revoluteJoint.maxMotorTorque = def.maxMotorTorque;
	joint.revoluteJoint.motorSpeed = def.motorSpeed;
	joint.revoluteJoint.enableSpring = def.enableSpring;
	joint.revoluteJoint.enableLimit = def.enableLimit;
	joint.revoluteJoint.enableMotor = def.enableMotor;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreateWeldJoint(b2WorldId worldId, const(b2WeldJointDef)* def)
{
	B2_CHECK_DEF( def );
	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_weldJoint );

	b2JointSim* joint = pair.jointSim;

	b2WeldJoint empty;
	joint.weldJoint = empty;
	joint.weldJoint.linearHertz = def.linearHertz;
	joint.weldJoint.linearDampingRatio = def.linearDampingRatio;
	joint.weldJoint.angularHertz = def.angularHertz;
	joint.weldJoint.angularDampingRatio = def.angularDampingRatio;
	joint.weldJoint.linearImpulse = b2Vec2.zero;
	joint.weldJoint.angularImpulse = 0.0f;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

b2JointId b2CreateWheelJoint(b2WorldId worldId, const(b2WheelJointDef)* def)
{
	B2_CHECK_DEF( def );
	B2_ASSERT( def.lowerTranslation <= def.upperTranslation );

	b2World* world = b2GetWorldFromId( worldId );

	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return b2JointId( 0 );
	}

	b2JointPair pair = b2CreateJoint( world, &def.base, b2_wheelJoint );

	b2JointSim* joint = pair.jointSim;

	joint.wheelJoint = b2WheelJoint( 0 );
	joint.wheelJoint.perpMass = 0.0f;
	joint.wheelJoint.axialMass = 0.0f;
	joint.wheelJoint.motorImpulse = 0.0f;
	joint.wheelJoint.lowerImpulse = 0.0f;
	joint.wheelJoint.upperImpulse = 0.0f;
	joint.wheelJoint.lowerTranslation = def.lowerTranslation;
	joint.wheelJoint.upperTranslation = def.upperTranslation;
	joint.wheelJoint.maxMotorTorque = def.maxMotorTorque;
	joint.wheelJoint.motorSpeed = def.motorSpeed;
	joint.wheelJoint.hertz = def.hertz;
	joint.wheelJoint.dampingRatio = def.dampingRatio;
	joint.wheelJoint.enableSpring = def.enableSpring;
	joint.wheelJoint.enableLimit = def.enableLimit;
	joint.wheelJoint.enableMotor = def.enableMotor;

	b2JointId jointId = { joint.jointId + 1, world.worldId, pair.joint.generation };
	return jointId;
}

void b2DestroyJointInternal(b2World* world, b2Joint* joint, bool wakeBodies)
{
	int jointId = joint.jointId;

	b2JointEdge* edgeA = joint.edges.ptr + 0;
	b2JointEdge* edgeB = joint.edges.ptr + 1;

	int idA = edgeA.bodyId;
	int idB = edgeB.bodyId;
	b2Body* bodyA = b2BodyArray_Get( world.bodies, idA );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, idB );

	// Remove from body A
	if ( edgeA.prevKey != B2_NULL_INDEX )
	{
		b2Joint* prevJoint = b2JointArray_Get( world.joints, edgeA.prevKey >> 1 );
		b2JointEdge* prevEdge = prevJoint.edges.ptr + ( edgeA.prevKey & 1 );
		prevEdge.nextKey = edgeA.nextKey;
	}

	if ( edgeA.nextKey != B2_NULL_INDEX )
	{
		b2Joint* nextJoint = b2JointArray_Get( world.joints, edgeA.nextKey >> 1 );
		b2JointEdge* nextEdge = nextJoint.edges.ptr + ( edgeA.nextKey & 1 );
		nextEdge.prevKey = edgeA.prevKey;
	}

	int edgeKeyA = ( jointId << 1 ) | 0;
	if ( bodyA.headJointKey == edgeKeyA )
	{
		bodyA.headJointKey = edgeA.nextKey;
	}

	bodyA.jointCount -= 1;

	// Remove from body B
	if ( edgeB.prevKey != B2_NULL_INDEX )
	{
		b2Joint* prevJoint = b2JointArray_Get( world.joints, edgeB.prevKey >> 1 );
		b2JointEdge* prevEdge = prevJoint.edges.ptr + ( edgeB.prevKey & 1 );
		prevEdge.nextKey = edgeB.nextKey;
	}

	if ( edgeB.nextKey != B2_NULL_INDEX )
	{
		b2Joint* nextJoint = b2JointArray_Get( world.joints, edgeB.nextKey >> 1 );
		b2JointEdge* nextEdge = nextJoint.edges.ptr + ( edgeB.nextKey & 1 );
		nextEdge.prevKey = edgeB.prevKey;
	}

	int edgeKeyB = ( jointId << 1 ) | 1;
	if ( bodyB.headJointKey == edgeKeyB )
	{
		bodyB.headJointKey = edgeB.nextKey;
	}

	bodyB.jointCount -= 1;

	if ( joint.islandId != B2_NULL_INDEX )
	{
		B2_ASSERT( joint.setIndex > b2_disabledSet );
		b2UnlinkJoint( world, joint );
	}
	else
	{
		B2_ASSERT( joint.setIndex <= b2_disabledSet );
	}

	// Remove joint from solver set that owns it
	int setIndex = joint.setIndex;
	int localIndex = joint.localIndex;

	if ( setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint.edges[0].bodyId, joint.edges[1].bodyId, joint.colorIndex, localIndex );
	}
	else
	{
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setIndex );
		int movedIndex = b2JointSimArray_RemoveSwap( set.jointSims, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// Fix moved joint
			b2JointSim* movedJointSim = set.jointSims.ptr + localIndex;
			int movedId = movedJointSim.jointId;
			b2Joint* movedJoint = b2JointArray_Get( world.joints, movedId );
			B2_ASSERT( movedJoint.localIndex == movedIndex );
			movedJoint.localIndex = localIndex;
		}
	}

	// Free joint and id (preserve joint generation)
	joint.setIndex = B2_NULL_INDEX;
	joint.localIndex = B2_NULL_INDEX;
	joint.colorIndex = B2_NULL_INDEX;
	joint.jointId = B2_NULL_INDEX;
	b2FreeId( &world.jointIdPool, jointId );

	if ( wakeBodies )
	{
		b2WakeBody( world, bodyA );
		b2WakeBody( world, bodyB );
	}

	b2ValidateSolverSets( world );
}

void b2DestroyJoint(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	B2_ASSERT( world.locked == false );

	if ( world.locked )
	{
		return;
	}

	b2Joint* joint = b2GetJointFullId( world, jointId );

	b2DestroyJointInternal( world, joint, true );
}

b2JointType b2Joint_GetType(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return joint.type;
}

b2BodyId b2Joint_GetBodyA(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return b2MakeBodyId( world, joint.edges[0].bodyId );
}

b2BodyId b2Joint_GetBodyB(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return b2MakeBodyId( world, joint.edges[1].bodyId );
}

b2WorldId b2Joint_GetWorld(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	return b2WorldId( cast(ushort)(jointId.world0 + 1), world.generation );
}

void b2Joint_SetLocalFrameA(b2JointId jointId, b2Transform localFrame)
{
	B2_ASSERT( b2IsValidTransform( localFrame ) );

	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* jointSim = b2GetJointSim( world, joint );
	jointSim.localFrameA = localFrame;
}

b2Transform b2Joint_GetLocalFrameA(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* jointSim = b2GetJointSim( world, joint );
	return jointSim.localFrameA;
}

void b2Joint_SetLocalFrameB(b2JointId jointId, b2Transform localFrame)
{
	B2_ASSERT( b2IsValidTransform( localFrame ) );

	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* jointSim = b2GetJointSim( world, joint );
	jointSim.localFrameB = localFrame;
}

b2Transform b2Joint_GetLocalFrameB(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* jointSim = b2GetJointSim( world, joint );
	return jointSim.localFrameB;
}

void b2Joint_SetCollideConnected(b2JointId jointId, bool shouldCollide)
{
	b2World* world = b2GetWorldLocked( jointId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Joint* joint = b2GetJointFullId( world, jointId );
	if ( joint.collideConnected == shouldCollide )
	{
		return;
	}

	joint.collideConnected = shouldCollide;

	b2Body* bodyA = b2BodyArray_Get( world.bodies, joint.edges[0].bodyId );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, joint.edges[1].bodyId );

	if ( shouldCollide )
	{
		// need to tell the broad-phase to look for new pairs for one of the
		// two bodies. Pick the one with the fewest shapes.
		int shapeCountA = bodyA.shapeCount;
		int shapeCountB = bodyB.shapeCount;

		int shapeId = shapeCountA < shapeCountB ? bodyA.headShapeId : bodyB.headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

			if ( shape.proxyKey != B2_NULL_INDEX )
			{
				b2BufferMove( &world.broadPhase, shape.proxyKey );
			}

			shapeId = shape.nextShapeId;
		}
	}
	else
	{
		b2DestroyContactsBetweenBodies( world, bodyA, bodyB );
	}
}

bool b2Joint_GetCollideConnected(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return joint.collideConnected;
}

void b2Joint_SetUserData(b2JointId jointId, void* userData)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	joint.userData = userData;
}

void* b2Joint_GetUserData(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return joint.userData;
}

void b2Joint_WakeBodies(b2JointId jointId)
{
	b2World* world = b2GetWorldLocked( jointId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2Body* bodyA = b2BodyArray_Get( world.bodies, joint.edges[0].bodyId );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, joint.edges[1].bodyId );

	b2WakeBody( world, bodyA );
	b2WakeBody( world, bodyB );
}

void b2GetJointReaction(b2JointSim* sim, float invTimeStep, float* force, float* torque)
{
	float linearImpulse = 0.0f;
	float angularImpulse = 0.0f;

	switch ( sim.type )
	{
		case b2_distanceJoint:
		{
			b2DistanceJoint* joint = &sim.distanceJoint;
			linearImpulse = b2AbsFloat( joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse );
		}
		break;

		case b2_motorJoint:
		{
			b2MotorJoint* joint = &sim.motorJoint;
			linearImpulse = b2Length( b2Add(joint.linearVelocityImpulse, joint.linearSpringImpulse) );
			angularImpulse = b2AbsFloat( joint.angularVelocityImpulse + joint.angularSpringImpulse );
		}
		break;

		case b2_mouseJoint:
		{
			b2MouseJoint* joint = &sim.mouseJoint;
			linearImpulse = b2Length( joint.linearImpulse );
			angularImpulse = b2AbsFloat( joint.angularImpulse );
		}
		break;

		case b2_prismaticJoint:
		{
			b2PrismaticJoint* joint = &sim.prismaticJoint;
			float perpImpulse = joint.impulse.x;
			float axialImpulse = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
			linearImpulse = sqrt( perpImpulse * perpImpulse + axialImpulse * axialImpulse );
			angularImpulse = b2AbsFloat( joint.impulse.y );
		}
		break;

		case b2_revoluteJoint:
		{
			b2RevoluteJoint* joint = &sim.revoluteJoint;

			linearImpulse = b2Length( joint.linearImpulse );
			angularImpulse = b2AbsFloat( joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse );
		}
		break;

		case b2_weldJoint:
		{
			b2WeldJoint* joint = &sim.weldJoint;
			linearImpulse = b2Length( joint.linearImpulse );
			angularImpulse = b2AbsFloat( joint.angularImpulse );
		}
		break;

		case b2_wheelJoint:
		{
			b2WheelJoint* joint = &sim.wheelJoint;
			float perpImpulse = joint.perpImpulse;
			float axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;
			linearImpulse = sqrt( perpImpulse * perpImpulse + axialImpulse * axialImpulse );
			angularImpulse = b2AbsFloat( joint.motorImpulse );
		}
		break;

		default:
			break;
	}

	*force = linearImpulse * invTimeStep;
	*torque = angularImpulse * invTimeStep;
}

private b2Vec2 b2GetJointConstraintForce(b2World* world, b2Joint* joint)
{
	b2JointSim* base = b2GetJointSim( world, joint );

	switch ( joint.type )
	{
		case b2_distanceJoint:
			return b2GetDistanceJointForce( world, base );

		case b2_motorJoint:
			return b2GetMotorJointForce( world, base );

		case b2_mouseJoint:
			return b2GetMouseJointForce( world, base );

		case b2_filterJoint:
			return b2Vec2.zero;

		case b2_prismaticJoint:
			return b2GetPrismaticJointForce( world, base );

		case b2_revoluteJoint:
			return b2GetRevoluteJointForce( world, base );

		case b2_weldJoint:
			return b2GetWeldJointForce( world, base );

		case b2_wheelJoint:
			return b2GetWheelJointForce( world, base );

		default:
			B2_ASSERT( false );
			return b2Vec2.zero;
	}
}

private float b2GetJointConstraintTorque(b2World* world, b2Joint* joint)
{
	b2JointSim* base = b2GetJointSim( world, joint );

	switch ( joint.type )
	{
		case b2_distanceJoint:
			return 0.0f;

		case b2_motorJoint:
			return b2GetMotorJointTorque( world, base );

		case b2_mouseJoint:
			return b2GetMouseJointTorque( world, base );

		case b2_filterJoint:
			return 0.0f;

		case b2_prismaticJoint:
			return b2GetPrismaticJointTorque( world, base );

		case b2_revoluteJoint:
			return b2GetRevoluteJointTorque( world, base );

		case b2_weldJoint:
			return b2GetWeldJointTorque( world, base );

		case b2_wheelJoint:
			return b2GetWheelJointTorque( world, base );

		default:
			B2_ASSERT( false );
			return 0.0f;
	}
}

b2Vec2 b2Joint_GetConstraintForce(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return b2GetJointConstraintForce( world, joint );
}

float b2Joint_GetConstraintTorque(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	return b2GetJointConstraintTorque( world, joint );
}

float b2Joint_GetLinearSeparation(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );

	b2Transform xfA = b2GetBodyTransform( world, joint.edges[0].bodyId );
	b2Transform xfB = b2GetBodyTransform( world, joint.edges[1].bodyId );

	b2Vec2 pA = b2TransformPoint( xfA, base.localFrameA.p );
	b2Vec2 pB = b2TransformPoint( xfB, base.localFrameB.p );
	b2Vec2 dp = b2Sub( pB, pA );

	switch ( joint.type )
	{
		case b2_distanceJoint:
		{
			b2DistanceJoint* distanceJoint = &base.distanceJoint;
			float length = b2Length( dp );
			if ( distanceJoint.enableSpring )
			{
				if ( distanceJoint.enableLimit )
				{
					if ( length < distanceJoint.minLength )
					{
						return distanceJoint.minLength - length;
					}

					if ( length > distanceJoint.maxLength )
					{
						return length - distanceJoint.maxLength;
					}

					return 0.0f;
				}

				return 0.0f;
			}

			return b2AbsFloat( length - distanceJoint.length );
		}

		case b2_motorJoint:
			return 0.0f;

		case b2_mouseJoint:
			return 0.0f;

		case b2_filterJoint:
			return 0.0f;

		case b2_prismaticJoint:
		{
			b2PrismaticJoint* prismaticJoint = &base.prismaticJoint;
			b2Vec2 axisA = b2RotateVector( xfA.q, b2Vec2( 1.0f, 0.0f ) );
			b2Vec2 perpA = b2LeftPerp( axisA );
			float perpendicularSeparation = b2AbsFloat( b2Dot( perpA, dp ) );
			float limitSeparation = 0.0f;

			if ( prismaticJoint.enableLimit )
			{
				float translation = b2Dot( axisA, dp );
				if ( translation < prismaticJoint.lowerTranslation )
				{
					limitSeparation = prismaticJoint.lowerTranslation - translation;
				}

				if ( prismaticJoint.upperTranslation < translation )
				{
					limitSeparation = translation - prismaticJoint.upperTranslation;
				}
			}

			return sqrt( perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation );
		}

		case b2_revoluteJoint:
			return b2Length( dp );

		case b2_weldJoint:
		{
			b2WeldJoint* weldJoint = &base.weldJoint;
			if ( weldJoint.linearHertz == 0.0f )
			{
				return b2Length( dp );
			}

			return 0.0f;
		}

		case b2_wheelJoint:
		{
			b2WheelJoint* wheelJoint = &base.wheelJoint;
			b2Vec2 axisA = b2RotateVector( xfA.q, b2Vec2( 1.0f, 0.0f ) );
			b2Vec2 perpA = b2LeftPerp( axisA );
			float perpendicularSeparation = b2AbsFloat( b2Dot( perpA, dp ) );
			float limitSeparation = 0.0f;

			if ( wheelJoint.enableLimit )
			{
				float translation = b2Dot( axisA, dp );
				if ( translation < wheelJoint.lowerTranslation )
				{
					limitSeparation = wheelJoint.lowerTranslation - translation;
				}

				if ( wheelJoint.upperTranslation < translation )
				{
					limitSeparation = translation - wheelJoint.upperTranslation;
				}
			}

			return sqrt( perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation );
		}

		default:
			B2_ASSERT( false );
			return 0.0f;
	}
}

float b2Joint_GetAngularSeparation(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );

	b2Transform xfA = b2GetBodyTransform( world, joint.edges[0].bodyId );
	b2Transform xfB = b2GetBodyTransform( world, joint.edges[1].bodyId );
	float relativeAngle = b2RelativeAngle( xfA.q, xfB.q );

	switch ( joint.type )
	{
		case b2_distanceJoint:
			return 0.0f;

		case b2_motorJoint:
			return 0.0f;

		case b2_mouseJoint:
			return 0.0f;

		case b2_filterJoint:
			return 0.0f;

		case b2_prismaticJoint:
		{
			return relativeAngle;
		}

		case b2_revoluteJoint:
		{
			b2RevoluteJoint* revoluteJoint = &base.revoluteJoint;
			if ( revoluteJoint.enableLimit )
			{
				float angle = relativeAngle;
				if ( angle < revoluteJoint.lowerAngle )
				{
					return revoluteJoint.lowerAngle - angle;
				}

				if ( revoluteJoint.upperAngle < angle )
				{
					return angle - revoluteJoint.upperAngle;
				}
			}

			return 0.0f;
		}

		case b2_weldJoint:
		{
			b2WeldJoint* weldJoint = &base.weldJoint;
			if ( weldJoint.angularHertz == 0.0f )
			{
				return relativeAngle;
			}

			return 0.0f;
		}

		case b2_wheelJoint:
			return 0.0f;

		default:
			B2_ASSERT( false );
			return 0.0f;
	}
}

void b2Joint_SetConstraintTuning(b2JointId jointId, float hertz, float dampingRatio)
{
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );

	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	base.constraintHertz = hertz;
	base.constraintDampingRatio = dampingRatio;
}

void b2Joint_GetConstraintTuning(b2JointId jointId, float* hertz, float* dampingRatio)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	*hertz = base.constraintHertz;
	*dampingRatio = base.constraintDampingRatio;
}

void b2Joint_SetForceThreshold(b2JointId jointId, float threshold)
{
	B2_ASSERT( b2IsValidFloat( threshold ) && threshold >= 0.0f );

	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	base.forceThreshold = threshold;
}

float b2Joint_GetForceThreshold(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	return base.forceThreshold;
}

void b2Joint_SetTorqueThreshold(b2JointId jointId, float threshold)
{
	B2_ASSERT( b2IsValidFloat( threshold ) && threshold >= 0.0f );

	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	base.torqueThreshold = threshold;
}

float b2Joint_GetTorqueThreshold(b2JointId jointId)
{
	b2World* world = b2GetWorld( jointId.world0 );
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	return base.torqueThreshold;
}

void b2PrepareJoint(b2JointSim* joint, b2StepContext* context)
{
	// Clamp joint hertz based on the time step to reduce jitter.
	float hertz = b2MinFloat( joint.constraintHertz, 0.25f * context.inv_h );
	joint.constraintSoftness = b2MakeSoft( hertz, joint.constraintDampingRatio, context.h );

	switch ( joint.type )
	{
		case b2_distanceJoint:
			b2PrepareDistanceJoint( joint, context );
			break;

		case b2_motorJoint:
			b2PrepareMotorJoint( joint, context );
			break;

		case b2_mouseJoint:
			b2PrepareMouseJoint( joint, context );
			break;

		case b2_filterJoint:
			break;

		case b2_prismaticJoint:
			b2PreparePrismaticJoint( joint, context );
			break;

		case b2_revoluteJoint:
			b2PrepareRevoluteJoint( joint, context );
			break;

		case b2_weldJoint:
			b2PrepareWeldJoint( joint, context );
			break;

		case b2_wheelJoint:
			b2PrepareWheelJoint( joint, context );
			break;

		default:
			B2_ASSERT( false );
	}
}

void b2WarmStartJoint(b2JointSim* joint, b2StepContext* context)
{
	switch ( joint.type )
	{
		case b2_distanceJoint:
			b2WarmStartDistanceJoint( joint, context );
			break;

		case b2_motorJoint:
			b2WarmStartMotorJoint( joint, context );
			break;

		case b2_mouseJoint:
			b2WarmStartMouseJoint( joint, context );
			break;

		case b2_filterJoint:
			break;

		case b2_prismaticJoint:
			b2WarmStartPrismaticJoint( joint, context );
			break;

		case b2_revoluteJoint:
			b2WarmStartRevoluteJoint( joint, context );
			break;

		case b2_weldJoint:
			b2WarmStartWeldJoint( joint, context );
			break;

		case b2_wheelJoint:
			b2WarmStartWheelJoint( joint, context );
			break;

		default:
			B2_ASSERT( false );
	}
}

void b2SolveJoint(b2JointSim* joint, b2StepContext* context, bool useBias)
{
	switch ( joint.type )
	{
		case b2_distanceJoint:
			b2SolveDistanceJoint( joint, context, useBias );
			break;

		case b2_motorJoint:
			b2SolveMotorJoint( joint, context );
			break;

		case b2_mouseJoint:
			b2SolveMouseJoint( joint, context );
			break;

		case b2_filterJoint:
			break;

		case b2_prismaticJoint:
			b2SolvePrismaticJoint( joint, context, useBias );
			break;

		case b2_revoluteJoint:
			b2SolveRevoluteJoint( joint, context, useBias );
			break;

		case b2_weldJoint:
			b2SolveWeldJoint( joint, context, useBias );
			break;

		case b2_wheelJoint:
			b2SolveWheelJoint( joint, context, useBias );
			break;

		default:
			B2_ASSERT( false );
	}
}

void b2PrepareOverflowJoints(b2StepContext* context)
{
	b2ConstraintGraph* graph = context.graph;
	b2JointSim* joints = graph.colors[B2_OVERFLOW_INDEX].jointSims.ptr;
	int jointCount = cast(int)graph.colors[B2_OVERFLOW_INDEX].jointSims.count;

	for ( int i = 0; i < jointCount; ++i )
	{
		b2JointSim* joint = joints + i;
		b2PrepareJoint( joint, context );
	}
}

void b2WarmStartOverflowJoints(b2StepContext* context)
{
	b2ConstraintGraph* graph = context.graph;
	b2JointSim* joints = graph.colors[B2_OVERFLOW_INDEX].jointSims.ptr;
	int jointCount = cast(int)graph.colors[B2_OVERFLOW_INDEX].jointSims.count;

	for ( int i = 0; i < jointCount; ++i )
	{
		b2JointSim* joint = joints + i;
		b2WarmStartJoint( joint, context );
	}
}

void b2SolveOverflowJoints(b2StepContext* context, bool useBias)
{
	b2ConstraintGraph* graph = context.graph;
	b2JointSim* joints = graph.colors[B2_OVERFLOW_INDEX].jointSims.ptr;
	int jointCount = cast(int)graph.colors[B2_OVERFLOW_INDEX].jointSims.count;

	for ( int i = 0; i < jointCount; ++i )
	{
		b2JointSim* joint = joints + i;
		b2SolveJoint( joint, context, useBias );
	}
}

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2Joint* joint)
{
	b2Body* bodyA = b2BodyArray_Get( world.bodies, joint.edges[0].bodyId );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, joint.edges[1].bodyId );
	if ( bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet )
	{
		return;
	}

	b2JointSim* jointSim = b2GetJointSim( world, joint );

	b2Transform transformA = b2GetBodyTransformQuick( world, bodyA );
	b2Transform transformB = b2GetBodyTransformQuick( world, bodyB );
	b2Vec2 pA = b2TransformPoint( transformA, jointSim.localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, jointSim.localFrameB.p );

	b2HexColor color = b2_colorDarkSeaGreen;

	switch ( joint.type )
	{
		case b2_distanceJoint:
			b2DrawDistanceJoint( draw, jointSim, transformA, transformB );
			break;

		case b2_mouseJoint:
			draw.DrawPointFcn( pA, 8.0f, b2_colorYellowGreen, draw.context );
			draw.DrawPointFcn( pB, 8.0f, b2_colorYellowGreen, draw.context );
			draw.DrawSegmentFcn( pA, pB, b2_colorLightGray, draw.context );
			break;

		case b2_filterJoint:
			draw.DrawSegmentFcn( pA, pB, b2_colorGold, draw.context );
			break;

		case b2_motorJoint:
			draw.DrawPointFcn( pA, 8.0f, b2_colorYellowGreen, draw.context );
			draw.DrawPointFcn( pB, 8.0f, b2_colorPlum, draw.context );
			break;

		case b2_prismaticJoint:
			b2DrawPrismaticJoint( draw, jointSim, transformA, transformB, joint.drawScale );
			break;

		case b2_revoluteJoint:
			b2DrawRevoluteJoint( draw, jointSim, transformA, transformB, joint.drawScale );
			break;

		case b2_weldJoint:
			b2DrawWeldJoint( draw, jointSim, transformA, transformB, joint.drawScale );
			break;

		case b2_wheelJoint:
			b2DrawWheelJoint( draw, jointSim, transformA, transformB );
			break;

		default:
			draw.DrawSegmentFcn( transformA.p, pA, color, draw.context );
			draw.DrawSegmentFcn( pA, pB, color, draw.context );
			draw.DrawSegmentFcn( transformB.p, pB, color, draw.context );
			break;
	}

	if ( draw.drawGraphColors )
	{
		b2HexColor[B2_GRAPH_COLOR_COUNT] graphColors = [
			b2_colorRed,	b2_colorOrange, b2_colorYellow,	   b2_colorGreen,	  b2_colorCyan,		b2_colorBlue,
			b2_colorViolet, b2_colorPink,	b2_colorChocolate, b2_colorGoldenRod, b2_colorCoral,	b2_colorRosyBrown,
			b2_colorAqua,	b2_colorPeru,	b2_colorLime,	   b2_colorGold,	  b2_colorPlum,		b2_colorSnow,
			b2_colorTeal,	b2_colorKhaki,	b2_colorSalmon,	   b2_colorPeachPuff, b2_colorHoneyDew, b2_colorBlack,
		];

		int colorIndex = joint.colorIndex;
		if ( colorIndex != B2_NULL_INDEX )
		{
			b2Vec2 p = b2Lerp( pA, pB, 0.5f );
			draw.DrawPointFcn( p, 5.0f, graphColors[colorIndex], draw.context );
		}
	}

	if ( draw.drawJointExtras )
	{
		b2Vec2 force = b2GetJointConstraintForce( world, joint );
		float torque = b2GetJointConstraintTorque( world, joint );
		b2Vec2 p = b2Lerp( pA, pB, 0.5f );

		draw.DrawSegmentFcn( p, b2MulAdd( p, 0.001f, force ), b2_colorAzure, draw.context );

		char[64] buffer = void;
		snprintf( buffer.ptr, 64, "f = [%g, %g], t = %g", force.x, force.y, torque );
		draw.DrawStringFcn( p, buffer.ptr, b2_colorAzure, draw.context );
	}
}
