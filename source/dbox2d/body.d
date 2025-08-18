module dbox2d.body;

public import dbox2d.array;
import dbox2d.box2d;

public import dbox2d.math_functions;
public import dbox2d.types;
import dbox2d.physics_world;
import dbox2d.island;
import dbox2d.contact;
import dbox2d.constants;
import dbox2d.core;
import dbox2d.joint;
import dbox2d.shape;
import dbox2d.sensor;

import dbox2d.solver_set;

mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));
mixin(B2_ARRAY_SOURCE!("b2ChainShape", "b2ChainShape"));
mixin(B2_ARRAY_SOURCE!("b2Joint","b2Joint"));
mixin(B2_ARRAY_SOURCE!("b2Shape","b2Shape"));
mixin(B2_ARRAY_SOURCE!("b2Contact","b2Contact"));
mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2Body","b2Body"));


enum b2BodyFlags
{
	// This body has fixed translation along the x-axis
	b2_lockLinearX = 0x00000001,

	// This body has fixed translation along the y-axis
	b2_lockLinearY = 0x00000002,

	// This body has fixed rotation
	b2_lockAngularZ = 0x00000004,

	// This flag is used for debug draw
	b2_isFast = 0x00000008,

	// This dynamic body does a final CCD pass against all body types, but not other bullets
	b2_isBullet = 0x00000010,

	// This body was speed capped in the current time step
	b2_isSpeedCapped = 0x00000020,
	
	// This body had a time of impact event in the current time step
	b2_hadTimeOfImpact = 0x00000040,

	// This body has no limit on angular velocity
	b2_allowFastRotation = 0x00000080,

	// This body need's to have its AABB increased
	b2_enlargeBounds = 0x00000100,

	// All lock flags
	b2_allLocks = b2_lockAngularZ | b2_lockLinearX | b2_lockLinearY,
}
alias b2_lockLinearX = b2BodyFlags.b2_lockLinearX;
alias b2_lockLinearY = b2BodyFlags.b2_lockLinearY;
alias b2_lockAngularZ = b2BodyFlags.b2_lockAngularZ;
alias b2_isFast = b2BodyFlags.b2_isFast;
alias b2_isBullet = b2BodyFlags.b2_isBullet;
alias b2_isSpeedCapped = b2BodyFlags.b2_isSpeedCapped;
alias b2_hadTimeOfImpact = b2BodyFlags.b2_hadTimeOfImpact;
alias b2_allowFastRotation = b2BodyFlags.b2_allowFastRotation;
alias b2_enlargeBounds = b2BodyFlags.b2_enlargeBounds;
alias b2_allLocks = b2BodyFlags.b2_allLocks;


// Body organizational details that are not used in the solver.
struct b2Body {
	char[32] name = 0;

	void* userData;

	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	int setIndex;

	// body sim and state index within set
	// may be B2_NULL_INDEX
	int localIndex;

	// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;

	// todo maybe move this to the body sim
	int headShapeId;
	int shapeCount;

	int headChainId;

	// [31 : jointId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;

	// All enabled dynamic and kinematic bodies are in an island.
	int islandId;

	// doubly-linked island list
	int islandPrev;
	int islandNext;

	float mass = 0;

	// Rotational inertia about the center of mass.
	float inertia = 0;

	float sleepThreshold = 0;
	float sleepTime = 0;

	// this is used to adjust the fellAsleep flag in the body move array
	int bodyMoveIndex;

	int id;

	// b2BodyFlags
	uint flags;

	b2BodyType type;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2BodyId
	ushort generation;

	// todo move into flags
	bool enableSleep;
	bool isMarked;
}

// Body State
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.

// 32 bytes
struct b2BodyState {
	b2Vec2 linearVelocity; // 8
	float angularVelocity = 0; // 4

	// b2BodyFlags
	uint flags; // 4

	// Using delta position reduces round-off error far from the origin
	b2Vec2 deltaPosition; // 8

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	b2Rot deltaRotation; // 8
}

// Identity body state, notice the deltaRotation is {1, 0}
const(b2BodyState) b2_identityBodyState = { { 0.0f, 0.0f }, 0.0f, 0, { 0.0f, 0.0f }, { 1.0f, 0.0f } };

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
struct b2BodySim {
	// transform for body origin
	b2Transform transform;

	// center of mass position in world space
	b2Vec2 center;

	// previous rotation and COM for TOI
	b2Rot rotation0;
	b2Vec2 center0;

	// location of center of mass relative to the body origin
	b2Vec2 localCenter;

	b2Vec2 force;
	float torque = 0;

	// inverse inertia
	float invMass = 0;
	float invInertia = 0;

	float minExtent = 0;
	float maxExtent = 0;
	float linearDamping = 0;
	float angularDamping = 0;
	float gravityScale = 0;

	// Index of b2Body
	int bodyId;

	// b2BodyFlags
	uint flags;
}

pragma(inline, true) b2Sweep b2MakeSweep(const(b2BodySim)* bodySim)
{
	b2Sweep s = void;
	s.c1 = bodySim.center0;
	s.c2 = bodySim.center;
	s.q1 = bodySim.rotation0;
	s.q2 = bodySim.transform.q;
	s.localCenter = bodySim.localCenter;
	return s;
}

import core.stdc.math;

void b2LimitVelocity(b2BodyState* state, float maxLinearSpeed)
{
	float v2 = b2LengthSquared( state.linearVelocity );
	if ( v2 > maxLinearSpeed * maxLinearSpeed )
	{
		state.linearVelocity = b2MulSV( maxLinearSpeed / sqrtf( v2 ), state.linearVelocity );
	}
}

// Get a validated body from a world using an id.
b2Body* b2GetBodyFullId(b2World* world, b2BodyId bodyId)
{
	assert( b2Body_IsValid( bodyId ) );

	// id index starts at one so that zero can represent null
	return b2BodyArray_Get( world.bodies, bodyId.index1 - 1 );
}

b2Transform b2GetBodyTransformQuick(b2World* world, b2Body* body)
{
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, body.setIndex );
	b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, body.localIndex );
	return bodySim.transform;
}

b2Transform b2GetBodyTransform(b2World* world, int bodyId)
{
	b2Body* body = b2BodyArray_Get( world.bodies, bodyId );
	return b2GetBodyTransformQuick( world, body );
}

// Create a b2BodyId from a raw id.
b2BodyId b2MakeBodyId(b2World* world, int bodyId)
{
	b2Body* body = b2BodyArray_Get( world.bodies, bodyId );
	return b2BodyId( bodyId + 1, world.worldId, body.generation );
}

b2BodySim* b2GetBodySim(b2World* world, b2Body* body)
{
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, body.setIndex );
	b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, body.localIndex );
	return bodySim;
}

b2BodyState* b2GetBodyState(b2World* world, b2Body* body)
{
	if ( body.setIndex == b2_awakeSet )
	{
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
		return b2BodyStateArray_Get( set.bodyStates, body.localIndex );
	}

	return null;
}

void b2CreateIslandForBody(b2World* world, int setIndex, b2Body* body)
{
	assert( body.islandId == B2_NULL_INDEX );
	assert( body.islandPrev == B2_NULL_INDEX );
	assert( body.islandNext == B2_NULL_INDEX );
	assert( setIndex != b2_disabledSet );

	b2Island* island = b2CreateIsland( world, setIndex );

	body.islandId = island.islandId;
	island.headBody = body.id;
	island.tailBody = body.id;
	island.bodyCount = 1;
}

void b2RemoveBodyFromIsland(b2World* world, b2Body* body)
{
	if ( body.islandId == B2_NULL_INDEX )
	{
		assert( body.islandPrev == B2_NULL_INDEX );
		assert( body.islandNext == B2_NULL_INDEX );
		return;
	}

	int islandId = body.islandId;
	b2Island* island = b2IslandArray_Get( world.islands, islandId );

	// Fix the island's linked list of sims
	if ( body.islandPrev != B2_NULL_INDEX )
	{
		b2Body* prevBody = b2BodyArray_Get( world.bodies, body.islandPrev );
		prevBody.islandNext = body.islandNext;
	}

	if ( body.islandNext != B2_NULL_INDEX )
	{
		b2Body* nextBody = b2BodyArray_Get( world.bodies, body.islandNext );
		nextBody.islandPrev = body.islandPrev;
	}

	assert( island.bodyCount > 0 );
	island.bodyCount -= 1;
	bool islandDestroyed = false;

	if ( island.headBody == body.id )
	{
		island.headBody = body.islandNext;

		if ( island.headBody == B2_NULL_INDEX )
		{
			// Destroy empty island
			assert( island.tailBody == body.id );
			assert( island.bodyCount == 0 );
			assert( island.contactCount == 0 );
			assert( island.jointCount == 0 );

			// Free the island
			b2DestroyIsland( world, island.islandId );
			islandDestroyed = true;
		}
	}
	else if ( island.tailBody == body.id )
	{
		island.tailBody = body.islandPrev;
	}

	if ( islandDestroyed == false )
	{
		b2ValidateIsland( world, islandId );
	}

	body.islandId = B2_NULL_INDEX;
	body.islandPrev = B2_NULL_INDEX;
	body.islandNext = B2_NULL_INDEX;
}

void b2DestroyBodyContacts(b2World* world, b2Body* body, bool wakeBodies)
{
	// Destroy the attached contacts
	int edgeKey = body.headContactKey;
	while ( edgeKey != B2_NULL_INDEX )
	{
		int contactId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );
		edgeKey = contact.edges[edgeIndex].nextKey;
		b2DestroyContact( world, contact, wakeBodies );
	}

	b2ValidateSolverSets( world );
}

b2BodyId b2CreateBody(b2WorldId worldId, const(b2BodyDef)* def)
{
	B2_CHECK_DEF(def);
	assert( b2IsValidVec2( def.position ) );
	assert( b2IsValidRotation( def.rotation ) );
	assert( b2IsValidVec2( def.linearVelocity ) );
	assert( b2IsValidFloat( def.angularVelocity ) );
	assert( b2IsValidFloat( def.linearDamping ) && def.linearDamping >= 0.0f );
	assert( b2IsValidFloat( def.angularDamping ) && def.angularDamping >= 0.0f );
	assert( b2IsValidFloat( def.sleepThreshold ) && def.sleepThreshold >= 0.0f );
	assert( b2IsValidFloat( def.gravityScale ) );

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );

	if ( world.locked )
	{
		return b2_nullBodyId;
	}

	bool isAwake = ( def.isAwake || def.enableSleep == false ) && def.isEnabled;

	// determine the solver set
	int setId = void;
	if ( def.isEnabled == false )
	{
		// any body type can be disabled
		setId = b2_disabledSet;
	}
	else if ( def.type == b2_staticBody )
	{
		setId = b2_staticSet;
	}
	else if ( isAwake == true )
	{
		setId = b2_awakeSet;
	}
	else
	{
		// new set for a sleeping body in its own island
		setId = b2AllocId( &world.solverSetIdPool );
		if ( setId == world.solverSets.length )
		{
			// Create a zero initialized solver set. All sub-arrays are also zero initialized.
			b2SolverSetArray_Push( world.solverSets, b2SolverSet() );
		}
		else
		{
			assert( world.solverSets[setId].setIndex == B2_NULL_INDEX );
		}

		world.solverSets[setId].setIndex = setId;
	}

	assert( 0 <= setId && setId < world.solverSets.length );

	int bodyId = b2AllocId( &world.bodyIdPool );

	uint lockFlags = 0;
	lockFlags |= def.motionLocks.linearX ? b2_lockLinearX : 0;
	lockFlags |= def.motionLocks.linearY ? b2_lockLinearY : 0;
	lockFlags |= def.motionLocks.angularZ ? b2_lockAngularZ : 0;

	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setId );
	b2BodySim* bodySim = b2BodySimArray_Add( set.bodySims );
	*bodySim = b2BodySim();
	bodySim.transform.p = def.position;
	bodySim.transform.q = def.rotation;
	bodySim.center = def.position;
	bodySim.rotation0 = bodySim.transform.q;
	bodySim.center0 = bodySim.center;
	bodySim.minExtent = B2_HUGE;
	bodySim.maxExtent = 0.0f;
	bodySim.linearDamping = def.linearDamping;
	bodySim.angularDamping = def.angularDamping;
	bodySim.gravityScale = def.gravityScale;
	bodySim.bodyId = bodyId;
	bodySim.flags = lockFlags;
	bodySim.flags |= def.isBullet ? b2_isBullet : 0;
	bodySim.flags |= def.allowFastRotation ? b2_allowFastRotation : 0;

	if ( setId == b2_awakeSet )
	{
		b2BodyState* bodyState = b2BodyStateArray_Add( set.bodyStates );
		assert( ( cast(uintptr_t)bodyState & 0x1F ) == 0 );

		*bodyState = b2BodyState();
		bodyState.linearVelocity = def.linearVelocity;
		bodyState.angularVelocity = def.angularVelocity;
		bodyState.deltaRotation = b2Rot_identity;
		bodyState.flags = lockFlags;
	}

	if ( bodyId == world.bodies.length )
	{
		b2BodyArray_Push( world.bodies, b2Body() );
	}
	else
	{
		assert( world.bodies[bodyId].id == B2_NULL_INDEX );
	}

	b2Body* body = b2BodyArray_Get( world.bodies, bodyId );

	if ( def.name )
	{
		int i = 0;
		while ( i < 31 && def.name[i] != 0 )
		{
			body.name[i] = def.name[i];
			i += 1;
		}

		while ( i < 32 )
		{
			body.name[i] = 0;
			i += 1;
		}
	}
	else
	{
		import core.stdc.string;
		memset( body.name.ptr, 0, cast(int)(32 * char.sizeof) );
	}

	body.userData = cast(void*)def.userData;
	body.setIndex = setId;
	body.localIndex = cast(int)(set.bodySims.count - 1);
	body.generation += 1;
	body.headShapeId = B2_NULL_INDEX;
	body.shapeCount = 0;
	body.headChainId = B2_NULL_INDEX;
	body.headContactKey = B2_NULL_INDEX;
	body.contactCount = 0;
	body.headJointKey = B2_NULL_INDEX;
	body.jointCount = 0;
	body.islandId = B2_NULL_INDEX;
	body.islandPrev = B2_NULL_INDEX;
	body.islandNext = B2_NULL_INDEX;
	body.bodyMoveIndex = B2_NULL_INDEX;
	body.id = bodyId;
	body.mass = 0.0f;
	body.inertia = 0.0f;
	body.sleepThreshold = def.sleepThreshold;
	body.sleepTime = 0.0f;
	body.type = def.type;
	body.flags = lockFlags;
	body.enableSleep = def.enableSleep;
	body.isMarked = false;

	// dynamic and kinematic bodies that are enabled need a island
	if ( setId >= b2_awakeSet )
	{
		b2CreateIslandForBody( world, setId, body );
	}

	b2ValidateSolverSets( world );

	b2BodyId id = { bodyId + 1, world.worldId, body.generation };
	return id;
}

bool b2WakeBody(b2World* world, b2Body* body)
{
	if ( body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, body.setIndex );
		b2ValidateSolverSets( world );
		return true;
	}

	return false;
}

void b2DestroyBody(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	// Wake bodies attached to this body, even if this body is static.
	bool wakeBodies = true;

	// Destroy the attached joints
	int edgeKey = body.headJointKey;
	while ( edgeKey != B2_NULL_INDEX )
	{
		int jointId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );
		edgeKey = joint.edges[edgeIndex].nextKey;

		// Careful because this modifies the list being traversed
		b2DestroyJointInternal( world, joint, wakeBodies );
	}

	// Destroy all contacts attached to this body.
	b2DestroyBodyContacts( world, body, wakeBodies );

	// Destroy the attached shapes and their broad-phase proxies.
	int shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

		if ( shape.sensorIndex != B2_NULL_INDEX )
		{
			b2DestroySensor( world, shape );
		}

		b2DestroyShapeProxy( shape, &world.broadPhase );

		// Return shape to free list.
		b2FreeId( &world.shapeIdPool, shapeId );
		shape.id = B2_NULL_INDEX;

		shapeId = shape.nextShapeId;
	}

	// Destroy the attached chains. The associated shapes have already been destroyed above.
	int chainId = body.headChainId;
	while ( chainId != B2_NULL_INDEX )
	{
		b2ChainShape* chain = b2ChainShapeArray_Get( world.chainShapes, chainId );

		b2FreeChainData( chain );

		// Return chain to free list.
		b2FreeId( &world.chainIdPool, chainId );
		chain.id = B2_NULL_INDEX;

		chainId = chain.nextChainId;
	}

	b2RemoveBodyFromIsland( world, body );

	// Remove body sim from solver set that owns it
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, body.setIndex );
	int movedIndex = b2BodySimArray_RemoveSwap( set.bodySims, body.localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved body index
		b2BodySim* movedSim = &set.bodySims[body.localIndex];
		int movedId = movedSim.bodyId;
		b2Body* movedBody = b2BodyArray_Get( world.bodies, movedId );
		assert( movedBody.localIndex == movedIndex );
		movedBody.localIndex = body.localIndex;
	}

	// Remove body state from awake set
	if ( body.setIndex == b2_awakeSet )
	{
		int result = b2BodyStateArray_RemoveSwap( set.bodyStates, body.localIndex );
		assert( result == movedIndex );
		// B2_UNUSED( result );
	}
	else if ( set.setIndex >= b2_firstSleepingSet && set.bodySims.count == 0 )
	{
		// Remove solver set if it's now an orphan.
		b2DestroySolverSet( world, set.setIndex );
	}

	// Free body and id (preserve body generation)
	b2FreeId( &world.bodyIdPool, body.id );

	body.setIndex = B2_NULL_INDEX;
	body.localIndex = B2_NULL_INDEX;
	body.id = B2_NULL_INDEX;

	b2ValidateSolverSets( world );
}

int b2Body_GetContactCapacity(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	// Conservative and fast
	return body.contactCount;
}

int b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int capacity)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	int contactKey = body.headContactKey;
	int index = 0;
	while ( contactKey != B2_NULL_INDEX && index < capacity )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );

		// Is contact touching?
		if ( contact.flags & b2_contactTouchingFlag )
		{
			b2Shape* shapeA = b2ShapeArray_Get( world.shapes, contact.shapeIdA );
			b2Shape* shapeB = b2ShapeArray_Get( world.shapes, contact.shapeIdB );

			contactData[index].contactId = b2ContactId( contact.contactId + 1, bodyId.world0, 0, contact.generation );
			contactData[index].shapeIdA = b2ShapeId( shapeA.id + 1, bodyId.world0, shapeA.generation );
			contactData[index].shapeIdB = b2ShapeId( shapeB.id + 1, bodyId.world0, shapeB.generation );

			b2ContactSim* contactSim = b2GetContactSim( world, contact );
			contactData[index].manifold = contactSim.manifold;

			index += 1;
		}

		contactKey = contact.edges[edgeIndex].nextKey;
	}

	assert( index <= capacity );

	return index;
}

b2AABB b2Body_ComputeAABB(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return b2AABB();
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body.headShapeId == B2_NULL_INDEX )
	{
		b2Transform transform = b2GetBodyTransform( world, body.id );
		return b2AABB( transform.p, transform.p );
	}

	b2Shape* shape = b2ShapeArray_Get( world.shapes, body.headShapeId );
	b2AABB aabb = shape.aabb;
	while ( shape.nextShapeId != B2_NULL_INDEX )
	{
		shape = b2ShapeArray_Get( world.shapes, shape.nextShapeId );
		aabb = b2AABB_Union( aabb, shape.aabb );
	}

	return aabb;
}

void b2UpdateBodyMassData(b2World* world, b2Body* body)
{
	b2BodySim* bodySim = b2GetBodySim( world, body );

	// Compute mass data from shapes. Each shape has its own density.
	body.mass = 0.0f;
	body.inertia = 0.0f;

	bodySim.invMass = 0.0f;
	bodySim.invInertia = 0.0f;
	bodySim.localCenter = b2Vec2_zero;
	bodySim.minExtent = B2_HUGE;
	bodySim.maxExtent = 0.0f;

	// Static and kinematic sims have zero mass.
	if ( body.type != b2_dynamicBody )
	{
		bodySim.center = bodySim.transform.p;

		// Need extents for kinematic bodies for sleeping to work correctly.
		if ( body.type == b2_kinematicBody )
		{
			int shapeId = body.headShapeId;
			while ( shapeId != B2_NULL_INDEX )
			{
				const(b2Shape)* s = b2ShapeArray_Get( world.shapes, shapeId );

				b2ShapeExtent extent = b2ComputeShapeExtent( s, b2Vec2_zero );
				bodySim.minExtent = b2MinFloat( bodySim.minExtent, extent.minExtent );
				bodySim.maxExtent = b2MaxFloat( bodySim.maxExtent, extent.maxExtent );

				shapeId = s.nextShapeId;
			}
		}

		return;
	}

	int shapeCount = body.shapeCount;
	b2MassData* masses = cast(b2MassData*)b2AllocateArenaItem( &world.arena, cast(int)(shapeCount * b2MassData.sizeof), "mass data" );

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int shapeId = body.headShapeId;
	int shapeIndex = 0;
	while ( shapeId != B2_NULL_INDEX )
	{
		const(b2Shape)* s = b2ShapeArray_Get( world.shapes, shapeId );
		shapeId = s.nextShapeId;

		if ( s.density == 0.0f )
		{
			masses[shapeIndex] = b2MassData( 0 );
			continue;
		}

		b2MassData massData = b2ComputeShapeMass( s );
		body.mass += massData.mass;
		localCenter = b2MulAdd( localCenter, massData.mass, massData.center );

		masses[shapeIndex] = massData;
		shapeIndex += 1;
	}

	// Compute center of mass.
	if ( body.mass > 0.0f )
	{
		bodySim.invMass = 1.0f / body.mass;
		localCenter = b2MulSV( bodySim.invMass, localCenter );
	}

	// Second loop to accumulate the rotational inertia about the center of mass
	for ( shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
	{
		b2MassData massData = masses[shapeIndex];
		if (massData.mass == 0.0f)
		{
			continue;
		}

		// Shift to center of mass. This is safe because it can only increase.
		b2Vec2 offset = b2Sub( localCenter, massData.center );
		float inertia = massData.rotationalInertia + massData.mass * b2Dot( offset, offset );
		body.inertia += inertia;
	}

	b2FreeArenaItem( &world.arena, masses );
	masses = null;

	assert( body.inertia >= 0.0f );

	if ( body.inertia > 0.0f )
	{
		bodySim.invInertia = 1.0f / body.inertia;
	}
	else
	{
		body.inertia = 0.0f;
		bodySim.invInertia = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = bodySim.center;
	bodySim.localCenter = localCenter;
	bodySim.center = b2TransformPoint( bodySim.transform, bodySim.localCenter );
	bodySim.center0 = bodySim.center;

	// Update center of mass velocity
	b2BodyState* state = b2GetBodyState( world, body );
	if ( state != null )
	{
		b2Vec2 deltaLinear = b2CrossSV( state.angularVelocity, b2Sub( bodySim.center, oldCenter ) );
		state.linearVelocity = b2Add( state.linearVelocity, deltaLinear );
	}

	// Compute body extents relative to center of mass
	shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		const(b2Shape)* s = b2ShapeArray_Get( world.shapes, shapeId );

		b2ShapeExtent extent = b2ComputeShapeExtent( s, localCenter );
		bodySim.minExtent = b2MinFloat( bodySim.minExtent, extent.minExtent );
		bodySim.maxExtent = b2MaxFloat( bodySim.maxExtent, extent.maxExtent );

		shapeId = s.nextShapeId;
	}
}

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return transform.p;
}

b2Rot b2Body_GetRotation(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return transform.q;
}

b2Transform b2Body_GetTransform(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return b2GetBodyTransformQuick( world, body );
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2InvTransformPoint( transform, worldPoint );
}

b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2TransformPoint( transform, localPoint );
}

b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2InvRotateVector( transform.q, worldVector );
}

b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	return b2RotateVector( transform.q, localVector );
}

void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, b2Rot rotation)
{
	assert( b2IsValidVec2( position ) );
	assert( b2IsValidRotation( rotation ) );
	assert( b2Body_IsValid( bodyId ) );
	b2World* world = b2GetWorld( bodyId.world0 );
	assert( world.locked == false );

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );

	bodySim.transform.p = position;
	bodySim.transform.q = rotation;
	bodySim.center = b2TransformPoint( bodySim.transform, bodySim.localCenter );

	bodySim.rotation0 = bodySim.transform.q;
	bodySim.center0 = bodySim.center;

	b2BroadPhase* broadPhase = &world.broadPhase;

	b2Transform transform = bodySim.transform;
	const(float) margin = B2_AABB_MARGIN;
	const(float) speculativeDistance = B2_SPECULATIVE_DISTANCE;

	int shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
		b2AABB aabb = b2ComputeShapeAABB( shape, transform );
		aabb.lowerBound.x -= speculativeDistance;
		aabb.lowerBound.y -= speculativeDistance;
		aabb.upperBound.x += speculativeDistance;
		aabb.upperBound.y += speculativeDistance;
		shape.aabb = aabb;

		if ( b2AABB_Contains( shape.fatAABB, aabb ) == false )
		{
			b2AABB fatAABB = void;
			fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
			fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
			fatAABB.upperBound.x = aabb.upperBound.x + margin;
			fatAABB.upperBound.y = aabb.upperBound.y + margin;
			shape.fatAABB = fatAABB;

			// They body could be disabled
			if ( shape.proxyKey != B2_NULL_INDEX )
			{
				b2BroadPhase_MoveProxy( broadPhase, shape.proxyKey, fatAABB );
			}
		}

		shapeId = shape.nextShapeId;
	}
}

b2Vec2 b2Body_GetLinearVelocity(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodyState* state = b2GetBodyState( world, body );
	if ( state != null )
	{
		return state.linearVelocity;
	}
	return b2Vec2_zero;
}

float b2Body_GetAngularVelocity(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodyState* state = b2GetBodyState( world, body );
	if ( state != null )
	{
		return state.angularVelocity;
	}
	return 0.0;
}

void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type == b2_staticBody )
	{
		return;
	}

	if ( b2LengthSquared( linearVelocity ) > 0.0f )
	{
		b2WakeBody( world, body );
	}

	b2BodyState* state = b2GetBodyState( world, body );
	if ( state == null )
	{
		return;
	}

	state.linearVelocity = linearVelocity;
}

void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type == b2_staticBody || ( body.flags & b2_lockAngularZ ) )
	{
		return;
	}

	if ( angularVelocity != 0.0f )
	{
		b2WakeBody( world, body );
	}

	b2BodyState* state = b2GetBodyState( world, body );
	if ( state == null )
	{
		return;
	}

	state.angularVelocity = angularVelocity;
}

void b2Body_SetTargetTransform(b2BodyId bodyId, b2Transform target, float timeStep)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( body.type == b2_staticBody || timeStep <= 0.0f )
	{
		return;
	}

	b2BodySim* sim = b2GetBodySim( world, body );

	// Compute linear velocity
	b2Vec2 center1 = sim.center;
	b2Vec2 center2 = b2TransformPoint( target, sim.localCenter );
	float invTimeStep = 1.0f / timeStep;
	b2Vec2 linearVelocity = b2MulSV( invTimeStep, b2Sub( center2, center1 ) );

	// Compute angular velocity
	b2Rot q1 = sim.transform.q;
	b2Rot q2 = target.q;
	float deltaAngle = b2RelativeAngle( q1, q2 );
	float angularVelocity = invTimeStep * deltaAngle;

	// Early out if the body is asleep already and the desired movement is small
	if ( body.setIndex != b2_awakeSet )
	{
		float maxVelocity = b2Length( linearVelocity ) + b2AbsFloat( angularVelocity ) * sim.maxExtent;

		// Return if velocity would be sleepy
		if ( maxVelocity < body.sleepThreshold )
		{
			return;
		}

		// Must wake for state to exist
		b2WakeBody( world, body );
	}

	assert( body.setIndex == b2_awakeSet );

	b2BodyState* state = b2GetBodyState( world, body );
	state.linearVelocity = linearVelocity;
	state.angularVelocity = angularVelocity;
}

b2Vec2 b2Body_GetLocalPointVelocity(b2BodyId bodyId, b2Vec2 localPoint)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodyState* state = b2GetBodyState( world, body );
	if ( state == null )
	{
		return b2Vec2_zero;
	}

	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, body.setIndex );
	b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, body.localIndex );

	b2Vec2 r = b2RotateVector( bodySim.transform.q, b2Sub( localPoint, bodySim.localCenter ) );
	b2Vec2 v = b2Add( state.linearVelocity, b2CrossSV( state.angularVelocity, r ) );
	return v;
}

b2Vec2 b2Body_GetWorldPointVelocity(b2BodyId bodyId, b2Vec2 worldPoint)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodyState* state = b2GetBodyState( world, body );
	if ( state == null )
	{
		return b2Vec2_zero;
	}

	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, body.setIndex );
	b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, body.localIndex );

	b2Vec2 r = b2Sub( worldPoint, bodySim.center );
	b2Vec2 v = b2Add( state.linearVelocity, b2CrossSV( state.angularVelocity, r ) );
	return v;
}

void b2Body_ApplyForce(b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type != b2_dynamicBody || body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body.setIndex == b2_awakeSet )
	{
		b2BodySim* bodySim = b2GetBodySim( world, body );
		bodySim.force = b2Add( bodySim.force, force );
		bodySim.torque += b2Cross( b2Sub( point, bodySim.center ), force );
	}
}

void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type != b2_dynamicBody || body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body.setIndex == b2_awakeSet )
	{
		b2BodySim* bodySim = b2GetBodySim( world, body );
		bodySim.force = b2Add( bodySim.force, force );
	}
}

void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type != b2_dynamicBody || body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body.setIndex == b2_awakeSet )
	{
		b2BodySim* bodySim = b2GetBodySim( world, body );
		bodySim.torque += torque;
	}
}

void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type != b2_dynamicBody || body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body.setIndex == b2_awakeSet )
	{
		int localIndex = body.localIndex;
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
		b2BodyState* state = b2BodyStateArray_Get( set.bodyStates, localIndex );
		b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, localIndex );
		state.linearVelocity = b2MulAdd( state.linearVelocity, bodySim.invMass, impulse );
		state.angularVelocity += bodySim.invInertia * b2Cross( b2Sub( point, bodySim.center ), impulse );

		b2LimitVelocity( state, world.maxLinearSpeed );
	}
}

void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type != b2_dynamicBody || body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}

	if ( body.setIndex == b2_awakeSet )
	{
		int localIndex = body.localIndex;
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
		b2BodyState* state = b2BodyStateArray_Get( set.bodyStates, localIndex );
		b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, localIndex );
		state.linearVelocity = b2MulAdd( state.linearVelocity, bodySim.invMass, impulse );

		b2LimitVelocity( state, world.maxLinearSpeed );
	}
}

void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake)
{
	assert( b2Body_IsValid( bodyId ) );
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( body.type != b2_dynamicBody || body.setIndex == b2_disabledSet )
	{
		return;
	}

	if ( wake && body.setIndex >= b2_firstSleepingSet )
	{
		// this will not invalidate body pointer
		b2WakeBody( world, body );
	}

	if ( body.setIndex == b2_awakeSet )
	{
		int localIndex = body.localIndex;
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
		b2BodyState* state = b2BodyStateArray_Get( set.bodyStates, localIndex );
		b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, localIndex );
		state.angularVelocity += bodySim.invInertia * impulse;
	}
}

b2BodyType b2Body_GetType(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.type;
}

// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
// todo with a bit more effort I could support an option to let the body sleep
//
// Revised steps:
// 1 Skip disabled bodies
// 2 Destroy all contacts on the body
// 3 Wake the body
// 4 For all joints attached to the body
//  - wake attached bodies
//  - remove from island
//  - move to static set temporarily
// 5 Change the body type and transfer the body
// 6 If the body was static
//   - create an island for the body
//   Else if the body is becoming static
//   - remove it from the island
// 7 For all joints
//  - if either body is non-static
//    - link into island
//    - transfer to constraint graph
// 8 For all shapes
//  - Destroy proxy in old tree
//  - Create proxy in new tree
// Notes:
// - the implementation below tries to minimize the number of predicates, so some
//   operations may have no effect, such as transferring a joint to the same set
void b2Body_SetType(b2BodyId bodyId, b2BodyType type)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	b2BodyType originalType = body.type;
	if ( originalType == type )
	{
		return;
	}

	// Stage 1: skip disabled bodies
	if ( body.setIndex == b2_disabledSet )
	{
		// Disabled bodies don't change solver sets or islands when they change type.
		body.type = type;

		// Body type affects the mass properties
		b2UpdateBodyMassData( world, body );
		return;
	}

	// Stage 2: destroy all contacts but don't wake bodies (because we don't need to)
	bool wakeBodies = false;
	b2DestroyBodyContacts( world, body, wakeBodies );

	// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
	// all bodies in the same sleeping solver set.
	b2WakeBody( world, body );

	// Stage 4: move joints to temporary storage
	b2SolverSet* staticSet = b2SolverSetArray_Get( world.solverSets, b2_staticSet );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );

	int jointKey = body.headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;
		
		b2Joint* joint = b2JointArray_Get( world.joints, jointId );
		jointKey = joint.edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if ( joint.setIndex == b2_disabledSet )
		{
			continue;
		}

		// Wake attached bodies. The b2WakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may have
		// no joints.
		b2Body* bodyA = b2BodyArray_Get( world.bodies, joint.edges[0].bodyId );
		b2Body* bodyB = b2BodyArray_Get( world.bodies, joint.edges[1].bodyId );
		b2WakeBody( world, bodyA );
		b2WakeBody( world, bodyB );

		// Remove joint from island
		b2UnlinkJoint( world, joint );

		// It is necessary to transfer all joints to the static set
		// so they can be added to the constraint graph below and acquire consistent colors.
		b2SolverSet* jointSourceSet = b2SolverSetArray_Get( world.solverSets, joint.setIndex );
		b2TransferJoint( world, staticSet, jointSourceSet, joint );
	}

	// Stage 5: change the body type and transfer body
	body.type = type;

	b2SolverSet* sourceSet = b2SolverSetArray_Get(world.solverSets, body.setIndex);
	b2SolverSet* targetSet = type == b2_staticBody ? staticSet : awakeSet;
	
	// Transfer body
	b2TransferBody( world, targetSet, sourceSet, body );

	// Stage 6: update island participation for the body
	if ( originalType == b2_staticBody )
	{
		// Create island for body
		b2CreateIslandForBody( world, b2_awakeSet, body );
	}
	else if ( type == b2_staticBody )
	{
		// Remove body from island.
		b2RemoveBodyFromIsland( world, body );
	}

	// Stage 7: Transfer joints to the target set
	jointKey = body.headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );

		jointKey = joint.edges[edgeIndex].nextKey;

		// Joint may be disabled by other body
		if (joint.setIndex == b2_disabledSet)
		{
			continue;
		}
		
		// All joints were transfered to the static set in an earlier stage
		assert ( joint.setIndex == b2_staticSet );
		
		b2Body* bodyA = b2BodyArray_Get( world.bodies, joint.edges[0].bodyId );
		b2Body* bodyB = b2BodyArray_Get( world.bodies, joint.edges[1].bodyId );
		assert(bodyA.setIndex == b2_staticSet || bodyA.setIndex == b2_awakeSet);
		assert(bodyB.setIndex == b2_staticSet || bodyB.setIndex == b2_awakeSet);
		
		if (bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet)
		{
			b2TransferJoint( world, awakeSet, staticSet, joint );
		}
	}

	// Recreate shape proxies in broadphase
   b2Transform transform = b2GetBodyTransformQuick( world, body );
   int shapeId = body.headShapeId;
   while ( shapeId != B2_NULL_INDEX )
   {
	   b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
	   shapeId = shape.nextShapeId;
	   b2DestroyShapeProxy( shape, &world.broadPhase );
	   bool forcePairCreation = true;
	   b2CreateShapeProxy( shape, &world.broadPhase, type, transform, forcePairCreation );
   }

	// Relink all joints
	jointKey = body.headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );
		jointKey = joint.edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		int otherBodyId = joint.edges[otherEdgeIndex].bodyId;
		b2Body* otherBody = b2BodyArray_Get( world.bodies, otherBodyId );

		if ( otherBody.setIndex == b2_disabledSet )
		{
			continue;
		}

		if ( body.type == b2_staticBody && otherBody.type == b2_staticBody )
		{
			continue;
		}

		b2LinkJoint( world, joint );
	}

	// Body type affects the mass
	b2UpdateBodyMassData( world, body );

	b2ValidateSolverSets( world );
	b2ValidateIsland(world, body.islandId);
}

void b2Body_SetName(b2BodyId bodyId, const(char)* name)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( name != null )
	{
		for ( int i = 0; i < 31; ++i )
		{
			body.name[i] = name[i];
		}

		body.name[31] = 0;
	}
	else
	{
		import core.stdc.string;
		memset( &body.name, 0, cast(int)(32 * char.sizeof) );
	}
}

const(char)* b2Body_GetName(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.name.stringof;
}

void b2Body_SetUserData(b2BodyId bodyId, void* userData)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	body.userData = userData;
}

void* b2Body_GetUserData(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.userData;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.mass;
}

float b2Body_GetRotationalInertia(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.inertia;
}

b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	return bodySim.localCenter;
}

b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	return bodySim.center;
}

void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData)
{
	assert( b2IsValidFloat( massData.mass ) && massData.mass >= 0.0f );
	assert( b2IsValidFloat( massData.rotationalInertia ) && massData.rotationalInertia >= 0.0f );
	assert( b2IsValidVec2( massData.center ) );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );

	body.mass = massData.mass;
	body.inertia = massData.rotationalInertia;
	bodySim.localCenter = massData.center;

	b2Vec2 center = b2TransformPoint( bodySim.transform, massData.center );
	bodySim.center = center;
	bodySim.center0 = center;

	bodySim.invMass = body.mass > 0.0f ? 1.0f / body.mass : 0.0f;
	bodySim.invInertia = body.inertia > 0.0f ? 1.0f / body.inertia : 0.0f;
}

b2MassData b2Body_GetMassData(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	b2MassData massData = { body.mass, bodySim.localCenter, body.inertia };
	return massData;
}

void b2Body_ApplyMassFromShapes(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2UpdateBodyMassData( world, body );
}

void b2Body_SetLinearDamping(b2BodyId bodyId, float linearDamping)
{
	assert( b2IsValidFloat( linearDamping ) && linearDamping >= 0.0f );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	bodySim.linearDamping = linearDamping;
}

float b2Body_GetLinearDamping(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	return bodySim.linearDamping;
}

void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping)
{
	assert( b2IsValidFloat( angularDamping ) && angularDamping >= 0.0f );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	bodySim.angularDamping = angularDamping;
}

float b2Body_GetAngularDamping(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	return bodySim.angularDamping;
}

void b2Body_SetGravityScale(b2BodyId bodyId, float gravityScale)
{
	assert( b2Body_IsValid( bodyId ) );
	assert( b2IsValidFloat( gravityScale ) );

	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	bodySim.gravityScale = gravityScale;
}

float b2Body_GetGravityScale(b2BodyId bodyId)
{
	assert( b2Body_IsValid( bodyId ) );
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	return bodySim.gravityScale;
}

bool b2Body_IsAwake(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.setIndex == b2_awakeSet;
}

void b2Body_SetAwake(b2BodyId bodyId, bool awake)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );

	if ( awake && body.setIndex >= b2_firstSleepingSet )
	{
		b2WakeBody( world, body );
	}
	else if ( awake == false && body.setIndex == b2_awakeSet )
	{
		b2Island* island = b2IslandArray_Get( world.islands, body.islandId );
		if ( island.constraintRemoveCount > 0 )
		{
			// Must split the island before sleeping. This is expensive.
			b2SplitIsland( world, body.islandId );
		}

		b2TrySleepIsland( world, body.islandId );
	}
}

bool b2Body_IsEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.setIndex != b2_disabledSet;
}

bool b2Body_IsSleepEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.enableSleep;
}

void b2Body_SetSleepThreshold(b2BodyId bodyId, float sleepThreshold)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	body.sleepThreshold = sleepThreshold;
}

float b2Body_GetSleepThreshold(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.sleepThreshold;
}

void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	body.enableSleep = enableSleep;

	if ( enableSleep == false )
	{
		b2WakeBody( world, body );
	}
}

// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
// The most challenging aspect is that joints may connect to bodies that are not disabled.
void b2Body_Disable(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body.setIndex == b2_disabledSet )
	{
		return;
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	bool wakeBodies = true;
	b2DestroyBodyContacts( world, body, wakeBodies );

	// The current solver set of the body
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, body.setIndex );

	// Disabled bodies and connected joints are moved to the disabled set
	b2SolverSet* disabledSet = b2SolverSetArray_Get( world.solverSets, b2_disabledSet );

	// Unlink joints and transfer them to the disabled set
	int jointKey = body.headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );
		jointKey = joint.edges[edgeIndex].nextKey;

		// joint may already be disabled by other body
		if ( joint.setIndex == b2_disabledSet )
		{
			continue;
		}

		assert( joint.setIndex == set.setIndex || set.setIndex == b2_staticSet );

		// Remove joint from island
		b2UnlinkJoint( world, joint );

		// Transfer joint to disabled set
		b2SolverSet* jointSet = b2SolverSetArray_Get( world.solverSets, joint.setIndex );
		b2TransferJoint( world, disabledSet, jointSet, joint );
	}
	
	// Remove shapes from broad-phase
	int shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
		shapeId = shape.nextShapeId;
		b2DestroyShapeProxy( shape, &world.broadPhase );
	}

	// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
	b2RemoveBodyFromIsland( world, body );

	// Transfer body sim
	b2TransferBody( world, disabledSet, set, body );

	b2ValidateConnectivity( world );
	b2ValidateSolverSets( world );
}

void b2Body_Enable(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body.setIndex != b2_disabledSet )
	{
		return;
	}

	b2SolverSet* disabledSet = b2SolverSetArray_Get( world.solverSets, b2_disabledSet );
	int setId = body.type == b2_staticBody ? b2_staticSet : b2_awakeSet;
	b2SolverSet* targetSet = b2SolverSetArray_Get( world.solverSets, setId );

	b2TransferBody( world, targetSet, disabledSet, body );

	b2Transform transform = b2GetBodyTransformQuick( world, body );

	// Add shapes to broad-phase
	b2BodyType proxyType = body.type;
	bool forcePairCreation = true;
	int shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
		shapeId = shape.nextShapeId;

		b2CreateShapeProxy( shape, &world.broadPhase, proxyType, transform, forcePairCreation );
	}

	if ( setId != b2_staticSet )
	{
		b2CreateIslandForBody( world, setId, body );
	}

	// Transfer joints. If the other body is disabled, don't transfer.
	// If the other body is sleeping, wake it.
	int jointKey = body.headJointKey;
	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );
		assert( joint.setIndex == b2_disabledSet );
		assert( joint.islandId == B2_NULL_INDEX );

		jointKey = joint.edges[edgeIndex].nextKey;

		b2Body* bodyA = b2BodyArray_Get( world.bodies, joint.edges[0].bodyId );
		b2Body* bodyB = b2BodyArray_Get( world.bodies, joint.edges[1].bodyId );

		if ( bodyA.setIndex == b2_disabledSet || bodyB.setIndex == b2_disabledSet )
		{
			// one body is still disabled
			continue;
		}

		// Transfer joint first
		int jointSetId = void;
		if ( bodyA.setIndex == b2_staticSet && bodyB.setIndex == b2_staticSet )
		{
			jointSetId = b2_staticSet;
		}
		else if ( bodyA.setIndex == b2_staticSet )
		{
			jointSetId = bodyB.setIndex;
		}
		else
		{
			jointSetId = bodyA.setIndex;
		}

		b2SolverSet* jointSet = b2SolverSetArray_Get( world.solverSets, jointSetId );
		b2TransferJoint( world, jointSet, disabledSet, joint );

		// Now that the joint is in the correct set, I can link the joint in the island.
		if ( jointSetId != b2_staticSet )
		{
			b2LinkJoint( world, joint );
		}
	}

	b2ValidateSolverSets( world );
}

void b2Body_SetMotionLocks(b2BodyId bodyId, b2MotionLocks locks)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	uint newFlags = 0;
	newFlags |= locks.linearX ? b2_lockLinearX : 0;
	newFlags |= locks.linearY ? b2_lockLinearY : 0;
	newFlags |= locks.angularZ ? b2_lockAngularZ : 0;

	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( ( body.flags & b2_allLocks ) != newFlags )
	{
		body.flags &= ~b2_allLocks;
		body.flags |= newFlags;

		b2BodySim* bodySim = b2GetBodySim( world, body );
		bodySim.flags &= ~b2_allLocks;
		bodySim.flags |= newFlags;

		b2BodyState* state = b2GetBodyState( world, body );

		if ( state != null )
		{
			state.flags = body.flags;

			if (locks.linearX)
			{
				state.linearVelocity.x = 0.0f;
			}

			if (locks.linearY)
			{
				state.linearVelocity.y = 0.0f;
			}

			if (locks.angularZ)
			{
				state.angularVelocity = 0.0f;
			}
		}
	}
}

b2MotionLocks b2Body_GetMotionLocks(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );

	b2MotionLocks locks = void;
	locks.linearX = cast(bool)( body.flags & b2_lockLinearX );
	locks.linearY = cast(bool)( body.flags & b2_lockLinearY );
	locks.angularZ = cast(bool)( body.flags & b2_lockAngularZ );
	return locks;
}

void b2Body_SetBullet(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorldLocked( bodyId.world0 );
	if ( world == null )
	{
		return;
	}

	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );

	if ( flag )
	{
		bodySim.flags |= b2_isBullet;
	}
	else
	{
		bodySim.flags &= ~b2_isBullet;
	}
}

bool b2Body_IsBullet(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );
	return ( bodySim.flags & b2_isBullet ) != 0;
}

void b2Body_EnableContactEvents(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
		shape.enableContactEvents = flag;
		shapeId = shape.nextShapeId;
	}
}

void b2Body_EnableHitEvents(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int shapeId = body.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
		shape.enableHitEvents = flag;
		shapeId = shape.nextShapeId;
	}
}

b2WorldId b2Body_GetWorld(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	return b2WorldId( cast(ushort)(bodyId.world0 + 1), world.generation );
}

int b2Body_GetShapeCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.shapeCount;
}

int b2Body_GetShapes(b2BodyId bodyId, b2ShapeId* shapeArray, int capacity)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int shapeId = body.headShapeId;
	int shapeCount = 0;
	while ( shapeId != B2_NULL_INDEX && shapeCount < capacity )
	{
		b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
		b2ShapeId id = { shape.id + 1, bodyId.world0, shape.generation };
		shapeArray[shapeCount] = id;
		shapeCount += 1;

		shapeId = shape.nextShapeId;
	}

	return shapeCount;
}

int b2Body_GetJointCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	return body.jointCount;
}

int b2Body_GetJoints(b2BodyId bodyId, b2JointId* jointArray, int capacity)
{
	b2World* world = b2GetWorld( bodyId.world0 );
	b2Body* body = b2GetBodyFullId( world, bodyId );
	int jointKey = body.headJointKey;

	int jointCount = 0;
	while ( jointKey != B2_NULL_INDEX && jointCount < capacity )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );

		b2JointId id = { jointId + 1, bodyId.world0, joint.generation };
		jointArray[jointCount] = id;
		jointCount += 1;

		jointKey = joint.edges[edgeIndex].nextKey;
	}

	return jointCount;
}

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	if ( bodyA.type != b2_dynamicBody && bodyB.type != b2_dynamicBody )
	{
		return false;
	}

	int jointKey = void;
	int otherBodyId = void;
	if ( bodyA.jointCount < bodyB.jointCount )
	{
		jointKey = bodyA.headJointKey;
		otherBodyId = bodyB.id;
	}
	else
	{
		jointKey = bodyB.headJointKey;
		otherBodyId = bodyA.id;
	}

	while ( jointKey != B2_NULL_INDEX )
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;
		int otherEdgeIndex = edgeIndex ^ 1;

		b2Joint* joint = b2JointArray_Get( world.joints, jointId );
		if ( joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			return false;
		}

		jointKey = joint.edges[edgeIndex].nextKey;
	}

	return true;
}
