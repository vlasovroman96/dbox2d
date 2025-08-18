module dbox2d.contact;
// @nogc nothrow:
// extern(C): __gshared:
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

//#pragma once

public import dbox2d.array;
public import dbox2d.core;
import dbox2d.island;

public import dbox2d.collision;
public import dbox2d.types;
import dbox2d.shape;
import dbox2d.physics_world;
import dbox2d.solver_set;
import dbox2d.body;
import dbox2d.manifold;

mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2Body","b2Body"));
mixin(B2_ARRAY_SOURCE!("b2Contact","b2Contact"));
mixin(B2_ARRAY_SOURCE!("b2ContactSim","b2ContactSim"));
mixin(B2_ARRAY_SOURCE!("b2Shape","b2Shape"));



enum b2ContactFlags
{
	// Set when the solid shapes are touching.
	b2_contactTouchingFlag = 0x00000001,

	// Contact has a hit event
	b2_contactHitEventFlag = 0x00000002,

	// This contact wants contact events
	b2_contactEnableContactEvents = 0x00000004,
}
alias b2_contactTouchingFlag = b2ContactFlags.b2_contactTouchingFlag;
alias b2_contactHitEventFlag = b2ContactFlags.b2_contactHitEventFlag;
alias b2_contactEnableContactEvents = b2ContactFlags.b2_contactEnableContactEvents;


// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
struct b2ContactEdge {
	int bodyId;
	int prevKey;
	int nextKey;
}

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
struct b2Contact {
	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array
	// B2_NULL_INDEX for non-touching or sleeping contacts
	// B2_NULL_INDEX when slot is free
	int colorIndex;

	// contact index within set or graph color
	// B2_NULL_INDEX when slot is free
	int localIndex;

	b2ContactEdge[2] edges;
	int shapeIdA;
	int shapeIdB;

	// A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
	int islandPrev;
	int islandNext;
	int islandId;

	int contactId;

	// b2ContactFlags
	uint flags;

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b2ContactId
	uint generation;

	bool isMarked;
}

// Shifted to be distinct from b2ContactFlags
enum b2ContactSimFlags
{
	// Set when the shapes are touching
	b2_simTouchingFlag = 0x00010000,

	// This contact no longer has overlapping AABBs
	b2_simDisjoint = 0x00020000,

	// This contact started touching
	b2_simStartedTouching = 0x00040000,

	// This contact stopped touching
	b2_simStoppedTouching = 0x00080000,

	// This contact has a hit event
	b2_simEnableHitEvent = 0x00100000,

	// This contact wants pre-solve events
	b2_simEnablePreSolveEvents = 0x00200000,
}
alias b2_simTouchingFlag = b2ContactSimFlags.b2_simTouchingFlag;
alias b2_simDisjoint = b2ContactSimFlags.b2_simDisjoint;
alias b2_simStartedTouching = b2ContactSimFlags.b2_simStartedTouching;
alias b2_simStoppedTouching = b2ContactSimFlags.b2_simStoppedTouching;
alias b2_simEnableHitEvent = b2ContactSimFlags.b2_simEnableHitEvent;
alias b2_simEnablePreSolveEvents = b2ContactSimFlags.b2_simEnablePreSolveEvents;


/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
struct b2ContactSim {
	int contactId;

static if (B2_VALIDATE) {
	int bodyIdA;
	int bodyIdB;
}

	int bodySimIndexA;
	int bodySimIndexB;

	int shapeIdA;
	int shapeIdB;

	float invMassA = 0;
	float invIA = 0;

	float invMassB = 0;
	float invIB = 0;

	b2Manifold manifold;

	// Mixed friction and restitution
	float friction = 0;
	float restitution = 0;
	float rollingResistance = 0;
	float tangentSpeed = 0;

	// b2ContactSimFlags
	uint simFlags;

	b2SimplexCache cache;
}

void b2InitializeContactRegisters();

void b2CreateContact(b2World* world, b2Shape* shapeA, b2Shape* shapeB);
void b2DestroyContact(b2World* world, b2Contact* contact, bool wakeBodies);

b2ContactSim* b2GetContactSim(b2World* world, b2Contact* contact);


bool b2UpdateContact(b2World* world, b2ContactSim* contactSim, b2Shape* shapeA, b2Transform transformA, b2Vec2 centerOffsetA, b2Shape* shapeB, b2Transform transformB, b2Vec2 centerOffsetB);

b2Manifold b2ComputeManifold(b2Shape* shapeA, b2Transform transformA, b2Shape* shapeB, b2Transform transformB);

// alias b2ContactArray = b2Contact[];
// B2_ARRAY_INLINE( b2Contact, b2Contact )

// alias b2ContactSimArray = b2ContactSim[];
// B2_ARRAY_INLINE( b2ContactSim, b2ContactSim )

private b2Contact* b2GetContactFullId(b2World* world, b2ContactId contactId)
{
	int id = contactId.index1 - 1;
	b2Contact* contact = b2ContactArray_Get( world.contacts, id );
	B2_ASSERT( contact.contactId == id && contact.generation == contactId.generation );
	return contact;
}

b2ContactData b2Contact_GetData(b2ContactId contactId)
{
	b2World* world = b2GetWorld( contactId.world0 );
	b2Contact* contact = b2GetContactFullId( world, contactId );
	b2ContactSim* contactSim = b2GetContactSim( world, contact );
	const(b2Shape)* shapeA = b2ShapeArray_Get( world.shapes, contact.shapeIdA );
	const(b2Shape)* shapeB = b2ShapeArray_Get( world.shapes, contact.shapeIdB );

	b2ContactData data = {
		contactId: contactId,
		shapeIdA:
			{
				index1: shapeA.id + 1,
				world0: cast(ushort)contactId.world0,
				generation: shapeA.generation,
			},
		shapeIdB:
			{
				index1: shapeB.id + 1,
				world0: cast(ushort)contactId.world0,
				generation: shapeB.generation,
			},
		manifold: contactSim.manifold,
	};

	return data;
}

alias b2ManifoldFcn = b2Manifold function(
	const b2Shape* shapeA,
	b2Transform xfA,
	const b2Shape* shapeB,
	b2Transform xfB,
	b2SimplexCache* cache
	);

struct b2ContactRegister
{
	b2ManifoldFcn fcn;
	bool primary;
}

private b2ContactRegister[b2_shapeTypeCount][b2_shapeTypeCount] s_registers;
private bool s_initialized = false;

private b2Manifold b2CircleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideCircles( &shapeA.circle, xfA, &shapeB.circle, xfB );
}

private b2Manifold b2CapsuleAndCircleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideCapsuleAndCircle( &shapeA.capsule, xfA, &shapeB.circle, xfB );
}

private b2Manifold b2CapsuleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideCapsules( &shapeA.capsule, xfA, &shapeB.capsule, xfB );
}

private b2Manifold b2PolygonAndCircleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollidePolygonAndCircle( &shapeA.polygon, xfA, &shapeB.circle, xfB );
}

private b2Manifold b2PolygonAndCapsuleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollidePolygonAndCapsule( &shapeA.polygon, xfA, &shapeB.capsule, xfB );
}

private b2Manifold b2PolygonManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollidePolygons( &shapeA.polygon, xfA, &shapeB.polygon, xfB );
}

private b2Manifold b2SegmentAndCircleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideSegmentAndCircle( &shapeA.segment, xfA, &shapeB.circle, xfB );
}

private b2Manifold b2SegmentAndCapsuleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideSegmentAndCapsule( &shapeA.segment, xfA, &shapeB.capsule, xfB );
}

private b2Manifold b2SegmentAndPolygonManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideSegmentAndPolygon( &shapeA.segment, xfA, &shapeB.polygon, xfB );
}

private b2Manifold b2ChainSegmentAndCircleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	// B2_UNUSED( cache );
	return b2CollideChainSegmentAndCircle( &shapeA.chainSegment, xfA, &shapeB.circle, xfB );
}

private b2Manifold b2ChainSegmentAndCapsuleManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	return b2CollideChainSegmentAndCapsule( &shapeA.chainSegment, xfA, &shapeB.capsule, xfB, cache );
}

private b2Manifold b2ChainSegmentAndPolygonManifold(const(b2Shape)* shapeA, b2Transform xfA, const(b2Shape)* shapeB, b2Transform xfB, b2SimplexCache* cache)
{
	return b2CollideChainSegmentAndPolygon( &shapeA.chainSegment, xfA, &shapeB.polygon, xfB, cache );
}

private void b2AddType(b2ManifoldFcn fcn, b2ShapeType type1, b2ShapeType type2)
{
	B2_ASSERT( 0 <= type1 && type1 < b2_shapeTypeCount );
	B2_ASSERT( 0 <= type2 && type2 < b2_shapeTypeCount );

	s_registers[type1][type2].fcn = fcn;
	s_registers[type1][type2].primary = true;

	if ( type1 != type2 )
	{
		s_registers[type2][type1].fcn = fcn;
		s_registers[type2][type1].primary = false;
	}
}

void b2InitializeContactRegisters()
{
	if ( s_initialized == false )
	{
		b2AddType( &b2CircleManifold, b2_circleShape, b2_circleShape );
		b2AddType( &b2CapsuleAndCircleManifold, b2_capsuleShape, b2_circleShape );
		b2AddType( &b2CapsuleManifold, b2_capsuleShape, b2_capsuleShape );
		b2AddType( &b2PolygonAndCircleManifold, b2_polygonShape, b2_circleShape );
		b2AddType( &b2PolygonAndCapsuleManifold, b2_polygonShape, b2_capsuleShape );
		b2AddType( &b2PolygonManifold, b2_polygonShape, b2_polygonShape );
		b2AddType( &b2SegmentAndCircleManifold, b2_segmentShape, b2_circleShape );
		b2AddType( &b2SegmentAndCapsuleManifold, b2_segmentShape, b2_capsuleShape );
		b2AddType( &b2SegmentAndPolygonManifold, b2_segmentShape, b2_polygonShape );
		b2AddType( &b2ChainSegmentAndCircleManifold, b2_chainSegmentShape, b2_circleShape );
		b2AddType( &b2ChainSegmentAndCapsuleManifold, b2_chainSegmentShape, b2_capsuleShape );
		b2AddType( &b2ChainSegmentAndPolygonManifold, b2_chainSegmentShape, b2_polygonShape );
		s_initialized = true;
	}
}

void b2CreateContact(b2World* world, b2Shape* shapeA, b2Shape* shapeB)
{
	b2ShapeType type1 = shapeA.type;
	b2ShapeType type2 = shapeB.type;

	B2_ASSERT( 0 <= type1 && type1 < b2_shapeTypeCount );
	B2_ASSERT( 0 <= type2 && type2 < b2_shapeTypeCount );

	if ( s_registers[type1][type2].fcn == null )
	{
		// For example, no segment vs segment collision
		return;
	}

	if ( s_registers[type1][type2].primary == false )
	{
		// flip order
		b2CreateContact( world, shapeB, shapeA );
		return;
	}

	b2Body* bodyA = b2BodyArray_Get( world.bodies, shapeA.bodyId );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, shapeB.bodyId );

	B2_ASSERT( bodyA.setIndex != b2_disabledSet && bodyB.setIndex != b2_disabledSet );
	B2_ASSERT( bodyA.setIndex != b2_staticSet || bodyB.setIndex != b2_staticSet );

	int setIndex = void;
	if ( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet )
	{
		setIndex = b2_awakeSet;
	}
	else
	{
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island
		setIndex = b2_disabledSet;
	}

	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setIndex );

	// Create contact key and contact
	int contactId = b2AllocId( &world.contactIdPool );
	if ( contactId == world.contacts.count )
	{
		b2ContactArray_Push( world.contacts, b2Contact( 0 ) );
	}

	int shapeIdA = shapeA.id;
	int shapeIdB = shapeB.id;

	b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );
	contact.contactId = contactId;
	contact.generation += 1;
	contact.setIndex = setIndex;
	contact.colorIndex = B2_NULL_INDEX;
	contact.localIndex = cast(int)set.contactSims.count;
	contact.islandId = B2_NULL_INDEX;
	contact.islandPrev = B2_NULL_INDEX;
	contact.islandNext = B2_NULL_INDEX;
	contact.shapeIdA = shapeIdA;
	contact.shapeIdB = shapeIdB;
	contact.isMarked = false;
	contact.flags = 0;

	B2_ASSERT( shapeA.sensorIndex == B2_NULL_INDEX && shapeB.sensorIndex == B2_NULL_INDEX );

	if ( shapeA.enableContactEvents || shapeB.enableContactEvents )
	{
		contact.flags |= b2_contactEnableContactEvents;
	}

	// Connect to body A
	{
		contact.edges[0].bodyId = shapeA.bodyId;
		contact.edges[0].prevKey = B2_NULL_INDEX;
		contact.edges[0].nextKey = bodyA.headContactKey;

		int keyA = ( contactId << 1 ) | 0;
		int headContactKey = bodyA.headContactKey;
		if ( headContactKey != B2_NULL_INDEX )
		{
			b2Contact* headContact = b2ContactArray_Get( world.contacts, headContactKey >> 1 );
			headContact.edges[headContactKey & 1].prevKey = keyA;
		}
		bodyA.headContactKey = keyA;
		bodyA.contactCount += 1;
	}

	// Connect to body B
	{
		contact.edges[1].bodyId = shapeB.bodyId;
		contact.edges[1].prevKey = B2_NULL_INDEX;
		contact.edges[1].nextKey = bodyB.headContactKey;

		int keyB = ( contactId << 1 ) | 1;
		int headContactKey = bodyB.headContactKey;
		if ( bodyB.headContactKey != B2_NULL_INDEX )
		{
			b2Contact* headContact = b2ContactArray_Get( world.contacts, headContactKey >> 1 );
			headContact.edges[headContactKey & 1].prevKey = keyB;
		}
		bodyB.headContactKey = keyB;
		bodyB.contactCount += 1;
	}

	// Add to pair set for fast lookup
	ulong pairKey = B2_SHAPE_PAIR_KEY( shapeIdA, shapeIdB );
	b2AddKey( &world.broadPhase.pairSet, pairKey );

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	b2ContactSim* contactSim = b2ContactSimArray_Add( set.contactSims );
	contactSim.contactId = contactId;

static if (B2_VALIDATE) {
	contactSim.bodyIdA = shapeA.bodyId;
	contactSim.bodyIdB = shapeB.bodyId;
}

	contactSim.bodySimIndexA = B2_NULL_INDEX;
	contactSim.bodySimIndexB = B2_NULL_INDEX;
	contactSim.invMassA = 0.0f;
	contactSim.invIA = 0.0f;
	contactSim.invMassB = 0.0f;
	contactSim.invIB = 0.0f;
	contactSim.shapeIdA = shapeIdA;
	contactSim.shapeIdB = shapeIdB;
	contactSim.cache = b2_emptySimplexCache;
	contactSim.manifold = b2Manifold();

	// These also get updated in the narrow phase
	contactSim.friction =
		world.frictionCallback( shapeA.friction, shapeA.userMaterialId, shapeB.friction, shapeB.userMaterialId );
	contactSim.restitution =
		world.restitutionCallback( shapeA.restitution, shapeA.userMaterialId, shapeB.restitution, shapeB.userMaterialId );

	contactSim.tangentSpeed = 0.0f;
	contactSim.simFlags = 0;

	if ( shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents )
	{
		contactSim.simFlags |= b2_simEnablePreSolveEvents;
	}
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
void b2DestroyContact(b2World* world, b2Contact* contact, bool wakeBodies)
{
	// Remove pair from set
	ulong pairKey = B2_SHAPE_PAIR_KEY( contact.shapeIdA, contact.shapeIdB );
	b2RemoveKey( &world.broadPhase.pairSet, pairKey );

	b2ContactEdge* edgeA = &contact.edges[0];
	b2ContactEdge* edgeB = &contact.edges[1];

	int bodyIdA = edgeA.bodyId;
	int bodyIdB = edgeB.bodyId;
	b2Body* bodyA = b2BodyArray_Get( world.bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, bodyIdB );

	uint flags = contact.flags;
	bool touching = ( flags & b2_contactTouchingFlag ) != 0;

	// End touch event
	if ( touching && ( flags & b2_contactEnableContactEvents ) != 0 )
	{
		ushort worldId = world.worldId;
		const(b2Shape)* shapeA = b2ShapeArray_Get( world.shapes, contact.shapeIdA );
		const(b2Shape)* shapeB = b2ShapeArray_Get( world.shapes, contact.shapeIdB );
		b2ShapeId shapeIdA = { shapeA.id + 1, worldId, shapeA.generation };
		b2ShapeId shapeIdB = { shapeB.id + 1, worldId, shapeB.generation };

		b2ContactId contactId = {
			index1: contact.contactId + 1,
			world0: world.worldId,
			padding: 0,
			generation: contact.generation,
		};

		b2ContactEndTouchEvent event = {
			shapeIdA: shapeIdA,
			shapeIdB: shapeIdB,
			contactId: contactId,
		};

		b2ContactEndTouchEventArray_Push( world.contactEndEvents[world.endEventArrayIndex], event );
	}

	// Remove from body A
	if ( edgeA.prevKey != B2_NULL_INDEX )
	{
		b2Contact* prevContact = b2ContactArray_Get( world.contacts, edgeA.prevKey >> 1 );
		b2ContactEdge* prevEdge = prevContact.edges.ptr + ( edgeA.prevKey & 1 );
		prevEdge.nextKey = edgeA.nextKey;
	}

	if ( edgeA.nextKey != B2_NULL_INDEX )
	{
		b2Contact* nextContact = b2ContactArray_Get( world.contacts, edgeA.nextKey >> 1 );
		b2ContactEdge* nextEdge = nextContact.edges.ptr + ( edgeA.nextKey & 1 );
		nextEdge.prevKey = edgeA.prevKey;
	}

	int contactId = contact.contactId;

	int edgeKeyA = ( contactId << 1 ) | 0;
	if ( bodyA.headContactKey == edgeKeyA )
	{
		bodyA.headContactKey = edgeA.nextKey;
	}

	bodyA.contactCount -= 1;

	// Remove from body B
	if ( edgeB.prevKey != B2_NULL_INDEX )
	{
		b2Contact* prevContact = b2ContactArray_Get( world.contacts, edgeB.prevKey >> 1 );
		b2ContactEdge* prevEdge = prevContact.edges.ptr + ( edgeB.prevKey & 1 );
		prevEdge.nextKey = edgeB.nextKey;
	}

	if ( edgeB.nextKey != B2_NULL_INDEX )
	{
		b2Contact* nextContact = b2ContactArray_Get( world.contacts, edgeB.nextKey >> 1 );
		b2ContactEdge* nextEdge = &nextContact.edges [edgeB.nextKey & 1 ];
		nextEdge.prevKey = edgeB.prevKey;
	}

	int edgeKeyB = ( contactId << 1 ) | 1;
	if ( bodyB.headContactKey == edgeKeyB )
	{
		bodyB.headContactKey = edgeB.nextKey;
	}

	bodyB.contactCount -= 1;

	// Remove contact from the array that owns it
	if ( contact.islandId != B2_NULL_INDEX )
	{
		b2UnlinkContact( world, contact );
	}

	if ( contact.colorIndex != B2_NULL_INDEX )
	{
		// contact is an active constraint
		B2_ASSERT( contact.setIndex == b2_awakeSet );
		b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex );
	}
	else
	{
		// contact is non-touching or is sleeping
		B2_ASSERT( contact.setIndex != b2_awakeSet || ( contact.flags & b2_contactTouchingFlag ) == 0 );
		b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, contact.setIndex );

		int movedIndex = b2ContactSimArray_RemoveSwap( set.contactSims, contact.localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			b2ContactSim* movedContactSim = &set.contactSims[contact.localIndex];
			b2Contact* movedContact = b2ContactArray_Get( world.contacts, movedContactSim.contactId );
			movedContact.localIndex = contact.localIndex;
		}
	}

	// Free contact and id (preserve generation)
	contact.contactId = B2_NULL_INDEX;
	contact.setIndex = B2_NULL_INDEX;
	contact.colorIndex = B2_NULL_INDEX;
	contact.localIndex = B2_NULL_INDEX;
	b2FreeId( &world.contactIdPool, contactId );

	if ( wakeBodies && touching )
	{
		b2WakeBody( world, bodyA );
		b2WakeBody( world, bodyB );
	}
}

b2ContactSim* b2GetContactSim(b2World* world, b2Contact* contact)
{
	if ( contact.setIndex == b2_awakeSet && contact.colorIndex != B2_NULL_INDEX )
	{
		// contact lives in constraint graph
		B2_ASSERT( 0 <= contact.colorIndex && contact.colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = world.constraintGraph.colors.ptr + contact.colorIndex;
		return b2ContactSimArray_Get( color.contactSims, contact.localIndex );
	}

	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, contact.setIndex );
	return b2ContactSimArray_Get( set.contactSims, contact.localIndex );
}

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
bool b2UpdateContact(b2World* world, b2ContactSim* contactSim, b2Shape* shapeA, b2Transform transformA, b2Vec2 centerOffsetA, b2Shape* shapeB, b2Transform transformB, b2Vec2 centerOffsetB)
{
	// Save old manifold
	b2Manifold oldManifold = contactSim.manifold;

	// Compute new manifold
	b2ManifoldFcn fcn = s_registers[shapeA.type][shapeB.type].fcn;
	contactSim.manifold = fcn( shapeA, transformA, shapeB, transformB, &contactSim.cache );

	// Keep these updated in case the values on the shapes are modified
	contactSim.friction =
		world.frictionCallback( shapeA.friction, shapeA.userMaterialId, shapeB.friction, shapeB.userMaterialId );
	contactSim.restitution =
		world.restitutionCallback( shapeA.restitution, shapeA.userMaterialId, shapeB.restitution, shapeB.userMaterialId );

	if ( shapeA.rollingResistance > 0.0f || shapeB.rollingResistance > 0.0f )
	{
		float radiusA = b2GetShapeRadius( shapeA );
		float radiusB = b2GetShapeRadius( shapeB );
		float maxRadius = b2MaxFloat( radiusA, radiusB );
		contactSim.rollingResistance = b2MaxFloat( shapeA.rollingResistance, shapeB.rollingResistance ) * maxRadius;
	}
	else
	{
		contactSim.rollingResistance = 0.0f;
	}

	contactSim.tangentSpeed = shapeA.tangentSpeed + shapeB.tangentSpeed;

	int pointCount = contactSim.manifold.pointCount;
	bool touching = pointCount > 0;

	if ( touching && world.preSolveFcn != null && ( contactSim.simFlags & b2_simEnablePreSolveEvents ) != 0 )
	{
		b2ShapeId shapeIdA = { shapeA.id + 1, world.worldId, shapeA.generation };
		b2ShapeId shapeIdB = { shapeB.id + 1, world.worldId, shapeB.generation };

		b2Manifold* manifold = &contactSim.manifold;
		float bestSeparation = manifold.points[0].separation;
		b2Vec2 bestPoint = manifold.points[0].point;

		// Get deepest point
		for ( int i = 1; i < manifold.pointCount; ++i )
		{
			float separation = manifold.points[i].separation;
			if ( separation < bestSeparation )
			{
				bestSeparation = separation;
				bestPoint = manifold.points[i].point;
			}
		}

		// this call assumes thread safety
		touching = world.preSolveFcn( shapeIdA, shapeIdB, bestPoint, manifold.normal, world.preSolveContext );
		if ( touching == false )
		{
			// disable contact
			pointCount = 0;
			manifold.pointCount = 0;
		}
	}

	// This flag is for testing
	if ( world.enableSpeculative == false && pointCount == 2 )
	{
		if ( contactSim.manifold.points[0].separation > 1.5f * B2_LINEAR_SLOP )
		{
			contactSim.manifold.points[0] = contactSim.manifold.points[1];
			contactSim.manifold.pointCount = 1;
		}
		else if ( contactSim.manifold.points[0].separation > 1.5f * B2_LINEAR_SLOP )
		{
			contactSim.manifold.pointCount = 1;
		}

		pointCount = contactSim.manifold.pointCount;
	}

	if ( touching && ( shapeA.enableHitEvents || shapeB.enableHitEvents ) )
	{
		contactSim.simFlags |= b2_simEnableHitEvent;
	}
	else
	{
		contactSim.simFlags &= ~b2_simEnableHitEvent;
	}

	if ( pointCount > 0 )
	{
		contactSim.manifold.rollingImpulse = oldManifold.rollingImpulse;
	}

	// Match old contact ids to new contact ids and copy the
	// stored impulses to warm start the solver.
	int unmatchedCount = 0;
	for ( int i = 0; i < pointCount; ++i )
	{
		b2ManifoldPoint* mp2 = contactSim.manifold.points.ptr + i;

		// shift anchors to be center of mass relative
		mp2.anchorA = b2Sub( mp2.anchorA, centerOffsetA );
		mp2.anchorB = b2Sub( mp2.anchorB, centerOffsetB );

		mp2.normalImpulse = 0.0f;
		mp2.tangentImpulse = 0.0f;
		mp2.totalNormalImpulse = 0.0f;
		mp2.normalVelocity = 0.0f;
		mp2.persisted = false;

		ushort id2 = mp2.id;

		for ( int j = 0; j < oldManifold.pointCount; ++j )
		{
			b2ManifoldPoint* mp1 = oldManifold.points.ptr + j;

			if ( mp1.id == id2 )
			{
				mp2.normalImpulse = mp1.normalImpulse;
				mp2.tangentImpulse = mp1.tangentImpulse;
				mp2.persisted = true;

				// clear old impulse
				mp1.normalImpulse = 0.0f;
				mp1.tangentImpulse = 0.0f;
				break;
			}
		}

		unmatchedCount += mp2.persisted ? 0 : 1;
	}

	// B2_UNUSED( unmatchedCount );

version (none) {
		// todo I haven't found an improvement from this yet
		// If there are unmatched new contact points, apply any left over old impulse.
		if (unmatchedCount > 0)
		{
			float unmatchedNormalImpulse = 0.0f;
			float unmatchedTangentImpulse = 0.0f;
			for (int i = 0; i < oldManifold.pointCount; ++i)
			{
				b2ManifoldPoint* mp = oldManifold.points + i;
				unmatchedNormalImpulse += mp.normalImpulse;
				unmatchedTangentImpulse += mp.tangentImpulse;
			}

			float inverse = 1.0f / unmatchedCount;
			unmatchedNormalImpulse *= inverse;
			unmatchedTangentImpulse *= inverse;

			for ( int i = 0; i < pointCount; ++i )
			{
				b2ManifoldPoint* mp2 = contactSim.manifold.points + i;

				if (mp2.persisted)
				{
					continue;
				}

				mp2.normalImpulse = unmatchedNormalImpulse;
				mp2.tangentImpulse = unmatchedTangentImpulse;
			}
		}
}

	if ( touching )
	{
		contactSim.simFlags |= b2_simTouchingFlag;
	}
	else
	{
		contactSim.simFlags &= ~b2_simTouchingFlag;
	}

	return touching;
}

b2Manifold b2ComputeManifold(b2Shape* shapeA, b2Transform transformA, b2Shape* shapeB, b2Transform transformB)
{
	b2ManifoldFcn fcn = s_registers[shapeA.type][shapeB.type].fcn;
	b2SimplexCache cache = { 0 };
	return fcn( shapeA, transformA, shapeB, transformB, &cache );
}
