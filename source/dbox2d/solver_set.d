module dbox2d.solver_set;

import dbox2d.array;
import dbox2d.body;
import dbox2d.island;
import dbox2d.physics_world;
import dbox2d.joint;
import dbox2d.contact;
import dbox2d.id_pool;
import dbox2d.core;
import dbox2d.constraint_graph;
import dbox2d.constants;
import dbox2d.bitset;

import core.stdc.string;

mixin(B2_ARRAY_SOURCE!("b2BodyState","b2BodyState"));
mixin(B2_ARRAY_SOURCE!("b2JointSim","b2JointSim"));
mixin(B2_ARRAY_SOURCE!("b2Joint","b2Joint"));
mixin(B2_ARRAY_SOURCE!("b2IslandSim","b2IslandSim"));
mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));
mixin(B2_ARRAY_SOURCE!("b2Body","b2Body"));
mixin(B2_ARRAY_SOURCE!("b2Contact","b2Contact"));
mixin(B2_ARRAY_SOURCE!("b2ContactSim","b2ContactSim"));
mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2BodyMoveEvent","b2BodyMoveEvent"));

// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
struct b2SolverSet {
	// Body array. Empty for unused set.
	b2BodySimArray bodySims;

	// Body state only exists for active set
	b2BodyStateArray bodyStates;

	// This holds sleeping/disabled joints. Empty for static/active set.
	b2JointSimArray jointSims;

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	b2ContactSimArray contactSims;

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	b2IslandSimArray islandSims;

	// Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
	int setIndex;
}

void b2DestroySolverSet(b2World* world, int setIndex)
{
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setIndex );
	b2BodySimArray_Destroy( set.bodySims );
	b2BodyStateArray_Destroy( set.bodyStates );
	b2ContactSimArray_Destroy( set.contactSims );
	b2JointSimArray_Destroy( set.jointSims );
	b2IslandSimArray_Destroy( set.islandSims );
	b2FreeId( &world.solverSetIdPool, setIndex );
	set =  new b2SolverSet;
	set.setIndex = B2_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
void b2WakeSolverSet(b2World* world, int setIndex)
{
	assert( setIndex >= b2_firstSleepingSet );
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setIndex );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	b2SolverSet* disabledSet = b2SolverSetArray_Get( world.solverSets, b2_disabledSet );

	b2Body* bodies = world.bodies.ptr;

	int bodyCount = cast(int)set.bodySims.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2BodySim* simSrc = &set.bodySims[i];

		b2Body* body = bodies + simSrc.bodyId;
		assert( body.setIndex == setIndex );
		body.setIndex = b2_awakeSet;
		body.localIndex = cast(int)awakeSet.bodySims.count;

		// Reset sleep timer
		body.sleepTime = 0.0f;

		b2BodySim* simDst = b2BodySimArray_Add( awakeSet.bodySims );
		memcpy( simDst, simSrc, b2BodySim.sizeof );

		b2BodyState* state = b2BodyStateArray_Add( awakeSet.bodyStates );
		*state = b2_identityBodyState;
		state.flags = body.flags;

		// move non-touching contacts from disabled set to awake set
		int contactKey = body.headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );

			contactKey = contact.edges[edgeIndex].nextKey;

			if ( contact.setIndex != b2_disabledSet )
			{
				assert( contact.setIndex == b2_awakeSet || contact.setIndex == setIndex );
				continue;
			}

			int localIndex = contact.localIndex;
			b2ContactSim* contactSim = b2ContactSimArray_Get( disabledSet.contactSims, localIndex );

			assert( ( contact.flags & b2_contactTouchingFlag ) == 0 && contactSim.manifold.pointCount == 0 );

			contact.setIndex = b2_awakeSet;
			contact.localIndex = cast(int)awakeSet.contactSims.count;
			b2ContactSim* awakeContactSim = b2ContactSimArray_Add( awakeSet.contactSims );
			memcpy( awakeContactSim, contactSim, b2ContactSim.sizeof );

			int movedLocalIndex = b2ContactSimArray_RemoveSwap( disabledSet.contactSims, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = &disabledSet.contactSims[localIndex];
				b2Contact* movedContact = b2ContactArray_Get( world.contacts, movedContactSim.contactId );
				assert( movedContact.localIndex == movedLocalIndex );
				movedContact.localIndex = localIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		int contactCount = cast(int)set.contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSim = &set.contactSims[i];
			b2Contact* contact = b2ContactArray_Get( world.contacts, contactSim.contactId );
			assert( contact.flags & b2_contactTouchingFlag );
			assert( contactSim.simFlags & b2_simTouchingFlag );
			assert( contactSim.manifold.pointCount > 0 );
			assert( contact.setIndex == setIndex );
			b2AddContactToGraph( world, contactSim, contact );
			contact.setIndex = b2_awakeSet;
		}
	}

	// transfer joints from sleeping set to awake set
	{
		int jointCount = cast(int)set.jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSim = set.jointSims.ptr + i;
			b2Joint* joint = b2JointArray_Get( world.joints, jointSim.jointId );
			assert( joint.setIndex == setIndex );
			b2AddJointToGraph( world, jointSim, joint );
			joint.setIndex = b2_awakeSet;
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		int islandCount = cast(int)set.islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = &set.islandSims[i];
			b2Island* island = b2IslandArray_Get( world.islands, islandSrc.islandId );
			island.setIndex = b2_awakeSet;
			island.localIndex = cast(int)awakeSet.islandSims.count;
			b2IslandSim* islandDst = b2IslandSimArray_Add( awakeSet.islandSims );
			memcpy( islandDst, islandSrc, b2IslandSim.sizeof );
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet( world, setIndex );
}

void b2TrySleepIsland(b2World* world, int islandId)
{
	b2Island* island = b2IslandArray_Get( world.islands, islandId );
	assert( island.setIndex == b2_awakeSet );

	// cannot put an island to sleep while it has a pending split
	if ( island.constraintRemoveCount > 0 )
	{
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	int sleepSetId = b2AllocId( &world.solverSetIdPool );
	if ( sleepSetId == cast(int)world.solverSets.count )
	{
		b2SolverSet set;
		set.setIndex = B2_NULL_INDEX;
		b2SolverSetArray_Push( world.solverSets, set );
	}

	b2SolverSet* sleepSet = b2SolverSetArray_Get( world.solverSets, sleepSetId );
	sleepSet =  new b2SolverSet;

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	assert( 0 <= island.localIndex && island.localIndex < awakeSet.islandSims.count );

	sleepSet.setIndex = sleepSetId;
	sleepSet.bodySims = b2BodySimArray_Create( island.bodyCount );
	sleepSet.contactSims = b2ContactSimArray_Create( island.contactCount );
	sleepSet.jointSims = b2JointSimArray_Create( island.jointCount );

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b2SolverSet* disabledSet = b2SolverSetArray_Get( world.solverSets, b2_disabledSet );
		int bodyId = island.headBody;
		while ( bodyId != B2_NULL_INDEX )
		{
			b2Body* body = b2BodyArray_Get( world.bodies, bodyId );
			assert( body.setIndex == b2_awakeSet );
			assert( body.islandId == islandId );

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if ( body.bodyMoveIndex != B2_NULL_INDEX )
			{
				b2BodyMoveEvent* moveEvent = b2BodyMoveEventArray_Get( world.bodyMoveEvents, body.bodyMoveIndex );
				assert( moveEvent.bodyId.index1 - 1 == bodyId );
				assert( moveEvent.bodyId.generation == body.generation );
				moveEvent.fellAsleep = true;
				body.bodyMoveIndex = B2_NULL_INDEX;
			}

			int awakeBodyIndex = body.localIndex;
			b2BodySim* awakeSim = b2BodySimArray_Get( awakeSet.bodySims, awakeBodyIndex );

			// move body sim to sleep set

			int sleepBodyIndex = cast(int)sleepSet.bodySims.count;
			b2BodySim* sleepBodySim = b2BodySimArray_Add( sleepSet.bodySims );
			memcpy( sleepBodySim, awakeSim, b2BodySim.sizeof );

			int movedIndex = b2BodySimArray_RemoveSwap( awakeSet.bodySims, awakeBodyIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix local index on moved element
				b2BodySim* movedSim = &awakeSet.bodySims[awakeBodyIndex];
				int movedId = movedSim.bodyId;
				b2Body* movedBody = b2BodyArray_Get( world.bodies, movedId );
				assert( movedBody.localIndex == movedIndex );
				movedBody.localIndex = awakeBodyIndex;
			}

			// destroy state, no need to clone
			b2BodyStateArray_RemoveSwap( awakeSet.bodyStates, awakeBodyIndex );

			body.setIndex = sleepSetId;
			body.localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body.headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );

				assert( contact.setIndex == b2_awakeSet || contact.setIndex == b2_disabledSet );
				contactKey = contact.edges[edgeIndex].nextKey;

				if ( contact.setIndex == b2_disabledSet )
				{
					// already moved to disabled set by another body in the island
					continue;
				}

				if ( contact.colorIndex != B2_NULL_INDEX )
				{
					// contact is touching and will be moved separately
					assert( ( contact.flags & b2_contactTouchingFlag ) != 0 );
					continue;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact.edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = b2BodyArray_Get( world.bodies, otherBodyId );
				if ( otherBody.setIndex == b2_awakeSet )
				{
					continue;
				}

				int localIndex = contact.localIndex;
				b2ContactSim* contactSim = b2ContactSimArray_Get( awakeSet.contactSims, localIndex );

				assert( contactSim.manifold.pointCount == 0 );
				assert( ( contact.flags & b2_contactTouchingFlag ) == 0 );

				// move the non-touching contact to the disabled set
				contact.setIndex = b2_disabledSet;
				contact.localIndex = cast(int)disabledSet.contactSims.count;
				b2ContactSim* disabledContactSim = b2ContactSimArray_Add( disabledSet.contactSims );
				memcpy( disabledContactSim, contactSim, b2ContactSim.sizeof );

				int movedLocalIndex = b2ContactSimArray_RemoveSwap( awakeSet.contactSims, localIndex );
				if ( movedLocalIndex != B2_NULL_INDEX )
				{
					// fix moved element
					b2ContactSim* movedContactSim = &awakeSet.contactSims[localIndex];
					b2Contact* movedContact = b2ContactArray_Get( world.contacts, movedContactSim.contactId );
					assert( movedContact.localIndex == movedLocalIndex );
					movedContact.localIndex = localIndex;
				}
			}

			bodyId = body.islandNext;
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		int contactId = island.headContact;
		while ( contactId != B2_NULL_INDEX )
		{
			b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );
			assert( contact.setIndex == b2_awakeSet );
			assert( contact.islandId == islandId );
			int colorIndex = contact.colorIndex;
			assert( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );

			b2GraphColor* color = &world.constraintGraph.colors[colorIndex];

			// Remove bodies from graph coloring associated with this constraint
			if ( colorIndex != B2_OVERFLOW_INDEX )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color.bodySet, contact.edges[0].bodyId );
				b2ClearBit( &color.bodySet, contact.edges[1].bodyId );
			}

			int localIndex = contact.localIndex;
			b2ContactSim* awakeContactSim = b2ContactSimArray_Get( color.contactSims, localIndex );

			int sleepContactIndex = cast(int)sleepSet.contactSims.count;
			b2ContactSim* sleepContactSim = b2ContactSimArray_Add( sleepSet.contactSims );
			memcpy( sleepContactSim, awakeContactSim, b2ContactSim.sizeof );

			int movedLocalIndex = b2ContactSimArray_RemoveSwap( color.contactSims, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = &color.contactSims[localIndex];
				b2Contact* movedContact = b2ContactArray_Get( world.contacts, movedContactSim.contactId );
				assert( movedContact.localIndex == movedLocalIndex );
				movedContact.localIndex = localIndex;
			}

			contact.setIndex = sleepSetId;
			contact.colorIndex = B2_NULL_INDEX;
			contact.localIndex = sleepContactIndex;

			contactId = contact.islandNext;
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		int jointId = island.headJoint;
		while ( jointId != B2_NULL_INDEX )
		{
			b2Joint* joint = b2JointArray_Get( world.joints, jointId );
			assert( joint.setIndex == b2_awakeSet );
			assert( joint.islandId == islandId );
			int colorIndex = joint.colorIndex;
			int localIndex = joint.localIndex;

			assert( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );

			b2GraphColor* color = &world.constraintGraph.colors[colorIndex];

			b2JointSim* awakeJointSim = b2JointSimArray_Get( color.jointSims, localIndex );

			if ( colorIndex != B2_OVERFLOW_INDEX )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color.bodySet, joint.edges[0].bodyId );
				b2ClearBit( &color.bodySet, joint.edges[1].bodyId );
			}

			int sleepJointIndex = cast(int)sleepSet.jointSims.count;
			b2JointSim* sleepJointSim = b2JointSimArray_Add( sleepSet.jointSims );
			memcpy( sleepJointSim, awakeJointSim, b2JointSim.sizeof );

			int movedIndex = b2JointSimArray_RemoveSwap( color.jointSims, localIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2JointSim* movedJointSim = &color.jointSims[localIndex];
				int movedId = movedJointSim.jointId;
				b2Joint* movedJoint = b2JointArray_Get( world.joints, movedId );
				assert( movedJoint.localIndex == movedIndex );
				movedJoint.localIndex = localIndex;
			}

			joint.setIndex = sleepSetId;
			joint.colorIndex = B2_NULL_INDEX;
			joint.localIndex = sleepJointIndex;

			jointId = joint.islandNext;
		}
	}

	// move island struct
	{
		assert( island.setIndex == b2_awakeSet );

		int islandIndex = island.localIndex;
		b2IslandSim* sleepIsland = b2IslandSimArray_Add( sleepSet.islandSims );
		sleepIsland.islandId = islandId;

		int movedIslandIndex = b2IslandSimArray_RemoveSwap( awakeSet.islandSims, islandIndex );
		if ( movedIslandIndex != B2_NULL_INDEX )
		{
			// fix index on moved element
			b2IslandSim* movedIslandSim = &awakeSet.islandSims[islandIndex];
			int movedIslandId = movedIslandSim.islandId;
			b2Island* movedIsland = b2IslandArray_Get( world.islands, movedIslandId );
			assert( movedIsland.localIndex == movedIslandIndex );
			movedIsland.localIndex = islandIndex;
		}

		island.setIndex = sleepSetId;
		island.localIndex = 0;
	}

	b2ValidateSolverSets( world );
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
void b2MergeSolverSets(b2World* world, int setId1, int setId2)
{
	assert( setId1 >= b2_firstSleepingSet );
	assert( setId2 >= b2_firstSleepingSet );
	b2SolverSet* set1 = b2SolverSetArray_Get( world.solverSets, setId1 );
	b2SolverSet* set2 = b2SolverSetArray_Get( world.solverSets, setId2 );

	// Move the fewest number of bodies
	if ( set1.bodySims.count < set2.bodySims.count )
	{
		b2SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// transfer bodies
	{
		b2Body* bodies = world.bodies.ptr;
		int bodyCount = cast(int)set2.bodySims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b2BodySim* simSrc = set2.bodySims.ptr + i;

			b2Body* body = bodies + simSrc.bodyId;
			assert( body.setIndex == setId2 );
			body.setIndex = setId1;
			body.localIndex = cast(int)set1.bodySims.count;

			b2BodySim* simDst = b2BodySimArray_Add( set1.bodySims );
			memcpy( simDst, simSrc, b2BodySim.sizeof );
		}
	}

	// transfer contacts
	{
		int contactCount = cast(int)set2.contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSrc = set2.contactSims.ptr + i;

			b2Contact* contact = b2ContactArray_Get( world.contacts, contactSrc.contactId );
			assert( contact.setIndex == setId2 );
			contact.setIndex = setId1;
			contact.localIndex = cast(int)set1.contactSims.count;

			b2ContactSim* contactDst = b2ContactSimArray_Add( set1.contactSims );
			memcpy( contactDst, contactSrc, b2ContactSim.sizeof );
		}
	}

	// transfer joints
	{
		int jointCount = cast(int)set2.jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSrc = set2.jointSims.ptr + i;

			b2Joint* joint = b2JointArray_Get( world.joints, jointSrc.jointId );
			assert( joint.setIndex == setId2 );
			joint.setIndex = setId1;
			joint.localIndex = cast(int)set1.jointSims.count;

			b2JointSim* jointDst = b2JointSimArray_Add( set1.jointSims );
			memcpy( jointDst, jointSrc, b2JointSim.sizeof );
		}
	}

	// transfer islands
	{
		int islandCount = cast(int)set2.islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set2.islandSims.ptr + i;
			int islandId = islandSrc.islandId;

			b2Island* island = b2IslandArray_Get( world.islands, islandId );
			island.setIndex = setId1;
			island.localIndex = cast(int)set1.islandSims.count;

			b2IslandSim* islandDst = b2IslandSimArray_Add( set1.islandSims );
			memcpy( islandDst, islandSrc, b2IslandSim.sizeof );
		}
	}

	// destroy the merged set
	b2DestroySolverSet( world, setId2 );

	b2ValidateSolverSets( world );
}

void b2TransferBody(b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Body* body)
{
	if (targetSet == sourceSet)
	{
		return;
	}

	int sourceIndex = body.localIndex;
	b2BodySim* sourceSim = b2BodySimArray_Get( sourceSet.bodySims, sourceIndex );

	int targetIndex = cast(int)targetSet.bodySims.count;
	b2BodySim* targetSim = b2BodySimArray_Add( targetSet.bodySims );
	memcpy( targetSim, sourceSim, b2BodySim.sizeof );

	// Clear transient body flags
	targetSim.flags &= ~(b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact);

	// Remove body sim from solver set that owns it
	int movedIndex = b2BodySimArray_RemoveSwap( sourceSet.bodySims, sourceIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved body index
		b2BodySim* movedSim = sourceSet.bodySims.ptr + sourceIndex;
		int movedId = movedSim.bodyId;
		b2Body* movedBody = b2BodyArray_Get( world.bodies, movedId );
		assert( movedBody.localIndex == movedIndex );
		movedBody.localIndex = sourceIndex;
	}

	if ( sourceSet.setIndex == b2_awakeSet )
	{
		b2BodyStateArray_RemoveSwap( sourceSet.bodyStates, sourceIndex );
	}
	else if ( targetSet.setIndex == b2_awakeSet )
	{
		b2BodyState* state = b2BodyStateArray_Add( targetSet.bodyStates );
		*state = b2_identityBodyState;
		state.flags = body.flags;
	}

	body.setIndex = targetSet.setIndex;
	body.localIndex = targetIndex;
}

void b2TransferJoint(b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Joint* joint)
{
	if (targetSet == sourceSet)
	{
		return;
	}

	int localIndex = joint.localIndex;
	int colorIndex = joint.colorIndex;

	// Retrieve source.
	b2JointSim* sourceSim = void;
	if ( sourceSet.setIndex == b2_awakeSet )
	{
		assert( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = &world.constraintGraph.colors[colorIndex];

		sourceSim = b2JointSimArray_Get( color.jointSims, localIndex );
	}
	else
	{
		assert( colorIndex == B2_NULL_INDEX );
		sourceSim = b2JointSimArray_Get( sourceSet.jointSims, localIndex );
	}

	// Create target and copy. Fix joint.
	if ( targetSet.setIndex == b2_awakeSet )
	{
		b2AddJointToGraph( world, sourceSim, joint );
		joint.setIndex = b2_awakeSet;
	}
	else
	{
		joint.setIndex = targetSet.setIndex;
		joint.localIndex = cast(int)targetSet.jointSims.count;
		joint.colorIndex = B2_NULL_INDEX;

		b2JointSim* targetSim = b2JointSimArray_Add( targetSet.jointSims );
		memcpy( targetSim, sourceSim, b2JointSim.sizeof );
	}

	// Destroy source.
	if ( sourceSet.setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint.edges[0].bodyId, joint.edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = b2JointSimArray_RemoveSwap( sourceSet.jointSims, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// fix swapped element
			b2JointSim* movedJointSim = sourceSet.jointSims.ptr + localIndex;
			int movedId = movedJointSim.jointId;
			b2Joint* movedJoint = b2JointArray_Get( world.joints, movedId );
			movedJoint.localIndex = localIndex;
		}
	}
}
