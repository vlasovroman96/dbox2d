module dbox2d.constraint_graph;

import std.math;
import std.algorithm;;

import dbox2d.core;
import dbox2d.array;
import dbox2d.bitset;
import dbox2d.constants;
import dbox2d.contact_solver;
import dbox2d.physics_world;
import dbox2d.contact;
import dbox2d.joint;
import dbox2d.shape;
import dbox2d.array;
import dbox2d.bitset;
import dbox2d.broad_phase;
import dbox2d.constraint_graph;
import dbox2d.id_pool;
import dbox2d.arena_allocator;
import dbox2d.ctz;
import dbox2d.body;
import dbox2d.solver_set;
import dbox2d.joint;
import dbox2d.contact;
import dbox2d.island;
import dbox2d.shape;
import dbox2d.sensor;
import dbox2d.types;
import dbox2d.atomic;
import dbox2d.physics_world;
import dbox2d.dynamic_tree;
import dbox2d.distance;
import dbox2d.timer;
import dbox2d.base;

mixin(B2_ARRAY_SOURCE!("b2Body", "b2Body"));
mixin(B2_ARRAY_SOURCE!("b2Int", "int"));
mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2Shape", "b2Shape"));
mixin(B2_ARRAY_SOURCE!("b2ChainShape", "b2ChainShape"));
mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2ContactHitEvent","b2ContactHitEvent"));
mixin(B2_ARRAY_SOURCE!("b2ContactBeginTouchEvent","b2ContactBeginTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2ContactEndTouchEvent","b2ContactEndTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2ContactSim","b2ContactSim"));
mixin(B2_ARRAY_SOURCE!("b2Visitor","b2Visitor"));
mixin(B2_ARRAY_SOURCE!("b2SensorBeginTouchEvent","b2SensorBeginTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2SensorEndTouchEvent","b2SensorEndTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2TaskContext","b2TaskContext"));
mixin(B2_ARRAY_SOURCE!("b2JointEvent","b2JointEvent"));
mixin(B2_ARRAY_SOURCE!("b2SensorTaskContext","b2SensorTaskContext"));
mixin(B2_ARRAY_SOURCE!("b2SensorHit","b2SensorHit"));
mixin(B2_ARRAY_SOURCE!("b2BodyMoveEvent","b2BodyMoveEvent"));
mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));
mixin(B2_ARRAY_SOURCE!("b2Joint","b2Joint"));
mixin(B2_ARRAY_SOURCE!("b2Contact","b2Contact"));

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
enum B2_OVERFLOW_INDEX = (B2_GRAPH_COLOR_COUNT - 1);

// This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
// involving a dynamic and static bodies. This reduces tunneling due to push through.
enum B2_DYNAMIC_COLOR_COUNT = (B2_GRAPH_COLOR_COUNT - 4);

struct b2GraphColor {
	// This bitset is indexed by bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
	// This bitset is unused on the overflow color.
	//
	// Dirk suggested having a uint64_t per body that tracks the graph color membership
	// but I think this would make debugging harder and be less flexible. With the bitset
	// I can trivially increase the number of graph colors beyond 64. See usage of b2CountSetBits
	// for validation.
	b2BitSet bodySet;

	// cache friendly arrays
	b2ContactSimArray contactSims;
	b2JointSimArray jointSims;

	// transient
	union  {
		b2ContactConstraintSIMD* simdConstraints;
		b2ContactConstraint* overflowConstraints;
	};
}

struct b2ConstraintGraph {
	// including overflow at the end
	b2GraphColor[B2_GRAPH_COLOR_COUNT] colors;
}

enum B2_FORCE_OVERFLOW = 0;

void b2CreateGraph(b2ConstraintGraph* graph, int bodyCapacity)
{
	static assert( B2_GRAPH_COLOR_COUNT >= 2, "must have at least two constraint graph colors" );
	static assert( B2_OVERFLOW_INDEX == B2_GRAPH_COLOR_COUNT - 1, "bad over flow index" );
	static assert( B2_DYNAMIC_COLOR_COUNT >= 2, "need more dynamic colors" );

	*graph =  b2ConstraintGraph ();

	bodyCapacity = max( bodyCapacity, 8 );

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for ( int i = 0; i < B2_OVERFLOW_INDEX; ++i )
	{
		b2GraphColor* color = graph.colors.ptr + i;
		color.bodySet = b2CreateBitSet( bodyCapacity );
		b2SetBitCountAndClear( &color.bodySet, bodyCapacity );
	}
}

void b2DestroyGraph(b2ConstraintGraph* graph)
{
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* color = graph.colors.ptr + i;

		// The bit set should never be used on the overflow color
		B2_ASSERT( i != B2_OVERFLOW_INDEX || color.bodySet.bits == null );

		b2DestroyBitSet( &color.bodySet );

		b2ContactSimArray_Destroy( color.contactSims );
		b2JointSimArray_Destroy( color.jointSims );
	}
}

// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
// todo maybe kinematic bodies should not go into graph
void b2AddContactToGraph(b2World* world, b2ContactSim* contactSim, b2Contact* contact)
{
	B2_ASSERT( contactSim.manifold.pointCount > 0 );
	B2_ASSERT( contactSim.simFlags & b2_simTouchingFlag );
	B2_ASSERT( contact.flags & b2_contactTouchingFlag );

	b2ConstraintGraph* graph = &world.constraintGraph;
	int colorIndex = B2_OVERFLOW_INDEX;

	int bodyIdA = contact.edges[0].bodyId;
	int bodyIdB = contact.edges[1].bodyId;
	b2Body* bodyA = b2BodyArray_Get( world.bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, bodyIdB );
	bool staticA = bodyA.type == b2_staticBody;
	bool staticB = bodyB.type == b2_staticBody;
	B2_ASSERT( staticA == false || staticB == false );

static if (B2_FORCE_OVERFLOW == 0) {
	if ( staticA == false && staticB == false )
	{
		// Dynamic constraint colors cannot encroach on colors reserved for static constraints
		for ( int i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i )
		{
			b2GraphColor* color = graph.colors.ptr + i;
			if ( b2GetBit( &color.bodySet, bodyIdA ) || b2GetBit( &color.bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color.bodySet, bodyIdA );
			b2SetBitGrow( &color.bodySet, bodyIdB );
			colorIndex = i;
			break;
		}
	}
	else if ( staticA == false )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph.colors.ptr + i;
			if ( b2GetBit( &color.bodySet, bodyIdA ) )
			{
				continue;
			}

			b2SetBitGrow( &color.bodySet, bodyIdA );
			colorIndex = i;
			break;
		}
	}
	else if ( staticB == false )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph.colors.ptr + i;
			if ( b2GetBit( &color.bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color.bodySet, bodyIdB );
			colorIndex = i;
			break;
		}
	}
}

	b2GraphColor* color = graph.colors.ptr + colorIndex;
	contact.colorIndex = colorIndex;
	contact.localIndex = cast(int)color.contactSims.count;

	b2ContactSim* newContact = b2ContactSimArray_Add( color.contactSims );
	*newContact = *contactSim;

	// todo perhaps skip this if the contact is already awake

	if ( staticA )
	{
		newContact.bodySimIndexA = B2_NULL_INDEX;
		newContact.invMassA = 0.0f;
		newContact.invIA = 0.0f;
	}
	else
	{
		B2_ASSERT( bodyA.setIndex == b2_awakeSet );
		b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );

		int localIndex = bodyA.localIndex;
		newContact.bodySimIndexA = localIndex;

		b2BodySim* bodySimA = b2BodySimArray_Get( awakeSet.bodySims, localIndex );
		newContact.invMassA = bodySimA.invMass;
		newContact.invIA = bodySimA.invInertia;
	}

	if ( staticB )
	{
		newContact.bodySimIndexB = B2_NULL_INDEX;
		newContact.invMassB = 0.0f;
		newContact.invIB = 0.0f;
	}
	else
	{
		B2_ASSERT( bodyB.setIndex == b2_awakeSet );
		b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );

		int localIndex = bodyB.localIndex;
		newContact.bodySimIndexB = localIndex;

		b2BodySim* bodySimB = b2BodySimArray_Get( awakeSet.bodySims, localIndex );
		newContact.invMassB = bodySimB.invMass;
		newContact.invIB = bodySimB.invInertia;
	}
}

void b2RemoveContactFromGraph(b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
{
	b2ConstraintGraph* graph = &world.constraintGraph;

	B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
	b2GraphColor* color = graph.colors.ptr + colorIndex;

	if ( colorIndex != B2_OVERFLOW_INDEX )
	{
		// might clear a bit for a static body, but this has no effect
		b2ClearBit( &color.bodySet, bodyIdA );
		b2ClearBit( &color.bodySet, bodyIdB );
	}

	int movedIndex = b2ContactSimArray_RemoveSwap( color.contactSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix index on swapped contact
		b2ContactSim* movedContactSim = color.contactSims.ptr + localIndex;

		// Fix moved contact
		int movedId = movedContactSim.contactId;
		b2Contact* movedContact = b2ContactArray_Get( world.contacts, movedId );
		B2_ASSERT( movedContact.setIndex == b2_awakeSet );
		B2_ASSERT( movedContact.colorIndex == colorIndex );
		B2_ASSERT( movedContact.localIndex == movedIndex );
		movedContact.localIndex = localIndex;
	}
}

private int b2AssignJointColor(b2ConstraintGraph* graph, int bodyIdA, int bodyIdB, bool staticA, bool staticB)
{
	B2_ASSERT( staticA == false || staticB == false );

static if (B2_FORCE_OVERFLOW == 0) {
	if ( staticA == false && staticB == false )
	{
		// Dynamic constraint colors cannot encroach on colors reserved for static constraints
		for ( int i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i )
		{
			b2GraphColor* color = graph.colors.ptr + i;
			if ( b2GetBit( &color.bodySet, bodyIdA ) || b2GetBit( &color.bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color.bodySet, bodyIdA );
			b2SetBitGrow( &color.bodySet, bodyIdB );
			return i;
		}
	}
	else if ( staticA == false )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph.colors.ptr + i;
			if ( b2GetBit( &color.bodySet, bodyIdA ) )
			{
				continue;
			}

			b2SetBitGrow( &color.bodySet, bodyIdA );
			return i;
		}
	}
	else if ( staticB == false )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph.colors.ptr + i;
			if ( b2GetBit( &color.bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color.bodySet, bodyIdB );
			return i;
		}
	}
} else {
	B2_UNUSED( graph, bodyIdA, bodyIdB, staticA, staticB );
}

	return B2_OVERFLOW_INDEX;
}

b2JointSim* b2CreateJointInGraph(b2World* world, b2Joint* joint)
{
	b2ConstraintGraph* graph = &world.constraintGraph;

	int bodyIdA = joint.edges[0].bodyId;
	int bodyIdB = joint.edges[1].bodyId;
	b2Body* bodyA = b2BodyArray_Get( world.bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( world.bodies, bodyIdB );
	bool staticA = bodyA.type == b2_staticBody;
	bool staticB = bodyB.type == b2_staticBody;

	int colorIndex = b2AssignJointColor( graph, bodyIdA, bodyIdB, staticA, staticB );

	b2JointSim* jointSim = b2JointSimArray_Add( graph.colors[colorIndex].jointSims );
	jointSim = new b2JointSim;

	joint.colorIndex = colorIndex;
	joint.localIndex = cast(int)(graph.colors[colorIndex].jointSims.count - 1);
	return jointSim;
}

void b2AddJointToGraph(b2World* world, b2JointSim* jointSim, b2Joint* joint)
{
	b2JointSim* jointDst = b2CreateJointInGraph( world, joint );
	(*jointDst) = *jointSim;
}

void b2RemoveJointFromGraph(b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
{
	b2ConstraintGraph* graph = &world.constraintGraph;

	B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
	b2GraphColor* color = graph.colors.ptr + colorIndex;

	if ( colorIndex != B2_OVERFLOW_INDEX )
	{
		// May clear static bodies, no effect
		b2ClearBit( &color.bodySet, bodyIdA );
		b2ClearBit( &color.bodySet, bodyIdB );
	}

	int movedIndex = b2JointSimArray_RemoveSwap( color.jointSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved joint
		b2JointSim* movedJointSim = color.jointSims.ptr + localIndex;
		int movedId = movedJointSim.jointId;
		b2Joint* movedJoint = b2JointArray_Get( world.joints, movedId );
		B2_ASSERT( movedJoint.setIndex == b2_awakeSet );
		B2_ASSERT( movedJoint.colorIndex == colorIndex );
		B2_ASSERT( movedJoint.localIndex == movedIndex );
		movedJoint.localIndex = localIndex;
	}
}
