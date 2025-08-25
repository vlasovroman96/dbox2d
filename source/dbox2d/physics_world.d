module dbox2d.physics_world;

import std.math;

import dbox2d.core;
import dbox2d.id;
import dbox2d.array;
import dbox2d.math;
import dbox2d.bitset;
import dbox2d.table;
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
// import dbox2d.atomic;
// import dbox2d.physics_world;
import dbox2d.dynamic_tree;
import dbox2d.distance;
import dbox2d.timer;
// import dbox2d.arena_allocator;
import dbox2d.solver;
import dbox2d.collision;
import dbox2d.constants;

import core.stdc.string;
import core.stdc.stdio;

mixin(B2_ARRAY_SOURCE!("b2Body", "b2Body"));
mixin(B2_ARRAY_SOURCE!("b2Int", "int"));
mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2Sensor","b2Sensor"));
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

enum b2SetType : int
{
	b2_staticSet = 0,
	b2_disabledSet = 1,
	b2_awakeSet = 2,
	b2_firstSleepingSet = 3,
}
alias b2_staticSet = b2SetType.b2_staticSet;
alias b2_disabledSet = b2SetType.b2_disabledSet;
alias b2_awakeSet = b2SetType.b2_awakeSet;
alias b2_firstSleepingSet = b2SetType.b2_firstSleepingSet;

// Per thread task storage
struct b2TaskContext {
	// Collect per thread sensor continuous hit events.
	b2SensorHitArray sensorHits;

	// These bits align with the contact id capacity and signal a change in contact status
	b2BitSet contactStateBitSet;

	// These bits align with the joint id capacity and signal a change in contact status
	b2BitSet jointStateBitSet;

	// Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
	// that is very large when there are many static shapes.
	b2BitSet enlargedSimBitSet;

	// Used to put islands to sleep
	b2BitSet awakeIslandBitSet;

	// Per worker split island candidate
	float splitSleepTime = 0;
	int splitIslandId;
}

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
struct b2World {
	b2ArenaAllocator arena;
	b2BroadPhase broadPhase;
	b2ConstraintGraph constraintGraph;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body. Aligns with b2Body.
	b2IdPool bodyIdPool;

	//mixin(B2_ARRAY_SOURCE!"b2Body");

	// This is a sparse array that maps body ids to the body
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	b2BodyArray bodies;

	// Provides free list for solver sets.
	b2IdPool solverSetIdPool;

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	b2SolverSetArray solverSets;

	// Used to create stable ids for joints
	b2IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint stored in the constraint graph
	// or in the solver sets.
	b2JointArray joints;

	// Used to create stable ids for contacts
	b2IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact stored in the constraint graph
	// or in the solver sets.
	b2ContactArray contacts;

	// Used to create stable ids for islands
	b2IdPool islandIdPool;

	// This is a sparse array that maps island ids to the island stored in the solver sets.
	b2IslandArray islands;

	b2IdPool shapeIdPool;
	b2IdPool chainIdPool;

	// These are sparse arrays that point into the pools above
	b2ShapeArray shapes;
	b2ChainShapeArray chainShapes;

	// This is a dense array of sensor.
	b2SensorArray sensors;

	// Per thread storage
	b2TaskContextArray taskContexts;
	b2SensorTaskContextArray sensorTaskContexts;

	b2BodyMoveEventArray bodyMoveEvents;
	b2SensorBeginTouchEventArray sensorBeginEvents;
	b2ContactBeginTouchEventArray contactBeginEvents;

	// End events are double buffered so that the user doesn't need to flush events
	b2SensorEndTouchEventArray[2] sensorEndEvents;
	b2ContactEndTouchEventArray[2] contactEndEvents;
	int endEventArrayIndex;

	b2ContactHitEventArray contactHitEvents;
	b2JointEventArray jointEvents;

	// todo consider deferred waking and impulses to make it possible
	// to apply forces and impulses from multiple threads
	// impulses must be deferred because sleeping bodies have no velocity state
	// Problems:
	// - multiple forces applied to the same body from multiple threads
	// Deferred wake
	//b2BitSet bodyWakeSet;
	//b2ImpulseArray deferredImpulses;

	// Used to track debug draw
	b2BitSet debugBodySet;
	b2BitSet debugJointSet;
	b2BitSet debugContactSet;
	b2BitSet debugIslandSet;

	// Id that is incremented every time step
	ulong stepIndex;

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	int splitIslandId;

	b2Vec2 gravity;
	float hitEventThreshold = 0;
	float restitutionThreshold = 0;
	float maxLinearSpeed = 0;
	float contactSpeed = 0;
	float contactHertz = 0;
	float contactDampingRatio = 0;

	b2FrictionCallback frictionCallback;
	b2RestitutionCallback restitutionCallback;

	ushort generation;

	b2Profile profile;

	b2PreSolveFcn preSolveFcn;
	void* preSolveContext;

	b2CustomFilterFcn customFilterFcn;
	void* customFilterContext;

	int workerCount;
	b2EnqueueTaskCallback enqueueTaskFcn;
	b2FinishTaskCallback finishTaskFcn;
	void* userTaskContext;
	void* userTreeTask;

	void* userData;

	// Remember type step used for reporting forces and torques
	float inv_h = 0;

	int activeTaskCount;
	int taskCount;

	ushort worldId;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSpeculative;
	bool inUse;
}
static assert( B2_MAX_WORLDS > 0, "must be 1 or more" );
static assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
b2World[B2_MAX_WORLDS] b2_worlds;

b2World* b2GetWorldFromId(b2WorldId id)
{
	assert( 1 <= id.index1 && id.index1 <= B2_MAX_WORLDS );
	b2World* world = b2_worlds.ptr + ( id.index1 - 1 );
	assert( id.index1 == (world.worldId + 1) );
	assert( id.generation == world.generation );
	return world;
}

b2World* b2GetWorld(int index)
{
	assert( 0 <= index && index < B2_MAX_WORLDS );
	b2World* world = b2_worlds.ptr + index;
	assert( world.worldId == index );
	return world;
}

b2World* b2GetWorldLocked(int index)
{
	assert( 0 <= index && index < B2_MAX_WORLDS );
	b2World* world = b2_worlds.ptr + index;
	assert( world.worldId == index );
	if ( world.locked )
	{
		assert( false );
		return null;
	}

	return world;
}

void* b2DefaultAddTaskFcn(b2TaskCallback task, int count, int minRange, void* taskContext, void* userContext)
{
	// B2_UNUSED( minRange, userContext );
	task( 0, count, 0, taskContext );
	return null;
}

void b2DefaultFinishTaskFcn(void* userTask, void* userContext)
{
	// B2_UNUSED( userTask, userContext );
}

float b2DefaultFrictionCallback(float frictionA, int materialA, float frictionB, int materialB)
{
	// B2_UNUSED( materialA, materialB );
	return sqrt( frictionA * frictionB );
}

float b2DefaultRestitutionCallback(float restitutionA, int materialA, float restitutionB, int materialB)
{
	// B2_UNUSED( materialA, materialB );
	return max( restitutionA, restitutionB );
}

b2WorldId b2CreateWorld(b2WorldDef* def)
{
	static assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
	B2_CHECK_DEF( def );

	int worldId = B2_NULL_INDEX;
	for ( int i = 0; i < B2_MAX_WORLDS; ++i )
	{
		if ( b2_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}

	if ( worldId == B2_NULL_INDEX )
	{
		return b2WorldId( 0 );
	}

	b2InitializeContactRegisters();

	b2World* world = b2_worlds.ptr + worldId;
	ushort generation = world.generation;

	*world = b2World();

	world.worldId = cast(ushort)worldId;
	world.generation = generation;
	world.inUse = true;

	world.arena = b2CreateArenaAllocator( 2048 );
	b2CreateBroadPhase( &world.broadPhase );
	b2CreateGraph( &world.constraintGraph, 16 );

	// pools
	world.bodyIdPool = b2CreateIdPool();
	world.bodies = b2BodyArray_Create( 16 );
	world.solverSets = b2SolverSetArray_Create( 8 );

	// add empty static, active, and disabled body sets
	world.solverSetIdPool = b2CreateIdPool();
	b2SolverSet set;

	// static set
	set.setIndex = b2AllocId( &world.solverSetIdPool );
	b2SolverSetArray_Push( world.solverSets, set );
	assert( world.solverSets[b2_staticSet].setIndex == b2_staticSet );

	// disabled set
	set.setIndex = b2AllocId( &world.solverSetIdPool );
	b2SolverSetArray_Push( world.solverSets, set );
	assert( world.solverSets[b2_disabledSet].setIndex == b2_disabledSet );

	// awake set
	set.setIndex = b2AllocId( &world.solverSetIdPool );
	b2SolverSetArray_Push( world.solverSets, set );
	assert( world.solverSets[b2_awakeSet].setIndex == b2_awakeSet );

	world.shapeIdPool = b2CreateIdPool();
	world.shapes = b2ShapeArray_Create( 16 );

	world.chainIdPool = b2CreateIdPool();
	world.chainShapes = b2ChainShapeArray_Create( 4 );

	world.contactIdPool = b2CreateIdPool();
	world.contacts = b2ContactArray_Create( 16 );

	world.jointIdPool = b2CreateIdPool();
	world.joints = b2JointArray_Create( 16 );

	world.islandIdPool = b2CreateIdPool();
	world.islands = b2IslandArray_Create( 8 );

	world.sensors = b2SensorArray_Create( 4 );

	world.bodyMoveEvents = b2BodyMoveEventArray_Create( 4 );
	world.sensorBeginEvents = b2SensorBeginTouchEventArray_Create( 4 );
	world.sensorEndEvents[0] = b2SensorEndTouchEventArray_Create( 4 );
	world.sensorEndEvents[1] = b2SensorEndTouchEventArray_Create( 4 );
	world.contactBeginEvents = b2ContactBeginTouchEventArray_Create( 4 );
	world.contactEndEvents[0] = b2ContactEndTouchEventArray_Create( 4 );
	world.contactEndEvents[1] = b2ContactEndTouchEventArray_Create( 4 );
	world.contactHitEvents = b2ContactHitEventArray_Create( 4 );
	world.jointEvents = b2JointEventArray_Create( 4 );
	world.endEventArrayIndex = 0;

	world.stepIndex = 0;
	world.splitIslandId = B2_NULL_INDEX;
	world.activeTaskCount = 0;
	world.taskCount = 0;
	world.gravity = def.gravity;
	world.hitEventThreshold = def.hitEventThreshold;
	world.restitutionThreshold = def.restitutionThreshold;
	world.maxLinearSpeed = def.maximumLinearSpeed;
	world.contactSpeed = def.contactSpeed;
	world.contactHertz = def.contactHertz;
	world.contactDampingRatio = def.contactDampingRatio;

	if ( def.frictionCallback == null )
	{
		world.frictionCallback = &b2DefaultFrictionCallback;
	}
	else
	{
		world.frictionCallback = *(def.frictionCallback);
	}

	if ( def.restitutionCallback == null )
	{
		world.restitutionCallback = &b2DefaultRestitutionCallback;
	}
	else
	{
		world.restitutionCallback = *(def.restitutionCallback);
	}

	world.enableSleep = def.enableSleep;
	world.locked = false;
	world.enableWarmStarting = true;
	world.enableContinuous = def.enableContinuous;
	world.enableSpeculative = true;
	world.userTreeTask = null;
	world.userData = def.userData;

	if ( def.workerCount > 0 && def.enqueueTask != null && def.finishTask != null )
	{
		world.workerCount = min( def.workerCount, B2_MAX_WORKERS );
		world.enqueueTaskFcn = *(def.enqueueTask);
		world.finishTaskFcn = *(def.finishTask);
		world.userTaskContext = def.userTaskContext;
	}
	else
	{
		world.workerCount = 1;
		world.enqueueTaskFcn = &b2DefaultAddTaskFcn;
		world.finishTaskFcn = &b2DefaultFinishTaskFcn;
		world.userTaskContext = null;
	}

	world.taskContexts = b2TaskContextArray_Create( world.workerCount );
	b2TaskContextArray_Resize( world.taskContexts, world.workerCount );

	world.sensorTaskContexts = b2SensorTaskContextArray_Create( world.workerCount );
	b2SensorTaskContextArray_Resize( world.sensorTaskContexts, world.workerCount );

	for ( int i = 0; i < world.workerCount; ++i )
	{
		world.taskContexts[i].sensorHits = b2SensorHitArray_Create( 8 );
		world.taskContexts[i].contactStateBitSet = b2CreateBitSet( 1024 );
		world.taskContexts[i].jointStateBitSet = b2CreateBitSet( 1024 );
		world.taskContexts[i].enlargedSimBitSet = b2CreateBitSet( 256 );
		world.taskContexts[i].awakeIslandBitSet = b2CreateBitSet( 256 );

		world.sensorTaskContexts[i].eventBits = b2CreateBitSet( 128 );
	}

	world.debugBodySet = b2CreateBitSet( 256 );
	world.debugJointSet = b2CreateBitSet( 256 );
	world.debugContactSet = b2CreateBitSet( 256 );
	world.debugIslandSet = b2CreateBitSet( 256 );

	// add one to worldId so that 0 represents a null b2WorldId
	return b2WorldId( cast(ushort)( worldId + 1 ), world.generation );
}

void b2DestroyWorld(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );

	b2DestroyBitSet( &world.debugBodySet );
	b2DestroyBitSet( &world.debugJointSet );
	b2DestroyBitSet( &world.debugContactSet );
	b2DestroyBitSet( &world.debugIslandSet );

	for ( int i = 0; i < world.workerCount; ++i )
	{
		b2SensorHitArray_Destroy( world.taskContexts[i].sensorHits );
		b2DestroyBitSet( &world.taskContexts[i].contactStateBitSet );
		b2DestroyBitSet( &world.taskContexts[i].jointStateBitSet );
		b2DestroyBitSet( &world.taskContexts[i].enlargedSimBitSet );
		b2DestroyBitSet( &world.taskContexts[i].awakeIslandBitSet );

		b2DestroyBitSet( &world.sensorTaskContexts[i].eventBits );
	}

	b2TaskContextArray_Destroy( world.taskContexts );
	b2SensorTaskContextArray_Destroy( world.sensorTaskContexts );

	b2BodyMoveEventArray_Destroy( world.bodyMoveEvents );
	b2SensorBeginTouchEventArray_Destroy( world.sensorBeginEvents );
	b2SensorEndTouchEventArray_Destroy( world.sensorEndEvents[0] );
	b2SensorEndTouchEventArray_Destroy( world.sensorEndEvents[1] );
	b2ContactBeginTouchEventArray_Destroy( world.contactBeginEvents );
	b2ContactEndTouchEventArray_Destroy( world.contactEndEvents[0] );
	b2ContactEndTouchEventArray_Destroy( world.contactEndEvents[1] );
	b2ContactHitEventArray_Destroy( world.contactHitEvents );
	b2JointEventArray_Destroy( world.jointEvents );

	int chainCapacity = cast(int)world.chainShapes.count;
	for ( int i = 0; i < chainCapacity; ++i )
	{
		b2ChainShape* chain = &world.chainShapes[i];
		if ( chain.id != B2_NULL_INDEX )
		{
			b2FreeChainData( chain );
		}
		else
		{
			assert( chain.shapeIndices == null );
			assert( chain.materials == null );
		}
	}

	int sensorCount = cast(int)world.sensors.count;
	for ( int i = 0; i < sensorCount; ++i )
	{
		b2VisitorArray_Destroy( world.sensors[i].hits );
		b2VisitorArray_Destroy( world.sensors[i].overlaps1 );
		b2VisitorArray_Destroy( world.sensors[i].overlaps2 );
	}

	b2SensorArray_Destroy( world.sensors );

	b2BodyArray_Destroy( world.bodies );
	b2ShapeArray_Destroy( world.shapes );
	b2ChainShapeArray_Destroy( world.chainShapes );
	b2ContactArray_Destroy( world.contacts );
	b2JointArray_Destroy( world.joints );
	b2IslandArray_Destroy( world.islands );

	// Destroy solver sets
	int setCapacity = cast(int)world.solverSets.length;
	for ( int i = 0; i < setCapacity; ++i )
	{
		b2SolverSet* set = &world.solverSets[i];
		if ( set.setIndex != B2_NULL_INDEX )
		{
			b2DestroySolverSet( world, i );
		}
	}

	b2SolverSetArray_Destroy( world.solverSets );

	b2DestroyGraph( &world.constraintGraph );
	b2DestroyBroadPhase( &world.broadPhase );

	b2DestroyIdPool( &world.bodyIdPool );
	b2DestroyIdPool( &world.shapeIdPool );
	b2DestroyIdPool( &world.chainIdPool );
	b2DestroyIdPool( &world.contactIdPool );
	b2DestroyIdPool( &world.jointIdPool );
	b2DestroyIdPool( &world.islandIdPool );
	b2DestroyIdPool( &world.solverSetIdPool );

	b2DestroyArenaAllocator( &world.arena );

	// Wipe world but preserve generation
	ushort generation = world.generation;
	*world = b2World();
	world.worldId = 0;
	world.generation = cast(ushort)(generation + 1);
}

void b2CollideTask(int startIndex, int endIndex, uint threadIndex, void* context)
{
	b2StepContext* stepContext = cast(b2StepContext*)context;
	b2World* world = stepContext.world;
	assert( cast(int)threadIndex < world.workerCount );
	b2TaskContext* taskContext = &world.taskContexts[threadIndex];
	b2ContactSim** contactSims = stepContext.contacts;
	b2Shape* shapes = world.shapes.ptr;
	b2Body* bodies = world.bodies.ptr;

	assert( startIndex < endIndex );

	for ( int contactIndex = startIndex; contactIndex < endIndex; ++contactIndex )
	{
		b2ContactSim* contactSim = contactSims[contactIndex];

		int contactId = contactSim.contactId;

		b2Shape* shapeA = shapes + contactSim.shapeIdA;
		b2Shape* shapeB = shapes + contactSim.shapeIdB;

		// Do proxies still overlap?
		bool overlap = shapeA.fatAABB.overlaps( shapeB.fatAABB );
		if ( overlap == false )
		{
			contactSim.simFlags |= b2_simDisjoint;
			contactSim.simFlags &= ~b2_simTouchingFlag;
			b2SetBit( &taskContext.contactStateBitSet, contactId );
		}
		else
		{
			bool wasTouching = cast(bool)( contactSim.simFlags & b2_simTouchingFlag );

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = bodies + shapeA.bodyId;
			b2Body* bodyB = bodies + shapeB.bodyId;
			b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
			b2BodySim* bodySimB = b2GetBodySim( world, bodyB );

			// avoid cache misses in b2PrepareContactsTask
			contactSim.bodySimIndexA = bodyA.setIndex == b2_awakeSet ? bodyA.localIndex : B2_NULL_INDEX;
			contactSim.invMassA = bodySimA.invMass;
			contactSim.invIA = bodySimA.invInertia;

			contactSim.bodySimIndexB = bodyB.setIndex == b2_awakeSet ? bodyB.localIndex : B2_NULL_INDEX;
			contactSim.invMassB = bodySimB.invMass;
			contactSim.invIB = bodySimB.invInertia;

			b2Transform transformA = bodySimA.transform;
			b2Transform transformB = bodySimB.transform;

			b2Vec2 centerOffsetA = b2RotateVector( transformA.q, bodySimA.localCenter );
			b2Vec2 centerOffsetB = b2RotateVector( transformB.q, bodySimB.localCenter );

			// This updates solid contacts
			bool touching = b2UpdateContact( world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB );

			// State changes that affect island connectivity. Also affects contact events.
			if ( touching == true && wasTouching == false )
			{
				contactSim.simFlags |= b2_simStartedTouching;
				b2SetBit( &taskContext.contactStateBitSet, contactId );
			}
			else if ( touching == false && wasTouching == true )
			{
				contactSim.simFlags |= b2_simStoppedTouching;
				b2SetBit( &taskContext.contactStateBitSet, contactId );
			}

			// To make this work, the time of impact code needs to adjust the target
			// distance based on the number of TOI events for a body.
			// if (touching && bodySimB->isFast)
			//{
			//	b2Manifold* manifold = &contactSim->manifold;
			//	int pointCount = manifold->pointCount;
			//	for (int i = 0; i < pointCount; ++i)
			//	{
			//		// trick the solver into pushing the fast shapes apart
			//		manifold->points[i].separation -= 0.25f * B2_SPECULATIVE_DISTANCE;
			//	}
			//}
		}
	}
}

void b2UpdateTreesTask(int startIndex, int endIndex, uint threadIndex, void* context)
{
	// B2_UNUSED( startIndex );
	// B2_UNUSED( endIndex );
	// B2_UNUSED( threadIndex );

	b2World* world = cast(b2World*)context;
	b2BroadPhase_RebuildTrees( &world.broadPhase );
}

void b2AddNonTouchingContact(b2World* world, b2Contact* contact, b2ContactSim* contactSim)
{
	assert( contact.setIndex == b2_awakeSet );
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	contact.colorIndex = B2_NULL_INDEX;
	contact.localIndex = cast(int)set.contactSims.length;

	b2ContactSim* newContactSim = b2ContactSimArray_Add( set.contactSims );
	memcpy( newContactSim, contactSim, b2ContactSim.sizeof );
}

void b2RemoveNonTouchingContact(b2World* world, int setIndex, int localIndex)
{
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, setIndex );
	int movedIndex = b2ContactSimArray_RemoveSwap( set.contactSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		b2ContactSim* movedContactSim = &set.contactSims[localIndex];
		b2Contact* movedContact = b2ContactArray_Get( world.contacts, movedContactSim.contactId );
		assert( movedContact.setIndex == setIndex );
		assert( movedContact.localIndex == movedIndex );
		assert( movedContact.colorIndex == B2_NULL_INDEX );
		movedContact.localIndex = localIndex;
	}
}

// Narrow-phase collision
void b2Collide(b2StepContext* context)
{
	b2World* world = context.world;

	assert( world.workerCount > 0 );

	// Task that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	// todo_erin move this to start when contacts are being created
	world.userTreeTask = world.enqueueTaskFcn( &b2UpdateTreesTask, 1, 1, world, world.userTaskContext );
	world.taskCount += 1;
	world.activeTaskCount += world.userTreeTask == null ? 0 : 1;

	// gather contacts into a single array for easier parallel-for
	int contactCount = 0;
	b2GraphColor* graphColors = world.constraintGraph.colors.ptr;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		contactCount += graphColors[i].contactSims.count;
	}

	int nonTouchingCount = cast(int)world.solverSets[b2_awakeSet].contactSims.count;
	contactCount += nonTouchingCount;

	if ( contactCount == 0 )
	{
		return;
	}

	b2ContactSim** contactSims = cast(b2ContactSim**)b2AllocateArenaItem( &world.arena, cast(int)(contactCount * (b2ContactSim*).sizeof), "contacts" );

	int contactIndex = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{

		b2GraphColor* color = &graphColors[i];
		int count = cast(int)color.contactSims.count;
		b2ContactSim* base = color.contactSims.ptr;
		for ( int j = 0; j < count; ++j )
		{
			contactSims[contactIndex] = base + j;
			contactIndex += 1;
		}
	}

	{
		b2ContactSim* base = world.solverSets[b2_awakeSet].contactSims.ptr;
		for ( int i = 0; i < nonTouchingCount; ++i )
		{
			contactSims[contactIndex] = base + i;
			contactIndex += 1;
		}
	}

	assert( contactIndex == contactCount );

	context.contacts = contactSims;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b2GetIdCapacity( &world.contactIdPool );
	for ( int i = 0; i < world.workerCount; ++i )
	{
		b2SetBitCountAndClear( &world.taskContexts[i].contactStateBitSet, contactIdCapacity );
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int minRange = 64;
	void* userCollideTask = world.enqueueTaskFcn( &b2CollideTask, contactCount, minRange, context, world.userTaskContext );
	world.taskCount += 1;
	if ( userCollideTask != null )
	{
		world.finishTaskFcn( userCollideTask, world.userTaskContext );
	}

	b2FreeArenaItem( &world.arena, contactSims );
	context.contacts = null;
	contactSims = null;

	// Serially update contact state
	// todo_erin bring this zone together with island merge
	// b2TracyCZoneNC( contact_state, "Contact State", b2_colorLightSlateGray, true );

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world.taskContexts[0].contactStateBitSet;
	for ( int i = 1; i < world.workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world.taskContexts[i].contactStateBitSet );
	}

	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );

	int endEventArrayIndex = world.endEventArrayIndex;

	const(b2Shape)* shapes = world.shapes.ptr;
	ushort worldId = world.worldId;

	// Process contact state changes. Iterate over set bits
	for ( uint k = 0; k < bitSet.blockCount; ++k )
	{
		ulong bits = bitSet.bits[k];
		while ( bits != 0 )
		{
			uint ctz = b2CTZ64( bits );
			int contactId = cast(int)( 64 * k + ctz );

			b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );
			assert( contact.setIndex == b2_awakeSet );

			int colorIndex = contact.colorIndex;
			int localIndex = contact.localIndex;

			b2ContactSim* contactSim = null;
			if ( colorIndex != B2_NULL_INDEX )
			{
				// contact lives in constraint graph
				assert( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
				b2GraphColor* color = graphColors + colorIndex;
				contactSim = b2ContactSimArray_Get( color.contactSims, localIndex );
			}
			else
			{
				contactSim = b2ContactSimArray_Get( awakeSet.contactSims, localIndex );
			}

			const(b2Shape)* shapeA = shapes + contact.shapeIdA;
			const(b2Shape)* shapeB = shapes + contact.shapeIdB;
			b2ShapeId shapeIdA = { shapeA.id + 1, worldId, shapeA.generation };
			b2ShapeId shapeIdB = { shapeB.id + 1, worldId, shapeB.generation };
			b2ContactId contactFullId = {
				index1: contactId + 1,
				world0: worldId,
				padding: 0,
				generation: contact.generation,
			};
			uint flags = contact.flags;
			uint simFlags = contactSim.simFlags;

			if ( simFlags & b2_simDisjoint )
			{
				// Bounding boxes no longer overlap
				b2DestroyContact( world, contact, false );
				contact = null;
				contactSim = null;
			}
			else if ( simFlags & b2_simStartedTouching )
			{
				assert( contact.islandId == B2_NULL_INDEX );

				if ( flags & b2_contactEnableContactEvents )
				{
					b2ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b2ContactBeginTouchEventArray_Push( world.contactBeginEvents, event );
				}

				assert( contactSim.manifold.pointCount > 0 );
				assert( contact.setIndex == b2_awakeSet );

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				contact.flags |= b2_contactTouchingFlag;
				b2LinkContact( world, contact );

				// Make sure these didn't change
				assert( contact.colorIndex == B2_NULL_INDEX );
				assert( contact.localIndex == localIndex );

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.
				contactSim = b2ContactSimArray_Get( awakeSet.contactSims, localIndex );

				contactSim.simFlags &= ~b2_simStartedTouching;

				b2AddContactToGraph( world, contactSim, contact );
				b2RemoveNonTouchingContact( world, b2_awakeSet, localIndex );
				contactSim = null;
			}
			else if ( simFlags & b2_simStoppedTouching )
			{
				contactSim.simFlags &= ~b2_simStoppedTouching;
				contact.flags &= ~b2_contactTouchingFlag;

				if ( contact.flags & b2_contactEnableContactEvents )
				{
					b2ContactEndTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b2ContactEndTouchEventArray_Push( world.contactEndEvents[endEventArrayIndex], event );
				}

				assert( contactSim.manifold.pointCount == 0 );

				b2UnlinkContact( world, contact );
				int bodyIdA = contact.edges[0].bodyId;
				int bodyIdB = contact.edges[1].bodyId;

				b2AddNonTouchingContact( world, contact, contactSim );
				b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
				contact = null;
				contactSim = null;
			}

			// Clear the smallest set bit
			bits = bits & ( bits - 1 );
		}
	}

	b2ValidateSolverSets( world );
	b2ValidateContacts( world );
}

void b2World_Step(b2WorldId worldId, float timeStep, int subStepCount)
{
	assert( b2IsValidFloat( timeStep ) );
	assert( 0 < subStepCount );

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	// Prepare to capture events
	// Ensure user does not access stale if there is an early return
	b2BodyMoveEventArray_Clear( world.bodyMoveEvents );
	b2SensorBeginTouchEventArray_Clear( world.sensorBeginEvents );
	b2ContactBeginTouchEventArray_Clear( world.contactBeginEvents );
	b2ContactHitEventArray_Clear( world.contactHitEvents );
	b2JointEventArray_Clear( world.jointEvents );

	world.profile = b2Profile( 0 );

	if ( timeStep == 0.0f )
	{
		// Swap end event array buffers
		world.endEventArrayIndex = 1 - world.endEventArrayIndex;
		b2SensorEndTouchEventArray_Clear( world.sensorEndEvents[world.endEventArrayIndex] );
		b2ContactEndTouchEventArray_Clear( world.contactEndEvents[world.endEventArrayIndex] );

		// todo_erin would be useful to still process collision while paused
		return;
	}

	world.locked = true;
	world.activeTaskCount = 0;
	world.taskCount = 0;

	ulong stepTicks = b2GetTicks();

	// Update collision pairs and create contacts
	{
		ulong pairTicks = b2GetTicks();
		b2UpdateBroadPhasePairs( world );
		world.profile.pairs = b2GetMilliseconds( pairTicks );
	}

	b2StepContext context = { 0 };
	context.world = world;
	context.dt = timeStep;
	context.subStepCount = max( 1, subStepCount );

	if ( timeStep > 0.0f )
	{
		context.inv_dt = 1.0f / timeStep;
		context.h = timeStep / context.subStepCount;
		context.inv_h = context.subStepCount * context.inv_dt;
	}
	else
	{
		context.inv_dt = 0.0f;
		context.h = 0.0f;
		context.inv_h = 0.0f;
	}

	world.inv_h = context.inv_h;

	// Hertz values get reduced for large time steps
	float contactHertz = min( world.contactHertz, 0.125f * context.inv_h );
	context.contactSoftness = b2MakeSoft( contactHertz, world.contactDampingRatio, context.h );
	context.staticSoftness = b2MakeSoft( 2.0f * contactHertz, world.contactDampingRatio, context.h );

	context.restitutionThreshold = world.restitutionThreshold;
	context.maxLinearVelocity = world.maxLinearSpeed;
	context.enableWarmStarting = world.enableWarmStarting;

	// Update contacts
	{
		ulong collideTicks = b2GetTicks();
		b2Collide( &context );
		world.profile.collide = b2GetMilliseconds( collideTicks );
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if ( context.dt > 0.0f )
	{
		ulong solveTicks = b2GetTicks();
		b2Solve( world, &context );
		world.profile.solve = b2GetMilliseconds( solveTicks );
	}

	// Update sensors
	{
		ulong sensorTicks = b2GetTicks();
		b2OverlapSensors( world );
		world.profile.sensors = b2GetMilliseconds( sensorTicks );
	}

	world.profile.step = b2GetMilliseconds( stepTicks );

	assert( b2GetArenaAllocation( &world.arena ) == 0 );

	// Ensure stack is large enough
	b2GrowArena( &world.arena );

	// Make sure all tasks that were started were also finished
	assert( world.activeTaskCount == 0 );

	// Swap end event array buffers
	world.endEventArrayIndex = 1 - world.endEventArrayIndex;
	b2SensorEndTouchEventArray_Clear( world.sensorEndEvents[world.endEventArrayIndex] );
	b2ContactEndTouchEventArray_Clear( world.contactEndEvents[world.endEventArrayIndex] );
	world.locked = false;
}

void b2DrawShape(b2DebugDraw* draw, b2Shape* shape, b2Transform xf, b2HexColor color)
{
	switch ( shape.type )
	{
		case b2_capsuleShape:
		{
			b2Capsule* capsule = &shape.capsule;
			b2Vec2 p1 = b2TransformPoint( xf, capsule.center1 );
			b2Vec2 p2 = b2TransformPoint( xf, capsule.center2 );
			draw.DrawSolidCapsuleFcn( p1, p2, capsule.radius, color, draw.context );
		}
		break;

		case b2_circleShape:
		{
			b2Circle* circle = &shape.circle;
			xf.p = b2TransformPoint( xf, circle.center );
			draw.DrawSolidCircleFcn( xf, circle.radius, color, draw.context );
		}
		break;

		case b2_polygonShape:
		{
			b2Polygon* poly = &shape.polygon;
			draw.DrawSolidPolygonFcn( xf, poly.vertices.ptr, poly.count, poly.radius, color, draw.context );
		}
		break;

		case b2_segmentShape:
		{
			b2Segment* segment = &shape.segment;
			b2Vec2 p1 = b2TransformPoint( xf, segment.point1 );
			b2Vec2 p2 = b2TransformPoint( xf, segment.point2 );
			draw.DrawSegmentFcn( p1, p2, color, draw.context );
		}
		break;

		case b2_chainSegmentShape:
		{
			b2Segment* segment = &shape.chainSegment.segment;
			b2Vec2 p1 = b2TransformPoint( xf, segment.point1 );
			b2Vec2 p2 = b2TransformPoint( xf, segment.point2 );
			draw.DrawSegmentFcn( p1, p2, color, draw.context );
			draw.DrawPointFcn( p2, 4.0f, color, draw.context );
			draw.DrawSegmentFcn( p1, b2Lerp( p1, p2, 0.1f ), b2_colorPaleGreen, draw.context );
		}
		break;

		default:
			break;
	}
}

struct DrawContext
{
	b2World* world;
	b2DebugDraw* draw;
}

bool DrawQueryCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	DrawContext* drawContext = cast(DrawContext*)context;
	b2World* world = drawContext.world;
	b2DebugDraw* draw = drawContext.draw;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );
	assert( shape.id == shapeId );

	b2SetBit( &world.debugBodySet, shape.bodyId );

	if ( draw.drawShapes )
	{
		b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
		b2BodySim* bodySim = b2GetBodySim( world, body );

		b2HexColor color = void;

		if ( shape.customColor != 0 )
		{
			color = cast(b2HexColor)shape.customColor;
		}
		else if ( body.type == b2_dynamicBody && body.mass == 0.0f )
		{
			// Bad body
			color = b2_colorRed;
		}
		else if ( body.setIndex == b2_disabledSet )
		{
			color = b2_colorSlateGray;
		}
		else if ( shape.sensorIndex != B2_NULL_INDEX )
		{
			color = b2_colorWheat;
		}
		else if ( body.flags & b2_hadTimeOfImpact )
		{
			color = b2_colorLime;
		}
		else if ( ( bodySim.flags & b2_isBullet ) && body.setIndex == b2_awakeSet )
		{
			color = b2_colorTurquoise;
		}
		else if ( body.flags & b2_isSpeedCapped )
		{
			color = b2_colorYellow;
		}
		else if ( bodySim.flags & b2_isFast )
		{
			color = b2_colorSalmon;
		}
		else if ( body.type == b2_staticBody )
		{
			color = b2_colorPaleGreen;
		}
		else if ( body.type == b2_kinematicBody )
		{
			color = b2_colorRoyalBlue;
		}
		else if ( body.setIndex == b2_awakeSet )
		{
			color = b2_colorPink;
		}
		else
		{
			color = b2_colorGray;
		}

		b2DrawShape( draw, shape, bodySim.transform, color );
	}

	if ( draw.drawBounds )
	{
		b2AABB aabb = shape.fatAABB;

		b2Vec2[4] vs = [ { aabb.lowerBound.x, aabb.lowerBound.y },
						 { aabb.upperBound.x, aabb.lowerBound.y },
						 { aabb.upperBound.x, aabb.upperBound.y },
						 { aabb.lowerBound.x, aabb.upperBound.y } ];

		draw.DrawPolygonFcn( vs.ptr, 4, b2_colorGold, draw.context );
	}

	return true;
}

// todo this has varying order for moving shapes, causing flicker when overlapping shapes are moving
// solution: display order by shape id modulus 3, keep 3 buckets in GLSolid* and flush in 3 passes.
void b2DrawWithBounds(b2World* world, b2DebugDraw* draw)
{
	assert( draw.drawingBounds.isValid() );

	const(float) k_impulseScale = 1.0f;
	const(float) k_axisScale = 0.3f;
	b2HexColor speculativeColor = b2_colorGainsboro;
	b2HexColor addColor = b2_colorGreen;
	b2HexColor persistColor = b2_colorBlue;
	b2HexColor normalColor = b2_colorDimGray;
	b2HexColor impulseColor = b2_colorMagenta;
	b2HexColor frictionColor = b2_colorYellow;

	b2HexColor[B2_GRAPH_COLOR_COUNT] graphColors = [
		b2_colorRed,	b2_colorOrange, b2_colorYellow,	   b2_colorGreen,	  b2_colorCyan,		b2_colorBlue,
		b2_colorViolet, b2_colorPink,	b2_colorChocolate, b2_colorGoldenRod, b2_colorCoral,	b2_colorRosyBrown,
		b2_colorAqua,	b2_colorPeru,	b2_colorLime,	   b2_colorGold,	  b2_colorPlum,		b2_colorSnow,
		b2_colorTeal,	b2_colorKhaki,	b2_colorSalmon,	   b2_colorPeachPuff, b2_colorHoneyDew, b2_colorBlack,
	];

	int bodyCapacity = b2GetIdCapacity( &world.bodyIdPool );
	b2SetBitCountAndClear( &world.debugBodySet, bodyCapacity );

	int jointCapacity = b2GetIdCapacity( &world.jointIdPool );
	b2SetBitCountAndClear( &world.debugJointSet, jointCapacity );

	int contactCapacity = b2GetIdCapacity( &world.contactIdPool );
	b2SetBitCountAndClear( &world.debugContactSet, contactCapacity );

	DrawContext drawContext = { world, draw };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Query( &world.broadPhase.trees[i], draw.drawingBounds, B2_DEFAULT_MASK_BITS, &DrawQueryCallback,
							 &drawContext );
	}

	uint wordCount = world.debugBodySet.blockCount;
	ulong* bits = world.debugBodySet.bits;
	for ( uint k = 0; k < wordCount; ++k )
	{
		ulong word = bits[k];
		while ( word != 0 )
		{
			uint ctz = b2CTZ64( word );
			uint bodyId = 64 * k + ctz;

			b2Body* body = b2BodyArray_Get( world.bodies, bodyId );

			if ( draw.drawBodyNames && body.name[0] != 0 )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2Transform transform = { bodySim.center, bodySim.transform.q };
				b2Vec2 p = b2TransformPoint( transform, offset );
				draw.DrawStringFcn( p, cast(const(char*))body.name, b2_colorBlueViolet, draw.context );
			}

			if ( draw.drawMass && body.type == b2_dynamicBody )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2Transform transform = { bodySim.center, bodySim.transform.q };
				draw.DrawTransformFcn( transform, draw.context );

				b2Vec2 p = b2TransformPoint( transform, offset );

				char[32] buffer = void;
				snprintf( buffer.ptr, 32, "  %.2f", body.mass );
				draw.DrawStringFcn( p, buffer.ptr, b2_colorWhite, draw.context );
			}

			if ( draw.drawJoints )
			{
				int jointKey = body.headJointKey;
				while ( jointKey != B2_NULL_INDEX )
				{
					int jointId = jointKey >> 1;
					int edgeIndex = jointKey & 1;
					b2Joint* joint = b2JointArray_Get( world.joints, jointId );

					// avoid double draw
					if ( b2GetBit( &world.debugJointSet, jointId ) == false )
					{
						b2DrawJoint( draw, world, joint );
						b2SetBit( &world.debugJointSet, jointId );
					}
					else
					{
						// todo testing
						edgeIndex += 0;
					}

					jointKey = joint.edges[edgeIndex].nextKey;
				}
			}

			const(float) linearSlop = B2_LINEAR_SLOP;
			if ( draw.drawContacts && body.type == b2_dynamicBody && body.setIndex == b2_awakeSet )
			{
				int contactKey = body.headContactKey;
				while ( contactKey != B2_NULL_INDEX )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;
					b2Contact* contact = b2ContactArray_Get( world.contacts, contactId );
					contactKey = contact.edges[edgeIndex].nextKey;

					if ( contact.setIndex != b2_awakeSet || contact.colorIndex == B2_NULL_INDEX )
					{
						continue;
					}

					// avoid double draw
					if ( b2GetBit( &world.debugContactSet, contactId ) == false )
					{
						assert( 0 <= contact.colorIndex && contact.colorIndex < B2_GRAPH_COLOR_COUNT );

						b2GraphColor* gc = &world.constraintGraph.colors[contact.colorIndex];
						b2ContactSim* contactSim = b2ContactSimArray_Get( gc.contactSims, contact.localIndex );
						int pointCount = contactSim.manifold.pointCount;
						b2Vec2 normal = contactSim.manifold.normal;
						char[32] buffer = void;

						for ( int j = 0; j < pointCount; ++j )
						{
							b2ManifoldPoint* point = &contactSim.manifold.points[j];

							if ( draw.drawGraphColors )
							{
								// graph color
								float pointSize = contact.colorIndex == B2_OVERFLOW_INDEX ? 7.5f : 5.0f;
								draw.DrawPointFcn( point.point, pointSize, graphColors[contact.colorIndex], draw.context );
								// m_context->draw.DrawString(point->position, "%d", point->color);
							}
							else if ( point.separation > linearSlop )
							{
								// Speculative
								draw.DrawPointFcn( point.point, 5.0f, speculativeColor, draw.context );
							}
							else if ( point.persisted == false )
							{
								// Add
								draw.DrawPointFcn( point.point, 10.0f, addColor, draw.context );
							}
							else if ( point.persisted == true )
							{
								// Persist
								draw.DrawPointFcn( point.point, 5.0f, persistColor, draw.context );
							}

							if ( draw.drawContactNormals )
							{
								b2Vec2 p1 = point.point;
								b2Vec2 p2 = b2MulAdd( p1, k_axisScale, normal );
								draw.DrawSegmentFcn( p1, p2, normalColor, draw.context );
							}
							else if ( draw.drawContactImpulses )
							{
								b2Vec2 p1 = point.point;
								b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point.normalImpulse, normal );
								draw.DrawSegmentFcn( p1, p2, impulseColor, draw.context );
								snprintf( buffer.ptr, buffer.length, "%.1f", 1000.0f * point.normalImpulse );
								draw.DrawStringFcn( p1, buffer.ptr, b2_colorWhite, draw.context );
							}

							if ( draw.drawContactFeatures )
							{
								snprintf( buffer.ptr, buffer.length, "%d", point.id );
								draw.DrawStringFcn( point.point, buffer.ptr, b2_colorOrange, draw.context );
							}

							if ( draw.drawFrictionImpulses )
							{
								b2Vec2 tangent = normal.rightPerp();
								b2Vec2 p1 = point.point;
								b2Vec2 p2 = b2MulAdd( p1, k_impulseScale * point.tangentImpulse, tangent );
								draw.DrawSegmentFcn( p1, p2, frictionColor, draw.context );
								snprintf( buffer.ptr, buffer.length, "%.1f", 1000.0f * point.tangentImpulse );
								draw.DrawStringFcn( p1, buffer.ptr, b2_colorWhite, draw.context );
							}
						}

						b2SetBit( &world.debugContactSet, contactId );
					}
					else
					{
						// todo testing
						edgeIndex += 0;
					}

					contactKey = contact.edges[edgeIndex].nextKey;
				}
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}
}

void b2World_Draw(b2WorldId worldId, b2DebugDraw* draw)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	b2DrawWithBounds( world, draw );
}

b2BodyEvents b2World_GetBodyEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return b2BodyEvents();
	}

	int count = cast(int)world.bodyMoveEvents.count;
	b2BodyEvents events = { world.bodyMoveEvents.ptr, count };
	return events;
}

b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return b2SensorEvents();
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world.endEventArrayIndex;

	int beginCount = cast(int)world.sensorBeginEvents.count;
	int endCount = cast(int)world.sensorEndEvents[endEventArrayIndex].count;

	b2SensorEvents events = {
		beginEvents: world.sensorBeginEvents.ptr,
		endEvents: world.sensorEndEvents[endEventArrayIndex].ptr,
		beginCount: beginCount,
		endCount: endCount,
	};
	return events;
}

b2ContactEvents b2World_GetContactEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return b2ContactEvents();
	}

	// Careful to use previous buffer
	int endEventArrayIndex = 1 - world.endEventArrayIndex;

	int beginCount = cast(int)world.contactBeginEvents.count;
	int endCount = cast(int)world.contactEndEvents[endEventArrayIndex].count;
	int hitCount = cast(int)world.contactHitEvents.count;

	b2ContactEvents events = {
		beginEvents: world.contactBeginEvents.ptr,
		endEvents: world.contactEndEvents[endEventArrayIndex].ptr,
		hitEvents: world.contactHitEvents.ptr,
		beginCount: beginCount,
		endCount: endCount,
		hitCount: hitCount,
	};

	return events;
}

b2JointEvents b2World_GetJointEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return b2JointEvents();
	}

	int count = cast(int)world.jointEvents.count;
	b2JointEvents events = { world.jointEvents.ptr, count };
	return events;
}

bool b2World_IsValid(b2WorldId id)
{
	if ( id.index1 < 1 || B2_MAX_WORLDS < id.index1 )
	{
		return false;
	}

	b2World* world = b2_worlds.ptr + ( id.index1 - 1 );

	if ( world.worldId != id.index1 - 1 )
	{
		// world is not allocated
		return false;
	}

	return id.generation == world.generation;
}

bool b2Body_IsValid(b2BodyId id)
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		// invalid world
		return false;
	}

	b2World* world = b2_worlds.ptr + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	if ( id.index1 < 1 || world.bodies.count < id.index1 )
	{
		// invalid index
		return false;
	}

	b2Body* body = &world.bodies[ id.index1 - 1 ];
	if ( body.setIndex == B2_NULL_INDEX )
	{
		// this was freed
		return false;
	}

	assert( body.localIndex != B2_NULL_INDEX );

	if ( body.generation != id.generation )
	{
		// this id is orphaned
		return false;
	}

	return true;
}

bool b2Shape_IsValid(b2ShapeId id)
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds.ptr + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int shapeId = id.index1 - 1;
	if ( shapeId < 0 || world.shapes.count <= shapeId )
	{
		return false;
	}

	b2Shape* shape = &world.shapes[shapeId];
	if ( shape.id == B2_NULL_INDEX )
	{
		// shape is free
		return false;
	}

	assert( shape.id == shapeId );

	return id.generation == shape.generation;
}

bool b2Chain_IsValid(b2ChainId id)
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds.ptr + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int chainId = id.index1 - 1;
	if ( chainId < 0 || world.chainShapes.count <= chainId )
	{
		return false;
	}

	b2ChainShape* chain = &world.chainShapes[chainId];
	if ( chain.id == B2_NULL_INDEX )
	{
		// chain is free
		return false;
	}

	assert( chain.id == chainId );

	return id.generation == chain.generation;
}

bool b2Joint_IsValid(b2JointId id)
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds.ptr + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int jointId = id.index1 - 1;
	if ( jointId < 0 || world.joints.count <= jointId )
	{
		return false;
	}

	b2Joint* joint = &world.joints[jointId];
	if ( joint.jointId == B2_NULL_INDEX )
	{
		// joint is free
		return false;
	}

	assert( joint.jointId == jointId );

	return id.generation == joint.generation;
}

bool b2Contact_IsValid(b2ContactId id)
{
	if ( B2_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b2World* world = b2_worlds.ptr + id.world0;
	if ( world.worldId != id.world0 )
	{
		// world is free
		return false;
	}

	int contactId = id.index1 - 1;
	if ( contactId < 0 || world.contacts.count <= contactId )
	{
		return false;
	}

	b2Contact* contact = &world.contacts[contactId];
	if ( contact.contactId == B2_NULL_INDEX )
	{
		// contact is free
		return false;
	}

	assert( contact.contactId == contactId );

	return id.generation == contact.generation;
}

void b2World_EnableSleeping(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	if ( flag == world.enableSleep )
	{
		return;
	}

	world.enableSleep = flag;

	if ( flag == false )
	{
		int setCount = cast(int)world.solverSets.count;
		for ( int i = b2_firstSleepingSet; i < setCount; ++i )
		{
			b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, i );
			if ( set.bodySims.count > 0 )
			{
				b2WakeSolverSet( world, i );
			}
		}
	}
}

bool b2World_IsSleepingEnabled(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.enableSleep;
}

void b2World_EnableWarmStarting(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.enableWarmStarting = flag;
}

bool b2World_IsWarmStartingEnabled(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.enableWarmStarting;
}

int b2World_GetAwakeBodyCount(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	return cast(int)awakeSet.bodySims.count;
}

void b2World_EnableContinuous(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.enableContinuous = flag;
}

bool b2World_IsContinuousEnabled(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.enableContinuous;
}

void b2World_SetRestitutionThreshold(b2WorldId worldId, float value)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.restitutionThreshold = clamp( value, 0.0f, float.max );
}

float b2World_GetRestitutionThreshold(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.restitutionThreshold;
}

void b2World_SetHitEventThreshold(b2WorldId worldId, float value)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.hitEventThreshold = clamp( value, 0.0f, float.max );
}

float b2World_GetHitEventThreshold(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.hitEventThreshold;
}

void b2World_SetContactTuning(b2WorldId worldId, float hertz, float dampingRatio, float pushSpeed)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.contactHertz = clamp( hertz, 0.0f, float.max );
	world.contactDampingRatio = clamp( dampingRatio, 0.0f, float.max );
	world.contactSpeed = clamp( pushSpeed, 0.0f, float.max );
}

void b2World_SetMaximumLinearSpeed(b2WorldId worldId, float maximumLinearSpeed)
{
	assert( b2IsValidFloat( maximumLinearSpeed ) && maximumLinearSpeed > 0.0f );

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	world.maxLinearSpeed = maximumLinearSpeed;
}

float b2World_GetMaximumLinearSpeed(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.maxLinearSpeed;
}

b2Profile b2World_GetProfile(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.profile;
}

b2Counters b2World_GetCounters(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	b2Counters s = { 0 };
	s.bodyCount = b2GetIdCount( &world.bodyIdPool );
	s.shapeCount = b2GetIdCount( &world.shapeIdPool );
	s.contactCount = b2GetIdCount( &world.contactIdPool );
	s.jointCount = b2GetIdCount( &world.jointIdPool );
	s.islandCount = b2GetIdCount( &world.islandIdPool );

	b2DynamicTree* staticTree = &world.broadPhase.trees[b2_staticBody];
	s.staticTreeHeight = b2DynamicTree_GetHeight( staticTree );

	b2DynamicTree* dynamicTree = &world.broadPhase.trees[b2_dynamicBody];
	b2DynamicTree* kinematicTree = &world.broadPhase.trees[b2_kinematicBody];
	s.treeHeight = max( b2DynamicTree_GetHeight( dynamicTree ), b2DynamicTree_GetHeight( kinematicTree ) );

	s.stackUsed = b2GetMaxArenaAllocation( &world.arena );
	s.byteCount = b2GetByteCount();
	s.taskCount = world.taskCount;

	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		s.colorCounts[i] = cast(int)(world.constraintGraph.colors[i].contactSims.count + world.constraintGraph.colors[i].jointSims.count);
	}
	return s;
}

void b2World_SetUserData(b2WorldId worldId, void* userData)
{
	b2World* world = b2GetWorldFromId( worldId );
	world.userData = userData;
}

void* b2World_GetUserData(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.userData;
}

void b2World_SetFrictionCallback(b2WorldId worldId, b2FrictionCallback callback)
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world.locked )
	{
		return;
	}

	if ( callback != null )
	{
		world.frictionCallback = callback;
	}
	else
	{
		world.frictionCallback = &b2DefaultFrictionCallback;
	}
}

void b2World_SetRestitutionCallback(b2WorldId worldId, b2RestitutionCallback callback)
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world.locked )
	{
		return;
	}

	if ( callback != null )
	{
		world.restitutionCallback = callback;
	}
	else
	{
		world.restitutionCallback = &b2DefaultRestitutionCallback;
	}
}

void b2World_DumpMemoryStats(b2WorldId worldId)
{
	FILE* file = fopen( "box2d_memory.txt", "w" );
	if ( file == null )
	{
		return;
	}

	b2World* world = b2GetWorldFromId( worldId );

	// id pools
	fprintf( file, "id pools\n" );
	fprintf( file, "body ids: %d\n", b2GetIdBytes( &world.bodyIdPool ) );
	fprintf( file, "solver set ids: %d\n", b2GetIdBytes( &world.solverSetIdPool ) );
	fprintf( file, "joint ids: %d\n", b2GetIdBytes( &world.jointIdPool ) );
	fprintf( file, "contact ids: %d\n", b2GetIdBytes( &world.contactIdPool ) );
	fprintf( file, "island ids: %d\n", b2GetIdBytes( &world.islandIdPool ) );
	fprintf( file, "shape ids: %d\n", b2GetIdBytes( &world.shapeIdPool ) );
	fprintf( file, "chain ids: %d\n", b2GetIdBytes( &world.chainIdPool ) );
	fprintf( file, "\n" );

	// world arrays
	fprintf( file, "world arrays\n" );
	fprintf( file, "bodies: %d\n", b2BodyArray_ByteCount( world.bodies ) );
	fprintf( file, "solver sets: %d\n", b2SolverSetArray_ByteCount( world.solverSets ) );
	fprintf( file, "joints: %d\n", b2JointArray_ByteCount( world.joints ) );
	fprintf( file, "contacts: %d\n", b2ContactArray_ByteCount( world.contacts ) );
	fprintf( file, "islands: %d\n", b2IslandArray_ByteCount( world.islands ) );
	fprintf( file, "shapes: %d\n", b2ShapeArray_ByteCount( world.shapes ) );
	fprintf( file, "chains: %d\n", b2ChainShapeArray_ByteCount( world.chainShapes ) );
	fprintf( file, "\n" );

	// broad-phase
	fprintf( file, "broad-phase\n" );
	fprintf( file, "static tree: %d\n", b2DynamicTree_GetByteCount( &world.broadPhase.trees[b2_staticBody] ) );
	fprintf( file, "kinematic tree: %d\n", b2DynamicTree_GetByteCount( &world.broadPhase.trees[b2_kinematicBody] ) );
	fprintf( file, "dynamic tree: %d\n", b2DynamicTree_GetByteCount( &world.broadPhase.trees[b2_dynamicBody] ) );
	b2HashSet* moveSet = &world.broadPhase.moveSet;
	fprintf( file, "moveSet: %d (%d, %d)\n", b2GetHashSetBytes( moveSet ), moveSet.count, moveSet.capacity );
	fprintf( file, "moveArray: %d\n", b2IntArray_ByteCount( world.broadPhase.moveArray ) );
	b2HashSet* pairSet = &world.broadPhase.pairSet;
	fprintf( file, "pairSet: %d (%d, %d)\n", b2GetHashSetBytes( pairSet ), pairSet.count, pairSet.capacity );
	fprintf( file, "\n" );

	// solver sets
	int bodySimCapacity = 0;
	int bodyStateCapacity = 0;
	int jointSimCapacity = 0;
	int contactSimCapacity = 0;
	int islandSimCapacity = 0;
	int solverSetCapacity = cast(int)world.solverSets.count;
	for ( int i = 0; i < solverSetCapacity; ++i )
	{
		b2SolverSet* set = &world.solverSets[i];
		if ( set.setIndex == B2_NULL_INDEX )
		{
			continue;
		}

		bodySimCapacity += set.bodySims.capacity;
		bodyStateCapacity += set.bodyStates.capacity;
		jointSimCapacity += set.jointSims.capacity;
		contactSimCapacity += set.contactSims.capacity;
		islandSimCapacity += set.islandSims.capacity;
	}

	fprintf( file, "solver sets\n" );
	fprintf( file, "body sim: %d\n", bodySimCapacity * cast(int)b2BodySim.sizeof );
	fprintf( file, "body state: %d\n", bodyStateCapacity * cast(int)b2BodyState.sizeof );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * cast(int)b2JointSim.sizeof );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * cast(int)b2ContactSim.sizeof );
	fprintf( file, "island sim: %d\n", islandSimCapacity * cast(int)islandSimCapacity.sizeof );
	fprintf( file, "\n" );

	// constraint graph
	int bodyBitSetBytes = 0;
	contactSimCapacity = 0;
	jointSimCapacity = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* c = &world.constraintGraph.colors[i];
		bodyBitSetBytes += b2GetBitSetBytes( &c.bodySet );
		contactSimCapacity += c.contactSims.capacity;
		jointSimCapacity += c.jointSims.capacity;
	}

	fprintf( file, "constraint graph\n" );
	fprintf( file, "body bit sets: %d\n", bodyBitSetBytes );
	fprintf( file, "joint sim: %d\n", jointSimCapacity * cast(int)b2JointSim.sizeof );
	fprintf( file, "contact sim: %d\n", contactSimCapacity * cast(int)b2ContactSim.sizeof );
	fprintf( file, "\n" );

	// stack allocator
	fprintf( file, "stack allocator: %d\n\n", world.arena.capacity );

	// chain shapes
	// todo

	fclose( file );
}

struct WorldQueryContext {
	b2World* world;
	b2OverlapResultFcn fcn;
	b2QueryFilter filter;
	void* userContext;
}

bool TreeQueryCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	WorldQueryContext* worldContext = cast(WorldQueryContext*)context;
	b2World* world = worldContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	if ( b2ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
	{
		return true;
	}

	b2ShapeId id = { shapeId + 1, world.worldId, shape.generation };
	bool result = worldContext.fcn( id, worldContext.userContext );
	return result;
}

b2TreeStats b2World_OverlapAABB(b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn fcn, void* context)
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	assert( aabb.isValid() );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_Query( &world.broadPhase.trees[i], aabb, filter.maskBits, &TreeQueryCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

struct WorldOverlapContext {
	b2World* world;
	b2OverlapResultFcn fcn;
	b2QueryFilter filter;
	const(b2ShapeProxy)* proxy;
	void* userContext;
}

bool TreeOverlapCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	WorldOverlapContext* worldContext = cast(WorldOverlapContext*)context;
	b2World* world = worldContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	if ( b2ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
	{
		return true;
	}

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input = void;
	input.proxyA = *worldContext.proxy;
	input.proxyB = b2MakeShapeDistanceProxy( shape );
	input.transformA = b2Transform.identity();
	input.transformB = transform;
	input.useRadii = true;

	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, null, 0 );

	float tolerance = 0.1f * B2_LINEAR_SLOP;
	if ( output.distance > tolerance )
	{
		return true;
	}

	b2ShapeId id = { shape.id + 1, world.worldId, shape.generation };
	bool result = worldContext.fcn( id, worldContext.userContext );
	return result;
}

b2TreeStats b2World_OverlapShape(b2WorldId worldId, const(b2ShapeProxy)* proxy, b2QueryFilter filter, b2OverlapResultFcn fcn, void* context)
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}


	b2AABB aabb = b2AABB.make( proxy.points.ptr, proxy.count, proxy.radius );
	WorldOverlapContext worldContext = {
		world, fcn, filter, proxy, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_Query( &world.broadPhase.trees[i], aabb, filter.maskBits, &TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}

	return treeStats;
}

struct WorldRayCastContext {
	b2World* world;
	b2CastResultFcn fcn;
	b2QueryFilter filter;
	float fraction = 0;
	void* userContext;
}

float RayCastCallback(const(b2RayCastInput)* input, int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	WorldRayCastContext* worldContext = cast(WorldRayCastContext*)context;
	b2World* world = worldContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	if ( b2ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
	{
		return input.maxFraction;
	}

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );
	b2CastOutput output = b2RayCastShape( input, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world.worldId, shape.generation };
		float fraction = worldContext.fcn( id, output.point, output.normal, output.fraction, worldContext.userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext.fraction = fraction;
		}

		return fraction;
	}

	return input.maxFraction;
}

b2TreeStats b2World_CastRay(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn fcn, void* context)
{
	b2TreeStats treeStats = { 0 };

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	assert( b2IsValidVec2( origin ) );
	assert( b2IsValidVec2( translation ) );

	b2RayCastInput input = { origin, translation, 1.0f };

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_RayCast( &world.broadPhase.trees[i], &input, filter.maskBits, &RayCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

// This callback finds the closest hit. This is the most common callback used in games.
float b2RayCastClosestFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	// Ignore initial overlap
	if ( fraction == 0.0f )
	{
		return -1.0f;
	}

	b2RayResult* rayResult = cast(b2RayResult*)context;
	rayResult.shapeId = shapeId;
	rayResult.point = point;
	rayResult.normal = normal;
	rayResult.fraction = fraction;
	rayResult.hit = true;
	return fraction;
}

b2RayResult b2World_CastRayClosest(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter)
{
	b2RayResult result;

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return result;
	}

	assert( b2IsValidVec2( origin ) );
	assert( b2IsValidVec2( translation ) );

	b2RayCastInput input = { origin, translation, 1.0f };
	WorldRayCastContext worldContext = { world, &b2RayCastClosestFcn, filter, 1.0f, &result };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_RayCast( &world.broadPhase.trees[i], &input, filter.maskBits, &RayCastCallback, &worldContext );
		result.nodeVisits += treeResult.nodeVisits;
		result.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return result;
		}

		input.maxFraction = worldContext.fraction;
	}

	return result;
}

float ShapeCastCallback(b2ShapeCastInput* input, int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	WorldRayCastContext* worldContext = cast(WorldRayCastContext*)context;
	b2World* world = worldContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	if ( b2ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
	{
		return input.maxFraction;
	}

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2CastOutput output = b2ShapeCastShape( input, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world.worldId, shape.generation };
		float fraction = worldContext.fcn( id, output.point, output.normal, output.fraction, worldContext.userContext );

		// The user may return -1 to skip this shape
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext.fraction = fraction;
		}

		return fraction;
	}

	return input.maxFraction;
}

b2TreeStats b2World_CastShape(b2WorldId worldId, const(b2ShapeProxy)* proxy, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn fcn, void* context)
{
	b2TreeStats treeStats;

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return treeStats;
	}

	assert( b2IsValidVec2( translation ) );

	b2ShapeCastInput input;
	input.proxy = *proxy;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		// b2TreeStats treeResult = b2DynamicTree_ShapeCast( &world.broadPhase.trees[i], &input, filter.maskBits, &ShapeCastCallback, worldContext );
		b2TreeStats treeResult = 
			b2DynamicTree_ShapeCast(&world.broadPhase.trees[i], &input, filter.maskBits, 
			&(ShapeCastCallback), cast(void*)&worldContext);
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			return treeStats;
		}

		input.maxFraction = worldContext.fraction;
	}

	return treeStats;
}

struct b2CharacterCallbackContext {
	b2World* world;
	b2QueryFilter filter;
	b2ShapeProxy proxy;
	b2Transform transform;
	void* userContext;
}

struct WorldMoverCastContext {
	b2World* world;
	b2QueryFilter filter;
	float fraction = 0;
}

float MoverCastCallback(const(b2ShapeCastInput)* input, int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;
	WorldMoverCastContext* worldContext = cast(WorldMoverCastContext*)context;
	b2World* world = worldContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	if ( b2ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
	{
		return worldContext.fraction;
	}

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2CastOutput output = b2ShapeCastShape( input, shape, transform );
	if ( output.fraction == 0.0f )
	{
		// Ignore overlapping shapes
		return worldContext.fraction;
	}

	worldContext.fraction = output.fraction;
	return output.fraction;
}

float b2World_CastMover(b2WorldId worldId, const(b2Capsule)* mover, b2Vec2 translation, b2QueryFilter filter)
{
	assert( b2IsValidVec2( translation ) );
	assert( mover.radius > 2.0f * B2_LINEAR_SLOP );

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return 1.0f;
	}

	b2ShapeCastInput input;
	input.proxy.points[0] = mover.center1;
	input.proxy.points[1] = mover.center2;
	input.proxy.count = 2;
	input.proxy.radius = mover.radius;
	input.translation = translation;
	input.maxFraction = 1.0f;
	input.canEncroach = true;

	WorldMoverCastContext worldContext = { world, filter, 1.0f };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_ShapeCast( &world.broadPhase.trees[i], &input, filter.maskBits, &MoverCastCallback, &worldContext );

		if ( worldContext.fraction == 0.0f )
		{
			return 0.0f;
		}

		input.maxFraction = worldContext.fraction;
	}

	return worldContext.fraction;
}

struct WorldMoverContext {
	b2World* world;
	b2PlaneResultFcn fcn;
	b2QueryFilter filter;
	b2Capsule mover;
	void* userContext;
}

bool TreeCollideCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;
	WorldMoverContext* worldContext = cast(WorldMoverContext*)context;
	b2World* world = worldContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	if ( b2ShouldQueryCollide( shape.filter, worldContext.filter ) == false )
	{
		return true;
	}

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2PlaneResult result = b2CollideMover( &worldContext.mover, shape, transform );

	// todo handle deep overlap
	if ( result.hit && b2IsNormalized( result.plane.normal ) )
	{
		b2ShapeId id = { shape.id + 1, world.worldId, shape.generation };
		return worldContext.fcn( id, &result, worldContext.userContext );
	}

	return true;
}

// It is tempting to use a shape proxy for the mover, but this makes handling deep overlap difficult and the generality may
// not be worth it.
void b2World_CollideMover(b2WorldId worldId, const(b2Capsule)* mover, b2QueryFilter filter, b2PlaneResultFcn fcn, void* context)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	b2Vec2 r = { mover.radius, mover.radius };

	b2AABB aabb = void;
	aabb.lowerBound = b2Min( mover.center1, mover.center2 ) - r;
	aabb.upperBound = b2Max( mover.center1, mover.center2 ) + r;

	WorldMoverContext worldContext = {
		world, fcn, filter, *mover, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Query( &world.broadPhase.trees[i], aabb, filter.maskBits, &TreeCollideCallback, &worldContext );
	}
}

version (none) {

void b2World_Dump()
{
	if (m_locked)
	{
		return;
	}

	b2OpenDump("box2d_dump.inl");

	b2Dump("b2Vec2 g(%.9g, %.9g);\n", m_gravity.x, m_gravity.y);
	b2Dump("m_world->SetGravity(g);\n");

	b2Dump("b2Body** sims = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);

	int i = 0;
	for (b2Body* b = m_bodyList; b; b = b.m_next)
	{
		b.m_islandIndex = i;
		b.Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		j.m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		if (j.m_type == e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j.Dump();
		b2Dump("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j.m_next)
	{
		if (j.m_type != e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j.Dump();
		b2Dump("}\n");
	}

	b2Dump("b2Free(joints);\n");
	b2Dump("b2Free(sims);\n");
	b2Dump("joints = nullptr;\n");
	b2Dump("sims = nullptr;\n");

	b2CloseDump();
}
}

void b2World_SetCustomFilterCallback(b2WorldId worldId, b2CustomFilterFcn fcn, void* context)
{
	b2World* world = b2GetWorldFromId( worldId );
	world.customFilterFcn = fcn;
	world.customFilterContext = context;
}

void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn fcn, void* context)
{
	b2World* world = b2GetWorldFromId( worldId );
	world.preSolveFcn = fcn;
	world.preSolveContext = context;
}

void b2World_SetGravity(b2WorldId worldId, b2Vec2 gravity)
{
	b2World* world = b2GetWorldFromId( worldId );
	world.gravity = gravity;
}

b2Vec2 b2World_GetGravity(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	return world.gravity;
}

struct ExplosionContext
{
	b2World* world;
	b2Vec2 position;
	float radius = 0;
	float falloff = 0;
	float impulsePerLength = 0;
}

bool ExplosionCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	ExplosionContext* explosionContext = cast(ExplosionContext*)context;
	b2World* world = explosionContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );
	assert( body.type == b2_dynamicBody );

	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input = void;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &explosionContext.position, 1, 0.0f );
	input.transformA = transform;
	input.transformB = b2Transform.identity();
	input.useRadii = true;

	b2SimplexCache cache;
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, null, 0 );

	float radius = explosionContext.radius;
	float falloff = explosionContext.falloff;
	if ( output.distance > radius + falloff )
	{
		return true;
	}

	b2WakeBody( world, body );

	if ( body.setIndex != b2_awakeSet )
	{
		return true;
	}

	b2Vec2 closestPoint = output.pointA;
	if ( output.distance == 0.0f )
	{
		b2Vec2 localCentroid = b2GetShapeCentroid( shape );
		closestPoint = b2TransformPoint( transform, localCentroid );
	}

	b2Vec2 direction = closestPoint - explosionContext.position;
	if ( b2LengthSquared( direction ) > 100.0f * float.epsilon * float.epsilon )
	{
		direction = direction.normalized;
	}
	else
	{
		direction = b2Vec2( 1.0f, 0.0f );
	}

	b2Vec2 localLine = b2InvRotateVector( transform.q, direction.leftPerp() );
	float perimeter = b2GetShapeProjectedPerimeter( shape, localLine );
	float scale = 1.0f;
	if ( output.distance > radius && falloff > 0.0f )
	{
		scale = clamp( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
	}

	float magnitude = explosionContext.impulsePerLength * perimeter * scale;
	b2Vec2 impulse = b2MulSV( magnitude, direction );

	int localIndex = body.localIndex;
	b2SolverSet* set = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	b2BodyState* state = b2BodyStateArray_Get( set.bodyStates, localIndex );
	b2BodySim* bodySim = b2BodySimArray_Get( set.bodySims, localIndex );
	state.linearVelocity = b2MulAdd( state.linearVelocity, bodySim.invMass, impulse );
	state.angularVelocity += bodySim.invInertia * (closestPoint - bodySim.center).cross( impulse );

	return true;
}

void b2World_Explode(b2WorldId worldId, const(b2ExplosionDef)* explosionDef)
{
	ulong maskBits = explosionDef.maskBits;
	b2Vec2 position = explosionDef.position;
	float radius = explosionDef.radius;
	float falloff = explosionDef.falloff;
	float impulsePerLength = explosionDef.impulsePerLength;

	assert( b2IsValidVec2( position ) );
	assert( b2IsValidFloat( radius ) && radius >= 0.0f );
	assert( b2IsValidFloat( falloff ) && falloff >= 0.0f );
	assert( b2IsValidFloat( impulsePerLength ) );

	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	ExplosionContext explosionContext = { world, position, radius, falloff, impulsePerLength };

	b2AABB aabb = void;
	aabb.lowerBound.x = position.x - ( radius + falloff );
	aabb.lowerBound.y = position.y - ( radius + falloff );
	aabb.upperBound.x = position.x + ( radius + falloff );
	aabb.upperBound.y = position.y + ( radius + falloff );

	b2DynamicTree_Query( &world.broadPhase.trees[b2_dynamicBody], aabb, maskBits, &ExplosionCallback, &explosionContext );
}

void b2World_RebuildStaticTree(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId( worldId );
	assert( world.locked == false );
	if ( world.locked )
	{
		return;
	}

	b2DynamicTree* staticTree = &world.broadPhase.trees[b2_staticBody];
	b2DynamicTree_Rebuild( staticTree, true );
}

void b2World_EnableSpeculative(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId( worldId );
	world.enableSpeculative = flag;
}

static if (B2_VALIDATE) {
version (none) {
// When validating islands ids I have to compare the root island
// ids because islands are not merged until the next time step.
int b2GetRootIslandId(b2World* world, int islandId)
{
	if ( islandId == B2_NULL_INDEX )
	{
		return B2_NULL_INDEX;
	}

	b2Island* island = b2IslandArray_Get( &world.islands, islandId );

	int rootId = islandId;
	b2Island* rootIsland = island;
	while ( rootIsland.parentIsland != B2_NULL_INDEX )
	{
		b2Island* parent = b2IslandArray_Get( &world.islands, rootIsland.parentIsland );
		rootId = rootIsland.parentIsland;
		rootIsland = parent;
	}

	return rootId;
}
}

// This validates island graph connectivity for each body
void b2ValidateConnectivity(b2World* world)
{
	b2Body* bodies = world.bodies;
	int bodyCapacity = world.bodies.count;

	for ( int bodyIndex = 0; bodyIndex < bodyCapacity; ++bodyIndex )
	{
		b2Body* body = bodies + bodyIndex;
		if ( body.id == B2_NULL_INDEX )
		{
			b2ValidateFreeId( &world.bodyIdPool, bodyIndex );
			continue;
		}

		b2ValidateUsedId( &world.bodyIdPool, bodyIndex );

		assert( bodyIndex == body.id );

		// Need to get the root island because islands are not merged until the next time step
		//int bodyIslandId = b2GetRootIslandId( world, body->islandId );
		int bodyIslandId = body.islandId;
		int bodySetIndex = body.setIndex;

		int contactKey = body.headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int contactId = contactKey >> 1;
			int edgeIndex = contactKey & 1;

			b2Contact* contact = b2ContactArray_Get( &world.contacts, contactId );

			bool touching = ( contact.flags & b2_contactTouchingFlag ) != 0;
			if ( touching )
			{
				if ( bodySetIndex != b2_staticSet )
				{
					//int contactIslandId = b2GetRootIslandId( world, contact->islandId );
					int contactIslandId = contact.islandId;
					assert( contactIslandId == bodyIslandId );
				}
			}
			else
			{
				assert( contact.islandId == B2_NULL_INDEX );
			}

			contactKey = contact.edges[edgeIndex].nextKey;
		}

		int jointKey = body.headJointKey;
		while ( jointKey != B2_NULL_INDEX )
		{
			int jointId = jointKey >> 1;
			int edgeIndex = jointKey & 1;

			b2Joint* joint = b2JointArray_Get( &world.joints, jointId );

			int otherEdgeIndex = edgeIndex ^ 1;

			b2Body* otherBody = b2BodyArray_Get( &world.bodies, joint.edges[otherEdgeIndex].bodyId );

			if ( bodySetIndex == b2_disabledSet || otherBody.setIndex == b2_disabledSet )
			{
				assert( joint.islandId == B2_NULL_INDEX );
			}
			else if ( bodySetIndex == b2_staticSet )
			{
				if ( otherBody.setIndex == b2_staticSet )
				{
					assert( joint.islandId == B2_NULL_INDEX );
				}
			}
			else
			{
				//int jointIslandId = b2GetRootIslandId( world, joint->islandId );
				int jointIslandId = joint.islandId;
				assert( jointIslandId == bodyIslandId );
			}

			jointKey = joint.edges[edgeIndex].nextKey;
		}
	}
}

// Validates solver sets, but not island connectivity
void b2ValidateSolverSets(b2World* world)
{
	assert( b2GetIdCapacity( &world.bodyIdPool ) == world.bodies.count );
	assert( b2GetIdCapacity( &world.contactIdPool ) == world.contacts.count );
	assert( b2GetIdCapacity( &world.jointIdPool ) == world.joints.count );
	assert( b2GetIdCapacity( &world.islandIdPool ) == world.islands.count );
	assert( b2GetIdCapacity( &world.solverSetIdPool ) == world.solverSets.count );

	int activeSetCount = 0;
	int totalBodyCount = 0;
	int totalJointCount = 0;
	int totalContactCount = 0;
	int totalIslandCount = 0;

	// Validate all solver sets
	int setCount = world.solverSets.count;
	for ( int setIndex = 0; setIndex < setCount; ++setIndex )
	{
		b2SolverSet* set = world.solverSets + setIndex;
		if ( set.setIndex != B2_NULL_INDEX )
		{
			activeSetCount += 1;

			if ( setIndex == b2_staticSet )
			{
				assert( set.contactSims.count == 0 );
				assert( set.islandSims.count == 0 );
				assert( set.bodyStates.count == 0 );
			}
			else if ( setIndex == b2_disabledSet )
			{
				assert( set.islandSims.count == 0 );
				assert( set.bodyStates.count == 0 );
			}
			else if ( setIndex == b2_awakeSet )
			{
				assert( set.bodySims.count == set.bodyStates.count );
				assert( set.jointSims.count == 0 );
			}
			else
			{
				assert( set.bodyStates.count == 0 );
			}

			// Validate bodies
			{
				b2Body* bodies = world.bodies;
				assert( set.bodySims.count >= 0 );
				totalBodyCount += set.bodySims.count;
				for ( int i = 0; i < set.bodySims.count; ++i )
				{
					b2BodySim* bodySim = set.bodySims + i;

					int bodyId = bodySim.bodyId;
					assert( 0 <= bodyId && bodyId < world.bodies.count );
					b2Body* body = bodies + bodyId;
					assert( body.setIndex == setIndex );
					assert( body.localIndex == i );
					assert( body.generation == body.generation );

					if ( setIndex == b2_disabledSet )
					{
						assert( body.headContactKey == B2_NULL_INDEX );
					}

					// Validate body shapes
					int prevShapeId = B2_NULL_INDEX;
					int shapeId = body.headShapeId;
					while ( shapeId != B2_NULL_INDEX )
					{
						b2Shape* shape = b2ShapeArray_Get( &world.shapes, shapeId );
						assert( shape.id == shapeId );
						assert( shape.prevShapeId == prevShapeId );

						if ( setIndex == b2_disabledSet )
						{
							assert( shape.proxyKey == B2_NULL_INDEX );
						}
						else if ( setIndex == b2_staticSet )
						{
							assert( B2_PROXY_TYPE( shape.proxyKey ) == b2_staticBody );
						}
						else
						{
							b2BodyType proxyType = B2_PROXY_TYPE( shape.proxyKey );
							assert( proxyType == b2_kinematicBody || proxyType == b2_dynamicBody );
						}

						prevShapeId = shapeId;
						shapeId = shape.nextShapeId;
					}

					// Validate body contacts
					int contactKey = body.headContactKey;
					while ( contactKey != B2_NULL_INDEX )
					{
						int contactId = contactKey >> 1;
						int edgeIndex = contactKey & 1;

						b2Contact* contact = b2ContactArray_Get( &world.contacts, contactId );
						assert( contact.setIndex != b2_staticSet );
						assert( contact.edges[0].bodyId == bodyId || contact.edges[1].bodyId == bodyId );
						contactKey = contact.edges[edgeIndex].nextKey;
					}

					// Validate body joints
					int jointKey = body.headJointKey;
					while ( jointKey != B2_NULL_INDEX )
					{
						int jointId = jointKey >> 1;
						int edgeIndex = jointKey & 1;

						b2Joint* joint = b2JointArray_Get( &world.joints, jointId );

						int otherEdgeIndex = edgeIndex ^ 1;

						b2Body* otherBody = b2BodyArray_Get( &world.bodies, joint.edges[otherEdgeIndex].bodyId );

						if ( setIndex == b2_disabledSet || otherBody.setIndex == b2_disabledSet )
						{
							assert( joint.setIndex == b2_disabledSet );
						}
						else if ( setIndex == b2_staticSet && otherBody.setIndex == b2_staticSet )
						{
							assert( joint.setIndex == b2_staticSet );
						}
						else if ( setIndex == b2_awakeSet )
						{
							assert( joint.setIndex == b2_awakeSet );
						}
						else if ( setIndex >= b2_firstSleepingSet )
						{
							assert( joint.setIndex == setIndex );
						}

						b2JointSim* jointSim = b2GetJointSim( world, joint );
						assert( jointSim.jointId == jointId );
						assert( jointSim.bodyIdA == joint.edges[0].bodyId );
						assert( jointSim.bodyIdB == joint.edges[1].bodyId );

						jointKey = joint.edges[edgeIndex].nextKey;
					}
				}
			}

			// Validate contacts
			{
				assert( set.contactSims.count >= 0 );
				totalContactCount += set.contactSims.count;
				for ( int i = 0; i < set.contactSims.count; ++i )
				{
					b2ContactSim* contactSim = set.contactSims + i;
					b2Contact* contact = b2ContactArray_Get( &world.contacts, contactSim.contactId );
					if ( setIndex == b2_awakeSet )
					{
						// contact should be non-touching if awake
						// or it could be this contact hasn't been transferred yet
						assert( contactSim.manifold.pointCount == 0 ||
								   ( contactSim.simFlags & b2_simStartedTouching ) != 0 );
					}
					assert( contact.setIndex == setIndex );
					assert( contact.colorIndex == B2_NULL_INDEX );
					assert( contact.localIndex == i );
				}
			}

			// Validate joints
			{
				assert( set.jointSims.count >= 0 );
				totalJointCount += set.jointSims.count;
				for ( int i = 0; i < set.jointSims.count; ++i )
				{
					b2JointSim* jointSim = set.jointSims + i;
					b2Joint* joint = b2JointArray_Get( &world.joints, jointSim.jointId );
					assert( joint.setIndex == setIndex );
					assert( joint.colorIndex == B2_NULL_INDEX );
					assert( joint.localIndex == i );
				}
			}

			// Validate islands
			{
				assert( set.islandSims.count >= 0 );
				totalIslandCount += set.islandSims.count;
				for ( int i = 0; i < set.islandSims.count; ++i )
				{
					b2IslandSim* islandSim = set.islandSims + i;
					b2Island* island = b2IslandArray_Get( &world.islands, islandSim.islandId );
					assert( island.setIndex == setIndex );
					assert( island.localIndex == i );
				}
			}
		}
		else
		{
			assert( set.bodySims.count == 0 );
			assert( set.contactSims.count == 0 );
			assert( set.jointSims.count == 0 );
			assert( set.islandSims.count == 0 );
			assert( set.bodyStates.count == 0 );
		}
	}

	int setIdCount = b2GetIdCount( &world.solverSetIdPool );
	assert( activeSetCount == setIdCount );

	int bodyIdCount = b2GetIdCount( &world.bodyIdPool );
	assert( totalBodyCount == bodyIdCount );

	int islandIdCount = b2GetIdCount( &world.islandIdPool );
	assert( totalIslandCount == islandIdCount );

	// Validate constraint graph
	for ( int colorIndex = 0; colorIndex < B2_GRAPH_COLOR_COUNT; ++colorIndex )
	{
		b2GraphColor* color = world.constraintGraph.colors + colorIndex;
		int bitCount = 0;

		assert( color.contactSims.count >= 0 );
		totalContactCount += color.contactSims.count;
		for ( int i = 0; i < color.contactSims.count; ++i )
		{
			b2ContactSim* contactSim = color.contactSims + i;
			b2Contact* contact = b2ContactArray_Get( &world.contacts, contactSim.contactId );
			// contact should be touching in the constraint graph or awaiting transfer to non-touching
			assert( contactSim.manifold.pointCount > 0 ||
					   ( contactSim.simFlags & ( b2_simStoppedTouching | b2_simDisjoint ) ) != 0 );
			assert( contact.setIndex == b2_awakeSet );
			assert( contact.colorIndex == colorIndex );
			assert( contact.localIndex == i );

			int bodyIdA = contact.edges[0].bodyId;
			int bodyIdB = contact.edges[1].bodyId;

			if ( colorIndex < B2_OVERFLOW_INDEX )
			{
				b2Body* bodyA = b2BodyArray_Get( &world.bodies, bodyIdA );
				b2Body* bodyB = b2BodyArray_Get( &world.bodies, bodyIdB );
				assert( b2GetBit( &color.bodySet, bodyIdA ) == ( bodyA.type != b2_staticBody ) );
				assert( b2GetBit( &color.bodySet, bodyIdB ) == ( bodyB.type != b2_staticBody ) );

				bitCount += bodyA.type == b2_staticBody ? 0 : 1;
				bitCount += bodyB.type == b2_staticBody ? 0 : 1;
			}
		}

		assert( color.jointSims.count >= 0 );
		totalJointCount += color.jointSims.count;
		for ( int i = 0; i < color.jointSims.count; ++i )
		{
			b2JointSim* jointSim = color.jointSims + i;
			b2Joint* joint = b2JointArray_Get( &world.joints, jointSim.jointId );
			assert( joint.setIndex == b2_awakeSet );
			assert( joint.colorIndex == colorIndex );
			assert( joint.localIndex == i );

			int bodyIdA = joint.edges[0].bodyId;
			int bodyIdB = joint.edges[1].bodyId;

			if ( colorIndex < B2_OVERFLOW_INDEX )
			{
				b2Body* bodyA = b2BodyArray_Get( &world.bodies, bodyIdA );
				b2Body* bodyB = b2BodyArray_Get( &world.bodies, bodyIdB );
				assert( b2GetBit( &color.bodySet, bodyIdA ) == ( bodyA.type != b2_staticBody ) );
				assert( b2GetBit( &color.bodySet, bodyIdB ) == ( bodyB.type != b2_staticBody ) );

				bitCount += bodyA.type == b2_staticBody ? 0 : 1;
				bitCount += bodyB.type == b2_staticBody ? 0 : 1;
			}
		}

		// Validate the bit population for this graph color
		assert( bitCount == b2CountSetBits( &color.bodySet ) );
	}

	int contactIdCount = b2GetIdCount( &world.contactIdPool );
	assert( totalContactCount == contactIdCount );
	assert( totalContactCount == cast(int)world.broadPhase.pairSet.count );

	int jointIdCount = b2GetIdCount( &world.jointIdPool );
	assert( totalJointCount == jointIdCount );

// Validate shapes
// This is very slow on compounds
version (none) {
	int shapeCapacity = b2Array(world.shapeArray).count;
	for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
	{
		b2Shape* shape = world.shapeArray + shapeIndex;
		if (shape.id != shapeIndex)
		{
			continue;
		}

		assert(0 <= shape.bodyId && shape.bodyId < b2Array(world.bodyArray).count);

		b2Body* body = world.bodyArray + shape.bodyId;
		assert(0 <= body.setIndex && body.setIndex < b2Array(world.solverSetArray).count);

		b2SolverSet* set = world.solverSetArray + body.setIndex;
		assert(0 <= body.localIndex && body.localIndex < set.sims.count);

		b2BodySim* bodySim = set.sims + body.localIndex;
		assert(bodySim.bodyId == shape.bodyId);

		bool found = false;
		int shapeCount = 0;
		int index = body.headShapeId;
		while (index != B2_NULL_INDEX)
		{
			b2CheckId(world.shapeArray, index);
			b2Shape* s = world.shapeArray + index;
			if (index == shapeIndex)
			{
				found = true;
			}

			index = s.nextShapeId;
			shapeCount += 1;
		}

		assert(found);
		assert(shapeCount == body.shapeCount);
	}
}
}

// Validate contact touching status.
	void b2ValidateContacts(b2World* world)
	{
		int contactCount = world.contacts.count;
		assert( contactCount == b2GetIdCapacity( &world.contactIdPool ) );
		int allocatedContactCount = 0;

		for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
		{
			b2Contact* contact = b2ContactArray_Get( &world.contacts, contactIndex );
			if ( contact.contactId == B2_NULL_INDEX )
			{
				continue;
			}

			assert( contact.contactId == contactIndex );

			allocatedContactCount += 1;

			bool touching = ( contact.flags & b2_contactTouchingFlag ) != 0;

			int setId = contact.setIndex;

			if ( setId == b2_awakeSet )
			{
				if ( touching )
				{
					assert( 0 <= contact.colorIndex && contact.colorIndex < B2_GRAPH_COLOR_COUNT );
				}
				else
				{
					assert( contact.colorIndex == B2_NULL_INDEX );
				}
			}
			else if ( setId >= b2_firstSleepingSet )
			{
				// Only touching contacts allowed in a sleeping set
				assert( touching == true );
			}
			else
			{
				// Sleeping and non-touching contacts belong in the disabled set
				assert( touching == false && setId == b2_disabledSet );
			}

			b2ContactSim* contactSim = b2GetContactSim( world, contact );
			assert( contactSim.contactId == contactIndex );
			assert( contactSim.bodyIdA == contact.edges[0].bodyId );
			assert( contactSim.bodyIdB == contact.edges[1].bodyId );

			bool simTouching = ( contactSim.simFlags & b2_simTouchingFlag ) != 0;
			assert( touching == simTouching );

			assert( 0 <= contactSim.manifold.pointCount && contactSim.manifold.pointCount <= 2 );
		}

		int contactIdCount = b2GetIdCount( &world.contactIdPool );
		assert( allocatedContactCount == contactIdCount );
	}

} else {

	void b2ValidateConnectivity(b2World* world)
	{
		// B2_UNUSED( world );
	}

	void b2ValidateSolverSets(b2World* world)
	{
		// B2_UNUSED( world );
	}

	void b2ValidateContacts(b2World* world)
	{
		// B2_UNUSED( world );
	}
}
