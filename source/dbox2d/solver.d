module dbox2d.solver;

public import dbox2d.core;
import dbox2d.constants;
import dbox2d.ctz;
public import dbox2d.math_functions;
import dbox2d.atomic;
public import core.stdc.stdlib;
public import core.stdc.stdint;
import dbox2d.island;
import dbox2d.constraint_graph;
import dbox2d.core;
import dbox2d.body;
import dbox2d.distance;
import dbox2d.joint;
import dbox2d.contact;
import dbox2d.contact_solver;
import dbox2d.physics_world;
import dbox2d.sensor;
import dbox2d.shape;
import dbox2d.dynamic_tree;
import dbox2d.solver_set;
import dbox2d.timer;

mixin(B2_ARRAY_SOURCE!("b2Body", "b2Body"));
mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2Sensor","b2Sensor"));
mixin(B2_ARRAY_SOURCE!("b2Shape", "b2Shape"));
mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
mixin(B2_ARRAY_SOURCE!("b2ContactHitEvent","b2ContactHitEvent"));
mixin(B2_ARRAY_SOURCE!("b2Visitor","b2Visitor"));
mixin(B2_ARRAY_SOURCE!("b2TaskContext","b2TaskContext"));
mixin(B2_ARRAY_SOURCE!("b2JointEvent","b2JointEvent"));
mixin(B2_ARRAY_SOURCE!("b2SensorHit","b2SensorHit"));
mixin(B2_ARRAY_SOURCE!("b2BodyMoveEvent","b2BodyMoveEvent"));
mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));

struct b2Softness {
	float biasRate = 0;
	float massScale = 0;
	float impulseScale = 0;
}

enum b2SolverStageType {
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageIntegrateVelocities,
	b2_stageWarmStart,
	b2_stageSolve,
	b2_stageIntegratePositions,
	b2_stageRelax,
	b2_stageRestitution,
	b2_stageStoreImpulses
}
alias b2_stagePrepareJoints = b2SolverStageType.b2_stagePrepareJoints;
alias b2_stagePrepareContacts = b2SolverStageType.b2_stagePrepareContacts;
alias b2_stageIntegrateVelocities = b2SolverStageType.b2_stageIntegrateVelocities;
alias b2_stageWarmStart = b2SolverStageType.b2_stageWarmStart;
alias b2_stageSolve = b2SolverStageType.b2_stageSolve;
alias b2_stageIntegratePositions = b2SolverStageType.b2_stageIntegratePositions;
alias b2_stageRelax = b2SolverStageType.b2_stageRelax;
alias b2_stageRestitution = b2SolverStageType.b2_stageRestitution;
alias b2_stageStoreImpulses = b2SolverStageType.b2_stageStoreImpulses;

enum b2SolverBlockType : short {
	b2_bodyBlock,
	b2_jointBlock,
	b2_contactBlock,
	b2_graphJointBlock,
	b2_graphContactBlock
}

alias b2_bodyBlock = b2SolverBlockType.b2_bodyBlock;
alias b2_jointBlock = b2SolverBlockType.b2_jointBlock;
alias b2_contactBlock = b2SolverBlockType.b2_contactBlock;
alias b2_graphJointBlock = b2SolverBlockType.b2_graphJointBlock;
alias b2_graphContactBlock = b2SolverBlockType.b2_graphContactBlock;

// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker
// claims a block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention
// on a single block index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver
// iteration) the same block of work is executed once per iteration and the atomic sync index is shared across iterations, so it
// increases monotonically.
struct b2SolverBlock {
	int startIndex;
	short count;
	short blockType; // b2SolverBlockType
	// todo consider false sharing of this atomic
	b2AtomicInt syncIndex;
}

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
struct b2SolverStage {
	b2SolverStageType type;
	b2SolverBlock* blocks;
	int blockCount;
	int colorIndex;
	// todo consider false sharing of this atomic
	b2AtomicInt completionCount;
}

// Context for a time step. Recreated each time step.
struct b2StepContext {
	// time step
	float dt = 0;

	// inverse time step (0 if dt == 0).
	float inv_dt = 0;

	// sub-step
	float h = 0;
	float inv_h = 0;

	int subStepCount;

	b2Softness contactSoftness;
	b2Softness staticSoftness;

	float restitutionThreshold = 0;
	float maxLinearVelocity = 0;

	b2World* world;
	b2ConstraintGraph* graph;

	// shortcut to body states from awake set
	b2BodyState* states;

	// shortcut to body sims from awake set
	b2BodySim* sims;

	// array of all shape ids for shapes that have enlarged AABBs
	int* enlargedShapes;
	int enlargedShapeCount;

	// Array of bullet bodies that need continuous collision handling
	int* bulletBodies;
	b2AtomicInt bulletBodyCount;

	// joint pointers for simplified parallel-for access.
	b2JointSim** joints;

	// contact pointers for simplified parallel-for access.
	// - parallel-for collide with no gaps
	// - parallel-for prepare and store contacts with NULL gaps for SIMD remainders
	// despite being an array of pointers, these are contiguous sub-arrays corresponding
	// to constraint graph colors
	b2ContactSim** contacts;

	b2ContactConstraintSIMD* simdContactConstraints;
	int activeColorCount;
	int workerCount;

	b2SolverStage* stages;
	int stageCount;
	bool enableWarmStarting;

	// todo padding to prevent false sharing
	char[64] dummy1 = 0;

	// sync index (16-bits) | stage type (16-bits)
	b2AtomicU32 atomicSyncBits;

	char[64] dummy2 = 0;
}

pragma(inline, true) b2Softness b2MakeSoft(float hertz, float zeta, float h)
{
	if ( hertz == 0.0f )
	{
		return b2Softness(
			biasRate: 0.0f,
			massScale: 0.0f,
			impulseScale: 0.0f,
		);
	}

	float omega = 2.0f * PI * hertz;
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / ( 1.0f + a2 );

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	return b2Softness(
		biasRate: omega / a1,
		massScale: a2 * a3,
		impulseScale: a3,
	);
}

void b2Solve(b2World* world, b2StepContext* stepContext);

enum ITERATIONS = 1;
enum RELAX_ITERATIONS = 1;

pragma(inline, true) void b2Pause()
	{
		asm {
			rep;
			nop;
		}
	}

// extern(C):
// Compare to SDL_CPUPauseInstruction
// static if (( HasVersion!"__GNUC__" || HasVersion!"__clang__" ) && ( HasVersion!"__i386__" || HasVersion!"__x86_64__" )) {
// 	pragma(inline, true) void b2Pause()
// 	{
// 		asm {
// 			rep;
// 			nop;
// 		}
// 	// __asm__ __volatile__( "pause\n" );
// 	}
// }
// } else static if (( HasVersion!"__arm__" && HasVersion!"__ARM_ARCH" && __ARM_ARCH >= 7 ) || HasVersion!"__aarch64__") {
// pragma(inline, true) private void b2Pause()
// {
// 	__asm__ __volatile__( "yield" ::: "memory" );
// }
// } else static if (HasVersion!"_MSC_VER" && ( HasVersion!"_M_IX86" || HasVersion!"_M_X64" )) {
// pragma(inline, true) private void b2Pause()
// {
// 	_mm_pause();
// }
// } else static if (HasVersion!"_MSC_VER" && ( HasVersion!"_M_ARM" || HasVersion!"_M_ARM64" )) {
// pragma(inline, true) private void b2Pause()
// {
// 	__yield();
// }
// } else {
// pragma(inline, true) private void b2Pause()
// {
// }
// }

struct b2WorkerContext {
	b2StepContext* context;
	int workerIndex;
	void* userTask;
}

// Integrate velocities and apply damping
private void b2IntegrateVelocitiesTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2BodyState* states = context.states;
	b2BodySim* sims = context.sims;

	b2Vec2 gravity = context.world.gravity;
	float h = context.h;
	float maxLinearSpeed = context.maxLinearVelocity;
	float maxAngularSpeed = B2_MAX_ROTATION * context.inv_dt;
	float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2BodySim* sim = sims + i;
		b2BodyState* state = states + i;

		b2Vec2 v = state.linearVelocity;
		float w = state.angularVelocity;

		// Apply forces, torque, gravity, and damping
		// Apply damping.
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		float linearDamping = 1.0f / ( 1.0f + h * sim.linearDamping );
		float angularDamping = 1.0f / ( 1.0f + h * sim.angularDamping );

		// Gravity scale will be zero for kinematic bodies
		float gravityScale = sim.invMass > 0.0f ? sim.gravityScale : 0.0f;

		// lvd = h * im * f + h * g
		b2Vec2 linearVelocityDelta = b2Add( b2MulSV( h * sim.invMass, sim.force ), b2MulSV( h * gravityScale, gravity ) );
		float angularVelocityDelta = h * sim.invInertia * sim.torque;

		v = b2MulAdd( linearVelocityDelta, linearDamping, v );
		w = angularVelocityDelta + angularDamping * w;

		// Clamp to max linear speed
		if ( b2Dot( v, v ) > maxLinearSpeedSquared )
		{
			float ratio = maxLinearSpeed / b2Length( v );
			v = b2MulSV( ratio, v );
			sim.flags |= b2_isSpeedCapped;
		}

		// Clamp to max angular speed
		if ( w * w > maxAngularSpeedSquared && ( sim.flags & b2_allowFastRotation ) == 0 )
		{
			float ratio = maxAngularSpeed / b2AbsFloat( w );
			w *= ratio;
			sim.flags |= b2_isSpeedCapped;
		}

		if ( state.flags & b2_lockLinearX )
		{
			v.x = 0.0f;
		}

		if ( state.flags & b2_lockLinearY )
		{
			v.y = 0.0f;
		}

		if ( state.flags & b2_lockAngularZ )
		{
			w = 0.0f;
		}

		state.linearVelocity = v;
		state.angularVelocity = w;
	}
}

private void b2PrepareJointsTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2JointSim** joints = context.joints;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2JointSim* joint = joints[i];
		b2PrepareJoint( joint, context );
	}
}

private void b2WarmStartJointsTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex)
{
	b2GraphColor* color = context.graph.colors.ptr + colorIndex;
	b2JointSim* joints = color.jointSims.ptr;
	B2_ASSERT( 0 <= startIndex && startIndex < color.jointSims.count );
	B2_ASSERT( startIndex <= endIndex && endIndex <= color.jointSims.count );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2JointSim* joint = joints + i;
		b2WarmStartJoint( joint, context );
	}
}

private void b2SolveJointsTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias, int workerIndex)
{
	b2GraphColor* color = context.graph.colors.ptr + colorIndex;
	b2JointSim* joints = color.jointSims.ptr;
	B2_ASSERT( 0 <= startIndex && startIndex < color.jointSims.count );
	B2_ASSERT( startIndex <= endIndex && endIndex <= color.jointSims.count );

	b2BitSet* jointStateBitSet = &context.world.taskContexts[workerIndex].jointStateBitSet;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2JointSim* joint = joints + i;
		b2SolveJoint( joint, context, useBias );

		if ( useBias && ( joint.forceThreshold < float.max || joint.torqueThreshold < float.max ) &&
			 b2GetBit( jointStateBitSet, joint.jointId ) == false )
		{
			float force = void, torque = void;
			b2GetJointReaction( joint, context.inv_h, &force, &torque );

			// Check thresholds. A zero threshold means all awake joints get reported.
			if ( force >= joint.forceThreshold || torque >= joint.torqueThreshold )
			{
				// Flag this joint for processing.
				b2SetBit( jointStateBitSet, joint.jointId );
			}
		}
	}
}

private void b2IntegratePositionsTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2BodyState* states = context.states;
	float h = context.h;

	B2_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2BodyState* state = states + i;

		if ( state.flags & b2_lockLinearX )
		{
			state.linearVelocity.x = 0.0f;
		}

		if ( state.flags & b2_lockLinearY )
		{
			state.linearVelocity.y = 0.0f;
		}

		if ( state.flags & b2_lockAngularZ )
		{
			state.angularVelocity = 0.0f;
		}

		state.deltaPosition = b2MulAdd( state.deltaPosition, h, state.linearVelocity );
		state.deltaRotation = b2IntegrateRotation( state.deltaRotation, h * state.angularVelocity );
	}
}

enum B2_MAX_CONTINUOUS_SENSOR_HITS = 8;

struct b2ContinuousContext
{
	b2World* world;
	b2BodySim* fastBodySim;
	b2Shape* fastShape;
	b2Vec2 centroid1, centroid2;
	b2Sweep sweep;
	float fraction = 0;
	b2SensorHit[B2_MAX_CONTINUOUS_SENSOR_HITS] sensorHits;
	float[B2_MAX_CONTINUOUS_SENSOR_HITS] sensorFractions = 0;
	int sensorCount;
}

enum B2_CORE_FRACTION = 0.25f;

// This is called from b2DynamicTree_Query for continuous collision
private bool b2ContinuousQueryCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	b2ContinuousContext* continuousContext = cast(b2ContinuousContext*) context;
	b2Shape* fastShape = continuousContext.fastShape;
	b2BodySim* fastBodySim = continuousContext.fastBodySim;

	B2_ASSERT( fastShape.sensorIndex == B2_NULL_INDEX );

	// Skip same shape
	if ( shapeId == fastShape.id )
	{
		return true;
	}

	b2World* world = continuousContext.world;

	b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

	// Skip same body
	if ( shape.bodyId == fastShape.bodyId )
	{
		return true;
	}

	bool isSensor = shape.sensorIndex != B2_NULL_INDEX;

	// Skip sensors unless the shapes want sensor events
	if ( isSensor && ( shape.enableSensorEvents == false || fastShape.enableSensorEvents == false ) )
	{
		return true;
	}

	// Skip filtered shapes
	bool canCollide = b2ShouldShapesCollide( fastShape.filter, shape.filter );
	if ( canCollide == false )
	{
		return true;
	}

	b2Body* body = b2BodyArray_Get( world.bodies, shape.bodyId );

	b2BodySim* bodySim = b2GetBodySim( world, body );
	B2_ASSERT( body.type == b2_staticBody || ( fastBodySim.flags & b2_isBullet ) );

	// Skip bullets
	if ( bodySim.flags & b2_isBullet )
	{
		return true;
	}

	// Skip filtered bodies
	b2Body* fastBody = b2BodyArray_Get( world.bodies, fastBodySim.bodyId );
	canCollide = b2ShouldBodiesCollide( world, fastBody, body );
	if ( canCollide == false )
	{
		return true;
	}

	// Custom user filtering
	if ( shape.enableCustomFiltering || fastShape.enableCustomFiltering )
	{
		b2CustomFilterFcn customFilterFcn = world.customFilterFcn;
		if ( customFilterFcn != null )
		{
			b2ShapeId idA = { shape.id + 1, world.worldId, shape.generation };
			b2ShapeId idB = { fastShape.id + 1, world.worldId, fastShape.generation };
			canCollide = customFilterFcn( idA, idB, world.customFilterContext );
			if ( canCollide == false )
			{
				return true;
			}
		}
	}

	// Early out on fast parallel movement over a chain shape.
	if ( shape.type == b2_chainSegmentShape )
	{
		b2Transform transform = bodySim.transform;
		b2Vec2 p1 = b2TransformPoint( transform, shape.chainSegment.segment.point1 );
		b2Vec2 p2 = b2TransformPoint( transform, shape.chainSegment.segment.point2 );
		b2Vec2 e = b2Sub( p2, p1 );
		float length = void;
		e = b2GetLengthAndNormalize( &length, e );
		if ( length > B2_LINEAR_SLOP )
		{
			b2Vec2 c1 = continuousContext.centroid1;
			float separation1 = b2Cross( b2Sub( c1, p1 ), e );
			b2Vec2 c2 = continuousContext.centroid2;
			float separation2 = b2Cross( b2Sub( c2, p1 ), e );

			float coreDistance = B2_CORE_FRACTION * fastBodySim.minExtent;
			
			if ( separation1 < 0.0f ||
				 (separation1 - separation2 < coreDistance && separation2 > coreDistance) )
			{
				// Minimal clipping
				return true;
			}
		}
	}

	// todo_erin testing early out for segments
version (none) {
	if ( shape.type == b2_segmentShape )
	{
		b2Transform transform = bodySim.transform;
		b2Vec2 p1 = b2TransformPoint( transform, shape.segment.point1 );
		b2Vec2 p2 = b2TransformPoint( transform, shape.segment.point2 );
		b2Vec2 e = b2Sub( p2, p1 );
		b2Vec2 c1 = continuousContext.centroid1;
		b2Vec2 c2 = continuousContext.centroid2;
		float offset1 = b2Cross( b2Sub( c1, p1 ), e );
		float offset2 = b2Cross( b2Sub( c2, p1 ), e );

		if ( offset1 > 0.0f && offset2 > 0.0f )
		{
			// Started behind or finished in front
			return true;
		}

		if ( offset1 < 0.0f && offset2 < 0.0f )
		{
			// Started behind or finished in front
			return true;
		}
	}
}

	b2TOIInput input = void;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeShapeDistanceProxy( fastShape );
	input.sweepA = b2MakeSweep( bodySim );
	input.sweepB = continuousContext.sweep;
	input.maxFraction = continuousContext.fraction;

	b2TOIOutput output = b2TimeOfImpact( &input );
	if ( isSensor )
	{
		// Only accept a sensor hit that is sooner than the current solid hit.
		if ( output.fraction <= continuousContext.fraction && continuousContext.sensorCount < B2_MAX_CONTINUOUS_SENSOR_HITS )
		{
			int index = continuousContext.sensorCount;

			// The hit shape is a sensor
			b2SensorHit sensorHit = {
				sensorId: shape.id,
				visitorId: fastShape.id,
			};

			continuousContext.sensorHits[index] = sensorHit;
			continuousContext.sensorFractions[index] = output.fraction;
			continuousContext.sensorCount += 1;
		}
	}
	else
	{
		float hitFraction = continuousContext.fraction;
		bool didHit = false;

		if ( 0.0f < output.fraction && output.fraction < continuousContext.fraction )
		{
			hitFraction = output.fraction;
			didHit = true;
		}
		else if ( 0.0f == output.fraction )
		{
			// fallback to TOI of a small circle around the fast shape centroid
			b2Vec2 centroid = b2GetShapeCentroid( fastShape );
			b2ShapeExtent extent = b2ComputeShapeExtent( fastShape, centroid );
			float radius = B2_CORE_FRACTION * extent.minExtent;
			input.proxyB = b2MakeProxy( &centroid, 1, radius );
			output = b2TimeOfImpact( &input );
			if ( 0.0f < output.fraction && output.fraction < continuousContext.fraction )
			{
				hitFraction = output.fraction;
				didHit = true;
			}
		}

		if ( didHit && ( shape.enablePreSolveEvents || fastShape.enablePreSolveEvents ) && world.preSolveFcn != null )
		{
			b2ShapeId shapeIdA = { shape.id + 1, world.worldId, shape.generation };
			b2ShapeId shapeIdB = { fastShape.id + 1, world.worldId, fastShape.generation };
			didHit = world.preSolveFcn( shapeIdA, shapeIdB, output.point, output.normal, world.preSolveContext );
		}

		if ( didHit )
		{
			fastBodySim.flags |= b2_hadTimeOfImpact;
			continuousContext.fraction = hitFraction;
		}
	}

	// Continue query
	return true;
}

private void b2SolveContinuous(b2World* world, int bodySimIndex, b2TaskContext* taskContext)
{
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	b2BodySim* fastBodySim = b2BodySimArray_Get( awakeSet.bodySims, bodySimIndex );
	B2_ASSERT( fastBodySim.flags & b2_isFast );

	b2Sweep sweep = b2MakeSweep( fastBodySim );

	b2Transform xf1 = void;
	xf1.q = sweep.q1;
	xf1.p = b2Sub( sweep.c1, b2RotateVector( sweep.q1, sweep.localCenter ) );

	b2Transform xf2 = void;
	xf2.q = sweep.q2;
	xf2.p = b2Sub( sweep.c2, b2RotateVector( sweep.q2, sweep.localCenter ) );

	b2DynamicTree* staticTree = world.broadPhase.trees.ptr + b2_staticBody;
	b2DynamicTree* kinematicTree = world.broadPhase.trees.ptr + b2_kinematicBody;
	b2DynamicTree* dynamicTree = world.broadPhase.trees.ptr + b2_dynamicBody;
	b2Body* fastBody = b2BodyArray_Get( world.bodies, fastBodySim.bodyId );

	b2ContinuousContext context;
	context.world = world;
	context.sweep = sweep;
	context.fastBodySim = fastBodySim;
	context.fraction = 1.0f;

	bool isBullet = ( fastBodySim.flags & b2_isBullet ) != 0;

	int shapeId = fastBody.headShapeId;
	while ( shapeId != B2_NULL_INDEX )
	{
		b2Shape* fastShape = b2ShapeArray_Get( world.shapes, shapeId );
		shapeId = fastShape.nextShapeId;

		context.fastShape = fastShape;
		context.centroid1 = b2TransformPoint( xf1, fastShape.localCentroid );
		context.centroid2 = b2TransformPoint( xf2, fastShape.localCentroid );

		b2AABB box1 = fastShape.aabb;
		b2AABB box2 = b2ComputeShapeAABB( fastShape, xf2 );

		// Store this to avoid double computation in the case there is no impact event
		fastShape.aabb = box2;

		// No continuous collision for sensors (but still need the updated bounds)
		if ( fastShape.sensorIndex != B2_NULL_INDEX )
		{
			continue;
		}

		b2AABB sweptBox = b2AABB_Union( box1, box2 );

		b2DynamicTree_Query( staticTree, sweptBox, B2_DEFAULT_MASK_BITS, &b2ContinuousQueryCallback, &context );

		if ( isBullet )
		{
			b2DynamicTree_Query( kinematicTree, sweptBox, B2_DEFAULT_MASK_BITS, &b2ContinuousQueryCallback, &context );
			b2DynamicTree_Query( dynamicTree, sweptBox, B2_DEFAULT_MASK_BITS, &b2ContinuousQueryCallback, &context );
		}
	}

	const(float) speculativeDistance = B2_SPECULATIVE_DISTANCE;
	const(float) aabbMargin = B2_AABB_MARGIN;

	if ( context.fraction < 1.0f )
	{
		// Handle time of impact event
		b2Rot q = b2NLerp( sweep.q1, sweep.q2, context.fraction );
		b2Vec2 c = b2Lerp( sweep.c1, sweep.c2, context.fraction );
		b2Vec2 origin = b2Sub( c, b2RotateVector( q, sweep.localCenter ) );

		// Advance body
		b2Transform transform = { origin, q };
		fastBodySim.transform = transform;
		fastBodySim.center = c;
		fastBodySim.rotation0 = q;
		fastBodySim.center0 = c;

		// Update body move event
		b2BodyMoveEvent* event = b2BodyMoveEventArray_Get( world.bodyMoveEvents, bodySimIndex );
		event.transform = transform;

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = fastBody.headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

			// Must recompute aabb at the interpolated transform
			b2AABB aabb = b2ComputeShapeAABB( shape, transform );
			aabb.lowerBound.x -= speculativeDistance;
			aabb.lowerBound.y -= speculativeDistance;
			aabb.upperBound.x += speculativeDistance;
			aabb.upperBound.y += speculativeDistance;
			shape.aabb = aabb;

			if ( b2AABB_Contains( shape.fatAABB, aabb ) == false )
			{
				b2AABB fatAABB = void;
				fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
				fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
				fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
				fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
				shape.fatAABB = fatAABB;

				shape.enlargedAABB = true;
				fastBodySim.flags |= b2_enlargeBounds;
			}

			shapeId = shape.nextShapeId;
		}
	}
	else
	{
		// No time of impact event

		// Advance body
		fastBodySim.rotation0 = fastBodySim.transform.q;
		fastBodySim.center0 = fastBodySim.center;

		// Prepare AABBs for broad-phase
		shapeId = fastBody.headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get( world.shapes, shapeId );

			// shape->aabb is still valid from above

			if ( b2AABB_Contains( shape.fatAABB, shape.aabb ) == false )
			{
				b2AABB fatAABB = void;
				fatAABB.lowerBound.x = shape.aabb.lowerBound.x - aabbMargin;
				fatAABB.lowerBound.y = shape.aabb.lowerBound.y - aabbMargin;
				fatAABB.upperBound.x = shape.aabb.upperBound.x + aabbMargin;
				fatAABB.upperBound.y = shape.aabb.upperBound.y + aabbMargin;
				shape.fatAABB = fatAABB;

				shape.enlargedAABB = true;
				fastBodySim.flags |= b2_enlargeBounds;
			}

			shapeId = shape.nextShapeId;
		}
	}

	// Push sensor hits on the the task context for serial processing.
	for ( int i = 0; i < context.sensorCount; ++i )
	{
		// Skip any sensor hits that occurred after a solid hit
		if ( context.sensorFractions[i] < context.fraction )
		{
			b2SensorHitArray_Push( taskContext.sensorHits, context.sensorHits[i] );
		}
	}
}

private void b2FinalizeBodiesTask(int startIndex, int endIndex, uint threadIndex, void* context)
{
	b2StepContext* stepContext = cast(b2StepContext*)context;
	b2World* world = stepContext.world;
	bool enableSleep = world.enableSleep;
	b2BodyState* states = stepContext.states;
	b2BodySim* sims = stepContext.sims;
	b2Body* bodies = world.bodies.ptr;
	float timeStep = stepContext.dt;
	float invTimeStep = stepContext.inv_dt;

	ushort worldId = world.worldId;

	// The body move event array should already have the correct size
	B2_ASSERT( endIndex <= world.bodyMoveEvents.count );
	b2BodyMoveEvent* moveEvents = world.bodyMoveEvents.ptr;

	b2BitSet* enlargedSimBitSet = &world.taskContexts[threadIndex].enlargedSimBitSet;
	b2BitSet* awakeIslandBitSet = &world.taskContexts[threadIndex].awakeIslandBitSet;
	b2TaskContext* taskContext = world.taskContexts.ptr + threadIndex;

	bool enableContinuous = world.enableContinuous;

	const(float) speculativeDistance = B2_SPECULATIVE_DISTANCE;
	const(float) aabbMargin = B2_AABB_MARGIN;

	B2_ASSERT( startIndex <= endIndex );

	for ( int simIndex = startIndex; simIndex < endIndex; ++simIndex )
	{
		b2BodyState* state = states + simIndex;
		b2BodySim* sim = sims + simIndex;

		if ( state.flags & b2_lockLinearX )
		{
			state.linearVelocity.x = 0.0f;
		}

		if ( state.flags & b2_lockLinearY )
		{
			state.linearVelocity.y = 0.0f;
		}

		if ( state.flags & b2_lockAngularZ )
		{
			state.angularVelocity = 0.0f;
		}

		b2Vec2 v = state.linearVelocity;
		float w = state.angularVelocity;

		B2_ASSERT( b2IsValidVec2( v ) );
		B2_ASSERT( b2IsValidFloat( w ) );

		sim.center = b2Add( sim.center, state.deltaPosition );
		sim.transform.q = b2NormalizeRot( b2MulRot( state.deltaRotation, sim.transform.q ) );

		// Use the velocity of the farthest point on the body to account for rotation.
		float maxVelocity = b2Length( v ) + b2AbsFloat( w ) * sim.maxExtent;

		// Sleep needs to observe position correction as well as true velocity.
		float maxDeltaPosition = b2Length( state.deltaPosition ) + b2AbsFloat( state.deltaRotation.s ) * sim.maxExtent;

		// Position correction is not as important for sleep as true velocity.
		float positionSleepFactor = 0.5f;

		float sleepVelocity = b2MaxFloat( maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition );

		// reset state deltas
		state.deltaPosition = b2Vec2.zero();
		state.deltaRotation = b2Rot_identity;

		sim.transform.p = b2Sub( sim.center, b2RotateVector( sim.transform.q, sim.localCenter ) );

		// cache miss here, however I need the shape list below
		b2Body* body = bodies + sim.bodyId;
		body.bodyMoveIndex = simIndex;
		moveEvents[simIndex].transform = sim.transform;
		moveEvents[simIndex].bodyId = b2BodyId( sim.bodyId + 1, worldId, body.generation );
		moveEvents[simIndex].userData = body.userData;
		moveEvents[simIndex].fellAsleep = false;

		// reset applied force and torque
		sim.force = b2Vec2.zero();
		sim.torque = 0.0f;

		body.flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );
		body.flags |= (sim.flags & (b2_isSpeedCapped | b2_hadTimeOfImpact));
		sim.flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );

		if ( enableSleep == false || body.enableSleep == false || sleepVelocity > body.sleepThreshold )
		{
			// Body is not sleepy
			body.sleepTime = 0.0f;

			if ( body.type == b2_dynamicBody && enableContinuous && maxVelocity * timeStep > 0.5f * sim.minExtent )
			{
				// This flag is only retained for debug draw
				sim.flags |= b2_isFast;

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if ( sim.flags & b2_isBullet )
				{
					int bulletIndex = b2AtomicFetchAddInt( &stepContext.bulletBodyCount, 1 );
					stepContext.bulletBodies[bulletIndex] = simIndex;
				}
				else
				{
					b2SolveContinuous( world, simIndex, taskContext );
				}
			}
			else
			{
				// Body is safe to advance
				sim.center0 = sim.center;
				sim.rotation0 = sim.transform.q;
			}
		}
		else
		{
			// Body is safe to advance and is falling asleep
			sim.center0 = sim.center;
			sim.rotation0 = sim.transform.q;
			body.sleepTime += timeStep;
		}

		// Any single body in an island can keep it awake
		b2Island* island = b2IslandArray_Get( world.islands, body.islandId );
		if ( body.sleepTime < B2_TIME_TO_SLEEP )
		{
			// keep island awake
			int islandIndex = island.localIndex;
			b2SetBit( awakeIslandBitSet, islandIndex );
		}
		else if ( island.constraintRemoveCount > 0 )
		{
			// body wants to sleep but its island needs splitting first
			if ( body.sleepTime > taskContext.splitSleepTime )
			{
				// pick the sleepiest candidate
				taskContext.splitIslandId = body.islandId;
				taskContext.splitSleepTime = body.sleepTime;
			}
		}

		// Update shapes AABBs
		b2Transform transform = sim.transform;
		bool isFast = ( sim.flags & b2_isFast ) != 0;
		int shapeId = body.headShapeId;
		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = b2ShapeArray_Get(world.shapes, shapeId );

			if ( isFast )
			{
				// For fast non-bullet bodies the AABB has already been updated in b2SolveContinuous
				// For fast bullet bodies the AABB will be updated at a later stage

				// Add to enlarged shapes regardless of AABB changes.
				// Bit-set to keep the move array sorted
				b2SetBit( enlargedSimBitSet, simIndex );
			}
			else
			{
				b2AABB aabb = b2ComputeShapeAABB( shape, transform );
				aabb.lowerBound.x -= speculativeDistance;
				aabb.lowerBound.y -= speculativeDistance;
				aabb.upperBound.x += speculativeDistance;
				aabb.upperBound.y += speculativeDistance;
				shape.aabb = aabb;

				B2_ASSERT( shape.enlargedAABB == false );

				if ( b2AABB_Contains( shape.fatAABB, aabb ) == false )
				{
					b2AABB fatAABB = void;
					fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
					fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
					fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
					fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
					shape.fatAABB = fatAABB;

					shape.enlargedAABB = true;

					// Bit-set to keep the move array sorted
					b2SetBit( enlargedSimBitSet, simIndex );
				}
			}

			shapeId = shape.nextShapeId;
		}
	}
}

private void b2ExecuteBlock(b2SolverStage* stage, b2StepContext* context, b2SolverBlock* block, int workerIndex)
{
	b2SolverStageType stageType = stage.type;
	b2SolverBlockType blockType = cast(b2SolverBlockType)block.blockType;
	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	switch ( stageType )
	{
		case b2_stagePrepareJoints:
			b2PrepareJointsTask( startIndex, endIndex, context );
			break;

		case b2_stagePrepareContacts:
			b2PrepareContactsTask( startIndex, endIndex, context );
			break;

		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask( startIndex, endIndex, context );
			break;

		case b2_stageWarmStart:
			if ( blockType == b2_graphContactBlock )
			{
				b2WarmStartContactsTask( startIndex, endIndex, context, stage.colorIndex );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				b2WarmStartJointsTask( startIndex, endIndex, context, stage.colorIndex );
			}
			break;

		case b2_stageSolve:
			if ( blockType == b2_graphContactBlock )
			{
				b2SolveContactsTask( startIndex, endIndex, context, stage.colorIndex, true );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				b2SolveJointsTask( startIndex, endIndex, context, stage.colorIndex, true, workerIndex );
			}
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask( startIndex, endIndex, context );
			break;

		case b2_stageRelax:
			if ( blockType == b2_graphContactBlock )
			{
				b2SolveContactsTask( startIndex, endIndex, context, stage.colorIndex, false );
			}
			else if ( blockType == b2_graphJointBlock )
			{
				b2SolveJointsTask( startIndex, endIndex, context, stage.colorIndex, false, workerIndex );
			}
			break;

		case b2_stageRestitution:
			if ( blockType == b2_graphContactBlock )
			{
				b2ApplyRestitutionTask( startIndex, endIndex, context, stage.colorIndex );
			}
			break;

		case b2_stageStoreImpulses:
			b2StoreImpulsesTask( startIndex, endIndex, context );
			break;
	default: break;}
}

pragma(inline, true) private int GetWorkerStartIndex(int workerIndex, int blockCount, int workerCount)
{
	if ( blockCount <= workerCount )
	{
		return workerIndex < blockCount ? workerIndex : B2_NULL_INDEX;
	}

	int blocksPerWorker = blockCount / workerCount;
	int remainder = blockCount - blocksPerWorker * workerCount;
	return blocksPerWorker * workerIndex + b2MinInt( remainder, workerIndex );
}

private void b2ExecuteStage(b2SolverStage* stage, b2StepContext* context, int previousSyncIndex, int syncIndex, int workerIndex)
{
	int completedCount = 0;
	b2SolverBlock* blocks = stage.blocks;
	int blockCount = stage.blockCount;

	int expectedSyncIndex = previousSyncIndex;

	int startIndex = GetWorkerStartIndex( workerIndex, blockCount, context.workerCount );
	if ( startIndex == B2_NULL_INDEX )
	{
		return;
	}

	B2_ASSERT( 0 <= startIndex && startIndex < blockCount );

	int blockIndex = startIndex;

	while ( b2AtomicCompareExchangeInt( &blocks[blockIndex].syncIndex, expectedSyncIndex, syncIndex ) == true )
	{
		B2_ASSERT( stage.type != b2_stagePrepareContacts || syncIndex < 2 );

		B2_ASSERT( completedCount < blockCount );

		b2ExecuteBlock( stage, context, blocks + blockIndex, workerIndex );

		completedCount += 1;
		blockIndex += 1;
		if ( blockIndex >= blockCount )
		{
			// Keep looking for work
			blockIndex = 0;
		}

		expectedSyncIndex = previousSyncIndex;
	}

	// Search backwards for blocks
	blockIndex = startIndex - 1;
	while ( true )
	{
		if ( blockIndex < 0 )
		{
			blockIndex = blockCount - 1;
		}

		expectedSyncIndex = previousSyncIndex;

		if ( b2AtomicCompareExchangeInt( &blocks[blockIndex].syncIndex, expectedSyncIndex, syncIndex ) == false )
		{
			break;
		}

		b2ExecuteBlock( stage, context, blocks + blockIndex, workerIndex );
		completedCount += 1;
		blockIndex -= 1;
	}

	cast(void)b2AtomicFetchAddInt( &stage.completionCount, completedCount );
}

private void b2ExecuteMainStage(b2SolverStage* stage, b2StepContext* context, uint syncBits)
{
	int blockCount = stage.blockCount;
	if ( blockCount == 0 )
	{
		return;
	}

	int workerIndex = 0;

	if ( blockCount == 1 )
	{
		b2ExecuteBlock( stage, context, stage.blocks, workerIndex );
	}
	else
	{
		b2AtomicStoreU32( &context.atomicSyncBits, syncBits );

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B2_ASSERT( syncIndex > 0 );
		int previousSyncIndex = syncIndex - 1;

		b2ExecuteStage( stage, context, previousSyncIndex, syncIndex, workerIndex );

		// todo consider using the cycle counter as well
		while ( b2AtomicLoadInt( &stage.completionCount ) != blockCount )
		{
			b2Pause();
		}

		b2AtomicStoreInt( &stage.completionCount, 0 );
	}
}

// This should not use the thread index because thread 0 can be called twice by enkiTS.
private void b2SolverTask(int startIndex, int endIndex, uint threadIndexIgnore, void* taskContext)
{
	// B2_UNUSED( startIndex, endIndex, threadIndexIgnore );

	b2WorkerContext* workerContext = cast(b2WorkerContext*)taskContext;
	int workerIndex = workerContext.workerIndex;
	b2StepContext* context = workerContext.context;
	int activeColorCount = context.activeColorCount;
	b2SolverStage* stages = context.stages;
	b2Profile* profile = &context.world.profile;

	if ( workerIndex == 0 )
	{
		// Main thread synchronizes the workers and does work itself.
		//
		// Stages are re-used by loops so that I don't need more stages for large iteration counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		b2_stagePrepareJoints,
		b2_stagePrepareContacts,
		b2_stageIntegrateVelocities,
		b2_stageWarmStart,
		b2_stageSolve,
		b2_stageIntegratePositions,
		b2_stageRelax,
		b2_stageRestitution,
		b2_stageStoreImpulses
		*/

		ulong ticks = b2GetTicks();

		int bodySyncIndex = 1;
		int stageIndex = 0;

		// This stage loops over all awake joints
		uint jointSyncIndex = 1;
		uint syncBits = ( jointSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stagePrepareJoints );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		jointSyncIndex += 1;

		// This stage loops over all contact constraints
		uint contactSyncIndex = 1;
		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stagePrepareContacts );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );
		stageIndex += 1;
		contactSyncIndex += 1;

		int graphSyncIndex = 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		b2PrepareOverflowJoints( context );
		b2PrepareOverflowContacts( context );

		profile.prepareConstraints += b2GetMillisecondsAndReset( &ticks );

		int subStepCount = context.subStepCount;
		for ( int i = 0; i < subStepCount; ++i )
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int iterStageIndex = stageIndex;

			// integrate velocities
			syncBits = ( bodySyncIndex << 16 ) | iterStageIndex;
			B2_ASSERT( stages[iterStageIndex].type == b2_stageIntegrateVelocities );
			b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
			iterStageIndex += 1;
			bodySyncIndex += 1;

			profile.integrateVelocities += b2GetMillisecondsAndReset( &ticks );

			// warm start constraints
			b2WarmStartOverflowJoints( context );
			b2WarmStartOverflowContacts( context );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageWarmStart );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile.warmStart += b2GetMillisecondsAndReset( &ticks );

			// solve constraints
			bool useBias = true;

			for ( int j = 0; j < ITERATIONS; ++j )
			{
				// Overflow constraints have lower priority
				b2SolveOverflowJoints( context, useBias );
				b2SolveOverflowContacts( context, useBias );

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
					B2_ASSERT( stages[iterStageIndex].type == b2_stageSolve );
					b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
					iterStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			profile.solveImpulses += b2GetMillisecondsAndReset( &ticks );

			// integrate positions
			B2_ASSERT( stages[iterStageIndex].type == b2_stageIntegratePositions );
			syncBits = ( bodySyncIndex << 16 ) | iterStageIndex;
			b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
			iterStageIndex += 1;
			bodySyncIndex += 1;

			profile.integratePositions += b2GetMillisecondsAndReset( &ticks );

			// relax constraints
			useBias = false;
			for ( int j = 0; j < RELAX_ITERATIONS; ++j )
			{
				b2SolveOverflowJoints( context, useBias );
				b2SolveOverflowContacts( context, useBias );

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
					B2_ASSERT( stages[iterStageIndex].type == b2_stageRelax );
					b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
					iterStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			profile.relaxImpulses += b2GetMillisecondsAndReset( &ticks );
		}

		// advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

		// Restitution
		{
			b2ApplyOverflowRestitution( context );

			int iterStageIndex = stageIndex;
			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageRestitution );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			// graphSyncIndex += 1;
			stageIndex += activeColorCount;
		}

		profile.applyRestitution += b2GetMillisecondsAndReset( &ticks );

		b2StoreOverflowImpulses( context );

		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stageStoreImpulses );
		b2ExecuteMainStage( stages + stageIndex, context, syncBits );

		profile.storeImpulses += b2GetMillisecondsAndReset( &ticks );

		// Signal workers to finish
		b2AtomicStoreU32( &context.atomicSyncBits, uint.max );

		B2_ASSERT( stageIndex + 1 == context.stageCount );
		return;
	}

	// Worker spins and waits for work
	uint lastSyncBits = 0;
	// uint64_t maxSpinTime = 10;
	while ( true )
	{
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		uint syncBits = void;
		int spinCount = 0;
		while ( ( syncBits = b2AtomicLoadU32( &context.atomicSyncBits ) ) == lastSyncBits )
		{
			if ( spinCount > 5 )
			{
				b2Yield();
				spinCount = 0;
			}
			else
			{
				// Using the cycle counter helps to account for variation in mm_pause timing across different
				// CPUs. However, this is X64 only.
				// uint64_t prev = __rdtsc();
				// do
				//{
				//	b2Pause();
				//}
				// while ((__rdtsc() - prev) < maxSpinTime);
				// maxSpinTime += 10;
				b2Pause();
				b2Pause();
				spinCount += 1;
			}
		}

		if ( syncBits == uint.max )
		{
			// sentinel hit
			break;
		}

		int stageIndex = syncBits & 0xFFFF;
		B2_ASSERT( stageIndex < context.stageCount );

		int syncIndex = ( syncBits >> 16 ) & 0xFFFF;
		B2_ASSERT( syncIndex > 0 );

		int previousSyncIndex = syncIndex - 1;

		b2SolverStage* stage = stages + stageIndex;
		b2ExecuteStage( stage, context, previousSyncIndex, syncIndex, workerIndex );

		lastSyncBits = syncBits;
	}
}

private void b2BulletBodyTask(int startIndex, int endIndex, uint threadIndex, void* context)
{
	// B2_UNUSED( threadIndex );

	// b2TracyCZoneNC( bullet_body_task, "Bullet", b2_colorLightSkyBlue, true );

	b2StepContext* stepContext = cast(b2StepContext*)context;
	b2TaskContext* taskContext = b2TaskContextArray_Get( stepContext.world.taskContexts, threadIndex );

	B2_ASSERT( startIndex <= endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		int simIndex = stepContext.bulletBodies[i];
		b2SolveContinuous( stepContext.world, simIndex, taskContext );
	}

	// b2TracyCZoneEnd( bullet_body_task );
}

static if (B2_SIMD_WIDTH == 8) {
enum B2_SIMD_SHIFT = 3;
} else static if (B2_SIMD_WIDTH == 4) {
enum B2_SIMD_SHIFT = 2;
} else {
enum B2_SIMD_SHIFT = 0;
}

// Solve with graph coloring
void b2Solve(b2World* world, b2StepContext* stepContext)
{
	world.stepIndex += 1;

	// Are there any awake bodies? This scenario should not be important for profiling.
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	int awakeBodyCount = cast(int)awakeSet.bodySims.count;
	if ( awakeBodyCount == 0 )
	{
		// Nothing to simulate, however the tree rebuild must be finished.
		if ( world.userTreeTask != null )
		{
			world.finishTaskFcn( world.userTreeTask, world.userTaskContext );
			world.userTreeTask = null;
			world.activeTaskCount -= 1;
		}

		b2ValidateNoEnlarged( &world.broadPhase );
		return;
	}

	// Solve constraints using graph coloring
	{
		// Prepare buffers for bullets
		b2AtomicStoreInt( &stepContext.bulletBodyCount, 0 );
		stepContext.bulletBodies = cast(int*)b2AllocateArenaItem( &world.arena, cast(int)(awakeBodyCount * int.sizeof), "bullet bodies" );

		// b2TracyCZoneNC( prepare_stages, "Prepare Stages", b2_colorDarkOrange, true );
		ulong prepareTicks = b2GetTicks();

		b2ConstraintGraph* graph = &world.constraintGraph;
		b2GraphColor* colors = graph.colors.ptr;

		stepContext.sims = awakeSet.bodySims.ptr;
		stepContext.states = awakeSet.bodyStates.ptr;

		// count contacts, joints, and colors
		int awakeJointCount = 0;
		int activeColorCount = 0;
		for ( int i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i )
		{
			int perColorContactCount = cast(int)colors[i].contactSims.count;
			int perColorJointCount = cast(int)colors[i].jointSims.count;
			int occupancyCount = perColorContactCount + perColorJointCount;
			activeColorCount += occupancyCount > 0 ? 1 : 0;
			awakeJointCount += perColorJointCount;
		}

		// prepare for move events
		b2BodyMoveEventArray_Resize( world.bodyMoveEvents, awakeBodyCount );

		// Each worker receives at most M blocks of work. The workers may receive less blocks if there is not sufficient work.
		// Each block of work has a minimum number of elements (block size). This in turn may limit the number of blocks.
		// If there are many elements then the block size is increased so there are still at most M blocks of work per worker.
		// M is a tunable number that has two goals:
		// 1. keep M small to reduce overhead
		// 2. keep M large enough for other workers to be able to steal work
		// The block size is a power of two to make math efficient.

		int workerCount = world.workerCount;
		const(int) blocksPerWorker = 4;
		const(int) maxBlockCount = blocksPerWorker * workerCount;

		// Configure blocks for tasks that parallel-for bodies
		int bodyBlockSize = 1 << 5;
		int bodyBlockCount = void;
		if ( awakeBodyCount > bodyBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			bodyBlockSize = awakeBodyCount / maxBlockCount;
			bodyBlockCount = maxBlockCount;
		}
		else
		{
			bodyBlockCount = ( ( awakeBodyCount - 1 ) >> 5 ) + 1;
		}

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of SIMD contact blocks and joint blocks
		int[B2_GRAPH_COLOR_COUNT] activeColorIndices = void;

		int[B2_GRAPH_COLOR_COUNT] colorContactCounts = void;
		int[B2_GRAPH_COLOR_COUNT] colorContactBlockSizes = void;
		int[B2_GRAPH_COLOR_COUNT] colorContactBlockCounts = void;

		int[B2_GRAPH_COLOR_COUNT] colorJointCounts = void;
		int[B2_GRAPH_COLOR_COUNT] colorJointBlockSizes = void;
		int[B2_GRAPH_COLOR_COUNT] colorJointBlockCounts = void;

		int graphBlockCount = 0;

		// c is the active color index
		int simdContactCount = 0;
		int c = 0;
		for ( int i = 0; i < B2_GRAPH_COLOR_COUNT - 1; ++i )
		{
			int colorContactCount = cast(int)colors[i].contactSims.count;
			int colorJointCount = cast(int)colors[i].jointSims.count;

			if ( colorContactCount + colorJointCount > 0 )
			{
				activeColorIndices[c] = i;

				// 4/8-way SIMD
				int colorContactCountSIMD = colorContactCount > 0 ? ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1 : 0;

				colorContactCounts[c] = colorContactCountSIMD;

				// determine the number of contact work blocks for this color
				if ( colorContactCountSIMD > blocksPerWorker * maxBlockCount )
				{
					// too many contact blocks
					colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
					colorContactBlockCounts[c] = maxBlockCount;
				}
				else if ( colorContactCountSIMD > 0 )
				{
					// dividing by blocksPerWorker (4)
					colorContactBlockSizes[c] = blocksPerWorker;
					colorContactBlockCounts[c] = ( ( colorContactCountSIMD - 1 ) >> 2 ) + 1;
				}
				else
				{
					// no contacts in this color
					colorContactBlockSizes[c] = 0;
					colorContactBlockCounts[c] = 0;
				}

				colorJointCounts[c] = colorJointCount;

				// determine number of joint work blocks for this color
				if ( colorJointCount > blocksPerWorker * maxBlockCount )
				{
					// too many joint blocks
					colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
					colorJointBlockCounts[c] = maxBlockCount;
				}
				else if ( colorJointCount > 0 )
				{
					// dividing by blocksPerWorker (4)
					colorJointBlockSizes[c] = blocksPerWorker;
					colorJointBlockCounts[c] = ( ( colorJointCount - 1 ) >> 2 ) + 1;
				}
				else
				{
					colorJointBlockSizes[c] = 0;
					colorJointBlockCounts[c] = 0;
				}

				graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
				simdContactCount += colorContactCountSIMD;
				c += 1;
			}
		}
		activeColorCount = c;

		// Gather contact pointers for easy parallel-for traversal. Some may be NULL due to SIMD remainders.
		b2ContactSim** contacts = cast(b2ContactSim**)b2AllocateArenaItem( &world.arena, cast(int)(B2_SIMD_WIDTH * simdContactCount * (b2ContactSim*).sizeof), "contact pointers" );

		// Gather joint pointers for easy parallel-for traversal.
		b2JointSim** joints = cast(b2JointSim**)b2AllocateArenaItem( &world.arena, cast(int)(awakeJointCount * (b2JointSim*).sizeof), "joint pointers" );

		int simdConstraintSize = b2GetContactConstraintSIMDByteCount();
		b2ContactConstraintSIMD* simdContactConstraints = cast(b2ContactConstraintSIMD*)b2AllocateArenaItem( &world.arena, cast(int)(simdContactCount * simdConstraintSize), "contact constraint" );

		int overflowContactCount = cast(int)colors[B2_OVERFLOW_INDEX].contactSims.count;
		b2ContactConstraint* overflowContactConstraints = cast(b2ContactConstraint*)b2AllocateArenaItem(
			&world.arena, cast(int)(overflowContactCount * b2ContactConstraint.sizeof), "overflow contact constraint" );

		graph.colors[B2_OVERFLOW_INDEX].overflowConstraints = overflowContactConstraints;

		// Distribute transient constraints to each graph color and build flat arrays of contact and joint pointers
		{
			int contactBase = 0;
			int jointBase = 0;
			for ( int i = 0; i < activeColorCount; ++i )
			{
				int j = activeColorIndices[i];
				b2GraphColor* color = colors + j;

				int colorContactCount = cast(int)color.contactSims.count;

				if ( colorContactCount == 0 )
				{
					color.simdConstraints = null;
				}
				else
				{
					color.simdConstraints =
						cast(b2ContactConstraintSIMD*)( cast(ubyte*)simdContactConstraints + contactBase * simdConstraintSize );

					for ( int k = 0; k < colorContactCount; ++k )
					{
						contacts[B2_SIMD_WIDTH * contactBase + k] = color.contactSims.ptr + k;
					}

					// remainder
					int colorContactCountSIMD = ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1;
					for ( int k = colorContactCount; k < B2_SIMD_WIDTH * colorContactCountSIMD; ++k )
					{
						contacts[B2_SIMD_WIDTH * contactBase + k] = null;
					}

					contactBase += colorContactCountSIMD;
				}

				int colorJointCount = cast(int)color.jointSims.count;
				for ( int k = 0; k < colorJointCount; ++k )
				{
					joints[jointBase + k] = color.jointSims.ptr + k;
				}
				jointBase += colorJointCount;
			}

			B2_ASSERT( contactBase == simdContactCount );
			B2_ASSERT( jointBase == awakeJointCount );
		}

		// Define work blocks for preparing contacts and storing contact impulses
		int contactBlockSize = blocksPerWorker;
		int contactBlockCount = simdContactCount > 0 ? ( ( simdContactCount - 1 ) >> 2 ) + 1 : 0;
		if ( simdContactCount > contactBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			contactBlockSize = simdContactCount / maxBlockCount;
			contactBlockCount = maxBlockCount;
		}

		// Define work blocks for preparing joints
		int jointBlockSize = blocksPerWorker;
		int jointBlockCount = awakeJointCount > 0 ? ( ( awakeJointCount - 1 ) >> 2 ) + 1 : 0;
		if ( awakeJointCount > jointBlockSize * maxBlockCount )
		{
			// Too many blocks, increase block size
			jointBlockSize = awakeJointCount / maxBlockCount;
			jointBlockCount = maxBlockCount;
		}

		int stageCount = 0;

		// b2_stagePrepareJoints
		stageCount += 1;
		// b2_stagePrepareContacts
		stageCount += 1;
		// b2_stageIntegrateVelocities
		stageCount += 1;
		// b2_stageWarmStart
		stageCount += activeColorCount;
		// b2_stageSolve
		stageCount += ITERATIONS * activeColorCount;
		// b2_stageIntegratePositions
		stageCount += 1;
		// b2_stageRelax
		stageCount += RELAX_ITERATIONS * activeColorCount;
		// b2_stageRestitution
		stageCount += activeColorCount;
		// b2_stageStoreImpulses
		stageCount += 1;

		b2SolverStage* stages = cast(b2SolverStage*)b2AllocateArenaItem( &world.arena, cast(int)(stageCount * b2SolverStage.sizeof), "stages" );
		b2SolverBlock* bodyBlocks = cast(b2SolverBlock*)b2AllocateArenaItem( &world.arena, cast(int)(bodyBlockCount * b2SolverBlock.sizeof), "body blocks" );
		b2SolverBlock* contactBlocks = cast(b2SolverBlock*)b2AllocateArenaItem( &world.arena, cast(int)(contactBlockCount * b2SolverBlock.sizeof), "contact blocks" );
		b2SolverBlock* jointBlocks = cast(b2SolverBlock*)b2AllocateArenaItem( &world.arena, cast(int)(jointBlockCount * b2SolverBlock.sizeof), "joint blocks" );
		b2SolverBlock* graphBlocks =cast(b2SolverBlock*) b2AllocateArenaItem( &world.arena, cast(int)(graphBlockCount * b2SolverBlock.sizeof), "graph blocks" );

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		void* splitIslandTask = null;
		if ( world.splitIslandId != B2_NULL_INDEX )
		{
			splitIslandTask = world.enqueueTaskFcn( &b2SplitIslandTask, 1, 1, world, world.userTaskContext );
			world.taskCount += 1;
			world.activeTaskCount += splitIslandTask == null ? 0 : 1;
		}

		// Prepare body work blocks
		for ( int i = 0; i < bodyBlockCount; ++i )
		{
			b2SolverBlock* block = bodyBlocks + i;
			block.startIndex = i * bodyBlockSize;
			block.count = cast(short)bodyBlockSize;
			block.blockType = b2_bodyBlock;
			b2AtomicStoreInt( &block.syncIndex, 0 );
		}
		bodyBlocks[bodyBlockCount - 1].count = cast(short)( awakeBodyCount - ( bodyBlockCount - 1 ) * bodyBlockSize );

		// Prepare joint work blocks
		for ( int i = 0; i < jointBlockCount; ++i )
		{
			b2SolverBlock* block = jointBlocks + i;
			block.startIndex = i * jointBlockSize;
			block.count = cast(short)jointBlockSize;
			block.blockType = b2_jointBlock;
			b2AtomicStoreInt( &block.syncIndex, 0 );
		}

		if ( jointBlockCount > 0 )
		{
			jointBlocks[jointBlockCount - 1].count = cast(short)( awakeJointCount - ( jointBlockCount - 1 ) * jointBlockSize );
		}

		// Prepare contact work blocks
		for ( int i = 0; i < contactBlockCount; ++i )
		{
			b2SolverBlock* block = contactBlocks + i;
			block.startIndex = i * contactBlockSize;
			block.count = cast(short)contactBlockSize;
			block.blockType = b2_contactBlock;
			b2AtomicStoreInt( &block.syncIndex, 0 );
		}

		if ( contactBlockCount > 0 )
		{
			contactBlocks[contactBlockCount - 1].count =
				cast(short)( simdContactCount - ( contactBlockCount - 1 ) * contactBlockSize );
		}

		// Prepare graph work blocks
		b2SolverBlock*[B2_GRAPH_COLOR_COUNT] graphColorBlocks = void;
		b2SolverBlock* baseGraphBlock = graphBlocks;

		for ( int i = 0; i < activeColorCount; ++i )
		{
			graphColorBlocks[i] = baseGraphBlock;

			int colorJointBlockCount = colorJointBlockCounts[i];
			int colorJointBlockSize = colorJointBlockSizes[i];
			for ( int j = 0; j < colorJointBlockCount; ++j )
			{
				b2SolverBlock* block = baseGraphBlock + j;
				block.startIndex = j * colorJointBlockSize;
				block.count = cast(short)colorJointBlockSize;
				block.blockType = b2_graphJointBlock;
				b2AtomicStoreInt( &block.syncIndex, 0 );
			}

			if ( colorJointBlockCount > 0 )
			{
				baseGraphBlock[colorJointBlockCount - 1].count =
					cast(short)( colorJointCounts[i] - ( colorJointBlockCount - 1 ) * colorJointBlockSize );
				baseGraphBlock += colorJointBlockCount;
			}

			int colorContactBlockCount = colorContactBlockCounts[i];
			int colorContactBlockSize = colorContactBlockSizes[i];
			for ( int j = 0; j < colorContactBlockCount; ++j )
			{
				b2SolverBlock* block = baseGraphBlock + j;
				block.startIndex = j * colorContactBlockSize;
				block.count = cast(short)colorContactBlockSize;
				block.blockType = b2_graphContactBlock;
				b2AtomicStoreInt( &block.syncIndex, 0 );
			}

			if ( colorContactBlockCount > 0 )
			{
				baseGraphBlock[colorContactBlockCount - 1].count =
					cast(short)( colorContactCounts[i] - ( colorContactBlockCount - 1 ) * colorContactBlockSize );
				baseGraphBlock += colorContactBlockCount;
			}
		}

		B2_ASSERT( cast(ptrdiff_t)( baseGraphBlock - graphBlocks ) == graphBlockCount );

		b2SolverStage* stage = stages;

		// Prepare joints
		stage.type = b2_stagePrepareJoints;
		stage.blocks = jointBlocks;
		stage.blockCount = jointBlockCount;
		stage.colorIndex = -1;
		b2AtomicStoreInt( &stage.completionCount, 0 );
		stage += 1;

		// Prepare contacts
		stage.type = b2_stagePrepareContacts;
		stage.blocks = contactBlocks;
		stage.blockCount = contactBlockCount;
		stage.colorIndex = -1;
		b2AtomicStoreInt( &stage.completionCount, 0 );
		stage += 1;

		// Integrate velocities
		stage.type = b2_stageIntegrateVelocities;
		stage.blocks = bodyBlocks;
		stage.blockCount = bodyBlockCount;
		stage.colorIndex = -1;
		b2AtomicStoreInt( &stage.completionCount, 0 );
		stage += 1;

		// Warm start
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage.type = b2_stageWarmStart;
			stage.blocks = graphColorBlocks[i];
			stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
			stage.colorIndex = activeColorIndices[i];
			b2AtomicStoreInt( &stage.completionCount, 0 );
			stage += 1;
		}

		// Solve graph
		for ( int j = 0; j < ITERATIONS; ++j )
		{
			for ( int i = 0; i < activeColorCount; ++i )
			{
				stage.type = b2_stageSolve;
				stage.blocks = graphColorBlocks[i];
				stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
				stage.colorIndex = activeColorIndices[i];
				b2AtomicStoreInt( &stage.completionCount, 0 );
				stage += 1;
			}
		}

		// Integrate positions
		stage.type = b2_stageIntegratePositions;
		stage.blocks = bodyBlocks;
		stage.blockCount = bodyBlockCount;
		stage.colorIndex = -1;
		b2AtomicStoreInt( &stage.completionCount, 0 );
		stage += 1;

		// Relax constraints
		for ( int j = 0; j < RELAX_ITERATIONS; ++j )
		{
			for ( int i = 0; i < activeColorCount; ++i )
			{
				stage.type = b2_stageRelax;
				stage.blocks = graphColorBlocks[i];
				stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
				stage.colorIndex = activeColorIndices[i];
				b2AtomicStoreInt( &stage.completionCount, 0 );
				stage += 1;
			}
		}

		// Restitution
		// Note: joint blocks mixed in, could have joint limit restitution
		for ( int i = 0; i < activeColorCount; ++i )
		{
			stage.type = b2_stageRestitution;
			stage.blocks = graphColorBlocks[i];
			stage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
			stage.colorIndex = activeColorIndices[i];
			b2AtomicStoreInt( &stage.completionCount, 0 );
			stage += 1;
		}

		// Store impulses
		stage.type = b2_stageStoreImpulses;
		stage.blocks = contactBlocks;
		stage.blockCount = contactBlockCount;
		stage.colorIndex = -1;
		b2AtomicStoreInt( &stage.completionCount, 0 );
		stage += 1;

		B2_ASSERT( cast(int)( stage - stages ) == stageCount );

		B2_ASSERT( workerCount <= B2_MAX_WORKERS );
		b2WorkerContext[B2_MAX_WORKERS] workerContext = void;

		stepContext.graph = graph;
		stepContext.joints = joints;
		stepContext.contacts = contacts;
		stepContext.simdContactConstraints = simdContactConstraints;
		stepContext.activeColorCount = activeColorCount;
		stepContext.workerCount = workerCount;
		stepContext.stageCount = stageCount;
		stepContext.stages = stages;
		b2AtomicStoreU32( &stepContext.atomicSyncBits, 0 );

		world.profile.prepareStages = b2GetMillisecondsAndReset( &prepareTicks );
		ulong constraintTicks = b2GetTicks();

		// Must use worker index because thread 0 can be assigned multiple tasks by enkiTS
		int jointIdCapacity = b2GetIdCapacity( &world.jointIdPool );
		for ( int i = 0; i < workerCount; ++i )
		{
			b2TaskContext* taskContext = b2TaskContextArray_Get( world.taskContexts, i );
			b2SetBitCountAndClear( &taskContext.jointStateBitSet, jointIdCapacity );

			workerContext[i].context = stepContext;
			workerContext[i].workerIndex = i;
			workerContext[i].userTask = world.enqueueTaskFcn( &b2SolverTask, 1, 1, workerContext.ptr + i, world.userTaskContext );
			world.taskCount += 1;
			world.activeTaskCount += workerContext[i].userTask == null ? 0 : 1;
		}

		// Finish island split
		if ( splitIslandTask != null )
		{
			world.finishTaskFcn( splitIslandTask, world.userTaskContext );
			world.activeTaskCount -= 1;
		}
		world.splitIslandId = B2_NULL_INDEX;

		// Finish constraint solve
		for ( int i = 0; i < workerCount; ++i )
		{
			if ( workerContext[i].userTask != null )
			{
				world.finishTaskFcn( workerContext[i].userTask, world.userTaskContext );
				world.activeTaskCount -= 1;
			}
		}

		world.profile.solveConstraints = b2GetMillisecondsAndReset( &constraintTicks );

		ulong transformTicks = b2GetTicks();

		// Prepare contact, enlarged body, and island bit sets used in body finalization.
		int awakeIslandCount = cast(int)awakeSet.islandSims.count;
		for ( int i = 0; i < world.workerCount; ++i )
		{
			b2TaskContext* taskContext = world.taskContexts.ptr + i;
			b2SensorHitArray_Clear(taskContext.sensorHits );
			b2SetBitCountAndClear(&taskContext.enlargedSimBitSet, awakeBodyCount );
			b2SetBitCountAndClear(&taskContext.awakeIslandBitSet, awakeIslandCount );
			taskContext.splitIslandId = B2_NULL_INDEX;
			taskContext.splitSleepTime = 0.0f;
		}

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		void* finalizeBodiesTask = world.enqueueTaskFcn( &b2FinalizeBodiesTask, awakeBodyCount, 64, stepContext, world.userTaskContext );
		world.taskCount += 1;
		if ( finalizeBodiesTask != null )
		{
			world.finishTaskFcn( finalizeBodiesTask, world.userTaskContext );
		}

		b2FreeArenaItem( &world.arena, graphBlocks );
		b2FreeArenaItem( &world.arena, jointBlocks );
		b2FreeArenaItem( &world.arena, contactBlocks );
		b2FreeArenaItem( &world.arena, bodyBlocks );
		b2FreeArenaItem( &world.arena, stages );
		b2FreeArenaItem( &world.arena, overflowContactConstraints );
		b2FreeArenaItem( &world.arena, simdContactConstraints );
		b2FreeArenaItem( &world.arena, joints );
		b2FreeArenaItem( &world.arena, contacts );

		world.profile.transforms = b2GetMilliseconds( transformTicks );
	}

	// Report joint events
	{
		ulong jointEventTicks = b2GetTicks();

		// Gather bits for all joints that have force/torque events
		b2BitSet* jointStateBitSet = &world.taskContexts[0].jointStateBitSet;
		for ( int i = 1; i < world.workerCount; ++i )
		{
			b2InPlaceUnion( jointStateBitSet, &world.taskContexts[i].jointStateBitSet );
		}

		{
			uint wordCount = jointStateBitSet.blockCount;
			ulong* bits = jointStateBitSet.bits;

			b2Joint* jointArray = world.joints.ptr;
			ushort worldIndex0 = world.worldId;

			for ( uint k = 0; k < wordCount; ++k )
			{
				ulong word = bits[k];
				while ( word != 0 )
				{
					uint ctz = b2CTZ64( word );
					int jointId = cast(int)( 64 * k + ctz );

					B2_ASSERT( jointId < world.joints.capacity );

					b2Joint* joint = jointArray + jointId;

					B2_ASSERT( joint.setIndex == b2_awakeSet );

					b2JointEvent event = {
						jointId:
							{
								index1: jointId + 1,
								world0: worldIndex0,
								generation: joint.generation,
							},
						userData: joint.userData,
					};

					b2JointEventArray_Push( world.jointEvents, event );

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		world.profile.jointEvents = b2GetMilliseconds( jointEventTicks );
	}

	// Report hit events
	// todo_erin perhaps optimize this with a bitset
	// todo_erin perhaps do this in parallel with other work below
	{
		ulong hitTicks = b2GetTicks();

		B2_ASSERT( world.contactHitEvents.count == 0 );

		float threshold = world.hitEventThreshold;
		b2GraphColor* colors = world.constraintGraph.colors.ptr;
		for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
		{
			b2GraphColor* color = colors + i;
			int contactCount = cast(int)color.contactSims.count;
			b2ContactSim* contactSims = color.contactSims.ptr;
			for ( int j = 0; j < contactCount; ++j )
			{
				b2ContactSim* contactSim = contactSims + j;
				if ( ( contactSim.simFlags & b2_simEnableHitEvent ) == 0 )
				{
					continue;
				}

				b2ContactHitEvent event;
				event.approachSpeed = threshold;

				bool hit = false;
				int pointCount = cast(int)contactSim.manifold.pointCount;
				for ( int k = 0; k < pointCount; ++k )
				{
					b2ManifoldPoint* mp = contactSim.manifold.points.ptr + k;
					float approachSpeed = -mp.normalVelocity;

					// Need to check total impulse because the point may be speculative and not colliding
					if ( approachSpeed > event.approachSpeed && mp.totalNormalImpulse > 0.0f )
					{
						event.approachSpeed = approachSpeed;
						event.point = mp.point;
						hit = true;
					}
				}

				if ( hit == true )
				{
					event.normal = contactSim.manifold.normal;

					b2Shape* shapeA = b2ShapeArray_Get( world.shapes, contactSim.shapeIdA );
					b2Shape* shapeB = b2ShapeArray_Get( world.shapes, contactSim.shapeIdB );

					event.shapeIdA = b2ShapeId( shapeA.id + 1, world.worldId, shapeA.generation );
					event.shapeIdB = b2ShapeId( shapeB.id + 1, world.worldId, shapeB.generation );

					b2ContactHitEventArray_Push( world.contactHitEvents, event );
				}
			}
		}

		world.profile.hitEvents = b2GetMilliseconds( hitTicks );
	}

	{
		ulong refitTicks = b2GetTicks();

		// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
		// broad-phase.
		if ( world.userTreeTask != null )
		{
			world.finishTaskFcn( world.userTreeTask, world.userTaskContext );
			world.userTreeTask = null;
			world.activeTaskCount -= 1;
		}

		b2ValidateNoEnlarged( &world.broadPhase );

		// Gather bits for all sim bodies that have enlarged AABBs
		b2BitSet* enlargedBodyBitSet = &world.taskContexts[0].enlargedSimBitSet;
		for ( int i = 1; i < world.workerCount; ++i )
		{
			b2InPlaceUnion( enlargedBodyBitSet, &world.taskContexts[i].enlargedSimBitSet );
		}

		// Enlarge broad-phase proxies and build move array
		// Apply shape AABB changes to broad-phase. This also create the move array which must be
		// in deterministic order. I'm tracking sim bodies because the number of shape ids can be huge.
		// This has to happen before bullets are processed.
		{
			b2BroadPhase* broadPhase = &world.broadPhase;
			uint wordCount = enlargedBodyBitSet.blockCount;
			ulong* bits = enlargedBodyBitSet.bits;

			// Fast array access is important here
			b2Body* bodyArray = world.bodies.ptr;
			b2BodySim* bodySimArray = awakeSet.bodySims.ptr;
			b2Shape* shapeArray = world.shapes.ptr;

			for ( uint k = 0; k < wordCount; ++k )
			{
				ulong word = bits[k];
				while ( word != 0 )
				{
					uint ctz = b2CTZ64( word );
					uint bodySimIndex = 64 * k + ctz;

					b2BodySim* bodySim = bodySimArray + bodySimIndex;

					b2Body* body = bodyArray + bodySim.bodyId;

					int shapeId = body.headShapeId;
					if ( ( bodySim.flags & ( b2_isBullet | b2_isFast ) ) == ( b2_isBullet | b2_isFast ) )
					{
						// Fast bullet bodies don't have their final AABB yet
						while ( shapeId != B2_NULL_INDEX )
						{
							b2Shape* shape = shapeArray + shapeId;

							// Shape is fast. It's aabb will be enlarged in continuous collision.
							// Update the move array here for determinism because bullets are processed
							// below in non-deterministic order.
							b2BufferMove( broadPhase, shape.proxyKey );

							shapeId = shape.nextShapeId;
						}
					}
					else
					{
						while ( shapeId != B2_NULL_INDEX )
						{
							b2Shape* shape = shapeArray + shapeId;

							// The AABB may not have been enlarged, despite the body being flagged as enlarged.
							// For example, a body with multiple shapes may have not have all shapes enlarged.
							// A fast body may have been flagged as enlarged despite having no shapes enlarged.
							if ( shape.enlargedAABB )
							{
								b2BroadPhase_EnlargeProxy( broadPhase, shape.proxyKey, shape.fatAABB );
								shape.enlargedAABB = false;
							}

							shapeId = shape.nextShapeId;
						}
					}

					// Clear the smallest set bit
					word = word & ( word - 1 );
				}
			}
		}

		b2ValidateBroadphase( &world.broadPhase );

		world.profile.refit = b2GetMilliseconds( refitTicks );
	}

	int bulletBodyCount = b2AtomicLoadInt( &stepContext.bulletBodyCount );
	if ( bulletBodyCount > 0 )
	{
		ulong bulletTicks = b2GetTicks();

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		int minRange = 8;
		void* userBulletBodyTask = world.enqueueTaskFcn( &b2BulletBodyTask, bulletBodyCount, minRange, stepContext, world.userTaskContext );
		world.taskCount += 1;
		if ( userBulletBodyTask != null )
		{
			world.finishTaskFcn( userBulletBodyTask, world.userTaskContext );
		}

		// Serially enlarge broad-phase proxies for bullet shapes
		b2BroadPhase* broadPhase = &world.broadPhase;
		b2DynamicTree* dynamicTree = broadPhase.trees.ptr + b2_dynamicBody;

		// Fast array access is important here
		b2Body* bodyArray = world.bodies.ptr;
		b2BodySim* bodySimArray = awakeSet.bodySims.ptr;
		b2Shape* shapeArray = world.shapes.ptr;

		// Serially enlarge broad-phase proxies for bullet shapes
		int* bulletBodySimIndices = stepContext.bulletBodies;

		// This loop has non-deterministic order but it shouldn't affect the result
		for ( int i = 0; i < bulletBodyCount; ++i )
		{
			b2BodySim* bulletBodySim = bodySimArray + bulletBodySimIndices[i];
			if ( ( bulletBodySim.flags & b2_enlargeBounds ) == 0 )
			{
				continue;
			}

			// Clear flag
			bulletBodySim.flags &= ~b2_enlargeBounds;

			int bodyId = bulletBodySim.bodyId;
			B2_ASSERT( 0 <= bodyId && bodyId < world.bodies.count );
			b2Body* bulletBody = bodyArray + bodyId;

			int shapeId = bulletBody.headShapeId;
			while ( shapeId != B2_NULL_INDEX )
			{
				b2Shape* shape = shapeArray + shapeId;
				if ( shape.enlargedAABB == false )
				{
					shapeId = shape.nextShapeId;
					continue;
				}

				// Clear flag
				shape.enlargedAABB = false;

				int proxyKey = shape.proxyKey;
				int proxyId = B2_PROXY_ID( proxyKey );
				B2_ASSERT( B2_PROXY_TYPE( proxyKey ) == b2_dynamicBody );

				// all fast bullet shapes should already be in the move buffer
				B2_ASSERT( b2ContainsKey( &broadPhase.moveSet, proxyKey + 1 ) );

				b2DynamicTree_EnlargeProxy( dynamicTree, proxyId, shape.fatAABB );

				shapeId = shape.nextShapeId;
			}
		}

		world.profile.bullets = b2GetMilliseconds( bulletTicks );
	}

	// Need to free this even if no bullets got processed.
	b2FreeArenaItem( &world.arena, stepContext.bulletBodies );
	stepContext.bulletBodies = null;
	b2AtomicStoreInt( &stepContext.bulletBodyCount, 0 );

	// Report sensor hits. This may include bullets sensor hits.
	{
		ulong sensorHitTicks = b2GetTicks();

		int workerCount = world.workerCount;
		B2_ASSERT( workerCount == world.taskContexts.count );

		for ( int i = 0; i < workerCount; ++i )
		{
			b2TaskContext* taskContext = world.taskContexts.ptr + i;
			int hitCount = cast(int)taskContext.sensorHits.count;
			b2SensorHit* hits = taskContext.sensorHits.ptr;

			for ( int j = 0; j < hitCount; ++j )
			{
				b2SensorHit hit = hits[j];
				b2Shape* sensorShape = b2ShapeArray_Get( world.shapes, hit.sensorId );
				b2Shape* visitor = b2ShapeArray_Get( world.shapes, hit.visitorId );

				b2Sensor* sensor = b2SensorArray_Get( world.sensors, sensorShape.sensorIndex );
				b2Visitor shapeRef = {
					shapeId: hit.visitorId,
					generation: visitor.generation,
				};
				b2VisitorArray_Push( sensor.hits, shapeRef );
			}
		}

		world.profile.sensorHits = b2GetMilliseconds( sensorHitTicks );
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if ( world.enableSleep == true )
	{
		ulong sleepTicks = b2GetTicks();

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		B2_ASSERT( world.splitIslandId == B2_NULL_INDEX );
		float splitSleepTimer = 0.0f;
		for ( int i = 0; i < world.workerCount; ++i )
		{
			b2TaskContext* taskContext = world.taskContexts.ptr + i;
			if ( taskContext.splitIslandId != B2_NULL_INDEX && taskContext.splitSleepTime >= splitSleepTimer )
			{
				B2_ASSERT( taskContext.splitSleepTime > 0.0f );

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if ( taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId )
				{
					continue;
				}

				world.splitIslandId = taskContext.splitIslandId;
				splitSleepTimer = taskContext.splitSleepTime;
			}
		}

		b2BitSet* awakeIslandBitSet = &world.taskContexts[0].awakeIslandBitSet;
		for ( int i = 1; i < world.workerCount; ++i )
		{
			b2InPlaceUnion( awakeIslandBitSet, &world.taskContexts[i].awakeIslandBitSet );
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		b2IslandSim* islands = awakeSet.islandSims.ptr;
		int count = cast(int)awakeSet.islandSims.count;
		for ( int islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1 )
		{
			if ( b2GetBit( awakeIslandBitSet, islandIndex ) == true )
			{
				// this island is still awake
				continue;
			}

			b2IslandSim* island = islands + islandIndex;
			int islandId = island.islandId;

			b2TrySleepIsland( world, islandId );
		}

		b2ValidateSolverSets( world );

		world.profile.sleepIslands = b2GetMilliseconds( sleepTicks );
	}
}

