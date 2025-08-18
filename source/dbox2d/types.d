module dbox2d.types;

public import dbox2d.base;
public import dbox2d.collision;
public import dbox2d.id;
public import dbox2d.math_functions;
import dbox2d.core;

public import core.stdc.stdlib;
public import core.stdc.stdint;

enum B2_DEFAULT_CATEGORY_BITS = 1;
enum B2_DEFAULT_MASK_BITS = UINT64_MAX;

/// Task interface
/// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
/// The task spans a range of the parallel-for: [startIndex, endIndex)
/// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
/// A worker must only exist on only one thread at a time and is analogous to the thread index.
/// The task context is the context pointer sent from Box2D when it is enqueued.
/// The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to b2EnqueueTaskCallback
/// below. Box2D expects startIndex < endIndex and will execute a loop like this:
///
/// @code{.c}
/// for (int i = startIndex; i < endIndex; ++i)
/// {
/// 	DoWork();
/// }
/// @endcode
/// @ingroup world
alias b2TaskCallback = void function(int starIndex, int endIndex, uint32_t workerIndex, void* taskContext);

/// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
/// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
/// serially within the callback and there is no need to call b2FinishTaskCallback.
/// The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
/// This is essentially a parallel-for. The minRange parameter is a suggestion of the minimum number of items to assign
/// per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A minRange of 8 suggests
/// that your task system should split the work items among just two workers, even if you have more available.
/// In general the range [startIndex, endIndex) send to b2TaskCallback should obey:
/// endIndex - startIndex >= minRange
/// The exception of course is when itemCount < minRange.
/// @ingroup world
alias b2EnqueueTaskCallback = void* function(b2TaskCallback task, int itemCount, int minRange, void* taskContext, void* userContext);

/// Finishes a user task object that wraps a Box2D task.
/// @ingroup world
alias b2FinishTaskCallback = void function(void* userTask, void* userContext);

/// Optional friction mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
/// @ingroup world
alias b2FrictionCallback = float function( float frictionA, int userMaterialIdA, float frictionB, int userMaterialIdB );

/// Optional restitution mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box2D state or user application state.
/// @ingroup world
alias b2RestitutionCallback = float function( float restitutionA, int userMaterialIdA, float restitutionB, int userMaterialIdB );

/// Result from b2World_RayCastClosest
/// If there is initial overlap the fraction and normal will be zero while the point is an arbitrary point in the overlap region.
/// @ingroup world
struct b2RayResult {
	b2ShapeId shapeId;
	b2Vec2 point;
	b2Vec2 normal;
	float fraction = 0;
	int nodeVisits;
	int leafVisits;
	bool hit;
}

/// World definition used to create a simulation world.
/// Must be initialized using b2DefaultWorldDef().
/// @ingroup world
struct b2WorldDef {
	/// Gravity vector. Box2D has no up-vector defined.
	b2Vec2 gravity;

	/// Restitution speed threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	float restitutionThreshold = 0;

	/// Threshold speed for hit events. Usually meters per second.
	float hitEventThreshold = 0;

	/// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
	float contactHertz = 0;

	/// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
	/// the trade-off that overlap resolution becomes more energetic.
	float contactDampingRatio = 0;

	/// This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
	/// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
	/// decreasing the damping ratio.
	float contactSpeed = 0;

	/// Maximum linear speed. Usually meters per second.
	float maximumLinearSpeed = 0;

	/// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	b2FrictionCallback* frictionCallback;

	/// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	b2RestitutionCallback* restitutionCallback;

	/// Can bodies go to sleep to improve performance
	bool enableSleep;

	/// Enable continuous collision
	bool enableContinuous;

	/// Number of workers to use with the provided task system. Box2D performs best when using only
	/// performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	/// little benefit and may even harm performance.
	/// @note Box2D does not create threads. This is the number of threads your applications has created
	/// that you are allocating to b2World_Step.
	/// @warning Do not modify the default value unless you are also providing a task system and providing
	/// task callbacks (enqueueTask and finishTask).
	int workerCount;

	/// Function to spawn tasks
	b2EnqueueTaskCallback* enqueueTask;

	/// Function to finish a task
	b2FinishTaskCallback* finishTask;

	/// User context that is provided to enqueueTask and finishTask
	void* userTaskContext;

	/// User data
	void* userData;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Use this to initialize your world definition
/// The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.
/// @ingroup body
enum b2BodyType : int {
	/// zero mass, zero velocity, may be manually moved
	b2_staticBody = 0,

	/// zero mass, velocity set by user, moved by solver
	b2_kinematicBody = 1,

	/// positive mass, velocity determined by forces, moved by solver
	b2_dynamicBody = 2,

	/// number of body types
	b2_bodyTypeCount,
}

alias b2_staticBody = b2BodyType.b2_staticBody;
alias b2_kinematicBody = b2BodyType.b2_kinematicBody;
alias b2_dynamicBody = b2BodyType.b2_dynamicBody;
alias b2_bodyTypeCount = b2BodyType.b2_bodyTypeCount;

/// Motion locks to restrict the body movement
struct b2MotionLocks {
	/// Prevent translation along the x-axis
	bool linearX;

	/// Prevent translation along the y-axis
	bool linearY;

	/// Prevent rotation around the z-axis
	bool angularZ;
}

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
/// Body definitions are temporary objects used to bundle creation parameters.
/// Must be initialized using b2DefaultBodyDef().
/// @ingroup body
struct b2BodyDef {
	/// The body type: static, kinematic, or dynamic.
	b2BodyType type;

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	/// if the body is moved after shapes have been added.
	b2Vec2 position;

	/// The initial world rotation of the body. Use b2MakeRot() if you have an angle.
	b2Rot rotation;

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	b2Vec2 linearVelocity;

	/// The initial angular velocity of the body. Radians per second.
	float angularVelocity = 0;

	/// Linear damping is used to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Generally linear damping is undesirable because it makes objects move slowly
	/// as if they are floating.
	float linearDamping = 0;

	/// Angular damping is used to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Angular damping can be use slow down rotating bodies.
	float angularDamping = 0;

	/// Scale the gravity applied to this body. Non-dimensional.
	float gravityScale = 0;

	/// Sleep speed threshold, default is 0.05 meters per second
	float sleepThreshold = 0;

	/// Optional body name for debugging. Up to 31 characters (excluding null termination)
	const(char)* name;

	/// Use this to store application specific body data.
	void* userData;

	/// Motions locks to restrict linear and angular movement.
	/// Caution: may lead to softer constraints along the locked direction
	b2MotionLocks motionLocks;

	/// Set this flag to false if this body should never fall asleep.
	bool enableSleep;

	/// Is this body initially awake or sleeping?
	bool isAwake;

	/// Treat this body as high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	/// @warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	/// continuous collision.
	bool isBullet;

	/// Used to disable a body. A disabled body does not move or collide.
	bool isEnabled;

	/// This allows this body to bypass rotational speed limits. Should only be used
	/// for circular objects, like wheels.
	bool allowFastRotation;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// This is used to filter collision on shapes. It affects shape-vs-shape collision
/// and shape-versus-query collision (such as b2World_CastRay).
/// @ingroup shape
struct b2Filter {
	/// The collision category bits. Normally you would just set one bit. The category bits should
	/// represent your application object types. For example:
	/// @code{.cpp}
	/// enum MyCategories
	/// {
	///    Static  = 0x00000001,
	///    Dynamic = 0x00000002,
	///    Debris  = 0x00000004,
	///    Player  = 0x00000008,
	///    // etc
	/// };
	/// @endcode
	ulong categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	/// For example, you may want your player to only collide with static objects
	/// and other players.
	/// @code{.c}
	/// maskBits = Static | Player;
	/// @endcode
	ulong maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	/// For example, you may want ragdolls to collide with other ragdolls but you don't want
	/// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	/// and apply that group index to all shapes on the ragdoll.
	int groupIndex;
}

/// The query filter is used to filter collisions between queries and shapes. For example,
/// you may want a ray-cast representing a projectile to hit players and the static environment
/// but not debris.
/// @ingroup shape
struct b2QueryFilter {
	/// The collision category bits of this query. Normally you would just set one bit.
	ulong categoryBits;

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	ulong maskBits;
}

/// Shape type
/// @ingroup shape
enum b2ShapeType {
	/// A circle with an offset
	b2_circleShape,

	/// A capsule is an extruded circle
	b2_capsuleShape,

	/// A line segment
	b2_segmentShape,

	/// A convex polygon
	b2_polygonShape,

	/// A line segment owned by a chain shape
	b2_chainSegmentShape,

	/// The number of shape types
	b2_shapeTypeCount
}

alias b2_circleShape = b2ShapeType.b2_circleShape;
alias b2_capsuleShape = b2ShapeType.b2_capsuleShape;
alias b2_segmentShape = b2ShapeType.b2_segmentShape;
alias b2_polygonShape = b2ShapeType.b2_polygonShape;
alias b2_chainSegmentShape = b2ShapeType.b2_chainSegmentShape;
alias b2_shapeTypeCount = b2ShapeType.b2_shapeTypeCount;

/// Surface materials allow chain shapes to have per segment surface properties.
/// @ingroup shape
struct b2SurfaceMaterial {
	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	float friction = 0;

	/// The coefficient of restitution (bounce) usually in the range [0,1].
	/// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	float restitution = 0;

	/// The rolling resistance usually in the range [0,1].
	float rollingResistance = 0;

	/// The tangent speed for conveyor belts
	float tangentSpeed = 0;

	/// User material identifier. This is passed with query results and to friction and restitution
	/// combining functions. It is not used internally.
	int userMaterialId;

	/// Custom debug draw color.
	uint customColor;
}

/// Used to create a shape.
/// This is a temporary object used to bundle shape creation parameters. You may use
/// the same shape definition to create multiple shapes.
/// Must be initialized using b2DefaultShapeDef().
/// @ingroup shape
struct b2ShapeDef {
	/// Use this to store application specific shape data.
	void* userData;

	/// The surface material for this shape.
	b2SurfaceMaterial material;

	/// The density, usually in kg/m^2.
	/// This is not part of the surface material because this is for the interior, which may have
	/// other considerations, such as being hollow. For example a wood barrel may be hollow or full of water.
	float density = 0;

	/// Collision filtering data.
	b2Filter filter;

	/// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b2WorldDef.
	bool enableCustomFiltering;

	/// A sensor shape generates overlap events but never generates a collision response.
	/// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
	/// Sensors still contribute to the body mass if they have non-zero density.
	/// @note Sensor events are disabled by default.
	/// @see enableSensorEvents
	bool isSensor;

	/// Enable sensor events for this shape. This applies to sensors and non-sensors. False by default, even for sensors.
	bool enableSensorEvents;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	bool enableContactEvents;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	bool enableHitEvents;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	/// and must be carefully handled due to multithreading. Ignored for sensors.
	bool enablePreSolveEvents;

	/// When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
	/// static body creation when there are many static shapes.
	/// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	bool invokeContactCreation;

	/// Should the body update the mass properties when this shape is created. Default is true.
	bool updateBodyMass;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Used to create a chain of line segments. This is designed to eliminate ghost collisions with some limitations.
/// - chains are one-sided
/// - chains have no mass and should be used on static bodies
/// - chains have a counter-clockwise winding order (normal points right of segment direction)
/// - chains are either a loop or open
/// - a chain must have at least 4 points
/// - the distance between any two points must be greater than B2_LINEAR_SLOP
/// - a chain shape should not self intersect (this is not validated)
/// - an open chain shape has NO COLLISION on the first and final edge
/// - you may overlap two open chains on their first three and/or last three points to get smooth collision
/// - a chain shape creates multiple line segment shapes on the body
/// https://en.wikipedia.org/wiki/Polygonal_chain
/// Must be initialized using b2DefaultChainDef().
/// @warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
/// @ingroup shape
struct b2ChainDef {
	/// Use this to store application specific shape data.
	void* userData;

	/// An array of at least 4 points. These are cloned and may be temporary.
	const(b2Vec2)* points;

	/// The point count, must be 4 or more.
	int count;

	/// Surface materials for each segment. These are cloned.
	const(b2SurfaceMaterial)* materials;

	/// The material count. Must be 1 or count. This allows you to provide one
	/// material for all segments or a unique material per segment.
	int materialCount;

	/// Contact filtering data.
	b2Filter filter;

	/// Indicates a closed chain formed by connecting the first and last points
	bool isLoop;

	/// Enable sensors to detect this chain. False by default.
	bool enableSensorEvents;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Profiling data. Times are in milliseconds.
struct b2Profile {
	float step = 0;
	float pairs = 0;
	float collide = 0;
	float solve = 0;
	float prepareStages = 0;
	float solveConstraints = 0;
	float prepareConstraints = 0;
	float integrateVelocities = 0;
	float warmStart = 0;
	float solveImpulses = 0;
	float integratePositions = 0;
	float relaxImpulses = 0;
	float applyRestitution = 0;
	float storeImpulses = 0;
	float splitIslands = 0;
	float transforms = 0;
	float sensorHits = 0;
	float jointEvents = 0;
	float hitEvents = 0;
	float refit = 0;
	float bullets = 0;
	float sleepIslands = 0;
	float sensors = 0;
}

/// Counters that give details of the simulation size.
struct b2Counters {
	int bodyCount;
	int shapeCount;
	int contactCount;
	int jointCount;
	int islandCount;
	int stackUsed;
	int staticTreeHeight;
	int treeHeight;
	int byteCount;
	int taskCount;
	int[24] colorCounts;
}

/// Joint type enumeration
///
/// This is useful because all joint types use b2JointId and sometimes you
/// want to get the type of a joint.
/// @ingroup joint
enum b2JointType {
	b2_distanceJoint,
	b2_filterJoint,
	b2_motorJoint,
	b2_mouseJoint,
	b2_prismaticJoint,
	b2_revoluteJoint,
	b2_weldJoint,
	b2_wheelJoint,
}

alias b2_distanceJoint = b2JointType.b2_distanceJoint;
alias b2_filterJoint = b2JointType.b2_filterJoint;
alias b2_motorJoint = b2JointType.b2_motorJoint;
alias b2_mouseJoint = b2JointType.b2_mouseJoint;
alias b2_prismaticJoint = b2JointType.b2_prismaticJoint;
alias b2_revoluteJoint = b2JointType.b2_revoluteJoint;
alias b2_weldJoint = b2JointType.b2_weldJoint;
alias b2_wheelJoint = b2JointType.b2_wheelJoint;

/// Base joint definition used by all joint types.
/// The local frames are measured from the body's origin rather than the center of mass because:
/// 1. you might not know where the center of mass will be
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
struct b2JointDef {
	/// User data pointer
	void* userData;

	/// The first attached body
	b2BodyId bodyIdA;

	/// The second attached body
	b2BodyId bodyIdB;

	/// The first local joint frame
	b2Transform localFrameA;

	/// The second local joint frame
	b2Transform localFrameB;

	/// Force threshold for joint events
	float forceThreshold = 0;

	/// Torque threshold for joint events
	float torqueThreshold = 0;

	/// Constraint hertz (advanced feature)
	float constraintHertz = 0;

	/// Constraint damping ratio (advanced feature)
	float constraintDampingRatio = 0;

	/// Debug draw scale
	float drawScale = 0;

	/// Set this flag to true if the attached bodies should collide
	bool collideConnected;

}

/// Distance joint definition
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
/// @ingroup distance_joint
struct b2DistanceJointDef {
	/// Base joint definition
	b2JointDef base;

	/// The rest length of this joint. Clamped to a stable minimum value.
	float length = 0;

	/// Enable the distance constraint to behave like a spring. If false
	/// then the distance joint will be rigid, overriding the limit and motor.
	bool enableSpring;

	/// The lower spring force controls how much tension it can sustain
	float lowerSpringForce = 0;

	/// The upper spring force controls how much compression it an sustain
	float upperSpringForce = 0;

	/// The spring linear stiffness Hertz, cycles per second
	float hertz = 0;

	/// The spring linear damping ratio, non-dimensional
	float dampingRatio = 0;

	/// Enable/disable the joint limit
	bool enableLimit;

	/// Minimum length. Clamped to a stable minimum value.
	float minLength = 0;

	/// Maximum length. Must be greater than or equal to the minimum length.
	float maxLength = 0;

	/// Enable/disable the joint motor
	bool enableMotor;

	/// The maximum motor force, usually in newtons
	float maxMotorForce = 0;

	/// The desired motor speed, usually in meters per second
	float motorSpeed = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// A motor joint is used to control the relative velocity and or transform between two bodies.
/// With a velocity of zero this acts like top-down friction.
/// @ingroup motor_joint
struct b2MotorJointDef {
	/// Base joint definition
	b2JointDef base;

	/// The desired linear velocity
	b2Vec2 linearVelocity;

	/// The maximum motor force in newtons
	float maxVelocityForce = 0;

	/// The desired angular velocity
	float angularVelocity = 0;

	/// The maximum motor torque in newton-meters
	float maxVelocityTorque = 0;

	/// Linear spring hertz for position control
	float linearHertz = 0;

	/// Linear spring damping ratio
	float linearDampingRatio = 0;

	/// Maximum spring force in newtons
	float maxSpringForce = 0;

	/// Angular spring hertz for position control
	float angularHertz = 0;

	/// Angular spring damping ratio
	float angularDampingRatio = 0;

	/// Maximum spring torque in newton-meters
	float maxSpringTorque = 0;

	/// The desired relative transform. Body B relative to bodyA.
	b2Transform relativeTransform;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// A mouse joint is used to make a point on body B track a point on body A.
/// You may move local frame A to change the target point.
/// This a soft constraint and allows the constraint to stretch without
/// applying huge forces. This also applies rotation constraint heuristic to improve control.
/// @ingroup mouse_joint
struct b2MouseJointDef {
	/// Base joint definition
	b2JointDef base;

	/// Stiffness in hertz
	float hertz = 0;

	/// Damping ratio, non-dimensional
	float dampingRatio = 0;

	/// Maximum force, typically in newtons
	float maxForce = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// A filter joint is used to disable collision between two specific bodies.
///
/// @ingroup filter_joint
struct b2FilterJointDef {
	/// Base joint definition
	b2JointDef base;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Prismatic joint definition
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup prismatic_joint
struct b2PrismaticJointDef {
	/// Base joint definition
	b2JointDef base;

	/// Enable a linear spring along the prismatic joint axis
	bool enableSpring;

	/// The spring stiffness Hertz, cycles per second
	float hertz = 0;

	/// The spring damping ratio, non-dimensional
	float dampingRatio = 0;

	/// The target translation for the joint in meters. The spring-damper will drive
	/// to this translation.
	float targetTranslation = 0;

	/// Enable/disable the joint limit
	bool enableLimit;

	/// The lower translation limit
	float lowerTranslation = 0;

	/// The upper translation limit
	float upperTranslation = 0;

	/// Enable/disable the joint motor
	bool enableMotor;

	/// The maximum motor force, typically in newtons
	float maxMotorForce = 0;

	/// The desired motor speed, typically in meters per second
	float motorSpeed = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Revolute joint definition
/// A point on body B is fixed to a point on body A. Allows relative rotation.
/// @ingroup revolute_joint
struct b2RevoluteJointDef {
	/// Base joint definition
	b2JointDef base;

	/// The target angle for the joint in radians. The spring-damper will drive
	/// to this angle.
	float targetAngle = 0;

	/// Enable a rotational spring on the revolute hinge axis
	bool enableSpring;

	/// The spring stiffness Hertz, cycles per second
	float hertz = 0;

	/// The spring damping ratio, non-dimensional
	float dampingRatio = 0;

	/// A flag to enable joint limits
	bool enableLimit;

	/// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	float lowerAngle = 0;

	/// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	float upperAngle = 0;

	/// A flag to enable the joint motor
	bool enableMotor;

	/// The maximum motor torque, typically in newton-meters
	float maxMotorTorque = 0;

	/// The desired motor speed in radians per second
	float motorSpeed = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Weld joint definition
/// Connects two bodies together rigidly. This constraint provides springs to mimic
/// soft-body simulation.
/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
/// @ingroup weld_joint
struct b2WeldJointDef {
	/// Base joint definition
	b2JointDef base;

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	float linearHertz = 0;

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	float angularHertz = 0;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float linearDampingRatio = 0;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float angularDampingRatio = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// Wheel joint definition
/// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup wheel_joint
struct b2WheelJointDef {
	/// Base joint definition
	b2JointDef base;

	/// Enable a linear spring along the local axis
	bool enableSpring;

	/// Spring stiffness in Hertz
	float hertz = 0;

	/// Spring damping ratio, non-dimensional
	float dampingRatio = 0;

	/// Enable/disable the joint linear limit
	bool enableLimit;

	/// The lower translation limit
	float lowerTranslation = 0;

	/// The upper translation limit
	float upperTranslation = 0;

	/// Enable/disable the joint rotational motor
	bool enableMotor;

	/// The maximum motor torque, typically in newton-meters
	float maxMotorTorque = 0;

	/// The desired motor speed in radians per second
	float motorSpeed = 0;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
}

/// The explosion definition is used to configure options for explosions. Explosions
/// consider shape geometry when computing the impulse.
/// @ingroup world
struct b2ExplosionDef {
	/// Mask bits to filter shapes
	ulong maskBits;

	/// The center of the explosion in world space
	b2Vec2 position;

	/// The radius of the explosion
	float radius = 0;

	/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	float falloff = 0;

	/// Impulse per unit length. This applies an impulse according to the shape perimeter that
	/// is facing the explosion. Explosions only apply to circles, capsules, and polygons. This
	/// may be negative for implosions.
	float impulsePerLength = 0;
}

/**
 * @defgroup events Events
 * World event types.
 *
 * Events are used to collect events that occur during the world time step. These events
 * are then available to query after the time step is complete. This is preferable to callbacks
 * because Box2D uses multithreaded simulation.
 *
 * Also when events occur in the simulation step it may be problematic to modify the world, which is
 * often what applications want to do when events occur.
 *
 * With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
 * that some event data may become invalid. There are several samples that show how to do this safely.
 *
 * @{
 */

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
struct b2SensorBeginTouchEvent {
	/// The id of the sensor shape
	b2ShapeId sensorShapeId;

	/// The id of the shape that began touching the sensor shape
	b2ShapeId visitorShapeId;
}

/// An end touch event is generated when a shape stops overlapping a sensor shape.
///	These include things like setting the transform, destroying a body or shape, or changing
///	a filter. You will also get an end event if the sensor or visitor are destroyed.
///	Therefore you should always confirm the shape id is valid using b2Shape_IsValid.
struct b2SensorEndTouchEvent {
	/// The id of the sensor shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	b2ShapeId sensorShapeId;

	/// The id of the shape that stopped touching the sensor shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	b2ShapeId visitorShapeId;

}

/// Sensor events are buffered in the world and are available
/// as begin/end overlap event arrays after the time step is complete.
/// Note: these may become invalid if bodies and/or shapes are destroyed
struct b2SensorEvents {
	/// Array of sensor begin touch events
	b2SensorBeginTouchEvent* beginEvents;

	/// Array of sensor end touch events
	b2SensorEndTouchEvent* endEvents;

	/// The number of begin touch events
	int beginCount;

	/// The number of end touch events
	int endCount;
}

/// A begin touch event is generated when two shapes begin touching.
struct b2ContactBeginTouchEvent {
	/// Id of the first shape
	b2ShapeId shapeIdA;

	/// Id of the second shape
	b2ShapeId shapeIdB;

	/// The transient contact id. This contact maybe destroyed automatically when the world is modified or simulated.
	/// Used b2Contact_IsValid before using this id.
	b2ContactId contactId;
}

/// An end touch event is generated when two shapes stop touching.
///	You will get an end event if you do anything that destroys contacts previous to the last
///	world step. These include things like setting the transform, destroying a body
///	or shape, or changing a filter or body type.
struct b2ContactEndTouchEvent {
	/// Id of the first shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	b2ShapeId shapeIdA;

	/// Id of the second shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	b2ShapeId shapeIdB;

	/// Id of the contact.
	///	@warning this contact may have been destroyed
	///	@see b2Contact_IsValid
	b2ContactId contactId;
}

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
/// This may be reported for speculative contacts that have a confirmed impulse.
struct b2ContactHitEvent {
	/// Id of the first shape
	b2ShapeId shapeIdA;

	/// Id of the second shape
	b2ShapeId shapeIdB;

	/// Point where the shapes hit at the beginning of the time step.
	/// This is a mid-point between the two surfaces. It could be at speculative
	/// point where the two shapes were not touching at the beginning of the time step.
	b2Vec2 point;

	/// Normal vector pointing from shape A to shape B
	b2Vec2 normal;

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	float approachSpeed = 0;
}

/// Contact events are buffered in the Box2D world and are available
/// as event arrays after the time step is complete.
/// Note: these may become invalid if bodies and/or shapes are destroyed
struct b2ContactEvents {
	/// Array of begin touch events
	b2ContactBeginTouchEvent* beginEvents;

	/// Array of end touch events
	b2ContactEndTouchEvent* endEvents;

	/// Array of hit events
	b2ContactHitEvent* hitEvents;

	/// Number of begin touch events
	int beginCount;

	/// Number of end touch events
	int endCount;

	/// Number of hit events
	int hitCount;
}

/// Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
/// This is an efficient way for an application to update game object transforms rather than
/// calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
/// and it is only populated with bodies that have moved.
/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
struct b2BodyMoveEvent {
	void* userData;
	b2Transform transform;
	b2BodyId bodyId;
	bool fellAsleep;
}

/// Body events are buffered in the Box2D world and are available
/// as event arrays after the time step is complete.
/// Note: this data becomes invalid if bodies are destroyed
struct b2BodyEvents {
	/// Array of move events
	b2BodyMoveEvent* moveEvents;

	/// Number of move events
	int moveCount;
}

/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
/// The observed forces and torques are not returned for efficiency reasons.
struct b2JointEvent {
	/// The joint id
	b2JointId jointId;

	/// The user data from the joint for convenience
	void* userData;
}

/// Joint events are buffered in the world and are available
/// as event arrays after the time step is complete.
/// Note: this data becomes invalid if joints are destroyed
struct b2JointEvents {
	/// Array of events
	b2JointEvent* jointEvents;

	/// Number of events
	int count;
}

/// The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B.
/// @see b2Shape_GetContactData() and b2Body_GetContactData()
struct b2ContactData {
	b2ContactId contactId;
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Manifold manifold;
}

/// Prototype for a contact filter callback.
/// This is called when a contact pair is considered for collision. This allows you to
/// perform custom logic to prevent collision between shapes. This is only called if
/// one of the two shapes has custom filtering enabled.
/// Notes:
/// - this function must be thread-safe
/// - this is only called if one of the two shapes has enabled custom filtering
/// - this may be called for awake dynamic bodies and sensors
/// Return false if you want to disable the collision
/// @see b2ShapeDef
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
alias b2CustomFilterFcn = bool function( b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context );

/// Prototype for a pre-solve callback.
/// This is called after a contact is updated. This allows you to inspect a
/// contact before it goes to the solver. If you are careful, you can modify the
/// contact manifold (e.g. modify the normal).
/// Notes:
/// - this function must be thread-safe
/// - this is only called if the shape has enabled pre-solve events
/// - this is called only for awake dynamic bodies
/// - this is not called for sensors
/// - the supplied manifold has impulse values from the previous step
/// Return false if you want to disable the contact this step
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
alias b2PreSolveFcn = bool function( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Vec2 point, b2Vec2 normal, void* context );

/// Prototype callback for overlap queries.
/// Called for each shape found in the query.
/// @see b2World_OverlapABB
/// @return false to terminate the query.
/// @ingroup world
alias b2OverlapResultFcn = bool function( b2ShapeId shapeId, void* context );

/// Prototype callback for ray and shape casts.
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:
/// return -1: ignore this shape and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// A cast with initial overlap will return a zero fraction and a zero normal.
/// @param shapeId the shape hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection, zero for a shape cast with initial overlap
/// @param fraction the fraction along the ray at the point of intersection, zero for a shape cast with initial overlap
/// @param context the user context
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
/// @see b2World_CastRay
/// @ingroup world
alias b2CastResultFcn = float function( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context );

// Used to collect collision planes for character movers.
// Return true to continue gathering planes.
alias b2PlaneResultFcn = bool function( b2ShapeId shapeId, const b2PlaneResult* plane, void* context );

/// These colors are used for debug draw and mostly match the named SVG colors.
/// See https://www.rapidtables.com/web/color/index.html
/// https://johndecember.com/html/spec/colorsvg.html
/// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
enum b2HexColor : uint {
	b2_colorAliceBlue = 0xF0F8FF,
	b2_colorAntiqueWhite = 0xFAEBD7,
	b2_colorAqua = 0x00FFFF,
	b2_colorAquamarine = 0x7FFFD4,
	b2_colorAzure = 0xF0FFFF,
	b2_colorBeige = 0xF5F5DC,
	b2_colorBisque = 0xFFE4C4,
	b2_colorBlack = 0x000000,
	b2_colorBlanchedAlmond = 0xFFEBCD,
	b2_colorBlue = 0x0000FF,
	b2_colorBlueViolet = 0x8A2BE2,
	b2_colorBrown = 0xA52A2A,
	b2_colorBurlywood = 0xDEB887,
	b2_colorCadetBlue = 0x5F9EA0,
	b2_colorChartreuse = 0x7FFF00,
	b2_colorChocolate = 0xD2691E,
	b2_colorCoral = 0xFF7F50,
	b2_colorCornflowerBlue = 0x6495ED,
	b2_colorCornsilk = 0xFFF8DC,
	b2_colorCrimson = 0xDC143C,
	b2_colorCyan = 0x00FFFF,
	b2_colorDarkBlue = 0x00008B,
	b2_colorDarkCyan = 0x008B8B,
	b2_colorDarkGoldenRod = 0xB8860B,
	b2_colorDarkGray = 0xA9A9A9,
	b2_colorDarkGreen = 0x006400,
	b2_colorDarkKhaki = 0xBDB76B,
	b2_colorDarkMagenta = 0x8B008B,
	b2_colorDarkOliveGreen = 0x556B2F,
	b2_colorDarkOrange = 0xFF8C00,
	b2_colorDarkOrchid = 0x9932CC,
	b2_colorDarkRed = 0x8B0000,
	b2_colorDarkSalmon = 0xE9967A,
	b2_colorDarkSeaGreen = 0x8FBC8F,
	b2_colorDarkSlateBlue = 0x483D8B,
	b2_colorDarkSlateGray = 0x2F4F4F,
	b2_colorDarkTurquoise = 0x00CED1,
	b2_colorDarkViolet = 0x9400D3,
	b2_colorDeepPink = 0xFF1493,
	b2_colorDeepSkyBlue = 0x00BFFF,
	b2_colorDimGray = 0x696969,
	b2_colorDodgerBlue = 0x1E90FF,
	b2_colorFireBrick = 0xB22222,
	b2_colorFloralWhite = 0xFFFAF0,
	b2_colorForestGreen = 0x228B22,
	b2_colorFuchsia = 0xFF00FF,
	b2_colorGainsboro = 0xDCDCDC,
	b2_colorGhostWhite = 0xF8F8FF,
	b2_colorGold = 0xFFD700,
	b2_colorGoldenRod = 0xDAA520,
	b2_colorGray = 0x808080,
	b2_colorGreen = 0x008000,
	b2_colorGreenYellow = 0xADFF2F,
	b2_colorHoneyDew = 0xF0FFF0,
	b2_colorHotPink = 0xFF69B4,
	b2_colorIndianRed = 0xCD5C5C,
	b2_colorIndigo = 0x4B0082,
	b2_colorIvory = 0xFFFFF0,
	b2_colorKhaki = 0xF0E68C,
	b2_colorLavender = 0xE6E6FA,
	b2_colorLavenderBlush = 0xFFF0F5,
	b2_colorLawnGreen = 0x7CFC00,
	b2_colorLemonChiffon = 0xFFFACD,
	b2_colorLightBlue = 0xADD8E6,
	b2_colorLightCoral = 0xF08080,
	b2_colorLightCyan = 0xE0FFFF,
	b2_colorLightGoldenRodYellow = 0xFAFAD2,
	b2_colorLightGray = 0xD3D3D3,
	b2_colorLightGreen = 0x90EE90,
	b2_colorLightPink = 0xFFB6C1,
	b2_colorLightSalmon = 0xFFA07A,
	b2_colorLightSeaGreen = 0x20B2AA,
	b2_colorLightSkyBlue = 0x87CEFA,
	b2_colorLightSlateGray = 0x778899,
	b2_colorLightSteelBlue = 0xB0C4DE,
	b2_colorLightYellow = 0xFFFFE0,
	b2_colorLime = 0x00FF00,
	b2_colorLimeGreen = 0x32CD32,
	b2_colorLinen = 0xFAF0E6,
	b2_colorMagenta = 0xFF00FF,
	b2_colorMaroon = 0x800000,
	b2_colorMediumAquaMarine = 0x66CDAA,
	b2_colorMediumBlue = 0x0000CD,
	b2_colorMediumOrchid = 0xBA55D3,
	b2_colorMediumPurple = 0x9370DB,
	b2_colorMediumSeaGreen = 0x3CB371,
	b2_colorMediumSlateBlue = 0x7B68EE,
	b2_colorMediumSpringGreen = 0x00FA9A,
	b2_colorMediumTurquoise = 0x48D1CC,
	b2_colorMediumVioletRed = 0xC71585,
	b2_colorMidnightBlue = 0x191970,
	b2_colorMintCream = 0xF5FFFA,
	b2_colorMistyRose = 0xFFE4E1,
	b2_colorMoccasin = 0xFFE4B5,
	b2_colorNavajoWhite = 0xFFDEAD,
	b2_colorNavy = 0x000080,
	b2_colorOldLace = 0xFDF5E6,
	b2_colorOlive = 0x808000,
	b2_colorOliveDrab = 0x6B8E23,
	b2_colorOrange = 0xFFA500,
	b2_colorOrangeRed = 0xFF4500,
	b2_colorOrchid = 0xDA70D6,
	b2_colorPaleGoldenRod = 0xEEE8AA,
	b2_colorPaleGreen = 0x98FB98,
	b2_colorPaleTurquoise = 0xAFEEEE,
	b2_colorPaleVioletRed = 0xDB7093,
	b2_colorPapayaWhip = 0xFFEFD5,
	b2_colorPeachPuff = 0xFFDAB9,
	b2_colorPeru = 0xCD853F,
	b2_colorPink = 0xFFC0CB,
	b2_colorPlum = 0xDDA0DD,
	b2_colorPowderBlue = 0xB0E0E6,
	b2_colorPurple = 0x800080,
	b2_colorRebeccaPurple = 0x663399,
	b2_colorRed = 0xFF0000,
	b2_colorRosyBrown = 0xBC8F8F,
	b2_colorRoyalBlue = 0x4169E1,
	b2_colorSaddleBrown = 0x8B4513,
	b2_colorSalmon = 0xFA8072,
	b2_colorSandyBrown = 0xF4A460,
	b2_colorSeaGreen = 0x2E8B57,
	b2_colorSeaShell = 0xFFF5EE,
	b2_colorSienna = 0xA0522D,
	b2_colorSilver = 0xC0C0C0,
	b2_colorSkyBlue = 0x87CEEB,
	b2_colorSlateBlue = 0x6A5ACD,
	b2_colorSlateGray = 0x708090,
	b2_colorSnow = 0xFFFAFA,
	b2_colorSpringGreen = 0x00FF7F,
	b2_colorSteelBlue = 0x4682B4,
	b2_colorTan = 0xD2B48C,
	b2_colorTeal = 0x008080,
	b2_colorThistle = 0xD8BFD8,
	b2_colorTomato = 0xFF6347,
	b2_colorTurquoise = 0x40E0D0,
	b2_colorViolet = 0xEE82EE,
	b2_colorWheat = 0xF5DEB3,
	b2_colorWhite = 0xFFFFFF,
	b2_colorWhiteSmoke = 0xF5F5F5,
	b2_colorYellow = 0xFFFF00,
	b2_colorYellowGreen = 0x9ACD32,

	b2_colorBox2DRed = 0xDC3132,
	b2_colorBox2DBlue = 0x30AEBF,
	b2_colorBox2DGreen = 0x8CC924,
	b2_colorBox2DYellow = 0xFFEE8C
}

alias b2_colorAliceBlue = b2HexColor.b2_colorAliceBlue;
alias b2_colorAntiqueWhite = b2HexColor.b2_colorAntiqueWhite;
alias b2_colorAqua = b2HexColor.b2_colorAqua;
alias b2_colorAquamarine = b2HexColor.b2_colorAquamarine;
alias b2_colorAzure = b2HexColor.b2_colorAzure;
alias b2_colorBeige = b2HexColor.b2_colorBeige;
alias b2_colorBisque = b2HexColor.b2_colorBisque;
alias b2_colorBlack = b2HexColor.b2_colorBlack;
alias b2_colorBlanchedAlmond = b2HexColor.b2_colorBlanchedAlmond;
alias b2_colorBlue = b2HexColor.b2_colorBlue;
alias b2_colorBlueViolet = b2HexColor.b2_colorBlueViolet;
alias b2_colorBrown = b2HexColor.b2_colorBrown;
alias b2_colorBurlywood = b2HexColor.b2_colorBurlywood;
alias b2_colorCadetBlue = b2HexColor.b2_colorCadetBlue;
alias b2_colorChartreuse = b2HexColor.b2_colorChartreuse;
alias b2_colorChocolate = b2HexColor.b2_colorChocolate;
alias b2_colorCoral = b2HexColor.b2_colorCoral;
alias b2_colorCornflowerBlue = b2HexColor.b2_colorCornflowerBlue;
alias b2_colorCornsilk = b2HexColor.b2_colorCornsilk;
alias b2_colorCrimson = b2HexColor.b2_colorCrimson;
alias b2_colorCyan = b2HexColor.b2_colorCyan;
alias b2_colorDarkBlue = b2HexColor.b2_colorDarkBlue;
alias b2_colorDarkCyan = b2HexColor.b2_colorDarkCyan;
alias b2_colorDarkGoldenRod = b2HexColor.b2_colorDarkGoldenRod;
alias b2_colorDarkGray = b2HexColor.b2_colorDarkGray;
alias b2_colorDarkGreen = b2HexColor.b2_colorDarkGreen;
alias b2_colorDarkKhaki = b2HexColor.b2_colorDarkKhaki;
alias b2_colorDarkMagenta = b2HexColor.b2_colorDarkMagenta;
alias b2_colorDarkOliveGreen = b2HexColor.b2_colorDarkOliveGreen;
alias b2_colorDarkOrange = b2HexColor.b2_colorDarkOrange;
alias b2_colorDarkOrchid = b2HexColor.b2_colorDarkOrchid;
alias b2_colorDarkRed = b2HexColor.b2_colorDarkRed;
alias b2_colorDarkSalmon = b2HexColor.b2_colorDarkSalmon;
alias b2_colorDarkSeaGreen = b2HexColor.b2_colorDarkSeaGreen;
alias b2_colorDarkSlateBlue = b2HexColor.b2_colorDarkSlateBlue;
alias b2_colorDarkSlateGray = b2HexColor.b2_colorDarkSlateGray;
alias b2_colorDarkTurquoise = b2HexColor.b2_colorDarkTurquoise;
alias b2_colorDarkViolet = b2HexColor.b2_colorDarkViolet;
alias b2_colorDeepPink = b2HexColor.b2_colorDeepPink;
alias b2_colorDeepSkyBlue = b2HexColor.b2_colorDeepSkyBlue;
alias b2_colorDimGray = b2HexColor.b2_colorDimGray;
alias b2_colorDodgerBlue = b2HexColor.b2_colorDodgerBlue;
alias b2_colorFireBrick = b2HexColor.b2_colorFireBrick;
alias b2_colorFloralWhite = b2HexColor.b2_colorFloralWhite;
alias b2_colorForestGreen = b2HexColor.b2_colorForestGreen;
alias b2_colorFuchsia = b2HexColor.b2_colorFuchsia;
alias b2_colorGainsboro = b2HexColor.b2_colorGainsboro;
alias b2_colorGhostWhite = b2HexColor.b2_colorGhostWhite;
alias b2_colorGold = b2HexColor.b2_colorGold;
alias b2_colorGoldenRod = b2HexColor.b2_colorGoldenRod;
alias b2_colorGray = b2HexColor.b2_colorGray;
alias b2_colorGreen = b2HexColor.b2_colorGreen;
alias b2_colorGreenYellow = b2HexColor.b2_colorGreenYellow;
alias b2_colorHoneyDew = b2HexColor.b2_colorHoneyDew;
alias b2_colorHotPink = b2HexColor.b2_colorHotPink;
alias b2_colorIndianRed = b2HexColor.b2_colorIndianRed;
alias b2_colorIndigo = b2HexColor.b2_colorIndigo;
alias b2_colorIvory = b2HexColor.b2_colorIvory;
alias b2_colorKhaki = b2HexColor.b2_colorKhaki;
alias b2_colorLavender = b2HexColor.b2_colorLavender;
alias b2_colorLavenderBlush = b2HexColor.b2_colorLavenderBlush;
alias b2_colorLawnGreen = b2HexColor.b2_colorLawnGreen;
alias b2_colorLemonChiffon = b2HexColor.b2_colorLemonChiffon;
alias b2_colorLightBlue = b2HexColor.b2_colorLightBlue;
alias b2_colorLightCoral = b2HexColor.b2_colorLightCoral;
alias b2_colorLightCyan = b2HexColor.b2_colorLightCyan;
alias b2_colorLightGoldenRodYellow = b2HexColor.b2_colorLightGoldenRodYellow;
alias b2_colorLightGray = b2HexColor.b2_colorLightGray;
alias b2_colorLightGreen = b2HexColor.b2_colorLightGreen;
alias b2_colorLightPink = b2HexColor.b2_colorLightPink;
alias b2_colorLightSalmon = b2HexColor.b2_colorLightSalmon;
alias b2_colorLightSeaGreen = b2HexColor.b2_colorLightSeaGreen;
alias b2_colorLightSkyBlue = b2HexColor.b2_colorLightSkyBlue;
alias b2_colorLightSlateGray = b2HexColor.b2_colorLightSlateGray;
alias b2_colorLightSteelBlue = b2HexColor.b2_colorLightSteelBlue;
alias b2_colorLightYellow = b2HexColor.b2_colorLightYellow;
alias b2_colorLime = b2HexColor.b2_colorLime;
alias b2_colorLimeGreen = b2HexColor.b2_colorLimeGreen;
alias b2_colorLinen = b2HexColor.b2_colorLinen;
alias b2_colorMagenta = b2HexColor.b2_colorMagenta;
alias b2_colorMaroon = b2HexColor.b2_colorMaroon;
alias b2_colorMediumAquaMarine = b2HexColor.b2_colorMediumAquaMarine;
alias b2_colorMediumBlue = b2HexColor.b2_colorMediumBlue;
alias b2_colorMediumOrchid = b2HexColor.b2_colorMediumOrchid;
alias b2_colorMediumPurple = b2HexColor.b2_colorMediumPurple;
alias b2_colorMediumSeaGreen = b2HexColor.b2_colorMediumSeaGreen;
alias b2_colorMediumSlateBlue = b2HexColor.b2_colorMediumSlateBlue;
alias b2_colorMediumSpringGreen = b2HexColor.b2_colorMediumSpringGreen;
alias b2_colorMediumTurquoise = b2HexColor.b2_colorMediumTurquoise;
alias b2_colorMediumVioletRed = b2HexColor.b2_colorMediumVioletRed;
alias b2_colorMidnightBlue = b2HexColor.b2_colorMidnightBlue;
alias b2_colorMintCream = b2HexColor.b2_colorMintCream;
alias b2_colorMistyRose = b2HexColor.b2_colorMistyRose;
alias b2_colorMoccasin = b2HexColor.b2_colorMoccasin;
alias b2_colorNavajoWhite = b2HexColor.b2_colorNavajoWhite;
alias b2_colorNavy = b2HexColor.b2_colorNavy;
alias b2_colorOldLace = b2HexColor.b2_colorOldLace;
alias b2_colorOlive = b2HexColor.b2_colorOlive;
alias b2_colorOliveDrab = b2HexColor.b2_colorOliveDrab;
alias b2_colorOrange = b2HexColor.b2_colorOrange;
alias b2_colorOrangeRed = b2HexColor.b2_colorOrangeRed;
alias b2_colorOrchid = b2HexColor.b2_colorOrchid;
alias b2_colorPaleGoldenRod = b2HexColor.b2_colorPaleGoldenRod;
alias b2_colorPaleGreen = b2HexColor.b2_colorPaleGreen;
alias b2_colorPaleTurquoise = b2HexColor.b2_colorPaleTurquoise;
alias b2_colorPaleVioletRed = b2HexColor.b2_colorPaleVioletRed;
alias b2_colorPapayaWhip = b2HexColor.b2_colorPapayaWhip;
alias b2_colorPeachPuff = b2HexColor.b2_colorPeachPuff;
alias b2_colorPeru = b2HexColor.b2_colorPeru;
alias b2_colorPink = b2HexColor.b2_colorPink;
alias b2_colorPlum = b2HexColor.b2_colorPlum;
alias b2_colorPowderBlue = b2HexColor.b2_colorPowderBlue;
alias b2_colorPurple = b2HexColor.b2_colorPurple;
alias b2_colorRebeccaPurple = b2HexColor.b2_colorRebeccaPurple;
alias b2_colorRed = b2HexColor.b2_colorRed;
alias b2_colorRosyBrown = b2HexColor.b2_colorRosyBrown;
alias b2_colorRoyalBlue = b2HexColor.b2_colorRoyalBlue;
alias b2_colorSaddleBrown = b2HexColor.b2_colorSaddleBrown;
alias b2_colorSalmon = b2HexColor.b2_colorSalmon;
alias b2_colorSandyBrown = b2HexColor.b2_colorSandyBrown;
alias b2_colorSeaGreen = b2HexColor.b2_colorSeaGreen;
alias b2_colorSeaShell = b2HexColor.b2_colorSeaShell;
alias b2_colorSienna = b2HexColor.b2_colorSienna;
alias b2_colorSilver = b2HexColor.b2_colorSilver;
alias b2_colorSkyBlue = b2HexColor.b2_colorSkyBlue;
alias b2_colorSlateBlue = b2HexColor.b2_colorSlateBlue;
alias b2_colorSlateGray = b2HexColor.b2_colorSlateGray;
alias b2_colorSnow = b2HexColor.b2_colorSnow;
alias b2_colorSpringGreen = b2HexColor.b2_colorSpringGreen;
alias b2_colorSteelBlue = b2HexColor.b2_colorSteelBlue;
alias b2_colorTan = b2HexColor.b2_colorTan;
alias b2_colorTeal = b2HexColor.b2_colorTeal;
alias b2_colorThistle = b2HexColor.b2_colorThistle;
alias b2_colorTomato = b2HexColor.b2_colorTomato;
alias b2_colorTurquoise = b2HexColor.b2_colorTurquoise;
alias b2_colorViolet = b2HexColor.b2_colorViolet;
alias b2_colorWheat = b2HexColor.b2_colorWheat;
alias b2_colorWhite = b2HexColor.b2_colorWhite;
alias b2_colorWhiteSmoke = b2HexColor.b2_colorWhiteSmoke;
alias b2_colorYellow = b2HexColor.b2_colorYellow;
alias b2_colorYellowGreen = b2HexColor.b2_colorYellowGreen;
alias b2_colorBox2DRed = b2HexColor.b2_colorBox2DRed;
alias b2_colorBox2DBlue = b2HexColor.b2_colorBox2DBlue;
alias b2_colorBox2DGreen = b2HexColor.b2_colorBox2DGreen;
alias b2_colorBox2DYellow = b2HexColor.b2_colorBox2DYellow;


/// This struct holds callbacks you can implement to draw a Box2D world.
/// This structure should be zero initialized.
/// @ingroup world
struct b2DebugDraw {
	/// Draw a closed polygon provided in CCW order.
	void function(const(b2Vec2)* vertices, int vertexCount, b2HexColor color, void* context) DrawPolygonFcn;

	/// Draw a solid closed polygon provided in CCW order.
	void function(b2Transform transform, const(b2Vec2)* vertices, int vertexCount, float radius, b2HexColor color, void* context) DrawSolidPolygonFcn;

	/// Draw a circle.
	void function(b2Vec2 center, float radius, b2HexColor color, void* context) DrawCircleFcn;

	/// Draw a solid circle.
	void function(b2Transform transform, float radius, b2HexColor color, void* context) DrawSolidCircleFcn;

	/// Draw a solid capsule.
	void function(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context) DrawSolidCapsuleFcn;

	/// Draw a line segment.
	void function(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context) DrawSegmentFcn;

	/// Draw a transform. Choose your own length scale.
	void function(b2Transform transform, void* context) DrawTransformFcn;

	/// Draw a point.
	void function(b2Vec2 p, float size, b2HexColor color, void* context) DrawPointFcn;

	/// Draw a string in world space
	void function(b2Vec2 p, const(char)* s, b2HexColor color, void* context) DrawStringFcn;

	/// Bounds to use if restricting drawing to a rectangular region
	b2AABB drawingBounds;

	/// Option to draw shapes
	bool drawShapes;

	/// Option to draw joints
	bool drawJoints;

	/// Option to draw additional information for joints
	bool drawJointExtras;

	/// Option to draw the bounding boxes for shapes
	bool drawBounds;

	/// Option to draw the mass and center of mass of dynamic bodies
	bool drawMass;

	/// Option to draw body names
	bool drawBodyNames;

	/// Option to draw contact points
	bool drawContacts;

	/// Option to visualize the graph coloring used for contacts and joints
	bool drawGraphColors;

	/// Option to draw contact normals
	bool drawContactNormals;

	/// Option to draw contact normal impulses
	bool drawContactImpulses;

	/// Option to draw contact feature ids
	bool drawContactFeatures;

	/// Option to draw contact friction impulses
	bool drawFrictionImpulses;

	/// Option to draw islands as bounding boxes
	bool drawIslands;

	/// User context that is passed as an argument to drawing callback functions
	void* context;
}

b2WorldDef b2DefaultWorldDef()
{
	b2WorldDef def;
	def.gravity.x = 0.0f;
	def.gravity.y = -10.0f;
	def.hitEventThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.contactSpeed = 3.0f * b2_lengthUnitsPerMeter;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 10.0f;

	// 400 meters per second, faster than the speed of sound
	def.maximumLinearSpeed = 400.0f * b2_lengthUnitsPerMeter;

	def.enableSleep = true;
	def.enableContinuous = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2BodyDef b2DefaultBodyDef()
{
	b2BodyDef def;
	def.type = b2_staticBody;
	def.rotation = b2Rot_identity;
	def.sleepThreshold = 0.05f * b2_lengthUnitsPerMeter;
	def.gravityScale = 1.0f;
	def.enableSleep = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2Filter b2DefaultFilter()
{
	b2Filter filter = { B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS, 0 };
	return filter;
}

b2QueryFilter b2DefaultQueryFilter()
{
	b2QueryFilter filter = { B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS };
	return filter;
}

b2ShapeDef b2DefaultShapeDef()
{
	b2ShapeDef def;
	def.material.friction = 0.6f;
	def.density = 1.0f;
	def.filter = b2DefaultFilter();
	def.updateBodyMass = true;
	def.invokeContactCreation = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2SurfaceMaterial b2DefaultSurfaceMaterial()
{
	b2SurfaceMaterial material = {
		friction: 0.6f,
	};

	return material;
}

b2ChainDef b2DefaultChainDef()
{
	static b2SurfaceMaterial defaultMaterial = {
		friction: 0.6f,
	};

	b2ChainDef def;
	def.materials = &defaultMaterial;
	def.materialCount = 1;
	def.filter = b2DefaultFilter();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

private void b2EmptyDrawPolygon(const(b2Vec2)* vertices, int vertexCount, b2HexColor color, void* context)
{
	// B2_UNUSED( vertices, vertexCount, color, context );
}

private void b2EmptyDrawSolidPolygon(b2Transform transform, const(b2Vec2)* vertices, int vertexCount, float radius, b2HexColor color, void* context)
{
	// B2_UNUSED( transform, vertices, vertexCount, radius, color, context );
}

private void b2EmptyDrawCircle(b2Vec2 center, float radius, b2HexColor color, void* context)
{
	// B2_UNUSED( center, radius, color, context );
}

private void b2EmptyDrawSolidCircle(b2Transform transform, float radius, b2HexColor color, void* context)
{
	// B2_UNUSED( transform, radius, color, context );
}

private void b2EmptyDrawSolidCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context)
{
	// B2_UNUSED( p1, p2, radius, color, context );
}

private void b2EmptyDrawSegment(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context)
{
	// B2_UNUSED( p1, p2, color, context );
}

private void b2EmptyDrawTransform(b2Transform transform, void* context)
{
	// B2_UNUSED( transform, context );
}

private void b2EmptyDrawPoint(b2Vec2 p, float size, b2HexColor color, void* context)
{
	// B2_UNUSED( p, size, color, context );
}

private void b2EmptyDrawString(b2Vec2 p, const(char)* s, b2HexColor color, void* context)
{
	// B2_UNUSED( p, s, color, context );
}

b2DebugDraw b2DefaultDebugDraw()
{
	b2DebugDraw draw;

	// These allow the user to skip some implementations and not hit null exceptions.
	draw.DrawPolygonFcn = &b2EmptyDrawPolygon;
	draw.DrawSolidPolygonFcn = &b2EmptyDrawSolidPolygon;
	draw.DrawCircleFcn = &b2EmptyDrawCircle;
	draw.DrawSolidCircleFcn = &b2EmptyDrawSolidCircle;
	draw.DrawSolidCapsuleFcn = &b2EmptyDrawSolidCapsule;
	draw.DrawSegmentFcn = &b2EmptyDrawSegment;
	draw.DrawTransformFcn = &b2EmptyDrawTransform;
	draw.DrawPointFcn = &b2EmptyDrawPoint;
	draw.DrawStringFcn = &b2EmptyDrawString;

	draw.drawingBounds.lowerBound = b2Vec2( -float.max, -float.max );
	draw.drawingBounds.upperBound = b2Vec2( float.max, float.max );

	draw.drawShapes = true;
	
	return draw;
}
