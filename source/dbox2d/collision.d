module dbox2d.collision;
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

//#pragma once

public import dbox2d.base;
import dbox2d.geometry;
import dbox2d.types;
import dbox2d.core;
import dbox2d.constants;
import dbox2d.aabb;
public import dbox2d.math_functions;
public import core.stdc.stdlib;
import core.stdc.stdint;
import dbox2d.dynamic_tree;
import core.stdc.string;
import dbox2d.collision;
import dbox2d.manifold;
import dbox2d.hull;
import dbox2d.geometry;
/**
 * @defgroup geometry Geometry
 * @brief Geometry types and algorithms
 *
 * Definitions of circles, capsules, segments, and polygons. Various algorithms to compute hulls, mass properties, and so on.
 * Functions should take the shape as the first argument to assist editor auto-complete.
 * @{
 */

/// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
/// don't use more vertices.
enum B2_MAX_POLYGON_VERTICES = 8;

/// Low level ray cast input data
struct b2RayCastInput {
	/// Start point of the ray cast
	b2Vec2 origin;

	/// Translation of the ray cast
	b2Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1
	float maxFraction = 0;
}

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
/// You can provide between 1 and B2_MAX_POLYGON_VERTICES and a radius.
struct b2ShapeProxy {
	/// The point cloud
	b2Vec2[B2_MAX_POLYGON_VERTICES] points;

	/// The number of points. Must be greater than 0.
	int count;

	/// The external radius of the point cloud. May be zero.
	float radius = 0;
}

/// Low level shape cast input in generic form. This allows casting an arbitrary point
/// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
struct b2ShapeCastInput {
	/// A generic shape
	b2ShapeProxy proxy;

	/// The translation of the shape cast
	b2Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1
	float maxFraction = 0;

	/// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
	bool canEncroach;
}

/// Low level ray cast or shape-cast output data. Returns a zero fraction and normal in the case of initial overlap.
struct b2CastOutput {
	/// The surface normal at the hit point
	b2Vec2 normal;

	/// The surface hit point
	b2Vec2 point;

	/// The fraction of the input translation at collision
	float fraction = 0;

	/// The number of iterations used
	int iterations;

	/// Did the cast hit?
	bool hit;
}

/// This holds the mass data computed for a shape.
struct b2MassData {
	/// The mass of the shape, usually in kilograms.
	float mass = 0;

	/// The position of the shape's centroid relative to the shape's origin.
	b2Vec2 center;

	/// The rotational inertia of the shape about the shape center.
	float rotationalInertia = 0;
}

/// A solid circle
struct b2Circle {
	/// The local center
	b2Vec2 center;

	/// The radius
	float radius = 0;
}

/// A solid capsule can be viewed as two semicircles connected
/// by a rectangle.
struct b2Capsule {
	/// Local center of the first semicircle
	b2Vec2 center1;

	/// Local center of the second semicircle
	b2Vec2 center2;

	/// The radius of the semicircles
	float radius = 0;
}

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
/// In most cases you should not need many vertices for a convex polygon.
/// @warning DO NOT fill this out manually, instead use a helper function like
/// b2MakePolygon or b2MakeBox.
struct b2Polygon {
	/// The polygon vertices
	b2Vec2[B2_MAX_POLYGON_VERTICES] vertices;

	/// The outward normal vectors of the polygon sides
	b2Vec2[B2_MAX_POLYGON_VERTICES] normals;

	/// The centroid of the polygon
	b2Vec2 centroid;

	/// The external radius for rounded polygons
	float radius = 0;

	/// The number of polygon vertices
	int count;
}

/// A line segment with two-sided collision.
struct b2Segment {
	/// The first point
	b2Vec2 point1;

	/// The second point
	b2Vec2 point2;
}

/// A line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
struct b2ChainSegment {
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Segment segment;

	/// The head ghost vertex
	b2Vec2 ghost2;

	/// The owning chain shape index (internal usage only)
	int chainId;
}

/// Validate ray cast input data (NaN, etc)
bool b2IsValidRay(const(b2RayCastInput)* input);

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
b2Polygon b2MakePolygon(const(b2Hull)* hull, float radius);

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
b2Polygon b2MakeOffsetPolygon(const(b2Hull)* hull, b2Vec2 position, b2Rot rotation);

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
b2Polygon b2MakeOffsetRoundedPolygon(const(b2Hull)* hull, b2Vec2 position, b2Rot rotation, float radius);

/// Make a square polygon, bypassing the need for a convex hull.
/// @param halfWidth the half-width
b2Polygon b2MakeSquare(float halfWidth);

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
// b2Polygon b2MakeBox(float halfWidth, float halfHeight);

/// Make a rounded box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param radius the radius of the rounded extension
b2Polygon b2MakeRoundedBox(float halfWidth, float halfHeight, float radius);

/// Make an offset box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param center the local center of the box
/// @param rotation the local rotation of the box
b2Polygon b2MakeOffsetBox(float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation);

/// Make an offset rounded box, bypassing the need for a convex hull.
/// @param halfWidth the half-width (x-axis)
/// @param halfHeight the half-height (y-axis)
/// @param center the local center of the box
/// @param rotation the local rotation of the box
/// @param radius the radius of the rounded extension
b2Polygon b2MakeOffsetRoundedBox(float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation, float radius);

/// Transform a polygon. This is useful for transferring a shape from one body to another.
b2Polygon b2TransformPolygon(b2Transform transform, const(b2Polygon)* polygon);

/// Compute mass properties of a circle
// b2MassData b2ComputeCircleMass(const(b2Circle)* shape, float density);

/// Compute mass properties of a capsule
// b2MassData b2ComputeCapsuleMass(const(b2Capsule)* shape, float density);

/// Compute mass properties of a polygon
// b2MassData b2ComputePolygonMass(const(b2Polygon)* shape, float density);

/// Compute the bounding box of a transformed circle
// b2AABB b2ComputeCircleAABB(const(b2Circle)* shape, b2Transform transform);

/// Compute the bounding box of a transformed capsule
// b2AABB b2ComputeCapsuleAABB(const(b2Capsule)* shape, b2Transform transform);

/// Compute the bounding box of a transformed polygon
// b2AABB b2ComputePolygonAABB(const(b2Polygon)* shape, b2Transform transform);

/// Compute the bounding box of a transformed line segment
// b2AABB b2ComputeSegmentAABB(const(b2Segment)* shape, b2Transform transform);

/// Test a point for overlap with a circle in local space
// bool b2PointInCircle(const(b2Circle)* shape, b2Vec2 point);

/// Test a point for overlap with a capsule in local space
// bool b2PointInCapsule(const(b2Capsule)* shape, b2Vec2 point);

/// Test a point for overlap with a convex polygon in local space
// bool b2PointInPolygon(const(b2Polygon)* shape, b2Vec2 point);

/// Ray cast versus circle shape in local space.
// b2CastOutput b2RayCastCircle(const(b2Circle)* shape, const(b2RayCastInput)* input);

/// Ray cast versus capsule shape in local space.
// b2CastOutput b2RayCastCapsule(const(b2Capsule)* shape, const(b2RayCastInput)* input);

/// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
// b2CastOutput b2RayCastSegment(const(b2Segment)* shape, const(b2RayCastInput)* input, bool oneSided);

/// Ray cast versus polygon shape in local space.
// b2CastOutput b2RayCastPolygon(const(b2Polygon)* shape, const(b2RayCastInput)* input);

/// Shape cast versus a circle.
// b2CastOutput b2ShapeCastCircle(const(b2Circle)* shape, const(b2ShapeCastInput)* input);

/// Shape cast versus a capsule.
// b2CastOutput b2ShapeCastCapsule(const(b2Capsule)* shape, const(b2ShapeCastInput)* input);

/// Shape cast versus a line segment.
// b2CastOutput b2ShapeCastSegment(const(b2Segment)* shape, const(b2ShapeCastInput)* input);

/// Shape cast versus a convex polygon.
// b2CastOutput b2ShapeCastPolygon(const(b2Polygon)* shape, const(b2ShapeCastInput)* input);

/// A convex hull. Used to create convex polygons.
/// @warning Do not modify these values directly, instead use b2ComputeHull()
struct b2Hull {
	/// The final points of the hull
	b2Vec2[B2_MAX_POLYGON_VERTICES] points;

	/// The number of points
	int count;
}

/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - less than 3 points
/// - more than B2_MAX_POLYGON_VERTICES points
/// This welds close points and removes collinear points.
/// @warning Do not modify a hull once it has been computed
// b2Hull b2ComputeHull(const(b2Vec2)* points, int count);

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
// bool b2ValidateHull(const(b2Hull)* hull);

/**@}*/

/**
 * @defgroup distance Distance
 * Functions for computing the distance between shapes.
 *
 * These are advanced functions you can use to perform distance calculations. There
 * are functions for computing the closest points between shapes, doing linear shape casts,
 * and doing rotational shape casts. The latter is called time of impact (TOI).
 * @{
 */

/// Result of computing the distance between two line segments
struct b2SegmentDistanceResult {
	/// The closest point on the first segment
	b2Vec2 closest1;

	/// The closest point on the second segment
	b2Vec2 closest2;

	/// The barycentric coordinate on the first segment
	float fraction1 = 0;

	/// The barycentric coordinate on the second segment
	float fraction2 = 0;

	/// The squared distance between the closest points
	float distanceSquared = 0;
}

/// Compute the distance between two line segments, clamping at the end points if needed.
// b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2);

/// Used to warm start the GJK simplex. If you call this function multiple times with nearby
/// transforms this might improve performance. Otherwise you can zero initialize this.
/// The distance cache must be initialized to zero on the first call.
/// Users should generally just zero initialize this structure for each call.
struct b2SimplexCache {
	/// The number of stored simplex points
	ushort count;

	/// The cached simplex indices on shape A
	ubyte[3] indexA;

	/// The cached simplex indices on shape B
	ubyte[3] indexB;
}

const(b2SimplexCache) b2_emptySimplexCache;

/// Input for b2ShapeDistance
struct b2DistanceInput {
	/// The proxy for shape A
	b2ShapeProxy proxyA;

	/// The proxy for shape B
	b2ShapeProxy proxyB;

	/// The world transform for shape A
	b2Transform transformA;

	/// The world transform for shape B
	b2Transform transformB;

	/// Should the proxy radius be considered?
	bool useRadii;
}

/// Output for b2ShapeDistance
struct b2DistanceOutput {
	b2Vec2 pointA;	  ///< Closest point on shapeA
	b2Vec2 pointB;	  ///< Closest point on shapeB
	b2Vec2 normal;	  ///< Normal vector that points from A to B. Invalid if distance is zero.
	float distance = 0;	  ///< The final distance, zero if overlapped
	int iterations;	  ///< Number of GJK iterations used
	int simplexCount; ///< The number of simplexes stored in the simplex array
}

/// Simplex vertex for debugging the GJK algorithm
struct b2SimplexVertex {
	b2Vec2 wA;	///< support point in proxyA
	b2Vec2 wB;	///< support point in proxyB
	b2Vec2 w;	///< wB - wA
	float a = 0;	///< barycentric coordinate for closest point
	int indexA; ///< wA index
	int indexB; ///< wB index
}

/// Simplex from the GJK algorithm
struct b2Simplex {
	b2SimplexVertex v1, v2, v3; ///< vertices
	int count;					///< number of valid vertices
}

/// Compute the closest points between two shapes represented as point clouds.
/// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
// b2DistanceOutput b2ShapeDistance(const(b2DistanceInput)* input, b2SimplexCache* cache, b2Simplex* simplexes, int simplexCapacity);

/// Input parameters for b2ShapeCast
struct b2ShapeCastPairInput {
	b2ShapeProxy proxyA;	///< The proxy for shape A
	b2ShapeProxy proxyB;	///< The proxy for shape B
	b2Transform transformA; ///< The world transform for shape A
	b2Transform transformB; ///< The world transform for shape B
	b2Vec2 translationB;	///< The translation of shape B
	float maxFraction = 0;		///< The fraction of the translation to consider, typically 1
	bool canEncroach;		///< Allows shapes with a radius to move slightly closer if already touching
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
/// Initially touching shapes are treated as a miss.
// b2CastOutput b2ShapeCast(const(b2ShapeCastPairInput)* input);

/// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.
// b2ShapeProxy b2MakeProxy(const(b2Vec2)* points, int count, float radius);

/// Make a proxy with a transform. This is a deep copy of the points.
b2ShapeProxy b2MakeOffsetProxy(const(b2Vec2)* points, int count, float radius, b2Vec2 position, b2Rot rotation);

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
struct b2Sweep {
	b2Vec2 localCenter; ///< Local center of mass position
	b2Vec2 c1;			///< Starting center of mass world position
	b2Vec2 c2;			///< Ending center of mass world position
	b2Rot q1;			///< Starting world rotation
	b2Rot q2;			///< Ending world rotation
}

/// Evaluate the transform sweep at a specific time.
b2Transform b2GetSweepTransform(const(b2Sweep)* sweep, float time);

/// Time of impact input
struct b2TOIInput {
	b2ShapeProxy proxyA; ///< The proxy for shape A
	b2ShapeProxy proxyB; ///< The proxy for shape B
	b2Sweep sweepA;		 ///< The movement of shape A
	b2Sweep sweepB;		 ///< The movement of shape B
	float maxFraction = 0;	 ///< Defines the sweep interval [0, maxFraction]
}

/// Describes the TOI output
enum b2TOIState {
	b2_toiStateUnknown,
	b2_toiStateFailed,
	b2_toiStateOverlapped,
	b2_toiStateHit,
	b2_toiStateSeparated
}
alias b2_toiStateUnknown = b2TOIState.b2_toiStateUnknown;
alias b2_toiStateFailed = b2TOIState.b2_toiStateFailed;
alias b2_toiStateOverlapped = b2TOIState.b2_toiStateOverlapped;
alias b2_toiStateHit = b2TOIState.b2_toiStateHit;
alias b2_toiStateSeparated = b2TOIState.b2_toiStateSeparated;


/// Time of impact output
struct b2TOIOutput {
	/// The type of result
	b2TOIState state;

	/// The hit point
	b2Vec2 point;

	/// The hit normal
	b2Vec2 normal;

	/// The sweep time of the collision 
	float fraction = 0;
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
// b2TOIOutput b2TimeOfImpact(const(b2TOIInput)* input);

/**@}*/

/**
 * @defgroup collision Collision
 * @brief Functions for colliding pairs of shapes
 * @{
 */

/// A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box2D uses speculative collision so some contact points may be separated.
/// You may use the totalNormalImpulse to determine if there was an interaction during
/// the time step.
struct b2ManifoldPoint {
	/// Location of the contact point in world space. Subject to precision loss at large coordinates.
	/// @note Should only be used for debugging.
	b2Vec2 point;

	/// Location of the contact point relative to shapeA's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	b2Vec2 anchorA;

	/// Location of the contact point relative to shapeB's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	b2Vec2 anchorB;

	/// The separation of the contact point, negative if penetrating
	float separation = 0;

	/// The impulse along the manifold normal vector.
	float normalImpulse = 0;

	/// The friction impulse
	float tangentImpulse = 0;

	/// The total normal impulse applied across sub-stepping and restitution. This is important
	/// to identify speculative contact points that had an interaction in the time step.
	float totalNormalImpulse = 0;

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	float normalVelocity = 0;

	/// Uniquely identifies a contact point between two shapes
	ushort id;

	/// Did this contact point exist the previous step?
	bool persisted;
}

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box2D uses speculative collision so some contact points may be separated.
struct b2Manifold {
	/// The unit normal vector in world space, points from shape A to bodyB
	b2Vec2 normal;

	/// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	float rollingImpulse = 0;

	/// The manifold points, up to two are possible in 2D
	b2ManifoldPoint[2] points;

	/// The number of contacts points, will be 0, 1, or 2
	int pointCount;

}

/// Compute the contact manifold between two circles
// b2Manifold b2CollideCircles(const(b2Circle)* circleA, b2Transform xfA, const(b2Circle)* circleB, b2Transform xfB);

/// Compute the contact manifold between a capsule and circle
// b2Manifold b2CollideCapsuleAndCircle(const(b2Capsule)* capsuleA, b2Transform xfA, const(b2Circle)* circleB, b2Transform xfB);

/// Compute the contact manifold between an segment and a circle
// b2Manifold b2CollideSegmentAndCircle(const(b2Segment)* segmentA, b2Transform xfA, const(b2Circle)* circleB, b2Transform xfB);

/// Compute the contact manifold between a polygon and a circle
// b2Manifold b2CollidePolygonAndCircle(const(b2Polygon)* polygonA, b2Transform xfA, const(b2Circle)* circleB, b2Transform xfB);

/// Compute the contact manifold between a capsule and circle
// b2Manifold b2CollideCapsules(const(b2Capsule)* capsuleA, b2Transform xfA, const(b2Capsule)* capsuleB, b2Transform xfB);

/// Compute the contact manifold between an segment and a capsule
// b2Manifold b2CollideSegmentAndCapsule(const(b2Segment)* segmentA, b2Transform xfA, const(b2Capsule)* capsuleB, b2Transform xfB);

/// Compute the contact manifold between a polygon and capsule
// b2Manifold b2CollidePolygonAndCapsule(const(b2Polygon)* polygonA, b2Transform xfA, const(b2Capsule)* capsuleB, b2Transform xfB);

/// Compute the contact manifold between two polygons
// b2Manifold b2CollidePolygons(const(b2Polygon)* polygonA, b2Transform xfA, const(b2Polygon)* polygonB, b2Transform xfB);

/// Compute the contact manifold between an segment and a polygon
// b2Manifold b2CollideSegmentAndPolygon(const(b2Segment)* segmentA, b2Transform xfA, const(b2Polygon)* polygonB, b2Transform xfB);

/// Compute the contact manifold between a chain segment and a circle
// b2Manifold b2CollideChainSegmentAndCircle(const(b2ChainSegment)* segmentA, b2Transform xfA, const(b2Circle)* circleB, b2Transform xfB);

/// Compute the contact manifold between a chain segment and a capsule
// b2Manifold b2CollideChainSegmentAndCapsule(const(b2ChainSegment)* segmentA, b2Transform xfA, const(b2Capsule)* capsuleB, b2Transform xfB, b2SimplexCache* cache);

/// Compute the contact manifold between a chain segment and a rounded polygon
// b2Manifold b2CollideChainSegmentAndPolygon(const(b2ChainSegment)* segmentA, b2Transform xfA, const(b2Polygon)* polygonB, b2Transform xfB, b2SimplexCache* cache);

/**@}*/

/**
 * @defgroup tree Dynamic Tree
 * The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
 *
 * Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
 * This data structure may have uses in games for organizing other geometry data and may be used independently
 * of Box2D rigid body simulation.
 *
 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as AABB queries and ray casts. Leaf nodes are proxies
 * with an AABB. These are used to hold a user collision object.
 * Nodes are pooled and relocatable, so I use node indices rather than pointers.
 * The dynamic tree is made available for advanced users that would like to use it to organize
 * spatial game data besides rigid bodies.
 * @{
 */



/// These are performance results returned by dynamic tree queries.
struct b2TreeStats {
	/// Number of internal nodes visited during the query
	int nodeVisits;

	/// Number of leaf nodes visited during the query
	int leafVisits;
}

/// Constructing the tree initializes the node pool.
// b2DynamicTree b2DynamicTree_Create();

/// Destroy the tree, freeing the node pool.
// void b2DynamicTree_Destroy(b2DynamicTree* tree);

/// Create a proxy. Provide an AABB and a userData value.
// int b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, ulong categoryBits, ulong userData);

/// Destroy a proxy. This asserts if the id is invalid.
// void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int proxyId);

/// Move a proxy to a new AABB by removing and reinserting into the tree.
// void b2DynamicTree_MoveProxy(b2DynamicTree* tree, int proxyId, b2AABB aabb);

/// Enlarge a proxy and enlarge ancestors as necessary.
// void b2DynamicTree_EnlargeProxy(b2DynamicTree* tree, int proxyId, b2AABB aabb);

/// Modify the category bits on a proxy. This is an expensive operation.
// void b2DynamicTree_SetCategoryBits(b2DynamicTree* tree, int proxyId, ulong categoryBits);

/// Get the category bits on a proxy.
// uint64_t b2DynamicTree_GetCategoryBits(b2DynamicTree* tree, int proxyId);

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
alias b2TreeQueryCallbackFcn = bool function( int proxyId, uint64_t userData, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
///	@return performance data
// b2TreeStats b2DynamicTree_Query(b2DynamicTree* tree, b2AABB aabb, ulong maskBits, b2TreeQueryCallbackFcn callback, void* context);

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
alias b2TreeRayCastCallbackFcn = float function(b2RayCastInput* input, int proxyId, uint64_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
///	However, this filtering may be approximate, so the user should still apply filtering to results.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
/// @param maskBits mask bit hint: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the ray
/// @param context user context that is passed to the callback
///	@return performance data
// b2TreeStats b2DynamicTree_RayCast(b2DynamicTree* tree, b2RayCastInput* input, ulong maskBits, b2TreeRayCastCallbackFcn callback, void* context);

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
alias b2TreeShapeCastCallbackFcn = float function(b2ShapeCastInput* input, int proxyId, ulong userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the shape
/// @param context user context that is passed to the callback
///	@return performance data
// b2TreeStats b2DynamicTree_ShapeCast(b2DynamicTree* tree, b2ShapeCastInput* input, ulong maskBits, 
	// b2TreeShapeCastCallbackFcn callback, void* context);

/// Get the height of the binary tree.
// int b2DynamicTree_GetHeight(const(b2DynamicTree)* tree);

/// Get the ratio of the sum of the node areas to the root area.
// float b2DynamicTree_GetAreaRatio(const(b2DynamicTree)* tree);

/// Get the bounding box that contains the entire tree
// b2AABB b2DynamicTree_GetRootBounds(const(b2DynamicTree)* tree);

/// Get the number of proxies created
// int b2DynamicTree_GetProxyCount(const(b2DynamicTree)* tree);

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
// int b2DynamicTree_Rebuild(b2DynamicTree* tree, bool fullBuild);

/// Get the number of bytes used by this tree
// int b2DynamicTree_GetByteCount(const(b2DynamicTree)* tree);

/// Get proxy user data
// uint64_t b2DynamicTree_GetUserData(const(b2DynamicTree)* tree, int proxyId);

/// Get the AABB of a proxy
// b2AABB b2DynamicTree_GetAABB(const(b2DynamicTree)* tree, int proxyId);

/// Validate this tree. For testing.
// void b2DynamicTree_Validate(const(b2DynamicTree)* tree);

/// Validate this tree has no enlarged AABBs. For testing.
// void b2DynamicTree_ValidateNoEnlarged(const(b2DynamicTree)* tree);

/**@}*/

/**
 * @defgroup character Character mover
 * Character movement solver
 * @{
 */

/// These are the collision planes returned from b2World_CollideMover
struct b2PlaneResult {
	/// The collision plane between the mover and a convex shape
	b2Plane plane;

	// The collision point on the shape.
	b2Vec2 point;

	/// Did the collision register a hit? If not this plane should be ignored.
	bool hit;
}

/// These are collision planes that can be fed to b2SolvePlanes. Normally
/// this is assembled by the user from plane results in b2PlaneResult
struct b2CollisionPlane {
	/// The collision plane between the mover and some shape
	b2Plane plane;

	/// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
	/// make the plane collision soft. Usually in meters.
	float pushLimit = 0;

	/// The push on the mover determined by b2SolvePlanes. Usually in meters.
	float push = 0;

	/// Indicates if b2ClipVector should clip against this plane. Should be false for soft collision.
	bool clipVelocity;
}

/// Result returned by b2SolvePlanes
struct b2PlaneSolverResult {
	/// The translation of the mover
	b2Vec2 translation;

	/// The number of iterations used by the plane solver. For diagnostics.
	int iterationCount;
}

/// Solves the position of a mover that satisfies the given collision planes.
/// @param targetDelta the desired movement from the position used to generate the collision planes
/// @param planes the collision planes
/// @param count the number of collision planes
b2PlaneSolverResult b2SolvePlanes(b2Vec2 targetDelta, b2CollisionPlane* planes, int count);

/// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
/// set to false are skipped.
b2Vec2 b2ClipVector(b2Vec2 vector, const(b2CollisionPlane)* planes, int count);

/**@}*/


enum B2_TREE_STACK_SIZE = 1024;

// todo externalize this to visualize internal nodes and speed up FindPairs

// A node in the dynamic tree.

private b2TreeNode b2_defaultTreeNode = {
	aabb: { { 0.0f, 0.0f }, { 0.0f, 0.0f } },
	categoryBits: B2_DEFAULT_CATEGORY_BITS,
	children:
		{
			child1: B2_NULL_INDEX,
			child2: B2_NULL_INDEX,
		},
	parent: B2_NULL_INDEX,
	height: 0,
	flags: b2_allocatedNode,
};

private bool b2IsLeaf(const(b2TreeNode)* node)
{
	return cast(bool)(node.flags & b2_leafNode);
}

private bool b2IsAllocated(const(b2TreeNode)* node)
{
	return cast(bool)(node.flags & b2_allocatedNode);
}

private ushort b2MaxUInt16(ushort a, ushort b)
{
	return a > b ? a : b;
}

// b2DynamicTree b2DynamicTree_Create()
// {
// 	b2DynamicTree tree = void;
// 	tree.root = B2_NULL_INDEX;

// 	tree.nodeCapacity = 16;
// 	tree.nodeCount = 0;
// 	tree.nodes = cast(b2TreeNode*)b2Alloc( cast(int)(tree.nodeCapacity * b2TreeNode.sizeof) );
// 	memset( tree.nodes, 0, tree.nodeCapacity * b2TreeNode.sizeof );

// 	// Build a linked list for the free list.
// 	for ( int i = 0; i < tree.nodeCapacity - 1; ++i )
// 	{
// 		tree.nodes[i].next = i + 1;
// 	}

// 	tree.nodes[tree.nodeCapacity - 1].next = B2_NULL_INDEX;
// 	tree.freeList = 0;

// 	tree.proxyCount = 0;

// 	tree.leafIndices = null;
// 	tree.leafBoxes = null;
// 	tree.leafCenters = null;
// 	tree.binIndices = null;
// 	tree.rebuildCapacity = 0;

// 	return tree;
// }

// void b2DynamicTree_Destroy(b2DynamicTree* tree)
// {
// 	b2Free( tree.nodes, cast(int)(tree.nodeCapacity * b2TreeNode.sizeof) );
// 	b2Free( tree.leafIndices, cast(int)(tree.rebuildCapacity * int.sizeof) );
// 	b2Free( tree.leafBoxes, cast(int)(tree.rebuildCapacity * b2AABB.sizeof) );
// 	b2Free( tree.leafCenters, cast(int)(tree.rebuildCapacity * b2Vec2.sizeof) );
// 	b2Free( tree.binIndices, cast(int)(tree.rebuildCapacity * int.sizeof) );

// 	memset( tree, 0, b2DynamicTree.sizeof );
// }

// Allocate a node from the pool. Grow the pool if necessary.
private int b2AllocateNode(b2DynamicTree* tree)
{
	// Expand the node pool as needed.
	if ( tree.freeList == B2_NULL_INDEX )
	{
		B2_ASSERT( tree.nodeCount == tree.nodeCapacity );

		// The free list is empty. Rebuild a bigger pool.
		b2TreeNode* oldNodes = tree.nodes;
		int oldCapacity = tree.nodeCapacity;
		tree.nodeCapacity += oldCapacity >> 1;
		tree.nodes = cast(b2TreeNode*)b2Alloc( cast(int)(tree.nodeCapacity * b2TreeNode.sizeof) );
		B2_ASSERT( oldNodes != null );
		memcpy( tree.nodes, oldNodes, tree.nodeCount * b2TreeNode.sizeof );
		memset( tree.nodes + tree.nodeCount, 0, ( tree.nodeCapacity - tree.nodeCount ) * b2TreeNode.sizeof );
		b2Free( oldNodes, cast(int)(oldCapacity * b2TreeNode.sizeof) );

		// Build a linked list for the free list. The parent pointer becomes the "next" pointer.
		// todo avoid building freelist?
		for ( int i = tree.nodeCount; i < tree.nodeCapacity - 1; ++i )
		{
			tree.nodes[i].next = i + 1;
		}

		tree.nodes[tree.nodeCapacity - 1].next = B2_NULL_INDEX;
		tree.freeList = tree.nodeCount;
	}

	// Peel a node off the free list.
	int nodeIndex = tree.freeList;
	b2TreeNode* node = tree.nodes + nodeIndex;
	tree.freeList = node.next;
	*node = b2_defaultTreeNode;
	++tree.nodeCount;
	return nodeIndex;
}

// Return a node to the pool.
private void b2FreeNode(b2DynamicTree* tree, int nodeId)
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree.nodeCapacity );
	B2_ASSERT( 0 < tree.nodeCount );
	tree.nodes[nodeId].next = tree.freeList;
	tree.nodes[nodeId].flags = 0;
	tree.freeList = nodeId;
	--tree.nodeCount;
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
private int b2FindBestSibling(const(b2DynamicTree)* tree, b2AABB boxD)
{
	b2Vec2 centerD = b2AABB_Center( boxD );
	float areaD = b2Perimeter( boxD );

	const(b2TreeNode)* nodes = tree.nodes;
	int rootIndex = tree.root;

	b2AABB rootBox = nodes[rootIndex].aabb;

	// Area of current node
	float areaBase = b2Perimeter( rootBox );

	// Area of inflated node
	float directCost = b2Perimeter( b2AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;

	int bestSibling = rootIndex;
	float bestCost = directCost;

	// Descend the tree from root, following a single greedy path.
	int index = rootIndex;
	while ( nodes[index].height > 0 )
	{
		int child1 = nodes[index].children.child1;
		int child2 = nodes[index].children.child2;

		// Cost of creating a new parent for this node and the new leaf
		float cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if ( cost < bestCost )
		{
			bestSibling = index;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		bool leaf1 = nodes[child1].height == 0;
		bool leaf2 = nodes[child2].height == 0;

		// Cost of descending into child 1
		float lowerCost1 = float.max;
		b2AABB box1 = nodes[child1].aabb;
		float directCost1 = b2Perimeter( b2AABB_Union( box1, boxD ) );
		float area1 = 0.0f;
		if ( leaf1 )
		{
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			float cost1 = directCost1 + inheritedCost;

			// Need this here due to while condition above
			if ( cost1 < bestCost )
			{
				bestSibling = child1;
				bestCost = cost1;
			}
		}
		else
		{
			// Child 1 is an internal node
			area1 = b2Perimeter( box1 );

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendent of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + b2MinFloat( areaD - area1, 0.0f );
		}

		// Cost of descending into child 2
		float lowerCost2 = float.max;
		b2AABB box2 = nodes[child2].aabb;
		float directCost2 = b2Perimeter( b2AABB_Union( box2, boxD ) );
		float area2 = 0.0f;
		if ( leaf2 )
		{
			float cost2 = directCost2 + inheritedCost;

			if ( cost2 < bestCost )
			{
				bestSibling = child2;
				bestCost = cost2;
			}
		}
		else
		{
			area2 = b2Perimeter( box2 );
			lowerCost2 = inheritedCost + directCost2 + b2MinFloat( areaD - area2, 0.0f );
		}

		if ( leaf1 && leaf2 )
		{
			break;
		}

		// Can the cost possibly be decreased?
		if ( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
		{
			break;
		}

		if ( lowerCost1 == lowerCost2 && leaf1 == false )
		{
			B2_ASSERT( lowerCost1 < float.max );
			B2_ASSERT( lowerCost2 < float.max );

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			b2Vec2 d1 = b2Sub( b2AABB_Center( box1 ), centerD );
			b2Vec2 d2 = b2Sub( b2AABB_Center( box2 ), centerD );
			lowerCost1 = b2LengthSquared( d1 );
			lowerCost2 = b2LengthSquared( d2 );
		}

		// Descend
		if ( lowerCost1 < lowerCost2 && leaf1 == false )
		{
			index = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			index = child2;
			areaBase = area2;
			directCost = directCost2;
		}

		B2_ASSERT( nodes[index].height > 0 );
	}

	return bestSibling;
}

enum b2RotateType
{
	b2_rotateNone,
	b2_rotateBF,
	b2_rotateBG,
	b2_rotateCD,
	b2_rotateCE
}
alias b2_rotateNone = b2RotateType.b2_rotateNone;
alias b2_rotateBF = b2RotateType.b2_rotateBF;
alias b2_rotateBG = b2RotateType.b2_rotateBG;
alias b2_rotateCD = b2RotateType.b2_rotateCD;
alias b2_rotateCE = b2RotateType.b2_rotateCE;


// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
private void b2RotateNodes(b2DynamicTree* tree, int iA)
{
	B2_ASSERT( iA != B2_NULL_INDEX );

	b2TreeNode* nodes = tree.nodes;

	b2TreeNode* A = nodes + iA;
	if ( A.height < 2 )
	{
		return;
	}

	int iB = A.children.child1;
	int iC = A.children.child2;
	B2_ASSERT( 0 <= iB && iB < tree.nodeCapacity );
	B2_ASSERT( 0 <= iC && iC < tree.nodeCapacity );

	b2TreeNode* B = nodes + iB;
	b2TreeNode* C = nodes + iC;

	if ( B.height == 0 )
	{
		// B is a leaf and C is internal
		B2_ASSERT( C.height > 0 );

		int iF = C.children.child1;
		int iG = C.children.child2;
		b2TreeNode* F = nodes + iF;
		b2TreeNode* G = nodes + iG;
		B2_ASSERT( 0 <= iF && iF < tree.nodeCapacity );
		B2_ASSERT( 0 <= iG && iG < tree.nodeCapacity );

		// Base cost
		float costBase = b2Perimeter( C.aabb );

		// Cost of swapping B and F
		b2AABB aabbBG = b2AABB_Union( B.aabb, G.aabb );
		float costBF = b2Perimeter( aabbBG );

		// Cost of swapping B and G
		b2AABB aabbBF = b2AABB_Union( B.aabb, F.aabb );
		float costBG = b2Perimeter( aabbBF );

		if ( costBase < costBF && costBase < costBG )
		{
			// Rotation does not improve cost
			return;
		}

		if ( costBF < costBG )
		{
			// Swap B and F
			A.children.child1 = iF;
			C.children.child1 = iB;

			B.parent = iC;
			F.parent = iA;

			C.aabb = aabbBG;

			C.height = cast(ushort)(1 + b2MaxUInt16( B.height, G.height ));
			A.height = cast(ushort)(1 + b2MaxUInt16( C.height, F.height ));
			C.categoryBits = B.categoryBits | G.categoryBits;
			A.categoryBits = C.categoryBits | F.categoryBits;
			C.flags |= ( B.flags | G.flags ) & b2_enlargedNode;
			A.flags |= ( C.flags | F.flags ) & b2_enlargedNode;
		}
		else
		{
			// Swap B and G
			A.children.child1 = iG;
			C.children.child2 = iB;

			B.parent = iC;
			G.parent = iA;

			C.aabb = aabbBF;

			C.height = cast(ushort)(1 + b2MaxUInt16( B.height, F.height ));
			A.height = cast(ushort)(1 + b2MaxUInt16( C.height, G.height ));
			C.categoryBits = B.categoryBits | F.categoryBits;
			A.categoryBits = C.categoryBits | G.categoryBits;
			C.flags |= ( B.flags | F.flags ) & b2_enlargedNode;
			A.flags |= ( C.flags | G.flags ) & b2_enlargedNode;
		}
	}
	else if ( C.height == 0 )
	{
		// C is a leaf and B is internal
		B2_ASSERT( B.height > 0 );

		int iD = B.children.child1;
		int iE = B.children.child2;
		b2TreeNode* D = nodes + iD;
		b2TreeNode* E = nodes + iE;
		B2_ASSERT( 0 <= iD && iD < tree.nodeCapacity );
		B2_ASSERT( 0 <= iE && iE < tree.nodeCapacity );

		// Base cost
		float costBase = b2Perimeter( B.aabb );

		// Cost of swapping C and D
		b2AABB aabbCE = b2AABB_Union( C.aabb, E.aabb );
		float costCD = b2Perimeter( aabbCE );

		// Cost of swapping C and E
		b2AABB aabbCD = b2AABB_Union( C.aabb, D.aabb );
		float costCE = b2Perimeter( aabbCD );

		if ( costBase < costCD && costBase < costCE )
		{
			// Rotation does not improve cost
			return;
		}

		if ( costCD < costCE )
		{
			// Swap C and D
			A.children.child2 = iD;
			B.children.child1 = iC;

			C.parent = iB;
			D.parent = iA;

			B.aabb = aabbCE;

			B.height = cast(ushort)(1 + b2MaxUInt16( C.height, E.height ));
			A.height = cast(ushort)(1 + b2MaxUInt16( B.height, D.height ));
			B.categoryBits = C.categoryBits | E.categoryBits;
			A.categoryBits = B.categoryBits | D.categoryBits;
			B.flags |= ( C.flags | E.flags ) & b2_enlargedNode;
			A.flags |= ( B.flags | D.flags ) & b2_enlargedNode;
		}
		else
		{
			// Swap C and E
			A.children.child2 = iE;
			B.children.child2 = iC;

			C.parent = iB;
			E.parent = iA;

			B.aabb = aabbCD;
			B.height = cast(ushort)(1 + b2MaxUInt16( C.height, D.height ));
			A.height = cast(ushort)(1 + b2MaxUInt16( B.height, E.height ));
			B.categoryBits = C.categoryBits | D.categoryBits;
			A.categoryBits = B.categoryBits | E.categoryBits;
			B.flags |= ( C.flags | D.flags ) & b2_enlargedNode;
			A.flags |= ( B.flags | E.flags ) & b2_enlargedNode;
		}
	}
	else
	{
		int iD = B.children.child1;
		int iE = B.children.child2;
		int iF = C.children.child1;
		int iG = C.children.child2;

		B2_ASSERT( 0 <= iD && iD < tree.nodeCapacity );
		B2_ASSERT( 0 <= iE && iE < tree.nodeCapacity );
		B2_ASSERT( 0 <= iF && iF < tree.nodeCapacity );
		B2_ASSERT( 0 <= iG && iG < tree.nodeCapacity );

		b2TreeNode* D = nodes + iD;
		b2TreeNode* E = nodes + iE;
		b2TreeNode* F = nodes + iF;
		b2TreeNode* G = nodes + iG;

		// Base cost
		float areaB = b2Perimeter( B.aabb );
		float areaC = b2Perimeter( C.aabb );
		float costBase = areaB + areaC;
		b2RotateType bestRotation = b2_rotateNone;
		float bestCost = costBase;

		// Cost of swapping B and F
		b2AABB aabbBG = b2AABB_Union( B.aabb, G.aabb );
		float costBF = areaB + b2Perimeter( aabbBG );
		if ( costBF < bestCost )
		{
			bestRotation = b2_rotateBF;
			bestCost = costBF;
		}

		// Cost of swapping B and G
		b2AABB aabbBF = b2AABB_Union( B.aabb, F.aabb );
		float costBG = areaB + b2Perimeter( aabbBF );
		if ( costBG < bestCost )
		{
			bestRotation = b2_rotateBG;
			bestCost = costBG;
		}

		// Cost of swapping C and D
		b2AABB aabbCE = b2AABB_Union( C.aabb, E.aabb );
		float costCD = areaC + b2Perimeter( aabbCE );
		if ( costCD < bestCost )
		{
			bestRotation = b2_rotateCD;
			bestCost = costCD;
		}

		// Cost of swapping C and E
		b2AABB aabbCD = b2AABB_Union( C.aabb, D.aabb );
		float costCE = areaC + b2Perimeter( aabbCD );
		if ( costCE < bestCost )
		{
			bestRotation = b2_rotateCE;
			// bestCost = costCE;
		}

		switch ( bestRotation )
		{
			case b2_rotateNone:
				break;

			case b2_rotateBF:
				A.children.child1 = iF;
				C.children.child1 = iB;

				B.parent = iC;
				F.parent = iA;

				C.aabb = aabbBG;
				C.height = cast(ushort)(1 + b2MaxUInt16( B.height, G.height ));
				A.height = cast(ushort)(1 + b2MaxUInt16( C.height, F.height ));
				C.categoryBits = B.categoryBits | G.categoryBits;
				A.categoryBits = C.categoryBits | F.categoryBits;
				C.flags |= ( B.flags | G.flags ) & b2_enlargedNode;
				A.flags |= ( C.flags | F.flags ) & b2_enlargedNode;
				break;

			case b2_rotateBG:
				A.children.child1 = iG;
				C.children.child2 = iB;

				B.parent = iC;
				G.parent = iA;

				C.aabb = aabbBF;
				C.height = cast(ushort)(1 + b2MaxUInt16( B.height, F.height ));
				A.height = cast(ushort)(1 + b2MaxUInt16( C.height, G.height ));
				C.categoryBits = B.categoryBits | F.categoryBits;
				A.categoryBits = C.categoryBits | G.categoryBits;
				C.flags |= ( B.flags | F.flags ) & b2_enlargedNode;
				A.flags |= ( C.flags | G.flags ) & b2_enlargedNode;
				break;

			case b2_rotateCD:
				A.children.child2 = iD;
				B.children.child1 = iC;

				C.parent = iB;
				D.parent = iA;

				B.aabb = aabbCE;
				B.height = cast(ushort)(1 + b2MaxUInt16( C.height, E.height ));
				A.height = cast(ushort)(1 + b2MaxUInt16( B.height, D.height ));
				B.categoryBits = C.categoryBits | E.categoryBits;
				A.categoryBits = B.categoryBits | D.categoryBits;
				B.flags |= ( C.flags | E.flags ) & b2_enlargedNode;
				A.flags |= ( B.flags | D.flags ) & b2_enlargedNode;
				break;

			case b2_rotateCE:
				A.children.child2 = iE;
				B.children.child2 = iC;

				C.parent = iB;
				E.parent = iA;

				B.aabb = aabbCD;
				B.height = cast(ushort)(1 + b2MaxUInt16( C.height, D.height ));
				A.height = cast(ushort)(1 + b2MaxUInt16( B.height, E.height ));
				B.categoryBits = C.categoryBits | D.categoryBits;
				A.categoryBits = B.categoryBits | E.categoryBits;
				B.flags |= ( C.flags | D.flags ) & b2_enlargedNode;
				A.flags |= ( B.flags | E.flags ) & b2_enlargedNode;
				break;

			default:
				B2_ASSERT( false );
				break;
		}
	}
}

private void b2InsertLeaf(b2DynamicTree* tree, int leaf, bool shouldRotate)
{
	if ( tree.root == B2_NULL_INDEX )
	{
		tree.root = leaf;
		tree.nodes[tree.root].parent = B2_NULL_INDEX;
		return;
	}

	// Stage 1: find the best sibling for this node
	b2AABB leafAABB = tree.nodes[leaf].aabb;
	int sibling = b2FindBestSibling( tree, leafAABB );

	// Stage 2: create a new parent for the leaf and sibling
	int oldParent = tree.nodes[sibling].parent;
	int newParent = b2AllocateNode( tree );

	// warning: node pointer can change after allocation
	b2TreeNode* nodes = tree.nodes;
	nodes[newParent].parent = oldParent;
	nodes[newParent].userData = UINT64_MAX;
	nodes[newParent].aabb = b2AABB_Union( leafAABB, nodes[sibling].aabb );
	nodes[newParent].categoryBits = nodes[leaf].categoryBits | nodes[sibling].categoryBits;
	nodes[newParent].height = cast(ushort)(nodes[sibling].height + 1);

	if ( oldParent != B2_NULL_INDEX )
	{
		// The sibling was not the root.
		if ( nodes[oldParent].children.child1 == sibling )
		{
			nodes[oldParent].children.child1 = newParent;
		}
		else
		{
			nodes[oldParent].children.child2 = newParent;
		}

		nodes[newParent].children.child1 = sibling;
		nodes[newParent].children.child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		nodes[newParent].children.child1 = sibling;
		nodes[newParent].children.child2 = leaf;
		nodes[sibling].parent = newParent;
		nodes[leaf].parent = newParent;
		tree.root = newParent;
	}

	// Stage 3: walk back up the tree fixing heights and AABBs
	int index = nodes[leaf].parent;
	while ( index != B2_NULL_INDEX )
	{
		int child1 = nodes[index].children.child1;
		int child2 = nodes[index].children.child2;

		B2_ASSERT( child1 != B2_NULL_INDEX );
		B2_ASSERT( child2 != B2_NULL_INDEX );

		nodes[index].aabb = b2AABB_Union( nodes[child1].aabb, nodes[child2].aabb );
		nodes[index].categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;
		nodes[index].height = cast(ushort)(1 + b2MaxUInt16( nodes[child1].height, nodes[child2].height ));
		nodes[index].flags |= ( nodes[child1].flags | nodes[child2].flags ) & b2_enlargedNode;

		if ( shouldRotate )
		{
			b2RotateNodes( tree, index );
		}

		index = nodes[index].parent;
	}
}

private void b2RemoveLeaf(b2DynamicTree* tree, int leaf)
{
	if ( leaf == tree.root )
	{
		tree.root = B2_NULL_INDEX;
		return;
	}

	b2TreeNode* nodes = tree.nodes;

	int parent = nodes[leaf].parent;
	int grandParent = nodes[parent].parent;
	int sibling = void;
	if ( nodes[parent].children.child1 == leaf )
	{
		sibling = nodes[parent].children.child2;
	}
	else
	{
		sibling = nodes[parent].children.child1;
	}

	if ( grandParent != B2_NULL_INDEX )
	{
		// Destroy parent and connect sibling to grandParent.
		if ( nodes[grandParent].children.child1 == parent )
		{
			nodes[grandParent].children.child1 = sibling;
		}
		else
		{
			nodes[grandParent].children.child2 = sibling;
		}
		nodes[sibling].parent = grandParent;
		b2FreeNode( tree, parent );

		// Adjust ancestor bounds.
		int index = grandParent;
		while ( index != B2_NULL_INDEX )
		{
			b2TreeNode* node = nodes + index;
			b2TreeNode* child1 = nodes + node.children.child1;
			b2TreeNode* child2 = nodes + node.children.child2;

			// Fast union using SSE
			//__m128 aabb1 = _mm_load_ps(&child1->aabb.lowerBound.x);
			//__m128 aabb2 = _mm_load_ps(&child2->aabb.lowerBound.x);
			//__m128 lower = _mm_min_ps(aabb1, aabb2);
			//__m128 upper = _mm_max_ps(aabb1, aabb2);
			//__m128 aabb = _mm_shuffle_ps(lower, upper, _MM_SHUFFLE(3, 2, 1, 0));
			//_mm_store_ps(&node->aabb.lowerBound.x, aabb);

			node.aabb = b2AABB_Union( child1.aabb, child2.aabb );
			node.categoryBits = child1.categoryBits | child2.categoryBits;
			node.height = cast(ushort)(1 + b2MaxUInt16( child1.height, child2.height ));

			index = node.parent;
		}
	}
	else
	{
		tree.root = sibling;
		tree.nodes[sibling].parent = B2_NULL_INDEX;
		b2FreeNode( tree, parent );
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
// int b2DynamicTree_CreateProxy(b2DynamicTree* tree, b2AABB aabb, ulong categoryBits, ulong userData)
// {
// 	B2_ASSERT( -B2_HUGE < aabb.lowerBound.x && aabb.lowerBound.x < B2_HUGE );
// 	B2_ASSERT( -B2_HUGE < aabb.lowerBound.y && aabb.lowerBound.y < B2_HUGE );
// 	B2_ASSERT( -B2_HUGE < aabb.upperBound.x && aabb.upperBound.x < B2_HUGE );
// 	B2_ASSERT( -B2_HUGE < aabb.upperBound.y && aabb.upperBound.y < B2_HUGE );

// 	int proxyId = b2AllocateNode( tree );
// 	b2TreeNode* node = tree.nodes + proxyId;

// 	node.aabb = aabb;
// 	node.userData = userData;
// 	node.categoryBits = categoryBits;
// 	node.height = 0;
// 	node.flags = b2_allocatedNode | b2_leafNode;

// 	bool shouldRotate = true;
// 	b2InsertLeaf( tree, proxyId, shouldRotate );

// 	tree.proxyCount += 1;

// 	return proxyId;
// }

// void b2DynamicTree_DestroyProxy(b2DynamicTree* tree, int proxyId)
// {
// 	B2_ASSERT( 0 <= proxyId && proxyId < tree.nodeCapacity );
// 	B2_ASSERT( b2IsLeaf( tree.nodes + proxyId ) );

// 	b2RemoveLeaf( tree, proxyId );
// 	b2FreeNode( tree, proxyId );

// 	B2_ASSERT( tree.proxyCount > 0 );
// 	tree.proxyCount -= 1;
// }

// int b2DynamicTree_GetProxyCount(const(b2DynamicTree)* tree)
// {
// 	return tree.proxyCount;
// }

// void b2DynamicTree_MoveProxy(b2DynamicTree* tree, int proxyId, b2AABB aabb)
// {
// 	B2_ASSERT( b2IsValidAABB( aabb ) );
// 	B2_ASSERT( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
// 	B2_ASSERT( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
// 	B2_ASSERT( 0 <= proxyId && proxyId < tree.nodeCapacity );
// 	B2_ASSERT( b2IsLeaf( tree.nodes + proxyId ) );

// 	b2RemoveLeaf( tree, proxyId );

// 	tree.nodes[proxyId].aabb = aabb;

// 	bool shouldRotate = false;
// 	b2InsertLeaf( tree, proxyId, shouldRotate );
// }

// void b2DynamicTree_SetCategoryBits(b2DynamicTree* tree, int proxyId, ulong categoryBits)
// {
// 	b2TreeNode* nodes = tree.nodes;

// 	B2_ASSERT( nodes[proxyId].children.child1 == B2_NULL_INDEX );
// 	B2_ASSERT( nodes[proxyId].children.child2 == B2_NULL_INDEX );
// 	B2_ASSERT( (nodes[proxyId].flags & b2_leafNode) == b2_leafNode );

// 	nodes[proxyId].categoryBits = categoryBits;

// 	// Fix up category bits in ancestor internal nodes
// 	int nodeIndex = nodes[proxyId].parent;
// 	while ( nodeIndex != B2_NULL_INDEX )
// 	{
// 		b2TreeNode* node = nodes + nodeIndex;
// 		int child1 = node.children.child1;
// 		B2_ASSERT( child1 != B2_NULL_INDEX );
// 		int child2 = node.children.child2;
// 		B2_ASSERT( child2 != B2_NULL_INDEX );
// 		node.categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;

// 		nodeIndex = node.parent;
// 	}
// }

// ulong b2DynamicTree_GetCategoryBits(b2DynamicTree* tree, int proxyId)
// {
// 	B2_ASSERT( 0 <= proxyId && proxyId < tree.nodeCapacity );
// 	return tree.nodes[proxyId].categoryBits;
// }

// float b2DynamicTree_GetAreaRatio(const(b2DynamicTree)* tree)
// {
// 	if ( tree.root == B2_NULL_INDEX )
// 	{
// 		return 0.0f;
// 	}

// 	const(b2TreeNode)* root = tree.nodes + tree.root;
// 	float rootArea = b2Perimeter( root.aabb );

// 	float totalArea = 0.0f;
// 	for ( int i = 0; i < tree.nodeCapacity; ++i )
// 	{
// 		const(b2TreeNode)* node = tree.nodes + i;
// 		if ( b2IsAllocated(node) == false || b2IsLeaf( node ) || i == tree.root )
// 		{
// 			continue;
// 		}

// 		totalArea += b2Perimeter( node.aabb );
// 	}

// 	return totalArea / rootArea;
// }

// b2AABB b2DynamicTree_GetRootBounds(const(b2DynamicTree)* tree)
// {
// 	if (tree.root != B2_NULL_INDEX)
// 	{
// 		return tree.nodes[tree.root].aabb;
// 	}

// 	b2AABB empty = { b2Vec2_zero, b2Vec2_zero };
// 	return empty;
// }

static if (B2_VALIDATE) {
// Compute the height of a sub-tree.
private int b2ComputeHeight(const(b2DynamicTree)* tree, int nodeId)
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree.nodeCapacity );
	b2TreeNode* node = tree.nodes + nodeId;

	if ( b2IsLeaf( node ) )
	{
		return 0;
	}

	int height1 = b2ComputeHeight( tree, node.children.child1 );
	int height2 = b2ComputeHeight( tree, node.children.child2 );
	return 1 + b2MaxInt( height1, height2 );
}

private void b2ValidateStructure(const(b2DynamicTree)* tree, int index)
{
	if ( index == B2_NULL_INDEX )
	{
		return;
	}

	if ( index == tree.root )
	{
		B2_ASSERT( tree.nodes[index].parent == B2_NULL_INDEX );
	}

	const(b2TreeNode)* node = tree.nodes + index;

	B2_ASSERT( node.flags == 0 || ( node.flags & b2_allocatedNode ) != 0 );

	if ( b2IsLeaf( node ) )
	{
		B2_ASSERT( node.height == 0 );
		return;
	}

	int child1 = node.children.child1;
	int child2 = node.children.child2;

	B2_ASSERT( 0 <= child1 && child1 < tree.nodeCapacity );
	B2_ASSERT( 0 <= child2 && child2 < tree.nodeCapacity );

	B2_ASSERT( tree.nodes[child1].parent == index );
	B2_ASSERT( tree.nodes[child2].parent == index );

	if ( ( tree.nodes[child1].flags | tree.nodes[child2].flags ) & b2_enlargedNode )
	{
		B2_ASSERT( node.flags & b2_enlargedNode );
	}

	b2ValidateStructure( tree, child1 );
	b2ValidateStructure( tree, child2 );
}

private void b2ValidateMetrics(const(b2DynamicTree)* tree, int index)
{
	if ( index == B2_NULL_INDEX )
	{
		return;
	}

	const(b2TreeNode)* node = tree.nodes + index;

	if ( b2IsLeaf( node ) )
	{
		B2_ASSERT( node.height == 0 );
		return;
	}

	int child1 = node.children.child1;
	int child2 = node.children.child2;

	B2_ASSERT( 0 <= child1 && child1 < tree.nodeCapacity );
	B2_ASSERT( 0 <= child2 && child2 < tree.nodeCapacity );

	int height1 = tree.nodes[child1].height;
	int height2 = tree.nodes[child2].height;
	int height = 1 + b2MaxInt( height1, height2 );
	B2_ASSERT( node.height == height );

	// b2AABB aabb = b2AABB_Union(tree->nodes[child1].aabb, tree->nodes[child2].aabb);

	B2_ASSERT( b2AABB_Contains( node.aabb, tree.nodes[child1].aabb ) );
	B2_ASSERT( b2AABB_Contains( node.aabb, tree.nodes[child2].aabb ) );

	// B2_ASSERT(aabb.lowerBound.x == node->aabb.lowerBound.x);
	// B2_ASSERT(aabb.lowerBound.y == node->aabb.lowerBound.y);
	// B2_ASSERT(aabb.upperBound.x == node->aabb.upperBound.x);
	// B2_ASSERT(aabb.upperBound.y == node->aabb.upperBound.y);

	ulong categoryBits = tree.nodes[child1].categoryBits | tree.nodes[child2].categoryBits;
	B2_ASSERT( node.categoryBits == categoryBits );

	b2ValidateMetrics( tree, child1 );
	b2ValidateMetrics( tree, child2 );
}
}

// Median split == 0, Surface area heuristic == 1
enum B2_TREE_HEURISTIC = 0;

static if (B2_TREE_HEURISTIC == 0) {

// Median split heuristic
private int b2PartitionMid(int* indices, b2Vec2* centers, int count)
{
	// Handle trivial case
	if ( count <= 2 )
	{
		return count / 2;
	}

	b2Vec2 lowerBound = centers[0];
	b2Vec2 upperBound = centers[0];

	for ( int i = 1; i < count; ++i )
	{
		lowerBound = b2Min( lowerBound, centers[i] );
		upperBound = b2Max( upperBound, centers[i] );
	}

	b2Vec2 d = b2Sub( upperBound, lowerBound );
	b2Vec2 c = { 0.5f * ( lowerBound.x + upperBound.x ), 0.5f * ( lowerBound.y + upperBound.y ) };

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	if ( d.x > d.y )
	{
		float pivot = c.x;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].x < pivot )
			{
				i1 += 1;
			}{}

			while ( i1 < i2 && centers[i2 - 1].x >= pivot )
			{
				i2 -= 1;
			}{}

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b2Vec2 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else
	{
		float pivot = c.y;

		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].y < pivot )
			{
				i1 += 1;
			}{}

			while ( i1 < i2 && centers[i2 - 1].y >= pivot )
			{
				i2 -= 1;
			}{}

			if ( i1 < i2 )
			{
				// Swap indices
				{
					int temp = indices[i1];
					indices[i1] = indices[i2 - 1];
					indices[i2 - 1] = temp;
				}

				// Swap centers
				{
					b2Vec2 temp = centers[i1];
					centers[i1] = centers[i2 - 1];
					centers[i2 - 1] = temp;
				}

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	B2_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}

	return count / 2;
}

} else {

enum B2_BIN_COUNT = 8;

struct b2TreeBin {
	b2AABB aabb;
	int count;
}

struct b2TreePlane {
	b2AABB leftAABB;
	b2AABB rightAABB;
	int leftCount;
	int rightCount;
}

// "On Fast Construction of SAH-based Bounding Volume Hierarchies" by Ingo Wald
// Returns the left child count
private int b2PartitionSAH(int* indices, int* binIndices, b2AABB* boxes, int count)
{
	B2_ASSERT( count > 0 );

	b2TreeBin[B2_BIN_COUNT] bins = void;
	b2TreePlane[B2_BIN_COUNT - 1] planes = void;

	b2Vec2 center = b2AABB_Center( boxes[0] );
	b2AABB centroidAABB = void;
	centroidAABB.lowerBound = center;
	centroidAABB.upperBound = center;

	for ( int i = 1; i < count; ++i )
	{
		center = b2AABB_Center( boxes[i] );
		centroidAABB.lowerBound = b2Min( centroidAABB.lowerBound, center );
		centroidAABB.upperBound = b2Max( centroidAABB.upperBound, center );
	}

	b2Vec2 d = b2Sub( centroidAABB.upperBound, centroidAABB.lowerBound );

	// Find longest axis
	int axisIndex = void;
	float invD = void;
	if ( d.x > d.y )
	{
		axisIndex = 0;
		invD = d.x;
	}
	else
	{
		axisIndex = 1;
		invD = d.y;
	}

	invD = invD > 0.0f ? 1.0f / invD : 0.0f;

	// Initialize bin bounds and count
	for ( int i = 0; i < B2_BIN_COUNT; ++i )
	{
		bins[i].aabb.lowerBound =  b2Vec2 ( float.max, float.max );
		bins[i].aabb.upperBound =  b2Vec2 ( -float.max, -float.max );
		bins[i].count = 0;
	}

	// Assign boxes to bins and compute bin boxes
	// TODO_ERIN optimize
	float binCount = B2_BIN_COUNT;
	float[2] lowerBoundArray = [ centroidAABB.lowerBound.x, centroidAABB.lowerBound.y ];
	float minC = lowerBoundArray[axisIndex];
	for ( int i = 0; i < count; ++i )
	{
		b2Vec2 c = b2AABB_Center( boxes[i] );
		float[2] cArray = [ c.x, c.y ];
		int binIndex = cast(int)( binCount * ( cArray[axisIndex] - minC ) * invD );
		binIndex = b2ClampInt( binIndex, 0, B2_BIN_COUNT - 1 );
		binIndices[i] = binIndex;
		bins[binIndex].count += 1;
		bins[binIndex].aabb = b2AABB_Union( bins[binIndex].aabb, boxes[i] );
	}

	int planeCount = B2_BIN_COUNT - 1;

	// Prepare all the left planes, candidates for left child
	planes[0].leftCount = bins[0].count;
	planes[0].leftAABB = bins[0].aabb;
	for ( int i = 1; i < planeCount; ++i )
	{
		planes[i].leftCount = planes[i - 1].leftCount + bins[i].count;
		planes[i].leftAABB = b2AABB_Union( planes[i - 1].leftAABB, bins[i].aabb );
	}

	// Prepare all the right planes, candidates for right child
	planes[planeCount - 1].rightCount = bins[planeCount].count;
	planes[planeCount - 1].rightAABB = bins[planeCount].aabb;
	for ( int i = planeCount - 2; i >= 0; --i )
	{
		planes[i].rightCount = planes[i + 1].rightCount + bins[i + 1].count;
		planes[i].rightAABB = b2AABB_Union( planes[i + 1].rightAABB, bins[i + 1].aabb );
	}

	// Find best split to minimize SAH
	float minCost = float.max;
	int bestPlane = 0;
	for ( int i = 0; i < planeCount; ++i )
	{
		float leftArea = b2Perimeter( planes[i].leftAABB );
		float rightArea = b2Perimeter( planes[i].rightAABB );
		int leftCount = planes[i].leftCount;
		int rightCount = planes[i].rightCount;

		float cost = leftCount * leftArea + rightCount * rightArea;
		if ( cost < minCost )
		{
			bestPlane = i;
			minCost = cost;
		}
	}

	// Partition node indices and boxes using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	int i1 = 0, i2 = count;
	while ( i1 < i2 )
	{
		while ( i1 < i2 && binIndices[i1] < bestPlane )
		{
			i1 += 1;
		}{}

		while ( i1 < i2 && binIndices[i2 - 1] >= bestPlane )
		{
			i2 -= 1;
		}{}

		if ( i1 < i2 )
		{
			// Swap indices
			{
				int temp = indices[i1];
				indices[i1] = indices[i2 - 1];
				indices[i2 - 1] = temp;
			}

			// Swap boxes
			{
				b2AABB temp = boxes[i1];
				boxes[i1] = boxes[i2 - 1];
				boxes[i2 - 1] = temp;
			}

			i1 += 1;
			i2 -= 1;
		}
	}
	B2_ASSERT( i1 == i2 );

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}
	else
	{
		return count / 2;
	}
}

}

// Temporary data used to track the rebuild of a tree node
struct b2RebuildItem
{
	int nodeIndex;
	int childCount;

	// Leaf indices
	int startIndex;
	int splitIndex;
	int endIndex;
}

// Returns root node index
private int b2BuildTree(b2DynamicTree* tree, int leafCount)
{
	b2TreeNode* nodes = tree.nodes;
	int* leafIndices = tree.leafIndices;

	if ( leafCount == 1 )
	{
		nodes[leafIndices[0]].parent = B2_NULL_INDEX;
		return leafIndices[0];
	}

static if (B2_TREE_HEURISTIC == 0) {
	b2Vec2* leafCenters = tree.leafCenters;
} else {
	b2AABB* leafBoxes = tree.leafBoxes;
	int* binIndices = tree.binIndices;
}

	// todo large stack item
	b2RebuildItem[B2_TREE_STACK_SIZE] stack = void;
	int top = 0;

	stack[0].nodeIndex = b2AllocateNode( tree );
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
static if (B2_TREE_HEURISTIC == 0) {
	stack[0].splitIndex = b2PartitionMid( leafIndices, leafCenters, leafCount );
} else {
	stack[0].splitIndex = b2PartitionSAH( leafIndices, binIndices, leafBoxes, leafCount );
}

	while ( true )
	{
		b2RebuildItem* item = stack.ptr + top;

		item.childCount += 1;

		if ( item.childCount == 2 )
		{
			// This internal node has both children established

			if ( top == 0 )
			{
				// all done
				break;
			}

			b2RebuildItem* parentItem = stack.ptr + ( top - 1 );
			b2TreeNode* parentNode = nodes + parentItem.nodeIndex;

			if ( parentItem.childCount == 0 )
			{
				B2_ASSERT( parentNode.children.child1 == B2_NULL_INDEX );
				parentNode.children.child1 = item.nodeIndex;
			}
			else
			{
				B2_ASSERT( parentItem.childCount == 1 );
				B2_ASSERT( parentNode.children.child2 == B2_NULL_INDEX );
				parentNode.children.child2 = item.nodeIndex;
			}

			b2TreeNode* node = nodes + item.nodeIndex;

			B2_ASSERT( node.parent == B2_NULL_INDEX );
			node.parent = parentItem.nodeIndex;

			B2_ASSERT( node.children.child1 != B2_NULL_INDEX );
			B2_ASSERT( node.children.child2 != B2_NULL_INDEX );
			b2TreeNode* child1 = nodes + node.children.child1;
			b2TreeNode* child2 = nodes + node.children.child2;

			node.aabb = b2AABB_Union( child1.aabb, child2.aabb );
			node.height = cast(ushort)(1 + b2MaxUInt16( child1.height, child2.height ));
			node.categoryBits = child1.categoryBits | child2.categoryBits;

			// Pop stack
			top -= 1;
		}
		else
		{
			int startIndex = void, endIndex = void;
			if ( item.childCount == 0 )
			{
				startIndex = item.startIndex;
				endIndex = item.splitIndex;
			}
			else
			{
				B2_ASSERT( item.childCount == 1 );
				startIndex = item.splitIndex;
				endIndex = item.endIndex;
			}

			int count = endIndex - startIndex;

			if ( count == 1 )
			{
				int childIndex = leafIndices[startIndex];
				b2TreeNode* node = nodes + item.nodeIndex;

				if ( item.childCount == 0 )
				{
					B2_ASSERT( node.children.child1 == B2_NULL_INDEX );
					node.children.child1 = childIndex;
				}
				else
				{
					B2_ASSERT( item.childCount == 1 );
					B2_ASSERT( node.children.child2 == B2_NULL_INDEX );
					node.children.child2 = childIndex;
				}

				b2TreeNode* childNode = nodes + childIndex;
				B2_ASSERT( childNode.parent == B2_NULL_INDEX );
				childNode.parent = item.nodeIndex;
			}
			else
			{
				B2_ASSERT( count > 0 );
				B2_ASSERT( top < B2_TREE_STACK_SIZE );

				top += 1;
				b2RebuildItem* newItem = stack.ptr + top;
				newItem.nodeIndex = b2AllocateNode( tree );
				newItem.childCount = -1;
				newItem.startIndex = startIndex;
				newItem.endIndex = endIndex;
static if (B2_TREE_HEURISTIC == 0) {
				newItem.splitIndex = b2PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
} else {
				newItem.splitIndex =
					b2PartitionSAH( leafIndices + startIndex, binIndices + startIndex, leafBoxes + startIndex, count );
}
				newItem.splitIndex += startIndex;
			}
		}
	}

	b2TreeNode* rootNode = nodes + stack[0].nodeIndex;
	B2_ASSERT( rootNode.parent == B2_NULL_INDEX );
	B2_ASSERT( rootNode.children.child1 != B2_NULL_INDEX );
	B2_ASSERT( rootNode.children.child2 != B2_NULL_INDEX );

	b2TreeNode* child1 = nodes + rootNode.children.child1;
	b2TreeNode* child2 = nodes + rootNode.children.child2;

	rootNode.aabb = b2AABB_Union( child1.aabb, child2.aabb );
	rootNode.height = cast(ushort)(1 + b2MaxUInt16( child1.height, child2.height ));
	rootNode.categoryBits = child1.categoryBits | child2.categoryBits;

	return stack[0].nodeIndex;
}