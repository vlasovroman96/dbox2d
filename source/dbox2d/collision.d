module dbox2d.collision;

import dbox2d.base;
import dbox2d.math;
import dbox2d.math.plane;
import dbox2d.types;
import dbox2d.core;
import dbox2d.constants;
import dbox2d.dynamic_tree;
import dbox2d.collision;
import dbox2d.manifold;
import dbox2d.hull;
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
		*tree.nodes = *oldNodes;
		(tree.nodes + tree.nodeCount)[0 .. ( tree.nodeCapacity - tree.nodeCount ) * b2TreeNode.sizeof ] = null;
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
	float areaD = boxD.perimeter;

	const(b2TreeNode)* nodes = tree.nodes;
	int rootIndex = tree.root;

	b2AABB rootBox = nodes[rootIndex].aabb;

	// Area of current node
	float areaBase = rootBox.perimeter;

	// Area of inflated node
	float directCost = b2AABB_Union( rootBox, boxD ).perimeter;
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
		float directCost1 = b2AABB_Union( box1, boxD ).perimeter;
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
			area1 = box1.perimeter();

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendent of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + min( areaD - area1, 0.0f );
		}

		// Cost of descending into child 2
		float lowerCost2 = float.max;
		b2AABB box2 = nodes[child2].aabb;
		float directCost2 = b2AABB_Union( box2, boxD ).perimeter;
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
			area2 = box2.perimeter();
			lowerCost2 = inheritedCost + directCost2 + min( areaD - area2, 0.0f );
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
			b2Vec2 d1 = b2AABB_Center( box1) - centerD;
			b2Vec2 d2 = b2AABB_Center( box2) - centerD;
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
		float costBase = C.aabb.perimeter();

		// Cost of swapping B and F
		b2AABB aabbBG = b2AABB_Union( B.aabb, G.aabb );
		float costBF = aabbBG.perimeter;

		// Cost of swapping B and G
		b2AABB aabbBF = b2AABB_Union( B.aabb, F.aabb );
		float costBG = aabbBF.perimeter;

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
		float costBase = B.aabb.perimeter();

		// Cost of swapping C and D
		b2AABB aabbCE = b2AABB_Union( C.aabb, E.aabb );
		float costCD = aabbCE.perimeter();

		// Cost of swapping C and E
		b2AABB aabbCD = b2AABB_Union( C.aabb, D.aabb );
		float costCE = aabbCD.perimeter();

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
		float areaB = B.aabb.perimeter();
		float areaC = C.aabb.perimeter();
		float costBase = areaB + areaC;
		b2RotateType bestRotation = b2_rotateNone;
		float bestCost = costBase;

		// Cost of swapping B and F
		b2AABB aabbBG = b2AABB_Union( B.aabb, G.aabb );
		float costBF = areaB + aabbBG.perimeter();
		if ( costBF < bestCost )
		{
			bestRotation = b2_rotateBF;
			bestCost = costBF;
		}

		// Cost of swapping B and G
		b2AABB aabbBF = b2AABB_Union( B.aabb, F.aabb );
		float costBG = areaB + aabbBF.perimeter();
		if ( costBG < bestCost )
		{
			bestRotation = b2_rotateBG;
			bestCost = costBG;
		}

		// Cost of swapping C and D
		b2AABB aabbCE = b2AABB_Union( C.aabb, E.aabb );
		float costCD = areaC + aabbCE.perimeter();
		if ( costCD < bestCost )
		{
			bestRotation = b2_rotateCD;
			bestCost = costCD;
		}

		// Cost of swapping C and E
		b2AABB aabbCD = b2AABB_Union( C.aabb, D.aabb );
		float costCE = areaC + aabbCD.perimeter();
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

static if (B2_VALIDATE) {
// Compute the height of a sub-tree.
private int b2ComputeHeight(const(b2DynamicTree)* tree, int nodeId)
{
	B2_ASSERT( 0 <= nodeId && nodeId < tree.nodeCapacity );
	b2TreeNode* node = &tree.nodes[nodeId];

	if ( b2IsLeaf( node ) )
	{
		return 0;
	}

	int height1 = b2ComputeHeight( tree, node.children.child1 );
	int height2 = b2ComputeHeight( tree, node.children.child2 );
	return 1 + max( height1, height2 );
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
	int height = 1 + max( height1, height2 );
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

	b2Vec2 d = upperBound - lowerBound;
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

	b2Vec2 d = centroidAABB.upperBound - centroidAABB.lowerBound;

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
		binIndex = clamp( binIndex, 0, B2_BIN_COUNT - 1 );
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
		float leftArea = planes[i].leftAABB.perimeter();
		float rightArea = planes[i].rightAABB.perimeter();
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