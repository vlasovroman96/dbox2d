module dbox2d.contact_solver;

public import dbox2d.solver;
import dbox2d.body;

mixin(B2_ARRAY_SOURCE!("b2BodyState","b2BodyState"));

struct b2ContactConstraintPoint {
	b2Vec2 anchorA, anchorB;
	float baseSeparation = 0;
	float relativeVelocity = 0;
	float normalImpulse = 0;
	float tangentImpulse = 0;
	float totalNormalImpulse = 0;
	float normalMass = 0;
	float tangentMass = 0;
}

struct b2ContactConstraint {
	int indexA;
	int indexB;
	b2ContactConstraintPoint[2] points;
	b2Vec2 normal;
	float invMassA = 0, invMassB = 0;
	float invIA = 0, invIB = 0;
	float friction = 0;
	float restitution = 0;
	float tangentSpeed = 0;
	float rollingResistance = 0;
	float rollingMass = 0;
	float rollingImpulse = 0;
	b2Softness softness;
	int pointCount;
}

import dbox2d.physics_world;

void b2PrepareOverflowContacts(b2StepContext* context)
{
	import dbox2d.contact;

	b2World* world = context.world;
	b2ConstraintGraph* graph = context.graph;
	b2GraphColor* color = &(graph.colors[B2_OVERFLOW_INDEX]);
	b2ContactConstraint* constraints = color.overflowConstraints;
	int contactCount = cast(int)color.contactSims.length;
	b2ContactSim* contacts = color.contactSims.ptr;
	b2BodyState* awakeStates = context.states;

static if (B2_VALIDATE) {
	b2Body* bodies = world.bodies.data;
}

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b2Softness contactSoftness = context.contactSoftness;
	b2Softness staticSoftness = context.staticSoftness;

	float warmStartScale = world.enableWarmStarting ? 1.0f : 0.0f;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactSim* contactSim = contacts + i;

		const(b2Manifold)* manifold = &contactSim.manifold;
		int pointCount = manifold.pointCount;

		B2_ASSERT( 0 < pointCount && pointCount <= 2 );

		int indexA = contactSim.bodySimIndexA;
		int indexB = contactSim.bodySimIndexB;

static if (B2_VALIDATE) {
		b2Body* bodyA = bodies + contactSim.bodyIdA;
		int validIndexA = bodyA.setIndex == b2_awakeSet ? bodyA.localIndex : B2_NULL_INDEX;
		B2_ASSERT( indexA == validIndexA );

		b2Body* bodyB = bodies + contactSim.bodyIdB;
		int validIndexB = bodyB.setIndex == b2_awakeSet ? bodyB.localIndex : B2_NULL_INDEX;
		B2_ASSERT( indexB == validIndexB );
}

		b2ContactConstraint* constraint = constraints + i;
		constraint.indexA = indexA;
		constraint.indexB = indexB;
		constraint.normal = manifold.normal;
		constraint.friction = contactSim.friction;
		constraint.restitution = contactSim.restitution;
		constraint.rollingResistance = contactSim.rollingResistance;
		constraint.rollingImpulse = warmStartScale * manifold.rollingImpulse;
		constraint.tangentSpeed = contactSim.tangentSpeed;
		constraint.pointCount = pointCount;

		b2Vec2 vA = b2Vec2.zero();
		float wA = 0.0f;
		float mA = contactSim.invMassA;
		float iA = contactSim.invIA;
		if ( indexA != B2_NULL_INDEX )
		{
			b2BodyState* stateA = awakeStates + indexA;
			vA = stateA.linearVelocity;
			wA = stateA.angularVelocity;
		}

		b2Vec2 vB = b2Vec2.zero();
		float wB = 0.0f;
		float mB = contactSim.invMassB;
		float iB = contactSim.invIB;
		if ( indexB != B2_NULL_INDEX )
		{
			b2BodyState* stateB = awakeStates + indexB;
			vB = stateB.linearVelocity;
			wB = stateB.angularVelocity;
		}

		if ( indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX )
		{
			constraint.softness = staticSoftness;
		}
		else
		{
			constraint.softness = contactSoftness;
		}

		// copy mass into constraint to avoid cache misses during sub-stepping
		constraint.invMassA = mA;
		constraint.invIA = iA;
		constraint.invMassB = mB;
		constraint.invIB = iB;

		{
			float k = iA + iB;
			constraint.rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
		}

		b2Vec2 normal = constraint.normal;
		b2Vec2 tangent = b2RightPerp( constraint.normal );

		for ( int j = 0; j < pointCount; ++j )
		{
			const(b2ManifoldPoint)* mp = &manifold.points[j];
			b2ContactConstraintPoint* cp = &constraint.points[j];

			cp.normalImpulse = warmStartScale * mp.normalImpulse;
			cp.tangentImpulse = warmStartScale * mp.tangentImpulse;
			cp.totalNormalImpulse = 0.0f;

			b2Vec2 rA = mp.anchorA;
			b2Vec2 rB = mp.anchorB;

			cp.anchorA = rA;
			cp.anchorB = rB;
			cp.baseSeparation = mp.separation - b2Dot( b2Sub( rB, rA ), normal );

			float rnA = b2Cross( rA, normal );
			float rnB = b2Cross( rB, normal );
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			cp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			float rtA = b2Cross( rA, tangent );
			float rtB = b2Cross( rB, tangent );
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			cp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Save relative velocity for restitution
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			cp.relativeVelocity = b2Dot( normal, b2Sub( vrB, vrA ) );
		}
	}
}

void b2WarmStartOverflowContacts(b2StepContext* context)
{
	import dbox2d.solver_set;
	mixin(B2_ARRAY_SOURCE!("b2SolverSet", "b2SolverSet"));
	b2ConstraintGraph* graph = context.graph;
	b2GraphColor* color = &graph.colors[B2_OVERFLOW_INDEX];
	b2ContactConstraint* constraints = color.overflowConstraints;
	int contactCount = cast(int)color.contactSims.length;
	b2World* world = context.world;
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet.bodyStates.ptr;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		const(b2ContactConstraint)* constraint = constraints + i;

		int indexA = constraint.indexA;
		int indexB = constraint.indexB;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;

		b2Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		b2Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		float mA = constraint.invMassA;
		float iA = constraint.invIA;
		float mB = constraint.invMassB;
		float iB = constraint.invIB;

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		b2Vec2 normal = constraint.normal;
		b2Vec2 tangent = b2RightPerp( constraint.normal );
		int pointCount = constraint.pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			const(b2ContactConstraintPoint)* cp = &(constraint.points[j]);

			// fixed anchors
			b2Vec2 rA = cp.anchorA;
			b2Vec2 rB = cp.anchorB;

			b2Vec2 P = b2Add( b2MulSV( cp.normalImpulse, normal ), b2MulSV( cp.tangentImpulse, tangent ) );
			wA -= iA * b2Cross( rA, P );
			vA = b2MulAdd( vA, -mA, P );
			wB += iB * b2Cross( rB, P );
			vB = b2MulAdd( vB, mB, P );
		}

		wA -= iA * constraint.rollingImpulse;
		wB += iB * constraint.rollingImpulse;

		stateA.linearVelocity = vA;
		stateA.angularVelocity = wA;
		stateB.linearVelocity = vB;
		stateB.angularVelocity = wB;
	}
}

import dbox2d.solver_set;

mixin(B2_ARRAY_SOURCE!("b2SolverSet", "b2SolverSet"));

void b2SolveOverflowContacts(b2StepContext* context, bool useBias)
{
	b2ConstraintGraph* graph = context.graph;
	b2GraphColor* color = &graph.colors[B2_OVERFLOW_INDEX];
	b2ContactConstraint* constraints = color.overflowConstraints;
	int contactCount = cast(int)color.contactSims.length;
	b2World* world = context.world;
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet.bodyStates.ptr;

	float inv_h = context.inv_h;
	const(float) contactSpeed = context.world.contactSpeed;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;
		float mA = constraint.invMassA;
		float iA = constraint.invIA;
		float mB = constraint.invMassB;
		float iB = constraint.invIB;

		b2BodyState* stateA = constraint.indexA == B2_NULL_INDEX ? &dummyState : states + constraint.indexA;
		b2Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;
		b2Rot dqA = stateA.deltaRotation;

		b2BodyState* stateB = constraint.indexB == B2_NULL_INDEX ? &dummyState : states + constraint.indexB;
		b2Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;
		b2Rot dqB = stateB.deltaRotation;

		b2Vec2 dp = b2Sub( stateB.deltaPosition, stateA.deltaPosition );

		b2Vec2 normal = constraint.normal;
		b2Vec2 tangent = b2RightPerp( normal );
		float friction = constraint.friction;
		b2Softness softness = constraint.softness;

		int pointCount = constraint.pointCount;
		float totalNormalImpulse = 0.0f;

		// Non-penetration
		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = &constraint.points[j];

			// fixed anchor points
			b2Vec2 rA = cp.anchorA;
			b2Vec2 rB = cp.anchorB;

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, rB ), b2RotateVector( dqA, rA ) ) );
			float s = cp.baseSeparation + b2Dot( ds, normal );

			float velocityBias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( s > 0.0f )
			{
				// speculative bias
				velocityBias = s * inv_h;
			}
			else if ( useBias )
			{
				velocityBias = max( softness.massScale * softness.biasRate * s, -contactSpeed );
				massScale = softness.massScale;
				impulseScale = softness.impulseScale;
			}

			// relative normal velocity at contact
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			// incremental normal impulse
			float impulse = -cp.normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp.normalImpulse;

			// clamp the accumulated impulse
			float newImpulse = max( cp.normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp.normalImpulse;
			cp.normalImpulse = newImpulse;
			cp.totalNormalImpulse += newImpulse;

			totalNormalImpulse += newImpulse;

			// apply normal impulse
			b2Vec2 P = b2MulSV( impulse, normal );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );

			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		// Friction
		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = &constraint.points[j];

			// fixed anchor points
			b2Vec2 rA = cp.anchorA;
			b2Vec2 rB = cp.anchorB;

			// relative tangent velocity at contact
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );

			// vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			//    = dot(vrB - vrA, tangent) - (sA + sB)

			float vt = b2Dot( b2Sub( vrB, vrA ), tangent ) - constraint.tangentSpeed;

			// incremental tangent impulse
			float impulse = cp.tangentMass * ( -vt );

			// clamp the accumulated force
			float maxFriction = friction * cp.normalImpulse;
			float newImpulse = clamp( cp.tangentImpulse + impulse, -maxFriction, maxFriction );
			impulse = newImpulse - cp.tangentImpulse;
			cp.tangentImpulse = newImpulse;

			// apply tangent impulse
			b2Vec2 P = b2MulSV( impulse, tangent );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		// Rolling resistance
		{
			float deltaLambda = -constraint.rollingMass * ( wB - wA );
			float lambda = constraint.rollingImpulse;
			float maxLambda = constraint.rollingResistance * totalNormalImpulse;
			constraint.rollingImpulse = clamp( lambda + deltaLambda, -maxLambda, maxLambda );
			deltaLambda = constraint.rollingImpulse - lambda;

			wA -= iA * deltaLambda;
			wB += iB * deltaLambda;
		}

		stateA.linearVelocity = vA;
		stateA.angularVelocity = wA;
		stateB.linearVelocity = vB;
		stateB.angularVelocity = wB;
	}
}

void b2ApplyOverflowRestitution(b2StepContext* context)
{
	b2ConstraintGraph* graph = context.graph;
	b2GraphColor* color = &graph.colors[B2_OVERFLOW_INDEX];
	b2ContactConstraint* constraints = color.overflowConstraints;
	int contactCount = cast(int)color.contactSims.length;
	b2World* world = context.world;
	b2SolverSet* awakeSet = b2SolverSetArray_Get( world.solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet.bodyStates.ptr;

	float threshold = context.world.restitutionThreshold;

	// dummy state to represent a static body
	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;

		float restitution = constraint.restitution;
		if ( restitution == 0.0f )
		{
			continue;
		}

		float mA = constraint.invMassA;
		float iA = constraint.invIA;
		float mB = constraint.invMassB;
		float iB = constraint.invIB;

		b2BodyState* stateA = constraint.indexA == B2_NULL_INDEX ? &dummyState : states + constraint.indexA;
		b2Vec2 vA = stateA.linearVelocity;
		float wA = stateA.angularVelocity;

		b2BodyState* stateB = constraint.indexB == B2_NULL_INDEX ? &dummyState : states + constraint.indexB;
		b2Vec2 vB = stateB.linearVelocity;
		float wB = stateB.angularVelocity;

		b2Vec2 normal = constraint.normal;
		int pointCount = constraint.pointCount;

		// it is possible to get more accurate restitution by iterating
		// this only makes a difference if there are two contact points
		// for (int iter = 0; iter < 10; ++iter)
		{
			for ( int j = 0; j < pointCount; ++j )
			{
				b2ContactConstraintPoint* cp = &constraint.points[j];

				// if the normal impulse is zero then there was no collision
				// this skips speculative contact points that didn't generate an impulse
				// The max normal impulse is used in case there was a collision that moved away within the sub-step process
				if ( cp.relativeVelocity > -threshold || cp.totalNormalImpulse == 0.0f )
				{
					continue;
				}

				// fixed anchor points
				b2Vec2 rA = cp.anchorA;
				b2Vec2 rB = cp.anchorB;

				// relative normal velocity at contact
				b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
				b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
				float vn = b2Dot( b2Sub( vrB, vrA ), normal );

				// compute normal impulse
				float impulse = -cp.normalMass * ( vn + restitution * cp.relativeVelocity );

				// clamp the accumulated impulse
				// todo should this be stored?
				float newImpulse = max( cp.normalImpulse + impulse, 0.0f );
				impulse = newImpulse - cp.normalImpulse;
				cp.normalImpulse = newImpulse;

				// Add the incremental impulse rather than the full impulse because this is not a sub-step
				cp.totalNormalImpulse += impulse;

				// apply contact impulse
				b2Vec2 P = b2MulSV( impulse, normal );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}
		}

		stateA.linearVelocity = vA;
		stateA.angularVelocity = wA;
		stateB.linearVelocity = vB;
		stateB.angularVelocity = wB;
	}
}

import dbox2d.contact;

void b2StoreOverflowImpulses(b2StepContext* context)
{
	b2ConstraintGraph* graph = context.graph;
	b2GraphColor* color = &graph.colors[B2_OVERFLOW_INDEX];
	b2ContactConstraint* constraints = color.overflowConstraints;
	b2ContactSim* contacts = color.contactSims.ptr;
	int contactCount = cast(int)color.contactSims.length;

	for ( int i = 0; i < contactCount; ++i )
	{
		const(b2ContactConstraint)* constraint = constraints + i;
		b2ContactSim* contact = contacts + i;
		b2Manifold* manifold = &contact.manifold;
		int pointCount = manifold.pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			manifold.points[j].normalImpulse = constraint.points[j].normalImpulse;
			manifold.points[j].tangentImpulse = constraint.points[j].tangentImpulse;
			manifold.points[j].totalNormalImpulse = constraint.points[j].totalNormalImpulse;
			manifold.points[j].normalVelocity = constraint.points[j].relativeVelocity;
		}

		manifold.rollingImpulse = constraint.rollingImpulse;
	}
}

version (B2_SIMD_AVX2) {

import immintrin;

// wide float holds 8 numbers
alias b2FloatW = __m256;

} else version (B2_SIMD_NEON) {

import arm_neon;

// wide float holds 4 numbers
alias b2FloatW = float32x4_t;

} else version (B2_SIMD_SSE2) {

import emmintrin;

// wide float holds 4 numbers
alias b2FloatW = __m128;

} else {

// scalar math
struct b2FloatW {
	float x = 0, y = 0, z = 0, w = 0;
}

}

// Wide vec2
struct b2Vec2W {
	b2FloatW X, Y;
}

// Wide rotation
struct b2RotW {
	b2FloatW C, S;
}

version (B2_SIMD_AVX2) {

pragma(inline, true) private b2FloatW b2ZeroW()
{
	return _mm256_setzero_ps();
}

pragma(inline, true) private b2FloatW b2SplatW(float scalar)
{
	return _mm256_set1_ps( scalar );
}

pragma(inline, true) private b2FloatW b2AddW(b2FloatW a, b2FloatW b)
{
	return _mm256_add_ps( a, b );
}

pragma(inline, true) private b2FloatW b2SubW(b2FloatW a, b2FloatW b)
{
	return _mm256_sub_ps( a, b );
}

pragma(inline, true) private b2FloatW b2MulW(b2FloatW a, b2FloatW b)
{
	return _mm256_mul_ps( a, b );
}

pragma(inline, true) private b2FloatW b2MulAddW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	// FMA can be emulated: https://github.com/lattera/glibc/blob/master/sysdeps/ieee754/dbl-64/s_fmaf.c#L34
	// return _mm256_fmadd_ps( b, c, a );
	return _mm256_add_ps( _mm256_mul_ps( b, c ), a );
}

pragma(inline, true) private b2FloatW b2MulSubW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	// return _mm256_fnmadd_ps(b, c, a);
	return _mm256_sub_ps( a, _mm256_mul_ps( b, c ) );
}

pragma(inline, true) private b2FloatW b2MinW(b2FloatW a, b2FloatW b)
{
	return _mm256_min_ps( a, b );
}

pragma(inline, true) private b2FloatW b2MaxW(b2FloatW a, b2FloatW b)
{
	return _mm256_max_ps( a, b );
}

// a = clamp(a, -b, b)
pragma(inline, true) private b2FloatW b2SymClampW(b2FloatW a, b2FloatW b)
{
	b2FloatW nb = _mm256_sub_ps( _mm256_setzero_ps(), b );
	return _mm256_max_ps( nb, _mm256_min_ps( a, b ) );
}

pragma(inline, true) private b2FloatW b2OrW(b2FloatW a, b2FloatW b)
{
	return _mm256_or_ps( a, b );
}

pragma(inline, true) private b2FloatW b2GreaterThanW(b2FloatW a, b2FloatW b)
{
	return _mm256_cmp_ps( a, b, _CMP_GT_OQ );
}

pragma(inline, true) private b2FloatW b2EqualsW(b2FloatW a, b2FloatW b)
{
	return _mm256_cmp_ps( a, b, _CMP_EQ_OQ );
}

pragma(inline, true) private bool b2AllZeroW(b2FloatW a)
{
	// Compare each element with zero
	b2FloatW zero = _mm256_setzero_ps();
	b2FloatW cmp = _mm256_cmp_ps( a, zero, _CMP_EQ_OQ );

	// Create a mask from the comparison results
	int mask = _mm256_movemask_ps( cmp );

	// If all elements are zero, the mask will be 0xFF (11111111 in binary)
	return mask == 0xFF;
}

// component-wise returns mask ? b : a
pragma(inline, true) private b2FloatW b2BlendW(b2FloatW a, b2FloatW b, b2FloatW mask)
{
	return _mm256_blendv_ps( a, b, mask );
}

} else version (B2_SIMD_NEON) {

pragma(inline, true) private b2FloatW b2ZeroW()
{
	return vdupq_n_f32( 0.0f );
}

pragma(inline, true) private b2FloatW b2SplatW(float scalar)
{
	return vdupq_n_f32( scalar );
}

pragma(inline, true) private b2FloatW b2SetW(float a, float b, float c, float d)
{
	float32_t[4] array = [ a, b, c, d ];
	return vld1q_f32( array.ptr );
}

pragma(inline, true) private b2FloatW b2AddW(b2FloatW a, b2FloatW b)
{
	return vaddq_f32( a, b );
}

pragma(inline, true) private b2FloatW b2SubW(b2FloatW a, b2FloatW b)
{
	return vsubq_f32( a, b );
}

pragma(inline, true) private b2FloatW b2MulW(b2FloatW a, b2FloatW b)
{
	return vmulq_f32( a, b );
}

pragma(inline, true) private b2FloatW b2MulAddW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	return vmlaq_f32( a, b, c );
}

pragma(inline, true) private b2FloatW b2MulSubW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	return vmlsq_f32( a, b, c );
}

pragma(inline, true) private b2FloatW b2MinW(b2FloatW a, b2FloatW b)
{
	return vminq_f32( a, b );
}

pragma(inline, true) private b2FloatW b2MaxW(b2FloatW a, b2FloatW b)
{
	return vmaxq_f32( a, b );
}

// a = clamp(a, -b, b)
pragma(inline, true) private b2FloatW b2SymClampW(b2FloatW a, b2FloatW b)
{
	b2FloatW nb = vnegq_f32( b );
	return vmaxq_f32( nb, vminq_f32( a, b ) );
}

pragma(inline, true) private b2FloatW b2OrW(b2FloatW a, b2FloatW b)
{
	return vreinterpretq_f32_u32( vorrq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

pragma(inline, true) private b2FloatW b2GreaterThanW(b2FloatW a, b2FloatW b)
{
	return vreinterpretq_f32_u32( vcgtq_f32( a, b ) );
}

pragma(inline, true) private b2FloatW b2EqualsW(b2FloatW a, b2FloatW b)
{
	return vreinterpretq_f32_u32( vceqq_f32( a, b ) );
}

pragma(inline, true) private bool b2AllZeroW(b2FloatW a)
{
	// Create a zero vector for comparison
	b2FloatW zero = vdupq_n_f32( 0.0f );

	// Compare the input vector with zero
	uint32x4_t cmp_result = vceqq_f32( a, zero );

// Check if all comparison results are non-zero using vminvq
version (__ARM_FEATURE_SVE) {
	// ARM v8.2+ has horizontal minimum instruction
	return vminvq_u32( cmp_result ) != 0;
} else {
	// For older ARM architectures, we need to manually check all lanes
	return vgetq_lane_u32( cmp_result, 0 ) != 0 && vgetq_lane_u32( cmp_result, 1 ) != 0 && vgetq_lane_u32( cmp_result, 2 ) != 0 &&
		   vgetq_lane_u32( cmp_result, 3 ) != 0;
}
}

// component-wise returns mask ? b : a
pragma(inline, true) private b2FloatW b2BlendW(b2FloatW a, b2FloatW b, b2FloatW mask)
{
	uint32x4_t mask32 = vreinterpretq_u32_f32( mask );
	return vbslq_f32( mask32, b, a );
}

pragma(inline, true) private b2FloatW b2LoadW(const(float32_t)* data)
{
	return vld1q_f32( data );
}

pragma(inline, true) private void b2StoreW(float32_t* data, b2FloatW a)
{
	vst1q_f32( data, a );
}

pragma(inline, true) private b2FloatW b2UnpackLoW(b2FloatW a, b2FloatW b)
{
version (__aarch64__) {
	return vzip1q_f32( a, b );
} else {
	float32x2_t a1 = vget_low_f32( a );
	float32x2_t b1 = vget_low_f32( b );
	float32x2x2_t result = vzip_f32( a1, b1 );
	return vcombine_f32( result.val[0], result.val[1] );
}
}

pragma(inline, true) private b2FloatW b2UnpackHiW(b2FloatW a, b2FloatW b)
{
version (__aarch64__) {
	return vzip2q_f32( a, b );
} else {
	float32x2_t a1 = vget_high_f32( a );
	float32x2_t b1 = vget_high_f32( b );
	float32x2x2_t result = vzip_f32( a1, b1 );
	return vcombine_f32( result.val[0], result.val[1] );
}
}

} else version (B2_SIMD_SSE2) {

pragma(inline, true) private b2FloatW b2ZeroW()
{
	return _mm_setzero_ps();
}

pragma(inline, true) private b2FloatW b2SplatW(float scalar)
{
	return _mm_set1_ps( scalar );
}

pragma(inline, true) private b2FloatW b2SetW(float a, float b, float c, float d)
{
	return _mm_setr_ps( a, b, c, d );
}

pragma(inline, true) private b2FloatW b2AddW(b2FloatW a, b2FloatW b)
{
	return _mm_add_ps( a, b );
}

pragma(inline, true) private b2FloatW b2SubW(b2FloatW a, b2FloatW b)
{
	return _mm_sub_ps( a, b );
}

pragma(inline, true) private b2FloatW b2MulW(b2FloatW a, b2FloatW b)
{
	return _mm_mul_ps( a, b );
}

pragma(inline, true) private b2FloatW b2MulAddW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	return _mm_add_ps( a, _mm_mul_ps( b, c ) );
}

pragma(inline, true) private b2FloatW b2MulSubW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	return _mm_sub_ps( a, _mm_mul_ps( b, c ) );
}

pragma(inline, true) private b2FloatW b2MinW(b2FloatW a, b2FloatW b)
{
	return _mm_min_ps( a, b );
}

pragma(inline, true) private b2FloatW b2MaxW(b2FloatW a, b2FloatW b)
{
	return _mm_max_ps( a, b );
}

// a = clamp(a, -b, b)
pragma(inline, true) private b2FloatW b2SymClampW(b2FloatW a, b2FloatW b)
{
	// Create a mask with the sign bit set for each element
	__m128 mask = _mm_set1_ps( -0.0f );

	// XOR the input with the mask to negate each element
	__m128 nb = _mm_xor_ps( b, mask );

	return _mm_max_ps( nb, _mm_min_ps( a, b ) );
}

pragma(inline, true) private b2FloatW b2OrW(b2FloatW a, b2FloatW b)
{
	return _mm_or_ps( a, b );
}

pragma(inline, true) private b2FloatW b2GreaterThanW(b2FloatW a, b2FloatW b)
{
	return _mm_cmpgt_ps( a, b );
}

pragma(inline, true) private b2FloatW b2EqualsW(b2FloatW a, b2FloatW b)
{
	return _mm_cmpeq_ps( a, b );
}

pragma(inline, true) private bool b2AllZeroW(b2FloatW a)
{
	// Compare each element with zero
	b2FloatW zero = _mm_setzero_ps();
	b2FloatW cmp = _mm_cmpeq_ps( a, zero );

	// Create a mask from the comparison results
	int mask = _mm_movemask_ps( cmp );

	// If all elements are zero, the mask will be 0xF (1111 in binary)
	return mask == 0xF;
}

// component-wise returns mask ? b : a
pragma(inline, true) private b2FloatW b2BlendW(b2FloatW a, b2FloatW b, b2FloatW mask)
{
	return _mm_or_ps( _mm_and_ps( mask, b ), _mm_andnot_ps( mask, a ) );
}

pragma(inline, true) private b2FloatW b2LoadW(const(float)* data)
{
	return _mm_load_ps( data );
}

pragma(inline, true) private void b2StoreW(float* data, b2FloatW a)
{
	_mm_store_ps( data, a );
}

pragma(inline, true) private b2FloatW b2UnpackLoW(b2FloatW a, b2FloatW b)
{
	return _mm_unpacklo_ps( a, b );
}

pragma(inline, true) private b2FloatW b2UnpackHiW(b2FloatW a, b2FloatW b)
{
	return _mm_unpackhi_ps( a, b );
}

} else {

pragma(inline, true) private b2FloatW b2ZeroW()
{
	return b2FloatW( 0.0f, 0.0f, 0.0f, 0.0f );
}

pragma(inline, true) private b2FloatW b2SplatW(float scalar)
{
	return b2FloatW( scalar, scalar, scalar, scalar );
}

pragma(inline, true) private b2FloatW b2AddW(b2FloatW a, b2FloatW b)
{
	return b2FloatW( a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w );
}

pragma(inline, true) private b2FloatW b2SubW(b2FloatW a, b2FloatW b)
{
	return b2FloatW( a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w );
}

pragma(inline, true) private b2FloatW b2MulW(b2FloatW a, b2FloatW b)
{
	return b2FloatW( a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w );
}

pragma(inline, true) private b2FloatW b2MulAddW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	return b2FloatW( a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w );
}

pragma(inline, true) private b2FloatW b2MulSubW(b2FloatW a, b2FloatW b, b2FloatW c)
{
	return b2FloatW( a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w );
}

pragma(inline, true) private b2FloatW b2MinW(b2FloatW a, b2FloatW b)
{
	b2FloatW r = void;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	return r;
}

pragma(inline, true) private b2FloatW b2MaxW(b2FloatW a, b2FloatW b)
{
	b2FloatW r = void;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

// a = clamp(a, -b, b)
pragma(inline, true) private b2FloatW b2SymClampW(b2FloatW a, b2FloatW b)
{
	b2FloatW r = void;
	r.x = clamp( a.x, -b.x, b.x );
	r.y = clamp( a.y, -b.y, b.y );
	r.z = clamp( a.z, -b.z, b.z );
	r.w = clamp( a.w, -b.w, b.w );
	return r;
}

pragma(inline, true) private b2FloatW b2OrW(b2FloatW a, b2FloatW b)
{
	b2FloatW r = void;
	r.x = a.x != 0.0f || b.x != 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f || b.y != 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f || b.z != 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f || b.w != 0.0f ? 1.0f : 0.0f;
	return r;
}

pragma(inline, true) private b2FloatW b2GreaterThanW(b2FloatW a, b2FloatW b)
{
	b2FloatW r = void;
	r.x = a.x > b.x ? 1.0f : 0.0f;
	r.y = a.y > b.y ? 1.0f : 0.0f;
	r.z = a.z > b.z ? 1.0f : 0.0f;
	r.w = a.w > b.w ? 1.0f : 0.0f;
	return r;
}

pragma(inline, true) private b2FloatW b2EqualsW(b2FloatW a, b2FloatW b)
{
	b2FloatW r = void;
	r.x = a.x == b.x ? 1.0f : 0.0f;
	r.y = a.y == b.y ? 1.0f : 0.0f;
	r.z = a.z == b.z ? 1.0f : 0.0f;
	r.w = a.w == b.w ? 1.0f : 0.0f;
	return r;
}

pragma(inline, true) private bool b2AllZeroW(b2FloatW a)
{
	return a.x == 0.0f && a.y == 0.0f && a.z == 0.0f && a.w == 0.0f;
}

// component-wise returns mask ? b : a
pragma(inline, true) private b2FloatW b2BlendW(b2FloatW a, b2FloatW b, b2FloatW mask)
{
	b2FloatW r = void;
	r.x = mask.x != 0.0f ? b.x : a.x;
	r.y = mask.y != 0.0f ? b.y : a.y;
	r.z = mask.z != 0.0f ? b.z : a.z;
	r.w = mask.w != 0.0f ? b.w : a.w;
	return r;
}

}

pragma(inline, true) private b2FloatW b2DotW(b2Vec2W a, b2Vec2W b)
{
	return b2AddW( b2MulW( a.X, b.X ), b2MulW( a.Y, b.Y ) );
}

pragma(inline, true) private b2FloatW b2CrossW(b2Vec2W a, b2Vec2W b)
{
	return b2SubW( b2MulW( a.X, b.Y ), b2MulW( a.Y, b.X ) );
}

pragma(inline, true) private b2Vec2W b2RotateVectorW(b2RotW q, b2Vec2W v)
{
	return b2Vec2W( b2SubW( b2MulW( q.C, v.X ), b2MulW( q.S, v.Y ) ), b2AddW( b2MulW( q.S, v.X ), b2MulW( q.C, v.Y ) ) );
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

struct b2ContactConstraintSIMD {
	int[B2_SIMD_WIDTH] indexA;
	int[B2_SIMD_WIDTH] indexB;

	b2FloatW invMassA, invMassB;
	b2FloatW invIA, invIB;
	b2Vec2W normal;
	b2FloatW friction;
	b2FloatW tangentSpeed;
	b2FloatW rollingResistance;
	b2FloatW rollingMass;
	b2FloatW rollingImpulse;
	b2FloatW biasRate;
	b2FloatW massScale;
	b2FloatW impulseScale;
	b2Vec2W anchorA1, anchorB1;
	b2FloatW normalMass1, tangentMass1;
	b2FloatW baseSeparation1;
	b2FloatW normalImpulse1;
	b2FloatW totalNormalImpulse1;
	b2FloatW tangentImpulse1;
	b2Vec2W anchorA2, anchorB2;
	b2FloatW baseSeparation2;
	b2FloatW normalImpulse2;
	b2FloatW totalNormalImpulse2;
	b2FloatW tangentImpulse2;
	b2FloatW normalMass2, tangentMass2;
	b2FloatW restitution;
	b2FloatW relativeVelocity1, relativeVelocity2;
}

int b2GetContactConstraintSIMDByteCount()
{
	return b2ContactConstraintSIMD.sizeof;
}

// wide version of b2BodyState
struct b2BodyStateW {
	b2Vec2W v;
	b2FloatW w;
	b2FloatW flags;
	b2Vec2W dp;
	b2RotW dq;
}

// Custom gather/scatter for each SIMD type
version (B2_SIMD_AVX2) {

// This is a load and 8x8 transpose
private b2BodyStateW b2GatherBodies(const(b2BodyState)* states, int* indices)
{
	_Static_assert( b2BodyState.sizeof == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( cast(uintptr_t)states & 0x1F ) == 0 );
	// b2BodyState b2_identityBodyState = {{0.0f, 0.0f}, 0.0f, 0, {0.0f, 0.0f}, {1.0f, 0.0f}};
	b2FloatW identity = _mm256_setr_ps( 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 1.0f, 0.0f );
	b2FloatW b0 = indices[0] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[0] ) );
	b2FloatW b1 = indices[1] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[1] ) );
	b2FloatW b2 = indices[2] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[2] ) );
	b2FloatW b3 = indices[3] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[3] ) );
	b2FloatW b4 = indices[4] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[4] ) );
	b2FloatW b5 = indices[5] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[5] ) );
	b2FloatW b6 = indices[6] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[6] ) );
	b2FloatW b7 = indices[7] == B2_NULL_INDEX ? identity : _mm256_load_ps( cast(float*)( states + indices[7] ) );

	b2FloatW t0 = _mm256_unpacklo_ps( b0, b1 );
	b2FloatW t1 = _mm256_unpackhi_ps( b0, b1 );
	b2FloatW t2 = _mm256_unpacklo_ps( b2, b3 );
	b2FloatW t3 = _mm256_unpackhi_ps( b2, b3 );
	b2FloatW t4 = _mm256_unpacklo_ps( b4, b5 );
	b2FloatW t5 = _mm256_unpackhi_ps( b4, b5 );
	b2FloatW t6 = _mm256_unpacklo_ps( b6, b7 );
	b2FloatW t7 = _mm256_unpackhi_ps( b6, b7 );
	b2FloatW tt0 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt1 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt2 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt3 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt4 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt5 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt6 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt7 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

	b2BodyStateW simdBody = void;
	simdBody.v.X = _mm256_permute2f128_ps( tt0, tt4, 0x20 );
	simdBody.v.Y = _mm256_permute2f128_ps( tt1, tt5, 0x20 );
	simdBody.w = _mm256_permute2f128_ps( tt2, tt6, 0x20 );
	simdBody.flags = _mm256_permute2f128_ps( tt3, tt7, 0x20 );
	simdBody.dp.X = _mm256_permute2f128_ps( tt0, tt4, 0x31 );
	simdBody.dp.Y = _mm256_permute2f128_ps( tt1, tt5, 0x31 );
	simdBody.dq.C = _mm256_permute2f128_ps( tt2, tt6, 0x31 );
	simdBody.dq.S = _mm256_permute2f128_ps( tt3, tt7, 0x31 );
	return simdBody;
}

// This writes everything back to the solver bodies but only the velocities change
private void b2ScatterBodies(b2BodyState* B2_RESTRICT, int* B2_RESTRICT, const(b2BodyStateW)* B2_RESTRICT)
{
	_Static_assert( b2BodyState.sizeof == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( cast(uintptr_t)states & 0x1F ) == 0 );
	b2FloatW t0 = _mm256_unpacklo_ps( simdBody.v.X, simdBody.v.Y );
	b2FloatW t1 = _mm256_unpackhi_ps( simdBody.v.X, simdBody.v.Y );
	b2FloatW t2 = _mm256_unpacklo_ps( simdBody.w, simdBody.flags );
	b2FloatW t3 = _mm256_unpackhi_ps( simdBody.w, simdBody.flags );
	b2FloatW t4 = _mm256_unpacklo_ps( simdBody.dp.X, simdBody.dp.Y );
	b2FloatW t5 = _mm256_unpackhi_ps( simdBody.dp.X, simdBody.dp.Y );
	b2FloatW t6 = _mm256_unpacklo_ps( simdBody.dq.C, simdBody.dq.S );
	b2FloatW t7 = _mm256_unpackhi_ps( simdBody.dq.C, simdBody.dq.S );
	b2FloatW tt0 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt1 = _mm256_shuffle_ps( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt2 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt3 = _mm256_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt4 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt5 = _mm256_shuffle_ps( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
	b2FloatW tt6 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
	b2FloatW tt7 = _mm256_shuffle_ps( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	// todo could add a check for kinematic bodies here

	if ( indices[0] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[0] ), _mm256_permute2f128_ps( tt0, tt4, 0x20 ) );
	if ( indices[1] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[1] ), _mm256_permute2f128_ps( tt1, tt5, 0x20 ) );
	if ( indices[2] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[2] ), _mm256_permute2f128_ps( tt2, tt6, 0x20 ) );
	if ( indices[3] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[3] ), _mm256_permute2f128_ps( tt3, tt7, 0x20 ) );
	if ( indices[4] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[4] ), _mm256_permute2f128_ps( tt0, tt4, 0x31 ) );
	if ( indices[5] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[5] ), _mm256_permute2f128_ps( tt1, tt5, 0x31 ) );
	if ( indices[6] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[6] ), _mm256_permute2f128_ps( tt2, tt6, 0x31 ) );
	if ( indices[7] != B2_NULL_INDEX )
		_mm256_store_ps( cast(float*)( states + indices[7] ), _mm256_permute2f128_ps( tt3, tt7, 0x31 ) );
}

} else version (B2_SIMD_NEON) {

// This is a load and transpose
private b2BodyStateW b2GatherBodies(const(b2BodyState)* B2_RESTRICT, int* B2_RESTRICT)
{
	_Static_assert( b2BodyState.sizeof == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( cast(uintptr_t)states & 0x1F ) == 0 );

	// [vx vy w flags]
	b2FloatW identityA = b2ZeroW();

	// [dpx dpy dqc dqs]

	b2FloatW identityB = b2SetW( 0.0f, 0.0f, 1.0f, 0.0f );

	b2FloatW b1a = indices[0] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[0] ) + 0 );
	b2FloatW b1b = indices[0] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[0] ) + 4 );
	b2FloatW b2a = indices[1] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[1] ) + 0 );
	b2FloatW b2b = indices[1] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[1] ) + 4 );
	b2FloatW b3a = indices[2] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[2] ) + 0 );
	b2FloatW b3b = indices[2] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[2] ) + 4 );
	b2FloatW b4a = indices[3] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[3] ) + 0 );
	b2FloatW b4b = indices[3] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[3] ) + 4 );

	// [vx1 vx3 vy1 vy3]
	b2FloatW t1a = b2UnpackLoW( b1a, b3a );

	// [vx2 vx4 vy2 vy4]
	b2FloatW t2a = b2UnpackLoW( b2a, b4a );

	// [w1 w3 f1 f3]
	b2FloatW t3a = b2UnpackHiW( b1a, b3a );

	// [w2 w4 f2 f4]
	b2FloatW t4a = b2UnpackHiW( b2a, b4a );

	b2BodyStateW simdBody = void;
	simdBody.v.X = b2UnpackLoW( t1a, t2a );
	simdBody.v.Y = b2UnpackHiW( t1a, t2a );
	simdBody.w = b2UnpackLoW( t3a, t4a );
	simdBody.flags = b2UnpackHiW( t3a, t4a );

	b2FloatW t1b = b2UnpackLoW( b1b, b3b );
	b2FloatW t2b = b2UnpackLoW( b2b, b4b );
	b2FloatW t3b = b2UnpackHiW( b1b, b3b );
	b2FloatW t4b = b2UnpackHiW( b2b, b4b );

	simdBody.dp.X = b2UnpackLoW( t1b, t2b );
	simdBody.dp.Y = b2UnpackHiW( t1b, t2b );
	simdBody.dq.C = b2UnpackLoW( t3b, t4b );
	simdBody.dq.S = b2UnpackHiW( t3b, t4b );

	return simdBody;
}

// This writes only the velocities back to the solver bodies
// https://developer.arm.com/documentation/102107a/0100/Floating-point-4x4-matrix-transposition
private void b2ScatterBodies(b2BodyState* B2_RESTRICT, int* B2_RESTRICT, const(b2BodyStateW)* B2_RESTRICT)
{
	_Static_assert( b2BodyState.sizeof == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( cast(uintptr_t)states & 0x1F ) == 0 );

	//	b2FloatW x = b2SetW(0.0f, 1.0f, 2.0f, 3.0f);
	//	b2FloatW y = b2SetW(4.0f, 5.0f, 6.0f, 7.0f);
	//	b2FloatW z = b2SetW(8.0f, 9.0f, 10.0f, 11.0f);
	//	b2FloatW w = b2SetW(12.0f, 13.0f, 14.0f, 15.0f);
	//
	//	float32x4x2_t rr1 = vtrnq_f32( x, y );
	//	float32x4x2_t rr2 = vtrnq_f32( z, w );
	//
	//	float32x4_t b1 = vcombine_f32(vget_low_f32(rr1.val[0]), vget_low_f32(rr2.val[0]));
	//	float32x4_t b2 = vcombine_f32(vget_low_f32(rr1.val[1]), vget_low_f32(rr2.val[1]));
	//	float32x4_t b3 = vcombine_f32(vget_high_f32(rr1.val[0]), vget_high_f32(rr2.val[0]));
	//	float32x4_t b4 = vcombine_f32(vget_high_f32(rr1.val[1]), vget_high_f32(rr2.val[1]));

	// transpose
	float32x4x2_t r1 = vtrnq_f32( simdBody.v.X, simdBody.v.Y );
	float32x4x2_t r2 = vtrnq_f32( simdBody.w, simdBody.flags );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if ( indices[0] != B2_NULL_INDEX )
	{
		float32x4_t body1 = vcombine_f32( vget_low_f32( r1.val[0] ), vget_low_f32( r2.val[0] ) );
		b2StoreW( cast(float*)( states + indices[0] ), body1 );
	}

	if ( indices[1] != B2_NULL_INDEX )
	{
		float32x4_t body2 = vcombine_f32( vget_low_f32( r1.val[1] ), vget_low_f32( r2.val[1] ) );
		b2StoreW( cast(float*)( states + indices[1] ), body2 );
	}

	if ( indices[2] != B2_NULL_INDEX )
	{
		float32x4_t body3 = vcombine_f32( vget_high_f32( r1.val[0] ), vget_high_f32( r2.val[0] ) );
		b2StoreW( cast(float*)( states + indices[2] ), body3 );
	}

	if ( indices[3] != B2_NULL_INDEX )
	{
		float32x4_t body4 = vcombine_f32( vget_high_f32( r1.val[1] ), vget_high_f32( r2.val[1] ) );
		b2StoreW( cast(float*)( states + indices[3] ), body4 );
	}
}

} else version (B2_SIMD_SSE2) {

// This is a load and transpose
private b2BodyStateW b2GatherBodies(const(b2BodyState)* B2_RESTRICT_B, int* B2_RESTRICT_I)
{
	_Static_assert( b2BodyState.sizeof == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( cast(uintptr_t)states & 0x1F ) == 0 );

	// [vx vy w flags]
	b2FloatW identityA = b2ZeroW();

	// [dpx dpy dqc dqs]
	b2FloatW identityB = b2SetW( 0.0f, 0.0f, 1.0f, 0.0f );

	b2FloatW b1a = indices[0] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[0] ) + 0 );
	b2FloatW b1b = indices[0] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[0] ) + 4 );
	b2FloatW b2a = indices[1] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[1] ) + 0 );
	b2FloatW b2b = indices[1] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[1] ) + 4 );
	b2FloatW b3a = indices[2] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[2] ) + 0 );
	b2FloatW b3b = indices[2] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[2] ) + 4 );
	b2FloatW b4a = indices[3] == B2_NULL_INDEX ? identityA : b2LoadW( cast(float*)( states + indices[3] ) + 0 );
	b2FloatW b4b = indices[3] == B2_NULL_INDEX ? identityB : b2LoadW( cast(float*)( states + indices[3] ) + 4 );

	// [vx1 vx3 vy1 vy3]
	b2FloatW t1a = b2UnpackLoW( b1a, b3a );

	// [vx2 vx4 vy2 vy4]
	b2FloatW t2a = b2UnpackLoW( b2a, b4a );

	// [w1 w3 f1 f3]
	b2FloatW t3a = b2UnpackHiW( b1a, b3a );

	// [w2 w4 f2 f4]
	b2FloatW t4a = b2UnpackHiW( b2a, b4a );

	b2BodyStateW simdBody = void;
	simdBody.v.X = b2UnpackLoW( t1a, t2a );
	simdBody.v.Y = b2UnpackHiW( t1a, t2a );
	simdBody.w = b2UnpackLoW( t3a, t4a );
	simdBody.flags = b2UnpackHiW( t3a, t4a );

	b2FloatW t1b = b2UnpackLoW( b1b, b3b );
	b2FloatW t2b = b2UnpackLoW( b2b, b4b );
	b2FloatW t3b = b2UnpackHiW( b1b, b3b );
	b2FloatW t4b = b2UnpackHiW( b2b, b4b );

	simdBody.dp.X = b2UnpackLoW( t1b, t2b );
	simdBody.dp.Y = b2UnpackHiW( t1b, t2b );
	simdBody.dq.C = b2UnpackLoW( t3b, t4b );
	simdBody.dq.S = b2UnpackHiW( t3b, t4b );

	return simdBody;
}

// This writes only the velocities back to the solver bodies
private void b2ScatterBodies(b2BodyState* states, int* indices, const(b2BodyStateW)* simdBody)
{
	_Static_assert( b2BodyState.sizeof == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( cast(uintptr_t)states & 0x1F ) == 0 );

	// [vx1 vy1 vx2 vy2]
	b2FloatW t1 = b2UnpackLoW( simdBody.v.X, simdBody.v.Y );
	// [vx3 vy3 vx4 vy4]
	b2FloatW t2 = b2UnpackHiW( simdBody.v.X, simdBody.v.Y );
	// [w1 f1 w2 f2]
	b2FloatW t3 = b2UnpackLoW( simdBody.w, simdBody.flags );
	// [w3 f3 w4 f4]
	b2FloatW t4 = b2UnpackHiW( simdBody.w, simdBody.flags );

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if ( indices[0] != B2_NULL_INDEX )
	{
		// [t1.x t1.y t3.x t3.y]
		b2StoreW( cast(float*)( states + indices[0] ), _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) ) );
	}

	if ( indices[1] != B2_NULL_INDEX )
	{
		// [t1.z t1.w t3.z t3.w]
		b2StoreW( cast(float*)( states + indices[1] ), _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) ) );
	}

	if ( indices[2] != B2_NULL_INDEX )
	{
		// [t2.x t2.y t4.x t4.y]
		b2StoreW( cast(float*)( states + indices[2] ), _mm_shuffle_ps( t2, t4, _MM_SHUFFLE( 1, 0, 1, 0 ) ) );
	}

	if ( indices[3] != B2_NULL_INDEX )
	{
		// [t2.z t2.w t4.z t4.w]
		b2StoreW( cast(float*)( states + indices[3] ), _mm_shuffle_ps( t2, t4, _MM_SHUFFLE( 3, 2, 3, 2 ) ) );
	}
}

} else {

// This is a load and transpose
private b2BodyStateW b2GatherBodies(const(b2BodyState)* states, int* indices)
{
	b2BodyState identity = b2_identityBodyState;

	b2BodyState s1 = indices[0] == B2_NULL_INDEX ? identity : states[indices[0]];
	b2BodyState s2 = indices[1] == B2_NULL_INDEX ? identity : states[indices[1]];
	b2BodyState s3 = indices[2] == B2_NULL_INDEX ? identity : states[indices[2]];
	b2BodyState s4 = indices[3] == B2_NULL_INDEX ? identity : states[indices[3]];

	b2BodyStateW simdBody = void;
	simdBody.v.X = b2FloatW( s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x );
	simdBody.v.Y = b2FloatW( s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y );
	simdBody.w = b2FloatW( s1.angularVelocity, s2.angularVelocity, s3.angularVelocity, s4.angularVelocity );
	simdBody.flags = b2FloatW( cast(float)s1.flags, cast(float)s2.flags, cast(float)s3.flags, cast(float)s4.flags );
	simdBody.dp.X = b2FloatW( s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x );
	simdBody.dp.Y = b2FloatW( s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y );
	simdBody.dq.C = b2FloatW( s1.deltaRotation.c, s2.deltaRotation.c, s3.deltaRotation.c, s4.deltaRotation.c );
	simdBody.dq.S = b2FloatW( s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s );

	return simdBody;
}

// This writes only the velocities back to the solver bodies
private void b2ScatterBodies(b2BodyState* states, int* indices, const(b2BodyStateW)* simdBody)
{
	// todo somehow skip writing to kinematic bodies

	if ( indices[0] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[0];
		state.linearVelocity.x = simdBody.v.X.x;
		state.linearVelocity.y = simdBody.v.Y.x;
		state.angularVelocity = simdBody.w.x;
	}

	if ( indices[1] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[1];
		state.linearVelocity.x = simdBody.v.X.y;
		state.linearVelocity.y = simdBody.v.Y.y;
		state.angularVelocity = simdBody.w.y;
	}

	if ( indices[2] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[2];
		state.linearVelocity.x = simdBody.v.X.z;
		state.linearVelocity.y = simdBody.v.Y.z;
		state.angularVelocity = simdBody.w.z;
	}

	if ( indices[3] != B2_NULL_INDEX )
	{
		b2BodyState* state = states + indices[3];
		state.linearVelocity.x = simdBody.v.X.w;
		state.linearVelocity.y = simdBody.v.Y.w;
		state.angularVelocity = simdBody.w.w;
	}
}

}

void b2PrepareContactsTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2World* world = context.world;
	b2ContactSim** contacts = context.contacts;
	b2ContactConstraintSIMD* constraints = context.simdContactConstraints;
	b2BodyState* awakeStates = context.states;
static if (B2_VALIDATE) {
	b2Body* bodies = world.bodies.data;
}

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b2Softness contactSoftness = context.contactSoftness;
	b2Softness staticSoftness = context.staticSoftness;

	float warmStartScale = world.enableWarmStarting ? 1.0f : 0.0f;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* constraint = constraints + i;

		for ( int j = 0; j < B2_SIMD_WIDTH; ++j )
		{
			b2ContactSim* contactSim = contacts[B2_SIMD_WIDTH * i + j];

			if ( contactSim != null )
			{
				const(b2Manifold)* manifold = &contactSim.manifold;

				int indexA = contactSim.bodySimIndexA;
				int indexB = contactSim.bodySimIndexB;

static if (B2_VALIDATE) {
				b2Body* bodyA = bodies + contactSim.bodyIdA;
				int validIndexA = bodyA.setIndex == b2_awakeSet ? bodyA.localIndex : B2_NULL_INDEX;
				b2Body* bodyB = bodies + contactSim.bodyIdB;
				int validIndexB = bodyB.setIndex == b2_awakeSet ? bodyB.localIndex : B2_NULL_INDEX;

				B2_ASSERT( indexA == validIndexA );
				B2_ASSERT( indexB == validIndexB );
}
				constraint.indexA[j] = indexA;
				constraint.indexB[j] = indexB;

				b2Vec2 vA = b2Vec2.zero();
				float wA = 0.0f;
				float mA = contactSim.invMassA;
				float iA = contactSim.invIA;
				if ( indexA != B2_NULL_INDEX )
				{
					b2BodyState* stateA = awakeStates + indexA;
					vA = stateA.linearVelocity;
					wA = stateA.angularVelocity;
				}

				b2Vec2 vB = b2Vec2.zero();
				float wB = 0.0f;
				float mB = contactSim.invMassB;
				float iB = contactSim.invIB;
				if ( indexB != B2_NULL_INDEX )
				{
					b2BodyState* stateB = awakeStates + indexB;
					vB = stateB.linearVelocity;
					wB = stateB.angularVelocity;
				}

				( cast(float*)&constraint.invMassA )[j] = mA;
				( cast(float*)&constraint.invMassB )[j] = mB;
				( cast(float*)&constraint.invIA )[j] = iA;
				( cast(float*)&constraint.invIB )[j] = iB;

				{
					float k = iA + iB;
					( cast(float*)&constraint.rollingMass )[j] = k > 0.0f ? 1.0f / k : 0.0f;
				}

				b2Softness soft = ( indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX ) ? staticSoftness : contactSoftness;

				b2Vec2 normal = manifold.normal;
				( cast(float*)&constraint.normal.X )[j] = normal.x;
				( cast(float*)&constraint.normal.Y )[j] = normal.y;

				( cast(float*)&constraint.friction )[j] = contactSim.friction;
				( cast(float*)&constraint.tangentSpeed )[j] = contactSim.tangentSpeed;
				( cast(float*)&constraint.restitution )[j] = contactSim.restitution;
				( cast(float*)&constraint.rollingResistance )[j] = contactSim.rollingResistance;
				( cast(float*)&constraint.rollingImpulse )[j] = warmStartScale * manifold.rollingImpulse;

				( cast(float*)&constraint.biasRate )[j] = soft.biasRate;
				( cast(float*)&constraint.massScale )[j] = soft.massScale;
				( cast(float*)&constraint.impulseScale )[j] = soft.impulseScale;

				b2Vec2 tangent = b2RightPerp( normal );

				{
					const(b2ManifoldPoint)* mp = &manifold.points[0];

					b2Vec2 rA = mp.anchorA;
					b2Vec2 rB = mp.anchorB;

					( cast(float*)&constraint.anchorA1.X )[j] = rA.x;
					( cast(float*)&constraint.anchorA1.Y )[j] = rA.y;
					( cast(float*)&constraint.anchorB1.X )[j] = rB.x;
					( cast(float*)&constraint.anchorB1.Y )[j] = rB.y;

					( cast(float*)&constraint.baseSeparation1 )[j] = mp.separation - b2Dot( b2Sub( rB, rA ), normal );

					( cast(float*)&constraint.normalImpulse1 )[j] = warmStartScale * mp.normalImpulse;
					( cast(float*)&constraint.tangentImpulse1 )[j] = warmStartScale * mp.tangentImpulse;
					( cast(float*)&constraint.totalNormalImpulse1 )[j] = 0.0f;

					float rnA = b2Cross( rA, normal );
					float rnB = b2Cross( rB, normal );
					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					( cast(float*)&constraint.normalMass1 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					float rtA = b2Cross( rA, tangent );
					float rtB = b2Cross( rB, tangent );
					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					( cast(float*)&constraint.tangentMass1 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// relative velocity for restitution
					b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
					b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
					( cast(float*)&constraint.relativeVelocity1 )[j] = b2Dot( normal, b2Sub( vrB, vrA ) );
				}

				int pointCount = manifold.pointCount;
				B2_ASSERT( 0 < pointCount && pointCount <= 2 );

				if ( pointCount == 2 )
				{
					const(b2ManifoldPoint)* mp = &manifold.points[1];

					b2Vec2 rA = mp.anchorA;
					b2Vec2 rB = mp.anchorB;

					( cast(float*)&constraint.anchorA2.X )[j] = rA.x;
					( cast(float*)&constraint.anchorA2.Y )[j] = rA.y;
					( cast(float*)&constraint.anchorB2.X )[j] = rB.x;
					( cast(float*)&constraint.anchorB2.Y )[j] = rB.y;

					( cast(float*)&constraint.baseSeparation2 )[j] = mp.separation - b2Dot( b2Sub( rB, rA ), normal );

					( cast(float*)&constraint.normalImpulse2 )[j] = warmStartScale * mp.normalImpulse;
					( cast(float*)&constraint.tangentImpulse2 )[j] = warmStartScale * mp.tangentImpulse;
					( cast(float*)&constraint.totalNormalImpulse2 )[j] = 0.0f;

					float rnA = b2Cross( rA, normal );
					float rnB = b2Cross( rB, normal );
					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					( cast(float*)&constraint.normalMass2 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					float rtA = b2Cross( rA, tangent );
					float rtB = b2Cross( rB, tangent );
					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					( cast(float*)&constraint.tangentMass2 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// relative velocity for restitution
					b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
					b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
					( cast(float*)&constraint.relativeVelocity2 )[j] = b2Dot( normal, b2Sub( vrB, vrA ) );
				}
				else
				{
					// dummy data that has no effect
					( cast(float*)&constraint.baseSeparation2 )[j] = 0.0f;
					( cast(float*)&constraint.normalImpulse2 )[j] = 0.0f;
					( cast(float*)&constraint.tangentImpulse2 )[j] = 0.0f;
					( cast(float*)&constraint.totalNormalImpulse2 )[j] = 0.0f;
					( cast(float*)&constraint.anchorA2.X )[j] = 0.0f;
					( cast(float*)&constraint.anchorA2.Y )[j] = 0.0f;
					( cast(float*)&constraint.anchorB2.X )[j] = 0.0f;
					( cast(float*)&constraint.anchorB2.Y )[j] = 0.0f;
					( cast(float*)&constraint.normalMass2 )[j] = 0.0f;
					( cast(float*)&constraint.tangentMass2 )[j] = 0.0f;
					( cast(float*)&constraint.relativeVelocity2 )[j] = 0.0f;
				}
			}
			else
			{
				// SIMD remainder
				constraint.indexA[j] = B2_NULL_INDEX;
				constraint.indexB[j] = B2_NULL_INDEX;

				( cast(float*)&constraint.invMassA )[j] = 0.0f;
				( cast(float*)&constraint.invMassB )[j] = 0.0f;
				( cast(float*)&constraint.invIA )[j] = 0.0f;
				( cast(float*)&constraint.invIB )[j] = 0.0f;

				( cast(float*)&constraint.normal.X )[j] = 0.0f;
				( cast(float*)&constraint.normal.Y )[j] = 0.0f;
				( cast(float*)&constraint.friction )[j] = 0.0f;
				( cast(float*)&constraint.tangentSpeed )[j] = 0.0f;
				( cast(float*)&constraint.rollingResistance )[j] = 0.0f;
				( cast(float*)&constraint.rollingMass )[j] = 0.0f;
				( cast(float*)&constraint.rollingImpulse )[j] = 0.0f;
				( cast(float*)&constraint.biasRate )[j] = 0.0f;
				( cast(float*)&constraint.massScale )[j] = 0.0f;
				( cast(float*)&constraint.impulseScale )[j] = 0.0f;

				( cast(float*)&constraint.anchorA1.X )[j] = 0.0f;
				( cast(float*)&constraint.anchorA1.Y )[j] = 0.0f;
				( cast(float*)&constraint.anchorB1.X )[j] = 0.0f;
				( cast(float*)&constraint.anchorB1.Y )[j] = 0.0f;
				( cast(float*)&constraint.baseSeparation1 )[j] = 0.0f;
				( cast(float*)&constraint.normalImpulse1 )[j] = 0.0f;
				( cast(float*)&constraint.tangentImpulse1 )[j] = 0.0f;
				( cast(float*)&constraint.totalNormalImpulse1 )[j] = 0.0f;
				( cast(float*)&constraint.normalMass1 )[j] = 0.0f;
				( cast(float*)&constraint.tangentMass1 )[j] = 0.0f;

				( cast(float*)&constraint.anchorA2.X )[j] = 0.0f;
				( cast(float*)&constraint.anchorA2.Y )[j] = 0.0f;
				( cast(float*)&constraint.anchorB2.X )[j] = 0.0f;
				( cast(float*)&constraint.anchorB2.Y )[j] = 0.0f;
				( cast(float*)&constraint.baseSeparation2 )[j] = 0.0f;
				( cast(float*)&constraint.normalImpulse2 )[j] = 0.0f;
				( cast(float*)&constraint.tangentImpulse2 )[j] = 0.0f;
				( cast(float*)&constraint.totalNormalImpulse2 )[j] = 0.0f;
				( cast(float*)&constraint.normalMass2 )[j] = 0.0f;
				( cast(float*)&constraint.tangentMass2 )[j] = 0.0f;

				( cast(float*)&constraint.restitution )[j] = 0.0f;
				( cast(float*)&constraint.relativeVelocity1 )[j] = 0.0f;
				( cast(float*)&constraint.relativeVelocity2 )[j] = 0.0f;
			}
		}
	}
}

void b2WarmStartContactsTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex)
{
	b2BodyState* states = context.states;
	b2ContactConstraintSIMD* constraints = context.graph.colors[colorIndex].simdConstraints;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;
		b2BodyStateW bA = b2GatherBodies( states, c.indexA.ptr );
		b2BodyStateW bB = b2GatherBodies( states, c.indexB.ptr );

		b2FloatW tangentX = c.normal.Y;
		b2FloatW tangentY = b2SubW( b2ZeroW(), c.normal.X );

		{
			// fixed anchors
			b2Vec2W rA = c.anchorA1;
			b2Vec2W rB = c.anchorB1;

			b2Vec2W P = void;
			P.X = b2AddW( b2MulW( c.normalImpulse1, c.normal.X ), b2MulW( c.tangentImpulse1, tangentX ) );
			P.Y = b2AddW( b2MulW( c.normalImpulse1, c.normal.Y ), b2MulW( c.tangentImpulse1, tangentY ) );
			bA.w = b2MulSubW( bA.w, c.invIA, b2CrossW( rA, P ) );
			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, P.X );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, P.Y );
			bB.w = b2MulAddW( bB.w, c.invIB, b2CrossW( rB, P ) );
			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, P.X );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, P.Y );
		}

		{
			// fixed anchors
			b2Vec2W rA = c.anchorA2;
			b2Vec2W rB = c.anchorB2;

			b2Vec2W P = void;
			P.X = b2AddW( b2MulW( c.normalImpulse2, c.normal.X ), b2MulW( c.tangentImpulse2, tangentX ) );
			P.Y = b2AddW( b2MulW( c.normalImpulse2, c.normal.Y ), b2MulW( c.tangentImpulse2, tangentY ) );
			bA.w = b2MulSubW( bA.w, c.invIA, b2CrossW( rA, P ) );
			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, P.X );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, P.Y );
			bB.w = b2MulAddW( bB.w, c.invIB, b2CrossW( rB, P ) );
			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, P.X );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, P.Y );
		}

		bA.w = b2MulSubW( bA.w, c.invIA, c.rollingImpulse );
		bB.w = b2MulAddW( bB.w, c.invIB, c.rollingImpulse );

		b2ScatterBodies( states, c.indexA.ptr, &bA );
		b2ScatterBodies( states, c.indexB.ptr, &bB );
	}
}

void b2SolveContactsTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias)
{
	b2BodyState* states = context.states;
	b2ContactConstraintSIMD* constraints = context.graph.colors[colorIndex].simdConstraints;
	b2FloatW inv_h = b2SplatW( context.inv_h );
	b2FloatW contactSpeed = b2SplatW( -context.world.contactSpeed );
	b2FloatW oneW = b2SplatW( 1.0f );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;

		b2BodyStateW bA = b2GatherBodies( states, c.indexA.ptr );
		b2BodyStateW bB = b2GatherBodies( states, c.indexB.ptr );

		b2FloatW biasRate = void, massScale = void, impulseScale = void;
		if ( useBias )
		{
			biasRate = b2MulW( c.massScale, c.biasRate );
			massScale = c.massScale;
			impulseScale = c.impulseScale;
		}
		else
		{
			biasRate = b2ZeroW();
			massScale = oneW;
			impulseScale = b2ZeroW();
		}

		b2FloatW totalNormalImpulse = b2ZeroW();

		b2Vec2W dp = { b2SubW( bB.dp.X, bA.dp.X ), b2SubW( bB.dp.Y, bA.dp.Y ) };

		// point1 non-penetration constraint
		{
			// Fixed anchors for impulses
			b2Vec2W rA = c.anchorA1;
			b2Vec2W rB = c.anchorB1;

			// Moving anchors for current separation
			b2Vec2W rsA = b2RotateVectorW( bA.dq, rA );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, rB );

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( c.normal, ds ), c.baseSeparation1 );

			// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
			// The contactSpeed is meant to limit stiffness, not increase it.
			b2FloatW mask = b2GreaterThanW( s, b2ZeroW() );
			b2FloatW specBias = b2MulW( s, inv_h );
			b2FloatW softBias = b2MaxW( b2MulW( biasRate, s ), contactSpeed );

			// todo try b2MaxW(softBias, specBias);
			b2FloatW bias = b2BlendW( softBias, specBias, mask );

			b2FloatW pointMassScale = b2BlendW( massScale, oneW, mask );
			b2FloatW pointImpulseScale = b2BlendW( impulseScale, b2ZeroW(), mask );

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c.normal.X ), b2MulW( dvy, c.normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2AddW( b2MulW( c.normalMass1, b2AddW( b2MulW( pointMassScale, vn ), bias ) ),
										  b2MulW( pointImpulseScale, c.normalImpulse1 ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c.normalImpulse1, negImpulse ), b2ZeroW() );
			b2FloatW impulse = b2SubW( newImpulse, c.normalImpulse1 );
			c.normalImpulse1 = newImpulse;
			c.totalNormalImpulse1 = b2AddW( c.totalNormalImpulse1, newImpulse );

			totalNormalImpulse = b2AddW( totalNormalImpulse, newImpulse );

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, c.normal.X );
			b2FloatW Py = b2MulW( impulse, c.normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = b2MulSubW( bA.w, c.invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = b2MulAddW( bB.w, c.invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// second point non-penetration constraint
		{
			// moving anchors for current separation
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c.anchorA2 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c.anchorB2 );

			// compute current separation
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( c.normal, ds ), c.baseSeparation2 );

			b2FloatW mask = b2GreaterThanW( s, b2ZeroW() );
			b2FloatW specBias = b2MulW( s, inv_h );
			b2FloatW softBias = b2MaxW( b2MulW( biasRate, s ), contactSpeed );
			b2FloatW bias = b2BlendW( softBias, specBias, mask );

			b2FloatW pointMassScale = b2BlendW( massScale, oneW, mask );
			b2FloatW pointImpulseScale = b2BlendW( impulseScale, b2ZeroW(), mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c.anchorA2;
			b2Vec2W rB = c.anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c.normal.X ), b2MulW( dvy, c.normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2AddW( b2MulW( c.normalMass2, b2MulW( pointMassScale, b2AddW( vn, bias ) ) ),
										  b2MulW( pointImpulseScale, c.normalImpulse2 ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c.normalImpulse2, negImpulse ), b2ZeroW() );
			b2FloatW impulse = b2SubW( newImpulse, c.normalImpulse2 );
			c.normalImpulse2 = newImpulse;
			c.totalNormalImpulse2 = b2AddW( c.totalNormalImpulse2, newImpulse );

			totalNormalImpulse = b2AddW( totalNormalImpulse, newImpulse );

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, c.normal.X );
			b2FloatW Py = b2MulW( impulse, c.normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = b2MulSubW( bA.w, c.invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = b2MulAddW( bB.w, c.invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		b2FloatW tangentX = c.normal.Y;
		b2FloatW tangentY = b2SubW( b2ZeroW(), c.normal.X );

		// point 1 friction constraint
		{
			// fixed anchors for Jacobians
			b2Vec2W rA = c.anchorA1;
			b2Vec2W rB = c.anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vt = b2AddW( b2MulW( dvx, tangentX ), b2MulW( dvy, tangentY ) );

			// Tangent speed (conveyor belt)
			vt = b2SubW( vt, c.tangentSpeed );

			// Compute tangent force
			b2FloatW negImpulse = b2MulW( c.tangentMass1, vt );

			// Clamp the accumulated force
			b2FloatW maxFriction = b2MulW( c.friction, c.normalImpulse1 );
			b2FloatW newImpulse = b2SubW( c.tangentImpulse1, negImpulse );
			newImpulse = b2MaxW( b2SubW( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			b2FloatW impulse = b2SubW( newImpulse, c.tangentImpulse1 );
			c.tangentImpulse1 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, tangentX );
			b2FloatW Py = b2MulW( impulse, tangentY );

			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = b2MulSubW( bA.w, c.invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = b2MulAddW( bB.w, c.invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// second point friction constraint
		{
			// fixed anchors for Jacobians
			b2Vec2W rA = c.anchorA2;
			b2Vec2W rB = c.anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vt = b2AddW( b2MulW( dvx, tangentX ), b2MulW( dvy, tangentY ) );

			// Tangent speed (conveyor belt)
			vt = b2SubW( vt, c.tangentSpeed );

			// Compute tangent force
			b2FloatW negImpulse = b2MulW( c.tangentMass2, vt );

			// Clamp the accumulated force
			b2FloatW maxFriction = b2MulW( c.friction, c.normalImpulse2 );
			b2FloatW newImpulse = b2SubW( c.tangentImpulse2, negImpulse );
			newImpulse = b2MaxW( b2SubW( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			b2FloatW impulse = b2SubW( newImpulse, c.tangentImpulse2 );
			c.tangentImpulse2 = newImpulse;

			// Apply contact impulse
			b2FloatW Px = b2MulW( impulse, tangentX );
			b2FloatW Py = b2MulW( impulse, tangentY );

			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = b2MulSubW( bA.w, c.invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = b2MulAddW( bB.w, c.invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// Rolling resistance
		{
			b2FloatW deltaLambda = b2MulW( c.rollingMass, b2SubW( bA.w, bB.w ) );
			b2FloatW lambda = c.rollingImpulse;
			b2FloatW maxLambda = b2MulW( c.rollingResistance, totalNormalImpulse );
			c.rollingImpulse = b2SymClampW( b2AddW( lambda, deltaLambda ), maxLambda );
			deltaLambda = b2SubW( c.rollingImpulse, lambda );

			bA.w = b2MulSubW( bA.w, c.invIA, deltaLambda );
			bB.w = b2MulAddW( bB.w, c.invIB, deltaLambda );
		}

		b2ScatterBodies( states, c.indexA.ptr, &bA );
		b2ScatterBodies( states, c.indexB.ptr, &bB );
	}
}

void b2ApplyRestitutionTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex)
{
	b2BodyState* states = context.states;
	b2ContactConstraintSIMD* constraints = context.graph.colors[colorIndex].simdConstraints;
	b2FloatW threshold = b2SplatW( context.world.restitutionThreshold );
	b2FloatW zero = b2ZeroW();

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactConstraintSIMD* c = constraints + i;

		if ( b2AllZeroW( c.restitution ) )
		{
			// No lanes have restitution. Common case.
			continue;
		}

		// Create a mask based on restitution so that lanes with no restitution are not affected
		// by the calculations below.
		b2FloatW restitutionMask = b2EqualsW( c.restitution, zero );

		b2BodyStateW bA = b2GatherBodies( states, c.indexA.ptr );
		b2BodyStateW bB = b2GatherBodies( states, c.indexB.ptr );

		// first point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			b2FloatW mask1 = b2GreaterThanW( b2AddW( c.relativeVelocity1, threshold ), zero );
			b2FloatW mask2 = b2EqualsW( c.totalNormalImpulse1, zero );
			b2FloatW mask = b2OrW( b2OrW( mask1, mask2 ), restitutionMask );
			b2FloatW mass = b2BlendW( c.normalMass1, zero, mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c.anchorA1;
			b2Vec2W rB = c.anchorB1;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c.normal.X ), b2MulW( dvy, c.normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2MulW( mass, b2AddW( vn, b2MulW( c.restitution, c.relativeVelocity1 ) ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c.normalImpulse1, negImpulse ), b2ZeroW() );
			b2FloatW deltaImpulse = b2SubW( newImpulse, c.normalImpulse1 );
			c.normalImpulse1 = newImpulse;

			// Add the incremental impulse rather than the full impulse because this is not a sub-step
			c.totalNormalImpulse1 = b2AddW( c.totalNormalImpulse1, deltaImpulse );

			// Apply contact impulse
			b2FloatW Px = b2MulW( deltaImpulse, c.normal.X );
			b2FloatW Py = b2MulW( deltaImpulse, c.normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = b2MulSubW( bA.w, c.invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = b2MulAddW( bB.w, c.invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		// second point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			b2FloatW mask1 = b2GreaterThanW( b2AddW( c.relativeVelocity2, threshold ), zero );
			b2FloatW mask2 = b2EqualsW( c.totalNormalImpulse2, zero );
			b2FloatW mask = b2OrW( b2OrW( mask1, mask2 ), restitutionMask );
			b2FloatW mass = b2BlendW( c.normalMass2, zero, mask );

			// fixed anchors for Jacobians
			b2Vec2W rA = c.anchorA2;
			b2Vec2W rB = c.anchorB2;

			// Relative velocity at contact
			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c.normal.X ), b2MulW( dvy, c.normal.Y ) );

			// Compute normal impulse
			b2FloatW negImpulse = b2MulW( mass, b2AddW( vn, b2MulW( c.restitution, c.relativeVelocity2 ) ) );

			// Clamp the accumulated impulse
			b2FloatW newImpulse = b2MaxW( b2SubW( c.normalImpulse2, negImpulse ), b2ZeroW() );
			b2FloatW deltaImpulse = b2SubW( newImpulse, c.normalImpulse2 );
			c.normalImpulse2 = newImpulse;

			// Add the incremental impulse rather than the full impulse because this is not a sub-step
			c.totalNormalImpulse2 = b2AddW( c.totalNormalImpulse2, deltaImpulse );

			// Apply contact impulse
			b2FloatW Px = b2MulW( deltaImpulse, c.normal.X );
			b2FloatW Py = b2MulW( deltaImpulse, c.normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c.invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c.invMassA, Py );
			bA.w = b2MulSubW( bA.w, c.invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c.invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c.invMassB, Py );
			bB.w = b2MulAddW( bB.w, c.invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		b2ScatterBodies( states, c.indexA.ptr, &bA );
		b2ScatterBodies( states, c.indexB.ptr, &bB );
	}
}

void b2StoreImpulsesTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2ContactSim** contacts = context.contacts;
	const(b2ContactConstraintSIMD)* constraints = context.simdContactConstraints;

	b2Manifold dummy;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const(b2ContactConstraintSIMD)* c = constraints + constraintIndex;
		const(float)* rollingImpulse = cast(const(float)*) (cast(float*)&c.rollingImpulse);
		const(float)* normalImpulse1 = cast(const(float)*) (cast(float*)&c.normalImpulse1);
		const(float)* normalImpulse2 = cast(const(float)*) (cast(float*)&c.normalImpulse2);
		const(float)* tangentImpulse1 = cast(const(float)*) (cast(float*)&c.tangentImpulse1);
		const(float)* tangentImpulse2 = cast(const(float)*) (cast(float*)&c.tangentImpulse2);
		const(float)* totalNormalImpulse1 = cast(const(float)*) (cast(float*)&c.totalNormalImpulse1);
		const(float)* totalNormalImpulse2 = cast(const(float)*) (cast(float*)&c.totalNormalImpulse2);
		const(float)* normalVelocity1 = cast(const(float)*) (cast(float*)&c.relativeVelocity1);
		const(float)* normalVelocity2 = cast(const(float)*) (cast(float*)&c.relativeVelocity2);

		int baseIndex = B2_SIMD_WIDTH * constraintIndex;

		for ( int laneIndex = 0; laneIndex < B2_SIMD_WIDTH; ++laneIndex )
		{
			b2Manifold* m = contacts[baseIndex + laneIndex] == null ? &dummy : &contacts[baseIndex + laneIndex].manifold;
			m.rollingImpulse = rollingImpulse[laneIndex];

			m.points[0].normalImpulse = normalImpulse1[laneIndex];
			m.points[0].tangentImpulse = tangentImpulse1[laneIndex];
			m.points[0].totalNormalImpulse = totalNormalImpulse1[laneIndex];
			m.points[0].normalVelocity = normalVelocity1[laneIndex];

			m.points[1].normalImpulse = normalImpulse2[laneIndex];
			m.points[1].tangentImpulse = tangentImpulse2[laneIndex];
			m.points[1].totalNormalImpulse = totalNormalImpulse2[laneIndex];
			m.points[1].normalVelocity = normalVelocity2[laneIndex];
		}
	}
}
