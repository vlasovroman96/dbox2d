module dbox2d.math_functions;

public import std.math.constants;

import dbox2d.base;

import core.stdc.math;

static assert( int.sizeof == int.sizeof, "Box2D expects int32_t and int to be the same" );

/// 2D vector
/// This can be used to represent a point or free vector
struct b2Vec2 {
	/// coordinates
	float x = 0, y = 0;

    auto opUnary(string op : "+")(b2Vec2 b) {
        this.x += b.x;
        this.y += b.y;
    }

    auto opUnary(string op : "-")(b2Vec2 b) {
        this.x += b.x;
        this.y += b.y;
    }

    auto opUnary(string op : "-")() {
        return b2Vec2(-this.x, -this.y);
    }

    auto opUnary(string op : "*")(float b) {
        this.x *= b;
        this.y *= b;
    }

    auto opBinary(string op : "+")(b2Vec2 b) {
        return b2Vec2(this.x + b.x, this.y + b.y);
    }

    auto opBinary(string op: "-")(b2Vec2 b) {
        return b2Vec2(this.x - b.x, this.y - b.y);
    }

    bool opEquals(b2Vec2 b) {
        return this.x == b.x && this.y == b.y;
    }

	static b2Vec2 zero() {
		return b2Vec2(0, 0);
	}
}

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
struct b2CosSin {
	/// cosine and sine
	float cosine = 0;
	float sine = 0;
}

/// 2D rotation
/// This is similar to using a complex number for rotation
struct b2Rot {
	/// cosine and sine
	float c = 0, s = 0;
}

/// A 2D rigid transform
struct b2Transform {
	b2Vec2 p;
	b2Rot q;
}

/// A 2-by-2 Matrix
struct b2Mat22 {
	/// columns
	b2Vec2 cx, cy;
}

/// Axis-aligned bounding box
struct b2AABB {
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
}

/// separation = dot(normal, point) - offset
struct b2Plane {
	b2Vec2 normal;
	float offset = 0;
}

const(b2Rot) b2Rot_identity = { 1.0f, 0.0f };
const(b2Transform) b2Transform_identity = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };
const(b2Mat22) b2Mat22_zero = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };

/// @return the minimum of two integers
int b2MinInt(int a, int b)
{
	return a < b ? a : b;
}

/// @return the maximum of two integers
int b2MaxInt(int a, int b)
{
	return a > b ? a : b;
}

/// @return the absolute value of an integer
int b2AbsInt(int a)
{
	return a < 0 ? -a : a;
}

/// @return an integer clamped between a lower and upper bound
int b2ClampInt(int a, int lower, int upper)
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// @return the minimum of two floats
float b2MinFloat(float a, float b)
{
	return a < b ? a : b;
}

/// @return the maximum of two floats
float b2MaxFloat(float a, float b)
{
	return a > b ? a : b;
}

/// @return the absolute value of a float
float b2AbsFloat(float a)
{
	return a < 0 ? -a : a;
}

/// @return a float clamped between a lower and upper bound
float b2ClampFloat(float a, float lower, float upper)
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// Vector dot product
float b2Dot(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
float b2Cross(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
b2Vec2 b2CrossVS( b2Vec2 v, float s )
{
	return b2Vec2( s * v.y, -s * v.x );
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
b2Vec2 b2CrossSV(float s, b2Vec2 v)
{
	return b2Vec2( -s * v.y, s * v.x );
}

/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
b2Vec2 b2LeftPerp(b2Vec2 v)
{
	return b2Vec2( -v.y, v.x );
}

/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
b2Vec2 b2RightPerp( b2Vec2 v )
{
	return b2Vec2( v.y, -v.x );
}

/// Vector addition
b2Vec2 b2Add( b2Vec2 a, b2Vec2 b )
{
	return b2Vec2( a.x + b.x, a.y + b.y );
}

/// Vector subtraction
b2Vec2 b2Sub( b2Vec2 a, b2Vec2 b )
{
	return b2Vec2( a.x - b.x, a.y - b.y );
}

/// Vector negation
b2Vec2 b2Neg(b2Vec2 a)
{
	return b2Vec2( -a.x, -a.y );
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t)
{
	return b2Vec2( ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y );
}

/// Component-wise multiplication
b2Vec2 b2Mul( b2Vec2 a, b2Vec2 b )
{
	return b2Vec2( a.x * b.x, a.y * b.y );
}

/// Multiply a scalar and vector
b2Vec2 b2MulSV( float s, b2Vec2 v )
{
	return b2Vec2( s * v.x, s * v.y );
}

/// a + s * b
b2Vec2 b2MulAdd( b2Vec2 a, float s, b2Vec2 b )
{
	return b2Vec2( a.x + s * b.x, a.y + s * b.y );
}

/// a - s * b
b2Vec2 b2MulSub( b2Vec2 a, float s, b2Vec2 b )
{
	return b2Vec2( a.x - s * b.x, a.y - s * b.y );
}

/// Component-wise absolute vector
b2Vec2 b2Abs(b2Vec2 a)
{
	b2Vec2 b = void;
	b.x = b2AbsFloat( a.x );
	b.y = b2AbsFloat( a.y );
	return b;
}

/// Component-wise minimum vector
b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = void;
	c.x = b2MinFloat( a.x, b.x );
	c.y = b2MinFloat( a.y, b.y );
	return c;
}

/// Component-wise maximum vector
b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = void;
	c.x = b2MaxFloat( a.x, b.x );
	c.y = b2MaxFloat( a.y, b.y );
	return c;
}

/// Component-wise clamp vector v into the range [a, b]
b2Vec2 b2Clamp(b2Vec2 v, b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = void;
	c.x = b2ClampFloat( v.x, a.x, b.x );
	c.y = b2ClampFloat( v.y, a.y, b.y );
	return c;
}

/// Get the length of this vector (the norm)
float b2Length(b2Vec2 v)
{
	return sqrtf( v.x * v.x + v.y * v.y );
}

/// Get the distance between two points
float b2Distance(b2Vec2 a, b2Vec2 b)
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrtf( dx * dx + dy * dy );
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
/// todo MSVC is not inlining this function in several places per warning 4710
b2Vec2 b2Normalize( b2Vec2 v )
{
	float length = sqrtf( v.x * v.x + v.y * v.y );
	if ( length < float.epsilon )
	{
		return b2Vec2( 0.0f, 0.0f );
	}

	float invLength = 1.0f / length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

/// Determines if the provided vector is normalized (norm(a) == 1).
bool b2IsNormalized(b2Vec2 a)
{
	float aa = b2Dot( a, a );
	return b2AbsFloat( 1.0f - aa ) < 100.0f * float.epsilon;
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
/// outputs the length.
b2Vec2 b2GetLengthAndNormalize( float* length, b2Vec2 v )
{
	*length = sqrtf( v.x * v.x + v.y * v.y );
	if ( *length < float.epsilon )
	{
		return b2Vec2( 0.0f, 0.0f );
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

/// Normalize rotation
b2Rot b2NormalizeRot(b2Rot q)
{
	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaAngle the angular displacement in radians
b2Rot b2IntegrateRotation(b2Rot q1, float deltaAngle)
{
	// dc/dt = -omega * sin(t)
	// ds/dt = omega * cos(t)
	// c2 = c1 - omega * h * s1
	// s2 = s1 + omega * h * c1
	b2Rot q2 = { q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c };
	float mag = sqrtf( q2.s * q2.s + q2.c * q2.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q2.c * invMag, q2.s * invMag };
	return qn;
}

/// Get the length squared of this vector
float b2LengthSquared(b2Vec2 v)
{
	return v.x * v.x + v.y * v.y;
}

/// Get the distance squared between points
float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = { b.x - a.x, b.y - a.y };
	return c.x * c.x + c.y * c.y;
}

/// Make a rotation using an angle in radians
b2Rot b2MakeRot( float radians )
{
	b2CosSin cs = b2ComputeCosSin( radians );
	return b2Rot( cs.cosine, cs.sine );
}

/// Make a rotation using a unit vector
b2Rot b2MakeRotFromUnitVector( b2Vec2 unitVector )
{
	B2_ASSERT( b2IsNormalized( unitVector ) );
	return b2Rot( unitVector.x, unitVector.y );
}

/// Compute the rotation between two unit vectors
b2Rot b2ComputeRotationBetweenUnitVectors(b2Vec2 v1, b2Vec2 v2);

/// Is this rotation normalized?
bool b2IsNormalizedRot(b2Rot q)
{
	// larger tolerance due to failure on mingw 32-bit
	float qq = q.s * q.s + q.c * q.c;
	return 1.0f - 0.0006f < qq && qq < 1.0f + 0.0006f;
}

/// Normalized linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
///	https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
b2Rot b2NLerp(b2Rot q1, b2Rot q2, float t)
{
	float omt = 1.0f - t;
	b2Rot q = {
		omt * q1.c + t * q2.c,
		omt * q1.s + t * q2.s,
	};

	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// Compute the angular velocity necessary to rotate between two rotations over a give time
/// @param q1 initial rotation
/// @param q2 final rotation
/// @param inv_h inverse time step
float b2ComputeAngularVelocity(b2Rot q1, b2Rot q2, float inv_h)
{
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1

	// omega * h * s1 = c1 - c2
	// omega * h * c1 = s2 - s1
	// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
	// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
	// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
	float omega = inv_h * ( q2.s * q1.c - q2.c * q1.s );
	return omega;
}

/// Get the angle in radians in the range [-pi, pi]
float b2Rot_GetAngle(b2Rot q)
{
	return b2Atan2( q.s, q.c );
}

/// Get the x-axis
b2Vec2 b2Rot_GetXAxis(b2Rot q)
{
	b2Vec2 v = { q.c, q.s };
	return v;
}

/// Get the y-axis
b2Vec2 b2Rot_GetYAxis(b2Rot q)
{
	b2Vec2 v = { -q.s, q.c };
	return v;
}

/// Multiply two rotations: q * r
b2Rot b2MulRot(b2Rot q, b2Rot r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s(q + r) = qs * rc + qc * rs
	// c(q + r) = qc * rc - qs * rs
	b2Rot qr = void;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: inv(a) * b
/// This rotates a vector local in frame b into a vector local in frame a
b2Rot b2InvMulRot(b2Rot a, b2Rot b)
{
	// [ ac as] * [bc -bs] = [ac*bc+qs*bs -ac*bs+as*bc]
	// [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
	// s(a - b) = ac * bs - as * bc
	// c(a - b) = ac * bc + as * bs
	b2Rot r = void;
	r.s = a.c * b.s - a.s * b.c;
	r.c = a.c * b.c + a.s * b.s;
	return r;
}

/// Relative angle between a and b
float b2RelativeAngle(b2Rot a, b2Rot b)
{
	// sin(b - a) = bs * ac - bc * as
	// cos(b - a) = bc * ac + bs * as
	float s = a.c * b.s - a.s * b.c;
	float c = a.c *b.c + a.s * b.s;
	return b2Atan2( s, c );
}

/// Convert any angle into the range [-pi, pi]
float b2UnwindAngle(float radians)
{
	// Assuming this is deterministic
	return remainderf( radians, 2.0f * PI );
}

/// Rotate a vector
b2Vec2 b2RotateVector( b2Rot q, b2Vec2 v )
{
	return b2Vec2( q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y );
}

/// Inverse rotate a vector
b2Vec2 b2InvRotateVector( b2Rot q, b2Vec2 v )
{
	return b2Vec2( q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y );
}

/// Transform a point (e.g. local space to world space)
b2Vec2 b2TransformPoint( b2Transform t, const(b2Vec2) p )
{
	float x = ( t.q.c * p.x - t.q.s * p.y ) + t.p.x;
	float y = ( t.q.s * p.x + t.q.c * p.y ) + t.p.y;

	return b2Vec2( x, y );
}

/// Inverse transform a point (e.g. world space to local space)
b2Vec2 b2InvTransformPoint(b2Transform t, const(b2Vec2) p)
{
	float vx = p.x - t.p.x;
	float vy = p.y - t.p.y;
	return b2Vec2( t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy );
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame.
/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
b2Transform b2MulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C = void;
	C.q = b2MulRot( A.q, B.q );
	C.p = b2Add( b2RotateVector( A.q, B.p ), A.p );
	return C;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
b2Transform b2InvMulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C = void;
	C.q = b2InvMulRot( A.q, B.q );
	C.p = b2InvRotateVector( A.q, b2Sub( B.p, A.p ) );
	return C;
}

/// Multiply a 2-by-2 matrix times a 2D vector
b2Vec2 b2MulMV(b2Mat22 A, b2Vec2 v)
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

/// Get the inverse of a 2-by-2 matrix
b2Mat22 b2GetInverse22(b2Mat22 A)
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}

		b2Mat22 B = {
			{ det * d, -det * c },
			{ -det * b, det * a },
		};
		return B;
	}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
b2Vec2 b2Solve22(b2Mat22 A, b2Vec2 b)
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}
	b2Vec2 x = { det * ( a22 * b.x - a12 * b.y ), det * ( a11 * b.y - a21 * b.x ) };
	return x;
}

/// Does a fully contain b
bool b2AABB_Contains(b2AABB a, b2AABB b)
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Get the center of the AABB.
b2Vec2 b2AABB_Center(b2AABB a)
{
	b2Vec2 b = { 0.5f * ( a.lowerBound.x + a.upperBound.x ), 0.5f * ( a.lowerBound.y + a.upperBound.y ) };
	return b;
}

/// Get the extents of the AABB (half-widths).
b2Vec2 b2AABB_Extents(b2AABB a)
{
	b2Vec2 b = { 0.5f * ( a.upperBound.x - a.lowerBound.x ), 0.5f * ( a.upperBound.y - a.lowerBound.y ) };
	return b;
}

/// Union of two AABBs
b2AABB b2AABB_Union(b2AABB a, b2AABB b)
{
	b2AABB c = void;
	c.lowerBound.x = b2MinFloat( a.lowerBound.x, b.lowerBound.x );
	c.lowerBound.y = b2MinFloat( a.lowerBound.y, b.lowerBound.y );
	c.upperBound.x = b2MaxFloat( a.upperBound.x, b.upperBound.x );
	c.upperBound.y = b2MaxFloat( a.upperBound.y, b.upperBound.y );
	return c;
}

/// Do a and b overlap
bool b2AABB_Overlaps(b2AABB a, b2AABB b)
{
	return !( b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x ||
			  a.lowerBound.y > b.upperBound.y );
}

/// Compute the bounding box of an array of circles
b2AABB b2MakeAABB(const(b2Vec2)* points, int count, float radius)
{
	B2_ASSERT( count > 0 );
	b2AABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = b2Min( a.lowerBound, points[i] );
		a.upperBound = b2Max( a.upperBound, points[i] );
	}

	b2Vec2 r = { radius, radius };
	a.lowerBound = b2Sub( a.lowerBound, r );
	a.upperBound = b2Add( a.upperBound, r );

	return a;
}

/// Signed separation of a point from a plane
float b2PlaneSeparation(b2Plane plane, b2Vec2 point)
{
	return b2Dot( plane.normal, point ) - plane.offset;
}

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += timeStep * newVelocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
float b2SpringDamper(float hertz, float dampingRatio, float position, float velocity, float timeStep)
{
	float omega = 2.0f * PI * hertz;
	float omegaH = omega * timeStep;
	return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
}

bool b2IsValidFloat(float a)
{
	if ( isnan( a ) )
	{
		return false;
	}

	if ( isinf( a ) )
	{
		return false;
	}

	return true;
}

bool b2IsValidVec2(b2Vec2 v)
{
	if ( isnan( v.x ) || isnan( v.y ) )
	{
		return false;
	}

	if ( isinf( v.x ) || isinf( v.y ) )
	{
		return false;
	}

	return true;
}

bool b2IsValidRotation(b2Rot q)
{
	if ( isnan( q.s ) || isnan( q.c ) )
	{
		return false;
	}

	if ( isinf( q.s ) || isinf( q.c ) )
	{
		return false;
	}

	return b2IsNormalizedRot( q );
}

bool b2IsValidTransform(b2Transform t)
{
	if (b2IsValidVec2(t.p) == false)
	{
		return false;
	}

	return b2IsValidRotation( t.q );
}

bool b2IsValidPlane(b2Plane a)
{
	return b2IsValidVec2( a.normal ) && b2IsNormalized( a.normal ) && b2IsValidFloat( a.offset );
}

// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
float b2Atan2(float y, float x)
{
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0.0f && y == 0.0f)
	{
		return 0.0f;
	}

	float ax = b2AbsFloat( x );
	float ay = b2AbsFloat( y );
	float mx = b2MaxFloat( ay, ax );
	float mn = b2MinFloat( ay, ax );
	float a = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	float s = a * a;
	float c = s * a;
	float q = s * s;
	float r = 0.024840285f * q + 0.18681418f;
	float t = -0.094097948f * q - 0.33213072f;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = 1.57079637f - r;
	}

	if ( x < 0 )
	{
		r = 3.14159274f - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return r;
}

// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
// this result.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b2CosSin b2ComputeCosSin(float radians)
{
	float x = b2UnwindAngle( radians );
	float pi2 = PI * PI;

	// cosine needs angle in [-pi/2, pi/2]
	float c = void;
	if ( x < -0.5f * PI )
	{
		float y = x + PI;
		float y2 = y * y;
		c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else if ( x > 0.5f * PI )
	{
		float y = x - PI;
		float y2 = y * y;
		c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else
	{
		float y2 = x * x;
		c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	float s = void;
	if ( x < 0.0f )
	{
		float y = x + PI;
		s = -16.0f * y * ( PI - y ) / ( 5.0f * pi2 - 4.0f * y * ( PI - y ) );
	}
	else
	{
		s = 16.0f * x * ( PI - x ) / ( 5.0f * pi2 - 4.0f * x * ( PI - x ) );
	}

	float mag = sqrtf( s * s + c * c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2CosSin cs = { c * invMag, s * invMag };
	return cs;
}

b2Rot b2ComputeRotationBetweenUnitVectors(b2Vec2 v1, b2Vec2 v2)
{
	B2_ASSERT( b2AbsFloat( 1.0f - b2Length( v1 ) ) < 100.0f * float.epsilon );
	B2_ASSERT( b2AbsFloat( 1.0f - b2Length( v2 ) ) < 100.0f * float.epsilon );

	b2Rot rot = void;
	rot.c = b2Dot( v1, v2 );
	rot.s = b2Cross( v1, v2 );
	return b2NormalizeRot( rot );
}
