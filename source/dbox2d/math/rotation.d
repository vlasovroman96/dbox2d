module dbox2d.math.rotation;

import std.math;

import dbox2d.base;
import dbox2d.math.funcs;
import dbox2d.math.cossin;
import dbox2d.math.vector;

/// 2D rotation
/// This is similar to using a complex number for rotation
struct b2Rot {
	/// cosine and sine
	float c = 0, s = 0;

	static b2Rot identity() {
		return b2Rot(1.0f, 0.0f);
	}

	/// Normalize rotation
	b2Rot getNormalized()
	{
		float mag = sqrt( this.s * this.s + this.c * this.c );
		float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
		b2Rot qn = { this.c * invMag, this.s * invMag };
		return qn;
	} 
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
	float mag = sqrt( q2.s * q2.s + q2.c * q2.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q2.c * invMag, q2.s * invMag };
	return qn;
}

/// Make a rotation using an angle in radians
b2Rot b2MakeRot( float radians )
{
	auto cs = b2CosSin( radians );
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

	float mag = sqrt( q.s * q.s + q.c * q.c );
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
	return remainder( radians, 2.0f * PI );
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


bool b2IsValidRotation(b2Rot q)
{
	if ( isNaN( q.s ) || isNaN( q.c ) )
	{
		return false;
	}

	if ( isInfinity( q.s ) || isInfinity( q.c ) )
	{
		return false;
	}

	return b2IsNormalizedRot( q );
}


b2Rot b2ComputeRotationBetweenUnitVectors(b2Vec2 v1, b2Vec2 v2)
{
	B2_ASSERT( abs( 1.0f - v1.length() ) < 100.0f * float.epsilon );
	B2_ASSERT( abs( 1.0f - v2.length() ) < 100.0f * float.epsilon );

	b2Rot rot = void;
	rot.c = v1.dot( v2 );
	rot.s = v1.cross( v2 );
	return rot.getNormalized();
}