module dbox2d.math.transform;

import dbox2d.math.funcs;
import dbox2d.math.rotation;
import dbox2d.math.vector;

/// A 2D rigid transform
struct b2Transform {
	b2Vec2 p;
	b2Rot q;

	static b2Transform identity() {
		auto _p = b2Vec2.init;
		auto _q = b2Rot.identity;

		return b2Transform(_p, _q);
	}
}

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
	C.p = B.p.getRotated( A.q ) + A.p;
	return C;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
b2Transform b2InvMulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C = void;
	C.q = b2InvMulRot( A.q, B.q );
	C.p = b2InvRotateVector( A.q, B.p - A.p );
	return C;
}


bool b2IsValidTransform(b2Transform t)
{
	if (t.p.isValid() == false)
	{
		return false;
	}

	return t.q.isValid();
}
