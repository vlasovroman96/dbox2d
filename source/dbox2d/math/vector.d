module dbox2d.math.vector;

import std.math;
import std.algorithm;

import dbox2d.math.rotation;

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

    auto opUnary(string op : "-")() const {
        return b2Vec2(-this.x, -this.y);
    }

    auto opUnary(string op : "*")(float b) {
        this.x *= b;
        this.y *= b;
    }

    auto opBinary(string op : "+")(b2Vec2 b) {
        return b2Vec2(this.x + b.x, this.y + b.y);
    }

    auto opBinary(string op: "-")(const b2Vec2 b) const {
        return b2Vec2(this.x - b.x, this.y - b.y);
    }

    bool opEquals(b2Vec2 b) {
        return this.x == b.x && this.y == b.y;
    }

	static b2Vec2 zero() {
		return b2Vec2(0, 0);
	}

	auto leftPerp() {
		return b2Vec2( -y, x );
	}

	/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
	auto rightPerp() const {
		return b2Vec2( y, -x );
	}

	/// Get the length of this vector (the norm)
	float length() {
		return hypot(this.x, this.y);
	}

    /// Vector dot product
    float dot( b2Vec2 b ) const {
        return this.x * b.x + this.y * b.y;
    }

    /// Vector cross product. In 2D this yields a scalar.
    float cross( b2Vec2 b) const {
        return this.x * b.y - this.y * b.x;
    }

    auto mul(b2Vec2 a) {
        return b2Vec2(this.x * a.x, this.y * a.y);
    }

    /// Component-wise absolute vector
    auto abs() {
        b2Vec2 b;
        b.x = std.math.abs( this.x );
        b.y = std.math.abs( this.y );
        return b;
    }

    /// Component-wise clamp vector v into the range [a, b]
    b2Vec2 clamp(b2Vec2 a, b2Vec2 b) {
        b2Vec2 c;
        c.x = std.algorithm.clamp( this.x, a.x, b.x );
        c.y = std.algorithm.clamp( this.y, a.y, b.y );
        return c;
    }

    /// Get the distance between two points
    float distanceTo(b2Vec2 a) {
        float dx = a.x - this.x;
        float dy = a.y - this.y;
        return sqrt( dx * dx + dy * dy );
    }

    /// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
    /// todo MSVC is not inlining this function in several places per warning 4710
    auto normalized() {
        float length = sqrt( this.x * this.x + this.y * this.y );
        if ( length < float.epsilon )
        {
            return b2Vec2( 0.0f, 0.0f );
        }

        float invLength = 1.0f / length;
        b2Vec2 n = { invLength * this.x, invLength * this.y };
        return n;
    }

    /// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
    b2Vec2 crossVS( float s ) {
        return b2Vec2( s * this.y, -s * this.x );
    }

    /// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
    b2Vec2 crossSV(float s) {
        return b2Vec2( -s * this.y, s * this.x );
    }

    /// Rotate a vector
    b2Vec2 getRotated( b2Rot q ) const {
        return b2Vec2( q.c * this.x - q.s * this.y, q.s * this.x + q.c * this.y );
    }

    /// Get the length squared of this vector
    float lengthSquared() {
        return this.x * this.x + this.y * this.y;
    }
}

/// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
b2Vec2 b2CrossVS( b2Vec2 v, float s )
{
	return v.crossVS(s);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
b2Vec2 b2CrossSV(float s, b2Vec2 v) {
	return v.crossSV(s);
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t)
{
	return b2Vec2( ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y );
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

/// Component-wise minimum vector
b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = void;
	c.x = min( a.x, b.x );
	c.y = min( a.y, b.y );
	return c;
}

/// Component-wise maximum vector
b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = void;
	c.x = max( a.x, b.x );
	c.y = max( a.y, b.y );
	return c;
}

/// Determines if the provided vector is normalized (norm(a) == 1).
bool b2IsNormalized(b2Vec2 a)
{
	float aa = a.dot( a );
	return abs( 1.0f - aa ) < 100.0f * float.epsilon;
}


bool b2IsValidVec2(b2Vec2 v)
{
	if ( isNaN( v.x ) || isNaN( v.y ) )
	{
		return false;
	}

	if ( isInfinity( v.x ) || isInfinity( v.y ) )
	{
		return false;
	}

	return true;
}

/// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
/// outputs the length.
b2Vec2 b2GetLengthAndNormalize( float* length, b2Vec2 v )
{
	*length = sqrt( v.x * v.x + v.y * v.y );
	if ( *length < float.epsilon )
	{
		return b2Vec2( 0.0f, 0.0f );
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = { invLength * v.x, invLength * v.y };
	return n;
}

/// Get the distance squared between points
float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = { b.x - a.x, b.y - a.y };
	return c.x * c.x + c.y * c.y;
}
