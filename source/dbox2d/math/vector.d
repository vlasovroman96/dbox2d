module dbox2d.math.vector;

import std.math;
import std.algorithm;

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
