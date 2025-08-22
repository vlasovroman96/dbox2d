module dbox2d.math.vector;

import std.math;

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
}
