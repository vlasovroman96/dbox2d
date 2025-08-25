module dbox2d.math.aabb;

import dbox2d.types;
import dbox2d.math.vector;
import dbox2d.math.funcs;
import dbox2d.collision;
import dbox2d.core;
import dbox2d.base;

/// Axis-aligned bounding box
struct b2AABB {
	b2Vec2 lowerBound;
	b2Vec2 upperBound;

	auto perimeter() const {
		auto wx = this.upperBound.x - this.lowerBound.x;
		auto wy = this.upperBound.y - this.lowerBound.y;
		return 2.0f * ( wx + wy );
	}

	/// Enlarge a to contain b
	/// @return true if the AABB grew
	bool enlargeAABB(b2AABB b) {
		bool changed = false;

		if ( b.lowerBound.x < this.lowerBound.x ) {
			this.lowerBound.x = b.lowerBound.x;
			changed = true;
		}

		if ( b.lowerBound.y < this.lowerBound.y ) {
			this.lowerBound.y = b.lowerBound.y;
			changed = true;
		}

		if ( this.upperBound.x < b.upperBound.x ) {
			this.upperBound.x = b.upperBound.x;
			changed = true;
		}

		if ( this.upperBound.y < b.upperBound.y ) {
			this.upperBound.y = b.upperBound.y;
			changed = true;
		}

		return changed;
	}

	bool isValid()	{
		b2Vec2 d = this.upperBound - this.lowerBound;
		bool valid = d.x >= 0.0f && d.y >= 0.0f;
		valid = valid && b2IsValidVec2( this.lowerBound ) && b2IsValidVec2( this.upperBound );
		return valid;
	}

	// From Real-time Collision Detection, p179.
	b2CastOutput rayCast(b2Vec2 p1, b2Vec2 p2) {
		// Radius not handled
		b2CastOutput output;

		float tmin = -float.max;
		float tmax = float.max;

		b2Vec2 p = p1;
		b2Vec2 d = p2 - p1 ;
		b2Vec2 absD = d.abs();

		b2Vec2 normal = b2Vec2.zero();

		// x-coordinate
		if ( absD.x < float.epsilon ) {
			// parallel
			if ( p.x < this.lowerBound.x || this.upperBound.x < p.x ) {
				return output;
			}
		}
		else {
			float inv_d = 1.0f / d.x;
			float t1 = ( this.lowerBound.x - p.x ) * inv_d;
			float t2 = ( this.upperBound.x - p.x ) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if ( t1 > t2 ) {
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
				s = 1.0f;
			}

			// Push the min up
			if ( t1 > tmin ) {
				normal.y = 0.0f;
				normal.x = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = min( tmax, t2 );

			if ( tmin > tmax ) {
				return output;
			}
		}

		// y-coordinate
		if ( absD.y < float.epsilon ) {
			// parallel
			if ( p.y < this.lowerBound.y || this.upperBound.y < p.y ) {
				return output;
			}
		}
		else {
			float inv_d = 1.0f / d.y;
			float t1 = ( this.lowerBound.y - p.y ) * inv_d;
			float t2 = ( this.upperBound.y - p.y ) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if ( t1 > t2 ) {
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
				s = 1.0f;
			}

			// Push the min up
			if ( t1 > tmin ) {
				normal.x = 0.0f;
				normal.y = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = min( tmax, t2 );

			if ( tmin > tmax ) {
				return output;
			}
		}

		// Does the ray start inside the box?
		// Does the ray intersect beyond the max fraction?
		if ( tmin < 0.0f || 1.0f < tmin ) {
			return output;
		}

		// Intersection.
		output.fraction = tmin;
		output.normal = normal;
		output.point = b2Lerp( p1, p2, tmin );
		output.hit = true;
		return output;
	}

	/// Does a fully contain b
	bool contains(b2AABB b) {
		bool s = true;
		s = s && this.lowerBound.x <= b.lowerBound.x;
		s = s && this.lowerBound.y <= b.lowerBound.y;
		s = s && b.upperBound.x <= this.upperBound.x;
		s = s && b.upperBound.y <= this.upperBound.y;
		return s;
	}

	/// Get the center of the AABB.
	b2Vec2 center() const {
		b2Vec2 b = { 0.5f * ( this.lowerBound.x + this.upperBound.x ), 0.5f * ( this.lowerBound.y + this.upperBound.y ) };
		return b;
	}

	/// Get the extents of the AABB (half-widths).
	b2Vec2 extents() const {
		b2Vec2 b = { 0.5f * ( this.upperBound.x - this.lowerBound.x ), 0.5f * ( this.upperBound.y - this.lowerBound.y ) };
		return b;
	}

	/// Union of two AABBs
	b2AABB _union(b2AABB b) {
		b2AABB c;
		c.lowerBound.x = min( this.lowerBound.x, b.lowerBound.x );
		c.lowerBound.y = min( this.lowerBound.y, b.lowerBound.y );
		c.upperBound.x = max( this.upperBound.x, b.upperBound.x );
		c.upperBound.y = max( this.upperBound.y, b.upperBound.y );
		return c;
	}
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
	a.lowerBound = a.lowerBound - r;
	a.upperBound = a.upperBound + r;

	return a;
}
