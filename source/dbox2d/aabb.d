module dbox2d.aabb;

public import dbox2d.types;

// Ray cast an AABB
b2CastOutput b2AABB_RayCast(b2AABB a, b2Vec2 p1, b2Vec2 p2);

// Get surface area of an AABB (the perimeter length)
pragma(inline, true) float b2Perimeter(b2AABB a)
{
	float wx = a.upperBound.x - a.lowerBound.x;
	float wy = a.upperBound.y - a.lowerBound.y;
	return 2.0f * ( wx + wy );
}

/// Enlarge a to contain b
/// @return true if the AABB grew
pragma(inline, true) bool b2EnlargeAABB(b2AABB* a, b2AABB b)
{
	bool changed = false;
	if ( b.lowerBound.x < a.lowerBound.x )
	{
		a.lowerBound.x = b.lowerBound.x;
		changed = true;
	}

	if ( b.lowerBound.y < a.lowerBound.y )
	{
		a.lowerBound.y = b.lowerBound.y;
		changed = true;
	}

	if ( a.upperBound.x < b.upperBound.x )
	{
		a.upperBound.x = b.upperBound.x;
		changed = true;
	}

	if ( a.upperBound.y < b.upperBound.y )
	{
		a.upperBound.y = b.upperBound.y;
		changed = true;
	}

	return changed;
}

bool b2IsValidAABB(b2AABB a)
{
	b2Vec2 d = b2Sub( a.upperBound, a.lowerBound );
	bool valid = d.x >= 0.0f && d.y >= 0.0f;
	valid = valid && b2IsValidVec2( a.lowerBound ) && b2IsValidVec2( a.upperBound );
	return valid;
}

// From Real-time Collision Detection, p179.
b2CastOutput b2AABB_RayCast(b2AABB a, b2Vec2 p1, b2Vec2 p2)
{
	// Radius not handled
	b2CastOutput output;

	float tmin = -float.max;
	float tmax = float.max;

	b2Vec2 p = p1;
	b2Vec2 d = b2Sub( p2, p1 );
	b2Vec2 absD = b2Abs( d );

	b2Vec2 normal = b2Vec2_zero;

	// x-coordinate
	if ( absD.x < float.epsilon )
	{
		// parallel
		if ( p.x < a.lowerBound.x || a.upperBound.x < p.x )
		{
			return output;
		}
	}
	else
	{
		float inv_d = 1.0f / d.x;
		float t1 = ( a.lowerBound.x - p.x ) * inv_d;
		float t2 = ( a.upperBound.x - p.x ) * inv_d;

		// Sign of the normal vector.
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		// Push the min up
		if ( t1 > tmin )
		{
			normal.y = 0.0f;
			normal.x = s;
			tmin = t1;
		}

		// Pull the max down
		tmax = b2MinFloat( tmax, t2 );

		if ( tmin > tmax )
		{
			return output;
		}
	}

	// y-coordinate
	if ( absD.y < float.epsilon )
	{
		// parallel
		if ( p.y < a.lowerBound.y || a.upperBound.y < p.y )
		{
			return output;
		}
	}
	else
	{
		float inv_d = 1.0f / d.y;
		float t1 = ( a.lowerBound.y - p.y ) * inv_d;
		float t2 = ( a.upperBound.y - p.y ) * inv_d;

		// Sign of the normal vector.
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		// Push the min up
		if ( t1 > tmin )
		{
			normal.x = 0.0f;
			normal.y = s;
			tmin = t1;
		}

		// Pull the max down
		tmax = b2MinFloat( tmax, t2 );

		if ( tmin > tmax )
		{
			return output;
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if ( tmin < 0.0f || 1.0f < tmin )
	{
		return output;
	}

	// Intersection.
	output.fraction = tmin;
	output.normal = normal;
	output.point = b2Lerp( p1, p2, tmin );
	output.hit = true;
	return output;
}

