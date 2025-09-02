module dbox2d.math.plane;

import dbox2d.math.vector;
import dbox2d.math.funcs;

/// separation = dot(normal, point) - offset
struct b2Plane {
	b2Vec2 normal;
	float offset = 0;
}

bool b2IsValidPlane(b2Plane a)
{
	return a.normal.isValid() && a.normal.isNormalized() && b2IsValidFloat( a.offset );
}

/// Signed separation of a point from a plane
float b2PlaneSeparation(b2Plane plane, b2Vec2 point)
{
	return plane.normal.dot( point ) - plane.offset;
}

