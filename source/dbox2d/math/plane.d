module dbox2d.math.plane;

import dbox2d.math.vector;
import dbox2d.math.funcs;

/// separation = dot(normal, point) - offset
struct b2Plane {
	b2Vec2 normal;
	float offset = 0;

	bool isValid() {
		return this.normal.isValid() && this.normal.isNormalized() && this.offset.b2IsValidFloat();
	}
}

/// Signed separation of a point from a plane
float b2PlaneSeparation(b2Plane plane, b2Vec2 point)
{
	return plane.normal.dot( point ) - plane.offset;
}

