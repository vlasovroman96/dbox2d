module dbox2d.math.transform;

import dbox2d.math.math_functions;
import dbox2d.math.rotation;
import dbox2d.math.vector;

/// A 2D rigid transform
struct b2Transform {
	b2Vec2 p;
	b2Rot q;

	static b2Transform identity() {
		auto _p = b2Vec2.zero;
		auto _q = b2Rot.identity;

		return b2Transform(_p, _q);
	}
}
