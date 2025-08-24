module dbox2d.math.mat22;

import dbox2d.math.vector;

/// A 2-by-2 Matrix
struct b2Mat22 {
	/// columns
	b2Vec2 cx, cy;

	static b2Mat22 zero() {
		return b2Mat22(b2Vec2(0 ,0), b2Vec2(0 ,0));
	} 
}
