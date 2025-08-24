module dbox2d.math.rotation;

/// 2D rotation
/// This is similar to using a complex number for rotation
struct b2Rot {
	/// cosine and sine
	float c = 0, s = 0;

	static b2Rot identity() {
		return b2Rot(1.0f, 0.0f);
	}
}
