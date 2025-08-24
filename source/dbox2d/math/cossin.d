module dbox2d.math.cossin;

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
struct b2CosSin {
	/// cosine and sine
	float cosine = 0;
	float sine = 0;
}
