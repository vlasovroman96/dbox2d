module dbox2d.math.cossin;

import dbox2d.math.funcs;
import dbox2d.math.rotation;
/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
struct b2CosSin {
	/// cosine and sine
	float cosine = 0;
	float sine = 0;

	// Approximate cosine and sine for determinism. In my testing cosf and sinf produced
	// the same results on x64 and ARM using MSVC, GCC, and Clang. However, I don't trust
	// this result.
	// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
	this(float radians) {
		float x = b2UnwindAngle( radians );
		enum pi2 = PI * PI;

		// cosine needs angle in [-pi/2, pi/2]
		float c;
		if ( x < -0.5f * PI ) {
			float y = x + PI;
			float y2 = y * y;
			c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
		}
		else if ( x > 0.5f * PI ) {
			float y = x - PI;
			float y2 = y * y;
			c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
		}
		else {
			float y2 = x * x;
			c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
		}

		// sine needs angle in [0, pi]
		float s;
		if ( x < 0.0f ) {
			float y = x + PI;
			s = -16.0f * y * ( PI - y ) / ( 5.0f * pi2 - 4.0f * y * ( PI - y ) );
		}
		else {
			s = 16.0f * x * ( PI - x ) / ( 5.0f * pi2 - 4.0f * x * ( PI - x ) );
		}

		float mag = sqrt( s * s + c * c );
		float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
		this.cosine = c * invMag;
		this.sine = s * invMag;
	}
}