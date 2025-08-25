module dbox2d.math.mat22;

import dbox2d.math.vector;
// import dbox2d.math.

/// A 2-by-2 Matrix
struct b2Mat22 {
	/// columns
	b2Vec2 cx, cy;

	static b2Mat22 zero() {
		return b2Mat22(b2Vec2(0 ,0), b2Vec2(0 ,0));
	} 
}

/// Multiply a 2-by-2 matrix times a 2D vector
b2Vec2 b2MulMV(b2Mat22 A, b2Vec2 v)
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

/// Get the inverse of a 2-by-2 matrix
b2Mat22 b2GetInverse22(b2Mat22 A)
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}

		b2Mat22 B = {
			{ det * d, -det * c },
			{ -det * b, det * a },
		};
		return B;
	}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
b2Vec2 b2Solve22(b2Mat22 A, b2Vec2 b)
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}
	b2Vec2 x = { det * ( a22 * b.x - a12 * b.y ), det * ( a11 * b.y - a21 * b.x ) };
	return x;
}
