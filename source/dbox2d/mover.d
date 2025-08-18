module dbox2d.mover;
import dbox2d.constants;

import dbox2d.collision;

b2PlaneSolverResult b2SolvePlanes(b2Vec2 targetDelta, b2CollisionPlane* planes, int count)
{
	for ( int i = 0; i < count; ++i )
	{
		planes[i].push = 0.0f;
	}

	b2Vec2 delta = targetDelta;
	float tolerance = B2_LINEAR_SLOP;

	int iteration = void;
	for ( iteration = 0; iteration < 20; ++iteration )
	{
		float totalPush = 0.0f;
		for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
		{
			b2CollisionPlane* plane = planes + planeIndex;

			// Add slop to prevent jitter
			float separation = b2PlaneSeparation( plane.plane, delta ) + B2_LINEAR_SLOP;
			// if (separation > 0.0f)
			//{
			//	continue;
			// }

			float push = -separation;

			// Clamp accumulated push
			float accumulatedPush = plane.push;
			plane.push = clamp( plane.push + push, 0.0f, plane.pushLimit );
			push = plane.push - accumulatedPush;
			delta = b2MulAdd( delta, push, plane.plane.normal );

			// Track maximum push for convergence
			totalPush += abs( push );
		}

		if ( totalPush < tolerance )
		{
			break;
		}
	}

	return b2PlaneSolverResult(
		translation: delta,
		iterationCount: iteration,
	);
}

b2Vec2 b2ClipVector(b2Vec2 vector, const(b2CollisionPlane)* planes, int count)
{
	b2Vec2 v = vector;

	for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
	{
		const(b2CollisionPlane)* plane = planes + planeIndex;
		if ( plane.push == 0.0f || plane.clipVelocity == false )
		{
			continue;
		}

		v = b2MulSub( v, min( 0.0f, b2Dot( v, plane.plane.normal ) ), plane.plane.normal );
	}

	return v;
}
