module dbox2d.math.funcs;

public import std.math;
public import std.algorithm;
public import std.stdint;

import dbox2d.base;
import dbox2d.math.vector;
import dbox2d.math.transform;
import dbox2d.math.cossin;
import dbox2d.math.rotation;
import dbox2d.math.mat22;

static assert( int32_t.sizeof == int.sizeof, "Box2D expects int32_t and int to be the same" );

/// One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
/// You can then compute the new position using:
/// position += timeStep * newVelocity
/// This drives towards a zero position. By using implicit integration we get a stable solution
/// that doesn't require transcendental functions.
float b2SpringDamper(float hertz, float dampingRatio, float position, float velocity, float timeStep)
{
	float omega = 2.0f * PI * hertz;
	float omegaH = omega * timeStep;
	return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
}

bool b2IsValidFloat(float a)
{
	if ( isNaN( a ) )
	{
		return false;
	}

	if ( isInfinity( a ) )
	{
		return false;
	}

	return true;
}

// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
float b2Atan2(float y, float x)
{
	// Added check for (0,0) to match atan2f and avoid NaN
	if (x == 0.0f && y == 0.0f)
	{
		return 0.0f;
	}

	float ax = abs( x );
	float ay = abs( y );
	float mx = max( ay, ax );
	float mn = min( ay, ax );
	float a = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	float s = a * a;
	float c = s * a;
	float q = s * s;
	float r = 0.024840285f * q + 0.18681418f;
	float t = -0.094097948f * q - 0.33213072f;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = 1.57079637f - r;
	}

	if ( x < 0 )
	{
		r = 3.14159274f - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return r;
}

