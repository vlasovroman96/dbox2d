module dbox2d.timer;

import dbox2d.base;

import core.sys.posix.sched;
import core.stdc.time;

import dbox2d.core;
import dbox2d.base;
import dbox2d.ct.templates;

import core.stdc.stddef;

// version (_MSC_VER) {

// enum WIN32_LEAN_AND_MEAN = 1;


// import core.sys.windows.windows;

// private double s_invFrequency = 0.0;

// ulong b2GetTicks()
// {
// 	LARGE_INTEGER counter = void;
// 	QueryPerformanceCounter( &counter );
// 	return cast(ulong)counter.QuadPart;
// }

// float b2GetMilliseconds(ulong ticks)
// {
// 	if ( s_invFrequency == 0.0 )
// 	{
// 		LARGE_INTEGER frequency = void;
// 		QueryPerformanceFrequency( &frequency );

// 		s_invFrequency = cast(double)frequency.QuadPart;
// 		if ( s_invFrequency > 0.0 )
// 		{
// 			s_invFrequency = 1000.0 / s_invFrequency;
// 		}
// 	}

// 	ulong ticksNow = b2GetTicks();
// 	return cast(float)( s_invFrequency * ( ticksNow - ticks ) );
// }

// float b2GetMillisecondsAndReset(ulong* ticks)
// {
// 	if ( s_invFrequency == 0.0 )
// 	{
// 		LARGE_INTEGER frequency = void;
// 		QueryPerformanceFrequency( &frequency );

// 		s_invFrequency = cast(double)frequency.QuadPart;
// 		if ( s_invFrequency > 0.0 )
// 		{
// 			s_invFrequency = 1000.0 / s_invFrequency;
// 		}
// 	}

// 	ulong ticksNow = b2GetTicks();
// 	float ms = cast(float)( s_invFrequency * ( ticksNow - *ticks ) );
// 	*ticks = ticksNow;
// 	return ms;
// }

// void b2Yield()
// {
// 	SwitchToThread();
// }

// struct b2Mutex {
// 	CRITICAL_SECTION cs;
// }

// b2Mutex* b2CreateMutex()
// {
// 	b2Mutex* m = b2Alloc( b2Mutex.sizeof );
// 	InitializeCriticalSection( &m.cs );
// 	return m;
// }

// void b2DestroyMutex(b2Mutex* m)
// {
// 	DeleteCriticalSection( &m.cs );
// 	*m = b2Mutex( 0 );
// 	b2Free( m, b2Mutex.sizeof );
// }

// void b2LockMutex(b2Mutex* m)
// {
// 	EnterCriticalSection( &m.cs );
// }

// void b2UnlockMutex(b2Mutex* m)
// {
// 	LeaveCriticalSection( &m.cs );
// }

// } else 
// static if (HasVersion!"linux" || HasVersion!"__EMSCRIPTEN__") {

import core.sys.posix.sched;
import core.stdc.time;

ulong b2GetTicks()
{
	timespec ts = void;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return ts.tv_sec * 1000000000L + ts.tv_nsec;
}

float b2GetMilliseconds(ulong ticks)
{
	ulong ticksNow = b2GetTicks();
	return cast(float)( ( ticksNow - ticks ) / 1000000.0 );
}

float b2GetMillisecondsAndReset(ulong* ticks)
{
	ulong ticksNow = b2GetTicks();
	float ms = cast(float)( ( ticksNow - *ticks ) / 1000000.0 );
	*ticks = ticksNow;
	return ms;
}

void b2Yield()
{
	sched_yield();
}

import core.sys.posix.pthread;
struct b2Mutex {
	pthread_mutex_t mtx;
}

b2Mutex* b2CreateMutex()
{
	b2Mutex* m = cast(b2Mutex*)b2Alloc( b2Mutex.sizeof );
	pthread_mutex_init( &m.mtx, null );
	return m;
}

void b2DestroyMutex(b2Mutex* m)
{
	pthread_mutex_destroy( &m.mtx );
	*m = b2Mutex(pthread_mutex_t(0));
	b2Free( m, b2Mutex.sizeof );
}

void b2LockMutex(b2Mutex* m)
{
	pthread_mutex_lock( &m.mtx );
}

void b2UnlockMutex(b2Mutex* m)
{
	pthread_mutex_unlock( &m.mtx );
}

// } 
// else version (OSX) {

// // import mach/mach_time;
// import core.sys.posix.sched;
// import core.sys.posix.sys.time;

// private double s_invFrequency = 0.0;

// ulong b2GetTicks()
// {
// 	return mach_absolute_time();
// }

// float b2GetMilliseconds(ulong ticks)
// {
// 	if ( s_invFrequency == 0 )
// 	{
// 		mach_timebase_info_data_t timebase = void;
// 		mach_timebase_info( &timebase );

// 		// convert to ns then to ms
// 		s_invFrequency = 1e-6 * cast(double)timebase.numer / cast(double)timebase.denom;
// 	}

// 	ulong ticksNow = b2GetTicks();
// 	return cast(float)( s_invFrequency * ( ticksNow - ticks ) );
// }

// float b2GetMillisecondsAndReset(ulong* ticks)
// {
// 	if ( s_invFrequency == 0 )
// 	{
// 		mach_timebase_info_data_t timebase = void;
// 		mach_timebase_info( &timebase );

// 		// convert to ns then to ms
// 		s_invFrequency = 1e-6 * cast(double)timebase.numer / cast(double)timebase.denom;
// 	}

// 	ulong ticksNow = b2GetTicks();
// 	float ms = cast(float)( s_invFrequency * ( ticksNow - *ticks ) );
// 	*ticks = ticksNow;
// 	return ms;
// }

// void b2Yield()
// {
// 	sched_yield();
// }

// import core.sys.posix.pthread;
// struct b2Mutex {
// 	pthread_mutex_t mtx;
// }

// b2Mutex* b2CreateMutex()
// {
// 	b2Mutex* m = b2Alloc( b2Mutex.sizeof );
// 	pthread_mutex_init( &m.mtx, null );
// 	return m;
// }

// void b2DestroyMutex(b2Mutex* m)
// {
// 	pthread_mutex_destroy( &m.mtx );
// 	*m = b2Mutex( 0 );
// 	b2Free( m, b2Mutex.sizeof );
// }

// void b2LockMutex(b2Mutex* m)
// {
// 	pthread_mutex_lock( &m.mtx );
// }

// void b2UnlockMutex(b2Mutex* m)
// {
// 	pthread_mutex_unlock( &m.mtx );
// }

// } else {

// ulong b2GetTicks()
// {
// 	return 0;
// }

// float b2GetMilliseconds(ulong ticks)
// {
// 	( cast(void)( ticks ) );
// 	return 0.0f;
// }

// float b2GetMillisecondsAndReset(ulong* ticks)
// {
// 	( cast(void)( ticks ) );
// 	return 0.0f;
// }

// void b2Yield()
// {
// }

// struct b2Mutex {
// 	int dummy;
// }

// b2Mutex* b2CreateMutex()
// {
// 	b2Mutex* m = b2Alloc( b2Mutex.sizeof );
// 	m.dummy = 42;
// 	return m;
// }

// void b2DestroyMutex(b2Mutex* m)
// {
// 	*m = b2Mutex( 0 );
// 	b2Free( m, b2Mutex.sizeof );
// }

// void b2LockMutex(b2Mutex* m)
// {
// 	cast(void)m;
// }

// void b2UnlockMutex(b2Mutex* m)
// {
// 	cast(void)m;
// }

// }

// djb2 hash
// https://en.wikipedia.org/wiki/List_of_hash_functions
uint b2Hash(uint hash, const(ubyte)* data, int count)
{
	uint result = hash;
	for ( int i = 0; i < count; i++ )
	{
		result = ( result << 5 ) + result + data[i];
	}

	return result;
}
