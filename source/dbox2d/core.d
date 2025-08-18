module dbox2d.core;

import dbox2d.math_functions;
import dbox2d.timer;
import dbox2d.base;
import dbox2d.atomic;
import dbox2d.timer;

import core.stdc.stdio;

import core.stdc.string;
import core.stdc.stdlib;

template HasVersion(string versionId) {
	mixin("version("~versionId~") {enum HasVersion = true;} else {enum HasVersion = false;}");
}
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

//#pragma once

// public import box2d.math_functions;

// clang-format off

enum B2_NULL_INDEX = -1 ;

// for performance comparisons
// alias B2_RESTRICT = restrict;

// void B2_ASSERT(bool e) {
// 	assert(e);
// }

version (NDEBUG) {
	enum B2_DEBUG = 0;
} else {
	enum B2_DEBUG = 1;
}

static if (HasVersion!"BOX2D_VALIDATE" && !HasVersion!"NDEBUG") {
	enum B2_VALIDATE = 1;
} else {
	enum B2_VALIDATE = 0;
}

// Define platform
static if (HasVersion!"Windows" || HasVersion!"_WIN64") {
	version = B2_PLATFORM_WINDOWS;
} else version (__ANDROID__) {
	version = B2_PLATFORM_ANDROID;
} else version (linux) {
	version = B2_PLATFORM_LINUX;
} else version (OSX) {
	public import TargetConditionals;
	static if (HasVersion!"TARGET_OS_IPHONE" && !TARGET_OS_IPHONE) {
		version = B2_PLATFORM_MACOS;
	}
	else {
		version = B2_PLATFORM_IOS;
	}
} else version (__EMSCRIPTEN__) {
	version = B2_PLATFORM_WASM;
} else {
	version = B2_PLATFORM_UNKNOWN;
}

// Define CPU
static if (HasVersion!"__x86_64__" || HasVersion!"_M_X64" || HasVersion!"__i386__" || HasVersion!"_M_IX86") {
	version = B2_CPU_X86_X64;
} else static if (HasVersion!"__aarch64__" || HasVersion!"_M_ARM64" || HasVersion!"__arm__" || HasVersion!"_M_ARM") {
	version = B2_CPU_ARM;
} else version (__EMSCRIPTEN__) {
	version = B2_CPU_WASM;
} else {
	version = B2_CPU_UNKNOWN;
}

// Define SIMD
version (BOX2D_DISABLE_SIMD) {
	version = B2_SIMD_NONE;
	// note: I tried width of 1 and got no performance change =
	enum B2_SIMD_WIDTH = 4;
} else {
	version (B2_CPU_X86_X64) {
		version (BOX2D_AVX2) {
			version = B2_SIMD_AVX2;
			enum B2_SIMD_WIDTH = 8;
		} else {
			version = B2_SIMD_SSE2;
			enum B2_SIMD_WIDTH = 4;
		}
	} else version (B2_CPU_ARM) {
		version = B2_SIMD_NEON;
		enum B2_SIMD_WIDTH = 4;
	} else version (B2_CPU_WASM) {
		version = B2_CPU_WASM;
		version = B2_SIMD_SSE2;
		enum B2_SIMD_WIDTH = 4;
	} else {
		version = B2_SIMD_NONE;
		enum B2_SIMD_WIDTH = 4;
	}
}

// Define compiler
static if(HasVersion!"__clang__") {
	version = B2_COMPILER_CLANG;
} else static if(HasVersion!"__GNUC__") {
	version = B2_COMPILER_GCC;
} else static if(HasVersion!"_MSC_VER") {
	version = B2_COMPILER_MSVC;
}

/// Tracy profiler instrumentation
/// https://github.com/wolfpld/tracy
version (BOX2D_PROFILE) {
	// public import tracy/TracyC;
	enum string b2TracyCZoneC( string ctx, string color, string active ) = `TracyCZoneC( ` ~ ctx ~ `, ` ~ color ~ `, ` ~ active ~ ` )`;
	enum string b2TracyCZoneNC( string ctx, string name, string color, string active ) = `TracyCZoneNC( ` ~ ctx ~ `, ` ~ name ~ `, ` ~ color ~ `, ` ~ active ~ ` )`;
	enum string b2TracyCZoneEnd( string ctx ) = `TracyCZoneEnd( ` ~ ctx ~ ` )`;
} else {
	enum string b2TracyCZoneC( string ctx, string color, string active ) = `#define b2TracyCZoneNC( ctx, name, color, active )`;
	// enum string b2TracyCZoneEnd( ctx )
}

// clang-format on

// Returns the number of elements of an array
enum string B2_ARRAY_COUNT( string A ) = `cast(int)( A.sizeof / typeof( ` ~ A ~ `[0] ).sizeof )`;

// Used to prevent the compiler from warning about unused variables
// enum string B2_UNUSED( ... ) = `cast(void)typeof( ( __VA_ARGS__, 0 ) ).sizeof`;
enum string B2_UNUSED = "";


// Use to validate definitions. Do not take my cookie.
enum B2_SECRET_COOKIE = 1152023;

// Snoop counters. These should be disabled in optimized builds because they are expensive.
version (box2d_EXPORTS) {
enum B2_SNOOP_TABLE_COUNTERS = B2_DEBUG;
enum B2_SNOOP_PAIR_COUNTERS = B2_DEBUG;
enum B2_SNOOP_TOI_COUNTERS = B2_DEBUG;
} else {
enum B2_SNOOP_TABLE_COUNTERS = 0;
enum B2_SNOOP_PAIR_COUNTERS = 0;
enum B2_SNOOP_TOI_COUNTERS = 0;
}

void B2_CHECK_DEF(T)(T def) { 
	assert(def.internalValue == B2_SECRET_COOKIE);
}

// enum string B2_CHECK_DEF( string DEF ) = "
// 	assert("~DEF~".internalValue === B2_SECRET_COOKIE);
// ";

struct b2AtomicInt {
	int value;
}

struct b2AtomicU32 {
	uint value;
}

void* b2Alloc(int size);
enum string B2_ALLOC_STRUCT( string type ) = `b2Alloc(` ~ type ~ `.sizeof)`;
enum string B2_ALLOC_ARRAY( string count, string type ) = `b2Alloc(` ~ count ~ ` * ` ~ type ~ `.sizeof)`;

void b2Free(void* mem, int size);
enum string B2_FREE_STRUCT( string mem, string type ) = `b2Free( ` ~ mem ~ `, ` ~ type ~ `.sizeof);`;
enum string B2_FREE_ARRAY( string mem, string count, string type ) = `b2Free(` ~ mem ~ `, ` ~ count ~ ` * ` ~ type ~ `.sizeof)`;

void* b2GrowAlloc(void* oldMem, int oldSize, int newSize);


b2Mutex* b2CreateMutex();
void b2DestroyMutex(b2Mutex* m);
void b2LockMutex(b2Mutex* m);
void b2UnlockMutex(b2Mutex* m);

static float b2_lengthUnitsPerMeter = 1.0f;

void b2SetLengthUnitsPerMeter( float lengthUnits )
{
	B2_ASSERT( b2IsValidFloat( lengthUnits ) && lengthUnits > 0.0f );
	b2_lengthUnitsPerMeter = lengthUnits;
}

float b2GetLengthUnitsPerMeter()
{
	return b2_lengthUnitsPerMeter;
}

static int b2DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );

	// return non-zero to break to debugger
	return 1;
}

b2AssertFcn b2AssertHandler = &b2DefaultAssertFcn;

void b2SetAssertFcn( b2AssertFcn assertFcn )
{
	B2_ASSERT( assertFcn != null );
	b2AssertHandler = assertFcn;
}

int b2InternalAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	return b2AssertHandler( condition, fileName, lineNumber );
}

b2Version b2GetVersion()
{	
	b2Version ver;
	with(ver) {
		major = 3;
		minor = 2;
		revision = 0;
	}

	return ver;
}

static b2AllocFcn b2_allocFcn = null;
static b2FreeFcn b2_freeFcn = null;

b2AtomicInt b2_byteCount;

void b2SetAllocator( b2AllocFcn allocFcn, b2FreeFcn freeFcn )
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

// Use 32 byte alignment for everything. Works with 256bit SIMD.
enum B2_ALIGNMENT = 32;

void* b2Alloc( int size )
{
	if ( size == 0 )
	{
		return null;
	}

	// This could cause some sharing issues, however Box2D rarely calls b2Alloc.
	b2AtomicFetchAddInt( &b2_byteCount, size );

	// Allocation must be a multiple of 32 or risk a seg fault
	// https://en.cppreference.com/w/c/memory/aligned_alloc
	int size32 = ( ( size - 1 ) | 0x1F ) + 1;

	if ( b2_allocFcn != null )
	{
		void* ptr = b2_allocFcn( size32, B2_ALIGNMENT );
		// b2TracyCAlloc( ptr, size );

		B2_ASSERT( ptr != null );
		B2_ASSERT( ( cast(uintptr_t)ptr & 0x1F ) == 0 );

		return ptr;
	}

// #ifdef B2_PLATFORM_WINDOWS
// 	void* ptr = _aligned_malloc( size32, B2_ALIGNMENT );
// #elif defined( B2_PLATFORM_ANDROID )
// 	void* ptr = null;
// 	if ( posix_memalign( &ptr, B2_ALIGNMENT, size32 ) != 0 )
// 	{
// 		// allocation failed, exit the application
// 		exit( EXIT_FAILURE );
// 	}
// #else
	void* ptr = aligned_alloc( B2_ALIGNMENT, size32 );
// #endif

	// b2TracyCAlloc( ptr, size );
// 
	assert( ptr != null );
	assert( ( cast(void*)(cast(int)ptr & 0x1F )) == null );

	return ptr;
}

void b2Free( void* mem, int size )
{
	if ( mem == null )
	{
		return;
	}

	// b2TracyCFree( mem );

	if ( b2_freeFcn != null )
	{
		b2_freeFcn( mem );
	}
	else
	{
// #ifdef B2_PLATFORM_WINDOWS
		// _aligned_free( mem );
// #else
		free( mem );
// #endif
	}

	b2AtomicFetchAddInt( &b2_byteCount, -size );
}

void* b2GrowAlloc( void* oldMem, int oldSize, int newSize )
{
	B2_ASSERT( newSize > oldSize );
	void* newMem = b2Alloc( newSize );
	if ( oldSize > 0 )
	{
		memcpy( newMem, oldMem, oldSize );
		b2Free( oldMem, oldSize );
	}
	return newMem;
}

int b2GetByteCount()
{
	return b2AtomicLoadInt( &b2_byteCount );
}

