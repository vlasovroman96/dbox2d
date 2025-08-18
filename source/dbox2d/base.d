module dbox2d.base;

import std.stdio;
import std.traits;

import dbox2d.timer;

import core.stdc.stdint;

// Shared library macros
// static if (is(Windows))
// {
//     // build the Windows DLL
//     mixin("define BOX2D_EXPORT __declspec( dllexport )");
// }
// else static if (is(Windows && defined(BOX2D_DLL)))
// {
//     // using the Windows DLL
//     mixin("define BOX2D_EXPORT __declspec( dllimport )");
// }
// else static if (defined(box2d_EXPORTS))
// {
//     // building or using the shared library
//     mixin("define BOX2D_EXPORT __attribute__( ( visibility( \"default\" ) ) )");
// }
// else
// {
//     // static library
//     mixin("define BOX2D_EXPORT");
// }

// // C++ macros
// static if (is(Cplusplus))
// {
//     mixin("define B2_API extern \"C\" BOX2D_EXPORT");
//     mixin("define B2_INLINE inline");
//     mixin("define B2_LITERAL(T) T");
//     mixin("define B2_ZERO_INIT {}");
// }
// else
// {
//     mixin("define B2_API BOX2D_EXPORT");
//     mixin("define B2_INLINE static inline");
//     mixin("define B2_LITERAL(T) (T)");
//     mixin("define B2_ZERO_INIT {0}");
// }

/**
 * @defgroup base Base
 * Base functionality
 * @{
 */

// Prototype for user allocation function
// @param size the allocation size in bytes
// @param alignment the required alignment, guaranteed to be a power of 2
alias b2AllocFcn = void* function(uint size, int alignment);

// Prototype for user free function
// @param mem the memory previously allocated through `b2AllocFcn`
alias b2FreeFcn = void function(void* mem);

// Prototype for the user assert callback. Return 0 to skip the debugger break.
alias b2AssertFcn = int function(const char* condition, const char* filename, int lineNumber);
// typedef int b2AssertFcn(const char* condition, const char* fileName, int lineNumber);

// This allows the user to override the allocation functions. These should be
// set during application startup.
void b2SetAllocator(b2AllocFcn allocFcn, b2FreeFcn freeFcn);

// @return the total bytes allocated by Box2D
// int b2GetByteCount();

// Override the default assert callback
// @param assertFcn a non-null assert callback
void b2SetAssertFcn(b2AssertFcn assertFcn);

// Version numbering scheme.
// See https://semver.org/
struct b2Version
{
    // Significant changes
    int major;

    // Incremental changes
    int minor;

    // Bug fixes
    int revision;
}

// Get the current version of Box2D
b2Version b2GetVersion();

//! @cond

// see https://github.com/scottt/debugbreak
// static if (is(Windows))
// {
//     mixin("define B2_BREAKPOINT __debugbreak()");
// }
// else static if (is(GCC) || is(Clang))
// {
//     mixin("define B2_BREAKPOINT __builtin_trap()");
// }
// else
// {
//     // Unknown compiler
//     import std.traits;
//     mixin("define B2_BREAKPOINT assert( 0 )");
// }

// static if (!defined(NDEBUG) || defined(B2_ENABLE_ASSERT))
// {
//     extern(C) int b2InternalAssertFcn(const char* condition, const char* fileName, int lineNumber);
//     mixin("define B2_ASSERT(condition) do { if (!(condition) && b2InternalAssertFcn(#condition, __FILE__, (int)__LINE__)) B2_BREAKPOINT; } while(0)");
// }
// else
// {
//     mixin("define B2_ASSERT(...) ((void)0)");
// }

// see https://github.com/scottt/debugbreak
void b2Breakpoint() { 
    import core.stdc.stdlib; 
    assert(0); 
}

alias B2_ASSERT = (a) {
    if(a)
        cast(void)0;
    else
        b2Breakpoint();
    // (a) ? cast(void)0 : b2Breakpoint();
};
// enum B2_ASSERT(bool a) = (a) ? cast(void)0 : b2Breakpoint();


// Get the absolute number of system ticks. The value is platform specific.
import core.sys.posix.sched;
import core.stdc.time;

// ulong b2GetTicks();


// Get the milliseconds passed from an initial tick value.
// float b2GetMilliseconds(ulong ticks);

// Get the milliseconds passed from an initial tick value. Resets the passed in
// value to the current tick value.
// float b2GetMillisecondsAndReset(ref ulong ticks);

// Yield to be used in a busy loop.
// void b2Yield();

// Simple djb2 hash function for determinism testing
static immutable int B2_HASH_INIT = 5381;
uint b2Hash(uint hash, const uint8_t[] data, int count);

//! @endcond