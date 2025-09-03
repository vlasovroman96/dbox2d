module dbox2d.atomic;

import core.atomic;

import std.stdint;

import dbox2d.core;
import dbox2d.ct.templates;

version (_MSC_VER) {
	import intrin;
}

pragma(inline, true) void b2AtomicStoreInt(b2AtomicInt* a, int value)
{
version (_MSC_VER) {
	cast(void)_InterlockedExchange( cast(c_long*)&a.value, value );
} else static if (HasVersion!"__GNUC__" || HasVersion!"__clang__" || HasVersion!"linux") {
	atomicStore( a.value, value );
} else {
static assert(0, "Unsupported platform");
}
}

pragma(inline, true) int b2AtomicLoadInt(b2AtomicInt* a)
{
version (_MSC_VER) {
	return _InterlockedOr( cast(c_long*)&a.value, 0 );
} else static if (HasVersion!"__GNUC__" || HasVersion!"__clang__" || HasVersion!"linux") {
	return atomicLoad( a.value);
} else {
static assert(0, "Unsupported platform");
}
}

pragma(inline, true) int b2AtomicFetchAddInt(b2AtomicInt* a, int increment)
{
version (_MSC_VER) {
	return _InterlockedExchangeAdd( cast(c_long*)&a.value, cast(c_long)increment );
} else static if (HasVersion!"__GNUC__" || HasVersion!"__clang__" || HasVersion!"linux") {
	return atomicFetchAdd( a.value, increment);
} else {
static assert(0, "Unsupported platform");
}
}

pragma(inline, true) bool b2AtomicCompareExchangeInt(b2AtomicInt* a, int expected, int desired)
{
version (_MSC_VER) {
	return _InterlockedCompareExchange( cast(c_long*)&a.value, cast(c_long)desired, cast(c_long)expected ) == expected;
} else static if (HasVersion!"__GNUC__" || HasVersion!"__clang__" || HasVersion!"linux") {
	// The value written to expected is ignored
	import core.stdc.stdatomic;
	return casWeak( &a.value, expected, desired);
	// return atomicExchange()
} else {
static assert(0, "Unsupported platform");
}
}

pragma(inline, true) void b2AtomicStoreU32(b2AtomicU32* a, uint value)
{
version (_MSC_VER) {
	cast(void)_InterlockedExchange( cast(c_long*)&a.value, value );
} else static if (HasVersion!"__GNUC__" || HasVersion!"__clang__" || HasVersion!"linux") {
	atomicStore( a.value, value);
} else {
static assert(0, "Unsupported platform");
}
}

pragma(inline, true) uint b2AtomicLoadU32(b2AtomicU32* a)
{
version (_MSC_VER) {
	return cast(uint)_InterlockedOr( cast(c_long*)&a.value, 0 );
} else static if (HasVersion!"__GNUC__" || HasVersion!"__clang__" || HasVersion!"linux") {
	return atomicLoad( a.value);
} else {
static assert(0, "Unsupported platform");
}
}
