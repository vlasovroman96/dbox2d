module dbox2d.ctz;

template HasVersion(string versionId) {
	mixin("version("~versionId~") {enum HasVersion = true;} else {enum HasVersion = false;}");
}
import core.stdc.config: c_long, c_ulong;

public import core.stdc.stdlib;
public import core.stdc.stdint;
import core.builtins;
import core.bitop;

static if (HasVersion!"_MSC_VER" && !HasVersion!"__clang__") {

// public import intrin;

// https://en.wikipedia.org/wiki/Find_first_set

pragma(inline, true) uint b2CTZ32(uint block)
{
	c_ulong index = void;
	_BitScanForward( &index, block );
	return index;
}

// This function doesn't need to be fast, so using the Ivy Bridge fallback.
pragma(inline, true) uint b2CLZ32(uint value)
{
	static if (1) {

	// Use BSR (Bit Scan Reverse) which is available on Ivy Bridge
	c_ulong index = void;
	if ( _BitScanReverse( &index, value ) )
	{
		// BSR gives the index of the most significant 1-bit
		// We need to invert this to get the number of leading zeros
		return 31 - index;
	}
	else
	{
		// If x is 0, BSR sets the zero flag and doesn't modify index
		// LZCNT should return 32 for an input of 0
		return 32;
	}

	} else {

	return __lzcnt( value );

	}
}

pragma(inline, true) uint b2CTZ64(ulong block)
{
	c_ulong index = void;

	version (_WIN64) {
	_BitScanForward64( &index, block );
	} else {
	// 32-bit fall back
	if ( cast(uint)block != 0 )
	{
		_BitScanForward( &index, cast(uint)block );
	}
	else
	{
		_BitScanForward( &index, cast(uint)( block >> 32 ) );
		index += 32;
	}
	}

	return index;
}

pragma(inline, true) int b2PopCount64(ulong block)
{
	return cast(int)_popcnt( block );
}
} else {

pragma(inline, true) uint b2CTZ32(uint block)
{
	return __builtin_ctz( block );
}

pragma(inline, true) uint b2CLZ32(uint value)
{
	return __builtin_clz( value );
}

pragma(inline, true) uint b2CTZ64(ulong block)
{
	return __builtin_ctzll( block );
}

pragma(inline, true) int b2PopCount64(ulong block)
{
	return cast(int)_popcnt( block );
}
}

pragma(inline, true) bool b2IsPowerOf2(int x)
{
	return ( x & ( x - 1 ) ) == 0;
}

pragma(inline, true) int b2BoundingPowerOf2(int x)
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 32 - cast(int)b2CLZ32( cast(uint)x - 1 );
}

pragma(inline, true) int b2RoundUpPowerOf2(int x)
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 1 << ( 32 - cast(int)b2CLZ32( cast(uint)x - 1 ) );
}

alias __builtin_ctzll = numTrailingBinaryZeros;
alias __builtin_ctz = numTrailingBinaryZeros;
alias __builtin_clz = clz;

int clz(T)(T v) {
    int count = T.sizeof;
    int bits = count / 2;
    do {
        int t = v >> bits;
        if (t != 0) {
            count -= bits;
            v = t;
        }
        bits >>= 1;
    } while (bits != 0);
    count -= v;
    return count;
}

int numTrailingBinaryZeros(T)(T n)
{
	int size = T.sizeof * 8;
    int mask = 1;
    for (int i = 0; i < size; i++, mask <<= 1)
        if ((n & mask) != 0)
            return i;

    return size;
}