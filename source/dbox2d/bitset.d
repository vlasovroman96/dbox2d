module dbox2d.bitset;

import std.stdint;

import dbox2d.core;

// Bit set provides fast operations on large arrays of bits.
struct b2BitSet {
	ulong[] bits;
	uint blockCapacity;
	uint blockCount;
}

pragma(inline, true) void b2SetBit(b2BitSet* bitSet, uint bitIndex)
{
	uint blockIndex = bitIndex / 64;
	assert( blockIndex < bitSet.blockCount );
	bitSet.bits[blockIndex] |= ( cast(ulong)1 << bitIndex % 64 );
}

pragma(inline, true) void b2SetBitGrow(b2BitSet* bitSet, uint bitIndex)
{
	uint blockIndex = bitIndex / 64;
	if ( blockIndex >= bitSet.blockCount )
	{
		b2GrowBitSet( bitSet, blockIndex + 1 );
	}
	bitSet.bits[blockIndex] |= ( cast(ulong)1 << bitIndex % 64 );
}

pragma(inline, true) void b2ClearBit(b2BitSet* bitSet, uint bitIndex)
{
	uint blockIndex = bitIndex / 64;
	if ( blockIndex >= bitSet.blockCount )
	{
		return;
	}
	bitSet.bits[blockIndex] &= ~( cast(ulong)1 << bitIndex % 64 );
}

pragma(inline, true) bool b2GetBit(const(b2BitSet)* bitSet, uint bitIndex)
{
	uint blockIndex = bitIndex / 64;
	if ( blockIndex >= bitSet.blockCount )
	{
		return false;
	}
	return ( bitSet.bits[blockIndex] & ( cast(ulong)1 << bitIndex % 64 ) ) != 0;
}

pragma(inline, true) int b2GetBitSetBytes(b2BitSet* bitSet)
{
	return cast(int)(bitSet.blockCapacity * ulong.sizeof);
}


b2BitSet b2CreateBitSet(uint bitCapacity)
{
	b2BitSet bitSet;

	bitSet.blockCapacity = ( bitCapacity + (ulong.sizeof * 8 - 1) ) / ( ulong.sizeof * 8 );
	bitSet.blockCount = 0;
	bitSet.bits.length = bitCapacity / 64;
	bitSet.bits[0..bitSet.blockCapacity * ulong.sizeof] = 0;
	return bitSet;
}

void b2DestroyBitSet(b2BitSet* bitSet)
{
	// b2Free( bitSet.bits, cast(int)(bitSet.blockCapacity * ulong.sizeof) );
	bitSet.blockCapacity = 0;
	bitSet.blockCount = 0;
	bitSet.bits = bitSet.bits.init;
}

void b2SetBitCountAndClear(b2BitSet* bitSet, uint bitCount)
{
	uint blockCount = ( bitCount + (ulong.sizeof * 8 - 1)) / ( ulong.sizeof * 8 );
	if ( bitSet.blockCapacity < blockCount )
	{
		b2DestroyBitSet( bitSet );
		uint newBitCapacity = bitCount + ( bitCount >> 1 );
		*bitSet = b2CreateBitSet( newBitCapacity );
	}

	bitSet.blockCount = blockCount;
	bitSet.bits[0..bitSet.blockCount * ulong.sizeof] = 0;
}

void b2GrowBitSet(b2BitSet* bitSet, uint blockCount)
{
	assert( blockCount > bitSet.blockCount );
	if ( blockCount > bitSet.blockCapacity )
	{
		uint oldCapacity = bitSet.blockCapacity;
		bitSet.blockCapacity = blockCount + blockCount / 2;
		// ulong* newBits = cast(ulong*)b2Alloc( cast(int)(bitSet.blockCapacity * ulong.sizeof) );
		// newBits = new ulong[bitSet.blockCapacity];
		// assert( bitSet.bits != null );
		// memcpy( newBits, bitSet.bits, oldCapacity * ulong.sizeof );
		// *newBits = *(bitSet.bits);
		bitSet.bits.length = blockCount;
		// b2Free( bitSet.bits, cast(int)(oldCapacity * ulong.sizeof) );
		// bitSet.bits = newBits;
	}

	bitSet.blockCount = blockCount;
}

void b2InPlaceUnion(b2BitSet* setA, const(b2BitSet)* setB)
{
	assert( setA.blockCount == setB.blockCount );
	uint blockCount = setA.blockCount;
	for ( uint i = 0; i < blockCount; ++i )
	{
		setA.bits[i] |= setB.bits[i];
	}
}
