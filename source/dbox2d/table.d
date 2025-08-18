module dbox2d.table;
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

//#pragma once

public import core.stdc.stdlib;
public import core.stdc.stdint;
public import core.stdc.string;


import dbox2d.core;
import dbox2d.bitset;
import dbox2d.ctz;

auto _SHAPE_KEY(K1, K2) (K1 k1, K2 k2) {
	return k1 < k2 ? cast(ulong)k1 << 32 | k2 : cast(ulong)k2 << 32 | k1;
}
alias B2_SHAPE_PAIR_KEY = _SHAPE_KEY;
// enum string B2_SHAPE_PAIR_KEY( string K1, string K2 ) = `` ~ K1 ~ ` < ` ~ K2 ~ ` ? cast(ulong)` ~ K1 ~ ` << 32 | cast(ulong)` ~ K2 ~ ` : cast(ulong)` ~ K2 ~ ` << 32 | cast(ulong)` ~ K1 ~ ``;

struct b2SetItem {
	ulong key;
	uint hash;
}

struct b2HashSet {
	b2SetItem* items;
	uint capacity;
	uint count;
}

b2HashSet b2CreateSet(int capacity);
void b2DestroySet(b2HashSet* set);

void b2ClearSet(b2HashSet* set);

// Returns true if key was already in set
bool b2AddKey(b2HashSet* set, ulong key);

// Returns true if the key was found
bool b2RemoveKey(b2HashSet* set, ulong key);

bool b2ContainsKey(const(b2HashSet)* set, ulong key);

int b2GetHashSetBytes(b2HashSet* set);

static if (B2_SNOOP_TABLE_COUNTERS) {
b2AtomicInt b2_findCount;
b2AtomicInt b2_probeCount;
}

// todo compare with https://github.com/skeeto/scratch/blob/master/set32/set32.h

b2HashSet b2CreateSet(int capacity)
{
	b2HashSet set;

	// Capacity must be a power of 2
	if ( capacity > 16 )
	{
		set.capacity = b2RoundUpPowerOf2( capacity );
	}
	else
	{
		set.capacity = 16;
	}

	set.count = 0;
	set.items = cast(b2SetItem*)b2Alloc( cast(int)(capacity * b2SetItem.sizeof) );
	memset( set.items, 0, capacity * b2SetItem.sizeof );

	return set;
}

void b2DestroySet(b2HashSet* set)
{
	b2Free( set.items, cast(int)(set.capacity * b2SetItem.sizeof) );
	set.items = null;
	set.count = 0;
	set.capacity = 0;
}

void b2ClearSet(b2HashSet* set)
{
	set.count = 0;
	memset( set.items, 0, set.capacity * b2SetItem.sizeof );
}

// I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// todo try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
// todo try:
// https://probablydance.com/2018/06/16/fibonacci-hashing-the-optimization-that-the-world-forgot-or-a-better-alternative-to-integer-modulo/
private uint b2KeyHash(ulong key)
{
	// Murmur hash
	ulong h = key;
	h ^= h >> 33;
	h *= 0xff51afd7ed558ccduL;
	h ^= h >> 33;
	h *= 0xc4ceb9fe1a85ec53uL;
	h ^= h >> 33;

	return cast(uint)h;
}

private int b2FindSlot(const(b2HashSet)* set, ulong key, uint hash)
{
static if (B2_SNOOP_TABLE_COUNTERS) {
	b2AtomicFetchAddInt( &b2_findCount, 1 );
}

	uint capacity = set.capacity;
	int index = hash & ( capacity - 1 );
	const(b2SetItem)* items = set.items;
	while ( items[index].hash != 0 && items[index].key != key )
	{
static if (B2_SNOOP_TABLE_COUNTERS) {
		b2AtomicFetchAddInt( &b2_probeCount, 1 );
}
		index = ( index + 1 ) & ( capacity - 1 );
	}

	return index;
}

private void b2AddKeyHaveCapacity(b2HashSet* set, ulong key, uint hash)
{
	int index = b2FindSlot( set, key, hash );
	b2SetItem* items = set.items;
	assert( items[index].hash == 0 );

	items[index].key = key;
	items[index].hash = hash;
	set.count += 1;
}

private void b2GrowTable(b2HashSet* set)
{
	uint oldCount = set.count;
	// B2_UNUSED( oldCount );

	uint oldCapacity = set.capacity;
	b2SetItem* oldItems = set.items;

	set.count = 0;
	// Capacity must be a power of 2
	set.capacity = 2 * oldCapacity;
	set.items = cast(b2SetItem*)b2Alloc( cast(int)(set.capacity * b2SetItem.sizeof) );
	memset( set.items, 0, set.capacity * b2SetItem.sizeof );

	// Transfer items into new array
	for ( uint i = 0; i < oldCapacity; ++i )
	{
		b2SetItem* item = oldItems + i;
		if ( item.hash == 0 )
		{
			// this item was empty
			continue;
		}

		b2AddKeyHaveCapacity( set, item.key, item.hash );
	}

	assert( set.count == oldCount );

	b2Free( oldItems, cast(int)(oldCapacity * b2SetItem.sizeof) );
}

bool b2ContainsKey(const(b2HashSet)* set, ulong key)
{
	// key of zero is a sentinel
	assert( key != 0 );
	uint hash = b2KeyHash( key );
	int index = b2FindSlot( set, key, hash );
	return set.items[index].key == key;
}

int b2GetHashSetBytes(b2HashSet* set)
{
	return set.capacity * cast(int)b2SetItem.sizeof;
}

bool b2AddKey(b2HashSet* set, ulong key)
{
	// key of zero is a sentinel
	assert( key != 0 );

	uint hash = b2KeyHash( key );
	assert( hash != 0 );

	int index = b2FindSlot( set, key, hash );
	if ( set.items[index].hash != 0 )
	{
		// Already in set
		assert( set.items[index].hash == hash && set.items[index].key == key );
		return true;
	}

	if ( 2 * set.count >= set.capacity )
	{
		b2GrowTable( set );
	}

	b2AddKeyHaveCapacity( set, key, hash );
	return false;
}

// See https://en.wikipedia.org/wiki/Open_addressing
bool b2RemoveKey(b2HashSet* set, ulong key)
{
	uint hash = b2KeyHash( key );
	int i = b2FindSlot( set, key, hash );
	b2SetItem* items = set.items;
	if ( items[i].hash == 0 )
	{
		// Not in set
		return false;
	}

	// Mark item i as unoccupied
	items[i].key = 0;
	items[i].hash = 0;

	assert( set.count > 0 );
	set.count -= 1;

	// Attempt to fill item i
	int j = i;
	uint capacity = set.capacity;
	for ( ;; )
	{
		j = ( j + 1 ) & ( capacity - 1 );
		if ( items[j].hash == 0 )
		{
			break;
		}

		// k is the first item for the hash of j
		int k = items[j].hash & ( capacity - 1 );

		// determine if k lies cyclically in (i,j]
		// i <= j: | i..k..j |
		// i > j: |.k..j  i....| or |....j     i..k.|
		if ( i <= j )
		{
			if ( i < k && k <= j )
			{
				continue;
			}
		}
		else
		{
			if ( i < k || k <= j )
			{
				continue;
			}
		}

		// Move j into i
		items[i] = items[j];

		// Mark item j as unoccupied
		items[j].key = 0;
		items[j].hash = 0;

		i = j;
	}

	return true;
}

// This function is here because ctz.h is included by
// this file but not in bitset.c
int b2CountSetBits(b2BitSet* bitSet)
{
	int popCount = 0;
	uint blockCount = bitSet.blockCount;
	for ( uint i = 0; i < blockCount; ++i )
	{
		popCount += b2PopCount64( bitSet.bits[i] );
	}

	return popCount;
}
