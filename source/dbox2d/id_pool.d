module dbox2d.id_pool;
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

//#pragma once

mixin(B2_ARRAY_SOURCE!("b2Int", "int"));

public import dbox2d.array;
import dbox2d.core;
import dbox2d.base;

struct b2IdPool {
	b2IntArray freeArray;
	int nextIndex;
}


b2IdPool b2CreateIdPool();
void b2DestroyIdPool(b2IdPool* pool);

int b2AllocId(b2IdPool* pool);
void b2FreeId(b2IdPool* pool, int id);
void b2ValidateFreeId(b2IdPool* pool, int id);
void b2ValidateUsedId(b2IdPool* pool, int id);

pragma(inline, true) int b2GetIdCount(b2IdPool* pool)
{
	return cast(int)(pool.nextIndex - pool.freeArray.length);
}

pragma(inline, true) int b2GetIdCapacity(b2IdPool* pool)
{
	return cast(int)(pool.nextIndex);
}

pragma(inline, true) int b2GetIdBytes(b2IdPool* pool)
{
	return b2IntArray_ByteCount(pool.freeArray);
}

b2IdPool b2CreateIdPool()
{
	b2IdPool pool;
	pool.freeArray = b2IntArray_Create( 32 );
	return pool;
}

void b2DestroyIdPool(b2IdPool* pool)
{
	b2IntArray_Destroy( pool.freeArray );
	*pool =  b2IdPool();
}

int b2AllocId(b2IdPool* pool)
{
	int count = cast(int)pool.freeArray.length;
	if ( count > 0 )
	{
		int id = b2IntArray_Pop( pool.freeArray );
		return id;
	}

	int id = pool.nextIndex;
	pool.nextIndex += 1;
	return id;
}

void b2FreeId(b2IdPool* pool, int id)
{
	B2_ASSERT( pool.nextIndex > 0 );
	B2_ASSERT( 0 <= id && id < pool.nextIndex );
	b2IntArray_Push( pool.freeArray, id );
}

static if (B2_VALIDATE) {

	void b2ValidateFreeId(b2IdPool* pool, int id)
	{
		int freeCount = pool.freeArray.count;
		for ( int i = 0; i < freeCount; ++i )
		{
			if ( pool.freeArray.data[i] == id )
			{
				return;
			}
		}

		B2_ASSERT( 0 );
	}

	void b2ValidateUsedId(b2IdPool* pool, int id)
	{
		int freeCount = pool.freeArray.count;
		for ( int i = 0; i < freeCount; ++i )
		{
			if ( pool.freeArray.data[i] == id )
			{
				B2_ASSERT( 0 );
			}
		}
	}

} else {

	void b2ValidateFreeId(b2IdPool* pool, int id)
	{
		// B2_UNUSED( pool, id );
	}

	void b2ValidateUsedId(b2IdPool* pool, int id)
	{
		// B2_UNUSED( pool, id );
	}
}

