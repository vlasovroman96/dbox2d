module dbox2d.arena_allocator;

public import dbox2d.array;
import dbox2d.core;
import dbox2d.base;

import core.stdc.stddef;
import core.stdc.inttypes;


struct b2ArenaEntry {
	char* data;
	const(char)* name;
	int size;
	bool usedMalloc;
}

mixin(B2_ARRAY_SOURCE!("b2ArenaEntry","b2ArenaEntry"));


// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B2_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
struct b2ArenaAllocator {
	char* data;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b2ArenaEntryArray entries;
}

b2ArenaAllocator b2CreateArenaAllocator(int capacity);
void b2DestroyArenaAllocator(b2ArenaAllocator* allocator);

void* b2AllocateArenaItem(b2ArenaAllocator* alloc, int size, const(char)* name);
void b2FreeArenaItem(b2ArenaAllocator* alloc, void* mem);

// Grow the arena based on usage
void b2GrowArena(b2ArenaAllocator* alloc);

int b2GetArenaCapacity(b2ArenaAllocator* alloc);
int b2GetArenaAllocation(b2ArenaAllocator* alloc);
int b2GetMaxArenaAllocation(b2ArenaAllocator* alloc);

// B2_ARRAY_INLINE( b2ArenaEntry, b2ArenaEntry )

b2ArenaAllocator b2CreateArenaAllocator(int capacity)
{
	B2_ASSERT( capacity >= 0 );
	b2ArenaAllocator allocator;
	allocator.capacity = capacity;
	allocator.data = cast(char*)b2Alloc( capacity );
	allocator.allocation = 0;
	allocator.maxAllocation = 0;
	allocator.index = 0;
	allocator.entries = b2ArenaEntryArray_Create( 32 );
	return allocator;
}

void b2DestroyArenaAllocator(b2ArenaAllocator* allocator)
{
	b2ArenaEntryArray_Destroy( allocator.entries );
	b2Free( allocator.data, allocator.capacity );
}

void* b2AllocateArenaItem(b2ArenaAllocator* alloc, int size, const(char)* name)
{
	// ensure allocation is 32 byte aligned to support 256-bit SIMD
	int size32 = ( ( size - 1 ) | 0x1F ) + 1;

	b2ArenaEntry entry = void;
	entry.size = size32;
	entry.name = name;
	if ( alloc.index + size32 > alloc.capacity )
	{
		// fall back to the heap (undesirable)
		entry.data = cast(char*)b2Alloc( size32 );
		entry.usedMalloc = true;

		B2_ASSERT( ( cast(uintptr_t)entry.data & 0x1F ) == 0 );
	}
	else
	{
		entry.data = alloc.data + alloc.index;
		entry.usedMalloc = false;
		alloc.index += size32;

		B2_ASSERT( ( cast(uintptr_t)entry.data & 0x1F ) == 0 );
	}

	alloc.allocation += size32;
	if ( alloc.allocation > alloc.maxAllocation )
	{
		alloc.maxAllocation = alloc.allocation;
	}

	b2ArenaEntryArray_Push( alloc.entries, entry );
	return entry.data;
}

void b2FreeArenaItem(b2ArenaAllocator* alloc, void* mem)
{
	int entryCount = cast(int)alloc.entries.count;
	B2_ASSERT( entryCount > 0 );
	b2ArenaEntry* entry = alloc.entries.ptr + ( entryCount - 1 );
	B2_ASSERT( mem == entry.data );
	if ( entry.usedMalloc )
	{
		b2Free( mem, entry.size );
	}
	else
	{
		alloc.index -= entry.size;
	}
	alloc.allocation -= entry.size;
	b2ArenaEntryArray_Pop( alloc.entries );
}

void b2GrowArena(b2ArenaAllocator* alloc)
{
	// Stack must not be in use
	B2_ASSERT( alloc.allocation == 0 );

	if ( alloc.maxAllocation > alloc.capacity )
	{
		b2Free( alloc.data, alloc.capacity );
		alloc.capacity = alloc.maxAllocation + alloc.maxAllocation / 2;
		alloc.data = cast(char*)b2Alloc( alloc.capacity );
	}
}

int b2GetArenaCapacity(b2ArenaAllocator* alloc)
{
	return alloc.capacity;
}

int b2GetArenaAllocation(b2ArenaAllocator* alloc)
{
	return alloc.allocation;
}

int b2GetMaxArenaAllocation(b2ArenaAllocator* alloc)
{
	return alloc.maxAllocation;
}
