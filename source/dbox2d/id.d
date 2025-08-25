module dbox2d.id;

import core.stdc.stdint;

// Note: this file should be stand-alone

/**
 * @defgroup id Ids
 * These ids serve as handles to internal Box2D objects.
 * These should be considered opaque data and passed by value.
 * Include this header if you need the id types and not the whole Box2D API.
 * All ids are considered null if initialized to zero.
 *
 * For example in C++:
 *
 * @code{.cxx}
 * b2WorldId worldId = {};
 * @endcode
 *
 * Or in C:
 *
 * @code{.c}
 * b2WorldId worldId = {0};
 * @endcode
 *
 * These are both considered null.
 *
 * @warning Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
 * @warning You should use ids to access objects in Box2D. Do not access files within the src folder. Such usage is unsupported.
 * @{
 */

/// World id references a world instance. This should be treated as an opaque handle.
struct b2WorldId {
	ushort index1;
	ushort generation;
}

/// Body id references a body instance. This should be treated as an opaque handle.
struct b2BodyId {
	int index1;
	ushort world0;
	ushort generation;
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
struct b2ShapeId {
	int index1;
	ushort world0;
	ushort generation;
}

/// Chain id references a chain instances. This should be treated as an opaque handle.
struct b2ChainId {
	int index1;
	ushort world0;
	ushort generation;
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
struct b2JointId {
	int index1;
	ushort world0;
	ushort generation;
}

/// Contact id references a contact instance. This should be treated as an opaque handled.
struct b2ContactId {
	int index1;
	ushort world0;
	short padding;
	uint generation;
}

version (none) {
	// alias B2_NULL_ID = {};
	// alias B2_ID_INLINE = inline;
} else {
	// alias B2_NULL_ID;
	// alias B2_ID_INLINE = inline;
}

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
// private const(b2WorldId) b2_nullWorldId = B2_NULL_ID;
const(b2BodyId) b2_nullBodyId;
// private const(b2ShapeId) b2_nullShapeId = B2_NULL_ID;
// private const(b2ChainId) b2_nullChainId = B2_NULL_ID;
// private const(b2JointId) b2_nullJointId = B2_NULL_ID;
// private const(b2ContactId) b2_nullContactId = B2_NULL_ID;

/// Macro to determine if any id is null.
enum string B2_IS_NULL( string id ) = `( (` ~ id ~ `).index1 == 0 )`;

/// Macro to determine if any id is non-null.
enum string B2_IS_NON_NULL( string id ) = `( (` ~ id ~ `).index1 != 0 )`;

/// Compare two ids for equality. Doesn't work for b2WorldId. Don't mix types.
// enum string B2_ID_EQUALS( string id1, string id2 ) = `( (` ~ id1 ~ `).index1 == (` ~ id2 ~ `).index1 && (` ~ id1 ~ `).world0 == (` ~ id2 ~ `).world0 && (` ~ id1 ~ `).generation == (` ~ id2 ~ `).generation )`;
bool B2_ID_EQUALS(A, B) (A a, B b) {
	return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

/// Store a world id into a uint32_t.
uint32_t b2StoreWorldId(b2WorldId id)
{
	return ( cast(uint)id.index1 << 16 ) | cast(uint)id.generation;
}

/// Load a uint32_t into a world id.
b2WorldId b2LoadWorldId(uint x)
{
	b2WorldId id = { cast(ushort)( x >> 16 ), cast(ushort)( x ) };
	return id;
}

/// Store a body id into a uint64_t.
uint64_t b2StoreBodyId(b2BodyId id)
{
	return ( cast(ulong)id.index1 << 32 ) | ( cast(ulong)id.world0 ) << 16 | cast(ulong)id.generation;
}

/// Load a uint64_t into a body id.
b2BodyId b2LoadBodyId(ulong x)
{
	b2BodyId id = { cast(int)( x >> 32 ), cast(ushort)( x >> 16 ), cast(ushort)( x ) };
	return id;
}

/// Store a shape id into a uint64_t.
uint64_t b2StoreShapeId(b2ShapeId id)
{
	return ( cast(ulong)id.index1 << 32 ) | ( cast(ulong)id.world0 ) << 16 | cast(ulong)id.generation;
}

/// Load a uint64_t into a shape id.
b2ShapeId b2LoadShapeId(ulong x)
{
	b2ShapeId id = { cast(int)( x >> 32 ), cast(ushort)( x >> 16 ), cast(ushort)( x ) };
	return id;
}

/// Store a chain id into a uint64_t.
uint64_t b2StoreChainId(b2ChainId id)
{
	return ( cast(ulong)id.index1 << 32 ) | ( cast(ulong)id.world0 ) << 16 | cast(ulong)id.generation;
}

/// Load a uint64_t into a chain id.
b2ChainId b2LoadChainId(ulong x)
{
	b2ChainId id = { cast(int)( x >> 32 ), cast(ushort)( x >> 16 ), cast(ushort)( x ) };
	return id;
}

/// Store a joint id into a uint64_t.
uint64_t b2StoreJointId(b2JointId id)
{
	return ( cast(ulong)id.index1 << 32 ) | ( cast(ulong)id.world0 ) << 16 | cast(ulong)id.generation;
}

/// Load a uint64_t into a joint id.
b2JointId b2LoadJointId(ulong x)
{
	b2JointId id = { cast(int)( x >> 32 ), cast(ushort)( x >> 16 ), cast(ushort)( x ) };
	return id;
}

/// Store a contact id into 16 bytes
void b2StoreContactId(b2ContactId id, uint* values)
{
	values[0] = cast(uint)id.index1;
	values[1] = cast(uint)id.world0;
	values[2] = cast(uint)id.generation;
}

/// Load a two uint64_t into a contact id.
b2ContactId b2LoadContactId(uint* values)
{
	b2ContactId id = void;
	id.index1 = cast(int)values[0];
	id.world0 = cast(ushort)values[1];
	id.padding = 0;
	id.generation = cast(uint)values[2];
	return id;
}
