module dbox2d.sensor;
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

//#pragma once

public import dbox2d.array;
public import dbox2d.bitset;

import dbox2d.physics_world;
import dbox2d.shape;
import dbox2d.body;
import dbox2d.dynamic_tree;
import dbox2d.ctz;
import dbox2d.distance;

import core.stdc.stdlib;
// import std.algorithm;

mixin(B2_ARRAY_SOURCE!("b2Body", "b2Body"));
// mixin(B2_ARRAY_SOURCE!("b2Int", "int"));
// mixin(B2_ARRAY_SOURCE!("b2Island","b2Island"));
mixin(B2_ARRAY_SOURCE!("b2Sensor","b2Sensor"));


mixin(B2_ARRAY_SOURCE!("b2Shape", "b2Shape"));
// mixin(B2_ARRAY_SOURCE!("b2ChainShape", "b2ChainShape"));
// mixin(B2_ARRAY_SOURCE!("b2SolverSet","b2SolverSet"));
// mixin(B2_ARRAY_SOURCE!("b2ContactHitEvent","b2ContactHitEvent"));
// mixin(B2_ARRAY_SOURCE!("b2ContactBeginTouchEvent","b2ContactBeginTouchEvent"));
// mixin(B2_ARRAY_SOURCE!("b2ContactEndTouchEvent","b2ContactEndTouchEvent"));
// mixin(B2_ARRAY_SOURCE!("b2ContactSim","b2ContactSim"));

mixin(B2_ARRAY_SOURCE!("b2Visitor","b2Visitor"));


mixin(B2_ARRAY_SOURCE!("b2SensorBeginTouchEvent","b2SensorBeginTouchEvent"));
mixin(B2_ARRAY_SOURCE!("b2SensorEndTouchEvent","b2SensorEndTouchEvent"));

// mixin(B2_ARRAY_SOURCE!("b2TaskContext","b2TaskContext"));
// mixin(B2_ARRAY_SOURCE!("b2JointEvent","b2JointEvent"));
// mixin(B2_ARRAY_SOURCE!("b2SensorTaskContext","b2SensorTaskContext"));
// mixin(B2_ARRAY_SOURCE!("b2SensorHit","b2SensorHit"));
// mixin(B2_ARRAY_SOURCE!("b2BodyMoveEvent","b2BodyMoveEvent"));
// mixin(B2_ARRAY_SOURCE!("b2BodySim","b2BodySim"));
// mixin(B2_ARRAY_SOURCE!("b2Joint","b2Joint"));

// mixin(B2_ARRAY_SOURCE!("b2Contact","b2Contact"));


// Used to track shapes that hit sensors using time of impact
struct b2SensorHit {
	int sensorId;
	int visitorId;
}

struct b2Visitor {
	int shapeId;
	ushort generation;
}

struct b2Sensor {
	// todo find a way to pool these
	b2VisitorArray hits;
	b2VisitorArray overlaps1;
	b2VisitorArray overlaps2;
	int shapeId;
}

struct b2SensorTaskContext {
	b2BitSet eventBits;
}

void b2OverlapSensors(b2World* world);

void b2DestroySensor(b2World* world, b2Shape* sensorShape);

// alias b2SensorArray = b2Sensor[];
// //B2_ARRAY_INLINE( b2Sensor, b2Sensor )

// alias b2SensorHitArray = b2SensorHit[];
// //B2_ARRAY_INLINE( b2SensorHit, b2SensorHit )

// alias b2SensorTaskContextArray = b2SensorTaskContext[];
// //B2_ARRAY_INLINE( b2SensorTaskContext, b2SensorTaskContext )

// alias b2VisitorArray = b2Visitor[];
//B2_ARRAY_INLINE( b2Visitor, b2Visitor )

struct b2SensorQueryContext
{
	b2World* world;
	b2SensorTaskContext* taskContext;
	b2Sensor* sensor;
	b2Shape* sensorShape;
	b2Transform transform;
}

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect shapes on the same body

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

private bool b2SensorQueryCallback(int proxyId, ulong userData, void* context)
{
	// B2_UNUSED( proxyId );

	int shapeId = cast(int)userData;

	b2SensorQueryContext* queryContext = cast(b2SensorQueryContext*)context;
	b2Shape* sensorShape = queryContext.sensorShape;
	int sensorShapeId = sensorShape.id;

	if ( shapeId == sensorShapeId )
	{
		return true;
	}

	b2World* world = queryContext.world;
	b2Shape* otherShape = b2ShapeArray_Get( world.shapes, shapeId );

	// Are sensor events enabled on the other shape?
	if ( otherShape.enableSensorEvents == false )
	{
		return true;
	}

	// Skip shapes on the same body
	if ( otherShape.bodyId == sensorShape.bodyId )
	{
		return true;
	}

	// Check filter
	if ( b2ShouldShapesCollide( sensorShape.filter, otherShape.filter ) == false )
	{
		return true;
	}

	// Custom user filter
	if ( sensorShape.enableCustomFiltering || otherShape.enableCustomFiltering )
	{
		b2CustomFilterFcn customFilterFcn = queryContext.world.customFilterFcn;
		if ( customFilterFcn != null )
		{
			b2ShapeId idA = { sensorShapeId + 1, world.worldId, sensorShape.generation };
			b2ShapeId idB = { shapeId + 1, world.worldId, otherShape.generation };
			bool shouldCollide = customFilterFcn( idA, idB, queryContext.world.customFilterContext );
			if ( shouldCollide == false )
			{
				return true;
			}
		}
	}

	b2Transform otherTransform = b2GetBodyTransform( world, otherShape.bodyId );

	b2DistanceInput input = void;
	input.proxyA = b2MakeShapeDistanceProxy( sensorShape );
	input.proxyB = b2MakeShapeDistanceProxy( otherShape );
	input.transformA = queryContext.transform;
	input.transformB = otherTransform;
	input.useRadii = true;
	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &input, &cache, null, 0 );

	bool overlaps = output.distance < 10.0f * float.epsilon;
	if ( overlaps == false )
	{
		return true;
	}

	// Record the overlap
	b2Sensor* sensor = queryContext.sensor;
	b2Visitor* shapeRef = b2VisitorArray_Add( sensor.overlaps2 );
	shapeRef.shapeId = shapeId;
	shapeRef.generation = otherShape.generation;

	return true;
}

extern(C) int b2CompareVisitors(const void* a, const  void* b)
{
	const(b2Visitor)* sa = cast(b2Visitor*)a;
	const(b2Visitor)* sb = cast(b2Visitor*)b;

	if ( sa.shapeId < sb.shapeId )
	{
		return -1;
	}

	return 1;
}

private void b2SensorTask(int startIndex, int endIndex, uint threadIndex, void* context)
{
	// b2TracyCZoneNC( sensor_task, "Overlap", b2_colorBrown, true );

	b2World* world = cast(b2World*)context;
	B2_ASSERT( cast(int)threadIndex < world.workerCount );
	b2SensorTaskContext* taskContext = world.sensorTaskContexts.ptr + threadIndex;

	B2_ASSERT( startIndex < endIndex );

	b2DynamicTree* trees = world.broadPhase.trees.ptr;
	for ( int sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex )
	{
		b2Sensor* sensor = b2SensorArray_Get( world.sensors, sensorIndex );
		b2Shape* sensorShape = b2ShapeArray_Get( world.shapes, sensor.shapeId );

		// Swap overlap arrays
		b2VisitorArray temp = sensor.overlaps1;
		sensor.overlaps1 = sensor.overlaps2;
		sensor.overlaps2 = temp;
		b2VisitorArray_Clear( sensor.overlaps2 );

		// Append sensor hits
		int hitCount = cast(int)sensor.hits.count;
		for ( int i = 0; i < hitCount; ++i )
		{
			b2VisitorArray_Push( sensor.overlaps2, sensor.hits[i] );
		}

		// Clear the hits
		b2VisitorArray_Clear( sensor.hits );

		b2Body* body = b2BodyArray_Get( world.bodies, sensorShape.bodyId );
		if ( body.setIndex == b2_disabledSet || sensorShape.enableSensorEvents == false )
		{
			if ( sensor.overlaps1.count != 0 )
			{
				// This sensor is dropping all overlaps because it has been disabled.
				b2SetBit( &taskContext.eventBits, sensorIndex );
			}
			continue;
		}

		b2Transform transform = b2GetBodyTransformQuick( world, body );

		b2SensorQueryContext queryContext = {
			world: world,
			taskContext: taskContext,
			sensor: sensor,
			sensorShape: sensorShape,
			transform: transform,
		};

		B2_ASSERT( sensorShape.sensorIndex == sensorIndex );
		b2AABB queryBounds = sensorShape.aabb;

		// Query all trees
		b2DynamicTree_Query( trees + 0, queryBounds, sensorShape.filter.maskBits, &b2SensorQueryCallback, &queryContext );
		b2DynamicTree_Query( trees + 1, queryBounds, sensorShape.filter.maskBits, &b2SensorQueryCallback, &queryContext );
		b2DynamicTree_Query( trees + 2, queryBounds, sensorShape.filter.maskBits, &b2SensorQueryCallback, &queryContext );

		// alias cmpV = int function(const void* a, const void* b);

		// cmpV  CmpV = &b2CompareVisitors;
		// extern(C) 
		
		// Sort the overlaps to enable finding begin and end events.
		qsort( cast(void*)sensor.overlaps2.ptr, cast(size_t)b2Visitor.sizeof, 
			cast(size_t)sensor.overlaps2.count, &b2CompareVisitors);
		// extern(D)
		// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		int uniqueCount = 0;
		int overlapCount = cast(int)sensor.overlaps2.count;
		b2Visitor* overlapData = sensor.overlaps2.ptr;
		for ( int i = 0; i < overlapCount; ++i )
		{
			if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
			{
				overlapData[uniqueCount] = overlapData[i];
				uniqueCount += 1;
			}
		}
		sensor.overlaps2.length = uniqueCount;

		int count1 = cast(int)sensor.overlaps1.count;
		int count2 = cast(int)sensor.overlaps2.count;
		if ( count1 != count2 )
		{
			// something changed
			b2SetBit( &taskContext.eventBits, sensorIndex );
		}
		else
		{
			for ( int i = 0; i < count1; ++i )
			{
				b2Visitor* s1 = sensor.overlaps1.ptr + i;
				b2Visitor* s2 = sensor.overlaps2.ptr + i;

				if ( s1.shapeId != s2.shapeId || s1.generation != s2.generation )
				{
					// something changed
					b2SetBit( &taskContext.eventBits, sensorIndex );
					break;
				}
			}
		}
	}

	// b2TracyCZoneEnd( sensor_task );
}

void b2OverlapSensors(b2World* world)
{
	int sensorCount = cast(int)world.sensors.count;
	if ( sensorCount == 0 )
	{
		return;
	}

	B2_ASSERT( world.workerCount > 0 );

	// b2TracyCZoneNC( overlap_sensors, "Sensors", b2_colorMediumPurple, true );

	for ( int i = 0; i < world.workerCount; ++i )
	{
		b2SetBitCountAndClear( &world.sensorTaskContexts[i].eventBits, sensorCount );
	}

	// Parallel-for sensors overlaps
	int minRange = 16;
	void* userSensorTask = world.enqueueTaskFcn( &b2SensorTask, sensorCount, minRange, world, world.userTaskContext );
	world.taskCount += 1;
	if ( userSensorTask != null )
	{
		world.finishTaskFcn( userSensorTask, world.userTaskContext );
	}

	// b2TracyCZoneNC( sensor_state, "Events", b2_colorLightSlateGray, true );

	b2BitSet* bitSet = &world.sensorTaskContexts[0].eventBits;
	for ( int i = 1; i < world.workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world.sensorTaskContexts[i].eventBits );
	}

	// Iterate sensors bits and publish events
	// Process sensor state changes. Iterate over set bits
	ulong* bits = bitSet.bits;
	uint blockCount = bitSet.blockCount;

	for ( uint k = 0; k < blockCount; ++k )
	{
		ulong word = bits[k];
		while ( word != 0 )
		{
			uint ctz = b2CTZ64( word );
			int sensorIndex = cast(int)( 64 * k + ctz );

			b2Sensor* sensor = b2SensorArray_Get( world.sensors, sensorIndex );
			b2Shape* sensorShape = b2ShapeArray_Get( world.shapes, sensor.shapeId );
			b2ShapeId sensorId = { sensor.shapeId + 1, world.worldId, sensorShape.generation };

			int count1 = cast(int)sensor.overlaps1.count;
			int count2 = cast(int)sensor.overlaps2.count;
			const(b2Visitor)* refs1 = sensor.overlaps1.ptr;
			const(b2Visitor)* refs2 = sensor.overlaps2.ptr;

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			int index1 = 0, index2 = 0;
			while ( index1 < count1 && index2 < count2 )
			{
				const(b2Visitor)* r1 = refs1 + index1;
				const(b2Visitor)* r2 = refs2 + index2;
				if ( r1.shapeId == r2.shapeId )
				{
					if ( r1.generation < r2.generation )
					{
						// end
						b2ShapeId visitorId = { r1.shapeId + 1, world.worldId, r1.generation };
						b2SensorEndTouchEvent event = {
							sensorShapeId: sensorId,
							visitorShapeId: visitorId,
						};
						b2SensorEndTouchEventArray_Push( world.sensorEndEvents[world.endEventArrayIndex], event );
						index1 += 1;
					}
					else if ( r1.generation > r2.generation )
					{
						// begin
						b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
						b2SensorBeginTouchEvent event = { sensorId, visitorId };
						b2SensorBeginTouchEventArray_Push( world.sensorBeginEvents, event );
						index2 += 1;
					}
					else
					{
						// persisted
						index1 += 1;
						index2 += 1;
					}
				}
				else if ( r1.shapeId < r2.shapeId )
				{
					// end
					b2ShapeId visitorId = { r1.shapeId + 1, world.worldId, r1.generation };
					b2SensorEndTouchEvent event = { sensorId, visitorId };
					b2SensorEndTouchEventArray_Push( world.sensorEndEvents[world.endEventArrayIndex], event );
					index1 += 1;
				}
				else
				{
					// begin
					b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
					b2SensorBeginTouchEvent event = { sensorId, visitorId };
					b2SensorBeginTouchEventArray_Push( world.sensorBeginEvents, event );
					index2 += 1;
				}
			}

			while ( index1 < count1 )
			{
				// end
				const(b2Visitor)* r1 = refs1 + index1;
				b2ShapeId visitorId = { r1.shapeId + 1, world.worldId, r1.generation };
				b2SensorEndTouchEvent event = { sensorId, visitorId };
				b2SensorEndTouchEventArray_Push( world.sensorEndEvents[world.endEventArrayIndex], event );
				index1 += 1;
			}

			while ( index2 < count2 )
			{
				// begin
				const(b2Visitor)* r2 = refs2 + index2;
				b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
				b2SensorBeginTouchEvent event = { sensorId, visitorId };
				b2SensorBeginTouchEventArray_Push( world.sensorBeginEvents, event );
				index2 += 1;
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}

	// b2TracyCZoneEnd( sensor_state );
	// b2TracyCZoneEnd( overlap_sensors );
}

void b2DestroySensor(b2World* world, b2Shape* sensorShape)
{
	b2Sensor* sensor = b2SensorArray_Get( world.sensors, sensorShape.sensorIndex );
	for ( int i = 0; i < sensor.overlaps2.count; ++i )
	{
		b2Visitor* ref_ = sensor.overlaps2.ptr + i;
		b2SensorEndTouchEvent event = {
			sensorShapeId:
				{
					index1: sensorShape.id + 1,
					world0: world.worldId,
					generation: sensorShape.generation,
				},
			visitorShapeId:
				{
					index1: ref_.shapeId + 1,
					world0: world.worldId,
					generation: ref_.generation,
				},
		};

		b2SensorEndTouchEventArray_Push( world.sensorEndEvents[world.endEventArrayIndex], event );
	}

	// Destroy sensor
	b2VisitorArray_Destroy( sensor.hits );
	b2VisitorArray_Destroy( sensor.overlaps1 );
	b2VisitorArray_Destroy( sensor.overlaps2 );

	int movedIndex = b2SensorArray_RemoveSwap( world.sensors, sensorShape.sensorIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fixup moved sensor
		b2Sensor* movedSensor = b2SensorArray_Get( world.sensors, sensorShape.sensorIndex );
		b2Shape* otherSensorShape = b2ShapeArray_Get( world.shapes, movedSensor.shapeId );
		otherSensorShape.sensorIndex = sensorShape.sensorIndex;
	}
}
