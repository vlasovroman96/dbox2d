module dbox2d.arenaEntry;
import dbox2d.array;

struct b2ArenaEntry {
	char* data;
	const(char)* name;
	int size;
	bool usedMalloc;
}

mixin(B2_ARRAY_SOURCE!("b2ArenaEntry","b2ArenaEntry"));
