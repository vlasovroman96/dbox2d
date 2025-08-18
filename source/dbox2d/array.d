module dbox2d.array;

import std.container.array;

// alias b2IntArray = Array!int;
// import dbox2d.contact;
// import dbox2d.body;
// import dbox2d.shape;
// alias b2ContactBeginTouchEventArray = Array!b2ContactBeginTouchEvent;

// alias b2ContactSimArray = Array!b2ContactSim;
// alias b2ContactSimArray = b2ContactSim[];

enum string B2_ARRAY_SOURCE(string PREFIX, string T) = 
"
    alias " ~ PREFIX~"Array = " ~ T ~"[];
    
    auto " ~PREFIX~"Array_Get("~ PREFIX ~ "Array array, int index) {
     
        return &(array[index]);
    }

    void " ~PREFIX~"Array_Clear("~ PREFIX ~ "Array array) {
        array.length = 0;
    }

    int " ~PREFIX~"Array_ByteCount("~PREFIX~"Array array) {
        return cast(int)("~T~".sizeof * array.length);
    }

    import std.algorithm;

    int "~PREFIX~"Array_RemoveSwap("~PREFIX~"Array array, int index) {
        int movedIndex = 0;

        if(index != cast(int)(array.length - 1)) {
            movedIndex = cast(int)(array.length - 1);

            array[index] = array[movedIndex];
        }

        array = array.remove(movedIndex);

        return movedIndex;
    }

    void "~PREFIX~"Array_Destroy("~PREFIX~"Array array) {
        array = [];
    }

    "~PREFIX~"Array "~PREFIX~"Array_Create(int size) {

        "~PREFIX~"Array array;
        array.length = size;
        return array;
    }

    void "~PREFIX~"Array_Push("~PREFIX~"Array array, "~T~" elem) {
        array ~= elem;
    }

    "~T~" "~PREFIX~"Array_Pop("~PREFIX~"Array array) {
        auto value = array[$-1];
        array = array[0..$-2];
        return value;
    }

    "~T~"* " ~PREFIX~"Array_Add("~PREFIX~"Array array) {
        array.length += 1;
        auto e = &(array[$ - 1]);
        return e;
    }
    
    void " ~PREFIX~"Array_Resize("~PREFIX~"Array array, int count) {
        array.length = count;
    }
";