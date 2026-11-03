// Force-included in front of kedge_dart.ino (g++ -include), doing the two things the Arduino builder does to a .ino
// before it compiles it as C++: include Arduino.h, and declare the sketch's functions so they can be called before
// their definition. Only one such forward use exists in kedge_dart.ino: inaCheckCal() calls hwLog(), defined later.
#pragma once
#include <Arduino.h>

void hwLog(const char* s);
