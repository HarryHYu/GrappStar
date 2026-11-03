# KEDGE dart host simulator.
#   make          build ./kedge_sim from the unchanged ../kedge_dart/kedge_dart.ino + kedge_logic.h
#   make test     build, then run the end to end HTTP test (python3 test_sim.py)
#   make clean
# The sketch is compiled as C++ with the stub Arduino headers in stubs/ first on the include path. sketch_prelude.h is
# force-included in front of it, doing what the Arduino builder does for a .ino (Arduino.h + function prototypes).

CXX      ?= g++
CXXFLAGS ?= -std=gnu++17 -O1 -g -Wall -Wextra -Wno-unused-parameter
CPPFLAGS += -Istubs -I. -I../kedge_dart
LDLIBS   += -pthread

SKETCH  := ../kedge_dart/kedge_dart.ino
HEADERS := $(wildcard stubs/*.h stubs/freertos/*.h) sim_world.h sketch_prelude.h ../kedge_dart/kedge_logic.h
OBJS    := build/sketch.o build/sim_main.o build/sim_world.o build/Arduino.o build/Wire.o build/WebServer.o \
           build/Adafruit_BNO08x.o build/esp_stubs.o

kedge_sim: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

build/sketch.o: $(SKETCH) $(HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -include sketch_prelude.h -x c++ -c $< -o $@

build/%.o: %.cpp $(HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

build/%.o: stubs/%.cpp $(HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

build:
	mkdir -p build

test: kedge_sim
	python3 test_sim.py

clean:
	rm -rf build kedge_sim

.PHONY: test clean
