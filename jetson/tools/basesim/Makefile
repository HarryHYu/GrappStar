# basesim: the real base firmware logic + its physics model (sim.h) behind a C1 JSON-lines port.
#   make                      build with the firmware in ../../../firmware/base (or the fallback extract)
#   make FW=/path/to/firmware/base
FW_DEFAULT  := ../../../firmware/base
FW_FALLBACK := /tmp/claude-0/basesim/firmware/base
FW ?= $(if $(wildcard $(FW_DEFAULT)/grappstar_base/base_logic.h),$(if $(wildcard $(FW_DEFAULT)/test/sim.h),$(FW_DEFAULT),$(FW_FALLBACK)),$(FW_FALLBACK))

CXX      ?= g++
CXXFLAGS ?= -O2 -g -Wall -Wno-unused-parameter -Wno-misleading-indentation -Wno-unused-variable
LOGIC    := $(FW)/grappstar_base/base_logic.h $(wildcard $(FW)/grappstar_base/*.h) $(FW)/test/sim.h

basesim: basesim.cpp $(LOGIC)
	@test -f $(FW)/grappstar_base/base_logic.h -a -f $(FW)/test/sim.h || { echo "basesim: no firmware at FW=$(FW)"; exit 1; }
	$(CXX) -std=gnu++17 $(CXXFLAGS) -I$(FW)/test -o $@ basesim.cpp
	@echo "built basesim from $(FW)"

clean:
	rm -f basesim

.PHONY: clean
