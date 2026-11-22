# Virtual SM11 for the laptop bench: the real tars.ino (CAR_SM11) + physics on a pty.
#   make -C bench/car          build build/vuno (host clang, macOS)
#   make -C bench/car test     physics unit tests (build/test_physics, no firmware) + brownout grid
#   make -C bench/car clean
# The robot never runs this. No firmware/test-mock edits: the sketch and the mocks
# are compiled straight from iter1-wasd/uno/.
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter
REPO     := ../..
INO      := $(REPO)/iter1-wasd/uno/tars
MOCK     := $(REPO)/iter1-wasd/uno/test/mock
BUILD    := build

all: $(BUILD)/vuno

$(BUILD)/vuno: vuno.cpp physics.h json.h $(INO)/tars.ino $(wildcard $(MOCK)/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(MOCK) -I$(INO) -o $@ vuno.cpp

$(BUILD)/test_physics: test_physics.cpp physics.h json.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ test_physics.cpp

test: $(BUILD)/test_physics
	$(BUILD)/test_physics --grid --profiles profiles

clean:
	rm -rf $(BUILD)

.PHONY: all test clean
