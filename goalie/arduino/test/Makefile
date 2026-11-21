# Host unit tests for goalie.ino (no Arduino needed).
#   make -C goalie/arduino/test
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-function -g -fsanitize=address,undefined
BUILD    := build

test: $(BUILD)/test_goalie
	./$(BUILD)/test_goalie

$(BUILD)/test_goalie: test_goalie.cpp mock/Arduino.h mock/Wire.h ../goalie/goalie.ino
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -Imock -o $@ test_goalie.cpp

clean:
	rm -rf $(BUILD)

.PHONY: test clean
