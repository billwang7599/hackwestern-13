# Host unit tests for balancer.ino (no Arduino needed).
#   make -C balancer/uno/test
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-function -g -fsanitize=address,undefined
BUILD    := build

test: $(BUILD)/test_balancer
	./$(BUILD)/test_balancer

$(BUILD)/test_balancer: test_balancer.cpp mock/Arduino.h mock/Wire.h mock/EEPROM.h ../balancer/balancer.ino
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -Imock -o $@ test_balancer.cpp

clean:
	rm -rf $(BUILD)

.PHONY: test clean
