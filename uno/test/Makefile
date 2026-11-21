# Host unit tests for tars.ino (no Arduino needed).
#   make -C uno/test            default build (2-PWM driver)
#                               runs everything twice: 2-PWM and EN+IN driver configs
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-function -g -fsanitize=address,undefined
BUILD    := build

test: $(BUILD)/test_tars $(BUILD)/test_tars_enin
	./$(BUILD)/test_tars
	./$(BUILD)/test_tars_enin

$(BUILD)/test_tars: test_tars.cpp mock/Arduino.h mock/Wire.h ../tars/tars.ino
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_enin: test_tars.cpp mock/Arduino.h mock/Wire.h ../tars/tars.ino
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DDRIVER_TYPE=2 -Imock -o $@ test_tars.cpp

clean:
	rm -rf $(BUILD)

.PHONY: test clean
