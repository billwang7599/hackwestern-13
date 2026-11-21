# Host unit tests for tars.ino (no Arduino needed).
#   make -C uno/test     builds and runs four configurations of the sketch:
#     test_tars          stepper wheels, no arms (the default build)
#     test_tars_arm1     stepper wheels + one arm on A0-A3
#     test_tars_dc       DC wheels, 2-PWM driver, two arms (the old build)
#     test_tars_dc_enin  DC wheels, EN+IN driver, two arms
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-function -g -fsanitize=address,undefined
BUILD    := build
DEPS     := test_tars.cpp mock/Arduino.h mock/Wire.h mock/EEPROM.h ../tars/tars.ino
VARIANTS := test_tars test_tars_arm1 test_tars_dc test_tars_dc_enin

test: $(addprefix $(BUILD)/,$(VARIANTS))
	@for v in $(VARIANTS); do echo "== $$v"; ./$(BUILD)/$$v || exit 1; done

$(BUILD)/test_tars: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_arm1: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DARM_COUNT=1 -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_dc: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DWHEEL_TYPE=1 -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_dc_enin: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DWHEEL_TYPE=1 -DDRIVER_TYPE=2 -Imock -o $@ test_tars.cpp

clean:
	rm -rf $(BUILD)

.PHONY: test clean
