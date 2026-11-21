# Host unit tests for tars.ino (no Arduino needed).
#   make -C iter1-wasd/uno/test     builds and runs five configurations of the sketch:
#     test_tars          Kuman SM11 kit: DC wheels, EN+IN driver, kit pins (the default build)
#     test_tars_stepper  custom: 28BYJ-48 stepper wheels, no arms
#     test_tars_arm1     custom: stepper wheels + one arm on A0-A3
#     test_tars_dc       custom: DC wheels, 2-PWM driver, two arms
#     test_tars_dc_enin  custom: DC wheels, EN+IN driver, two arms
CXX      ?= c++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wno-unused-function -g -fsanitize=address,undefined
BUILD    := build
DEPS     := test_tars.cpp mock/Arduino.h mock/Wire.h mock/EEPROM.h ../tars/tars.ino
VARIANTS := test_tars test_tars_stepper test_tars_arm1 test_tars_dc test_tars_dc_enin

test: $(addprefix $(BUILD)/,$(VARIANTS))
	@for v in $(VARIANTS); do echo "== $$v"; ./$(BUILD)/$$v || exit 1; done

$(BUILD)/test_tars: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_stepper: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DWHEEL_TYPE=2 -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_arm1: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DWHEEL_TYPE=2 -DARM_COUNT=1 -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_dc: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DWHEEL_TYPE=1 -Imock -o $@ test_tars.cpp

$(BUILD)/test_tars_dc_enin: $(DEPS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DWHEEL_TYPE=1 -DDRIVER_TYPE=2 -Imock -o $@ test_tars.cpp

clean:
	rm -rf $(BUILD)

.PHONY: test clean
