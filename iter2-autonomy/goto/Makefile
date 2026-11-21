# iter2-autonomy/goto/: go-to-object controller (C) + detector (C++), QNX native.
#
#   make               cross-compile goto for QNX aarch64le (inside docker/run.sh
#                      if qcc isn't on PATH)
#   make detect        cross-compile detect (C++, needs ONNX Runtime built for QNX,
#                      see README "Build ONNX Runtime for QNX"; ORT_ROOT=...)
#   make host          build goto + tests with the host cc (macOS clang)
#   make test          host build, C unit tests, Python 2D sim + socket tests
#                      (includes test_nav, the nav grid/A*/LDR1 tests)
#   make test-nav-asan test_nav + 1M-packet LDR1 fuzz under ASan/UBSan
#   make test-nav-parity  nav.c A* vs iter2-autonomy/dashboard/mapping.py on shared fixtures
#   make test-fullstack   scripted full-stack sim (obstacle, occlusion, moved object, STOP, visual, voice)
#   make test-fullstack-dc  the same with sloppy DC-car motion (+-25% MOVE/TURN, 15 deg/m drift)
#   (make test also runs tests/test_dc_car.py: the 2D sim on the SM11 DC car, sloppy moves)
#   make deploy        scp goto (+ detect if built) and the config to the Pi
#
# Override: make deploy PI_HOST=qnxuser@192.168.1.42

PI_HOST  ?= qnxuser@tars.local
PI_DIR   ?= /data/home/qnxuser/goto
ORT_ROOT ?= $(QNX_TARGET)/aarch64le/usr/local
ORT_HOST ?= /opt/homebrew

CFLAGS   := -Wall -Wextra -O2 -std=gnu11
CXXFLAGS := -Wall -Wextra -O2 -std=gnu++17
CORE     := src/goto_core.c
NAV      := src/nav.c
GOTO_SRC := src/goto.c $(CORE) $(NAV)
DET_SRC  := src/detect.cpp
DET_HDR  := src/yolo_post.hpp src/camera.hpp

QCC      := qcc -V gcc_ntoaarch64le
QXX      := q++ -V gcc_ntoaarch64le_cxx
REPO     := $(abspath ../..)

.PHONY: all detect host host-detect test test-nav-asan test-nav-parity test-fullstack test-fullstack-dc deploy clean

all: build/goto

ifneq ($(shell command -v qcc 2>/dev/null),)
build/goto: $(GOTO_SRC) src/*.h | build
	$(QCC) $(CFLAGS) -o $@ $(GOTO_SRC) -lsocket -lm
	@file $@ 2>/dev/null || true

# Camera library (camapi) + ONNX Runtime C++ API. -DHAVE_ORT -DHAVE_CAMAPI
build/detect: $(DET_SRC) $(DET_HDR) | build
	$(QXX) $(CXXFLAGS) -DHAVE_ORT -DHAVE_CAMAPI -I$(ORT_ROOT)/include -I$(ORT_ROOT)/include/onnxruntime \
	    -o $@ $(DET_SRC) -L$(ORT_ROOT)/lib -lonnxruntime -lcamapi -lsocket -lm
	@file $@ 2>/dev/null || true
else
build/goto: $(GOTO_SRC) src/*.h
	@echo ">> qcc not found, building inside the QNX container"
	"$(REPO)/docker/run.sh" make -C "$(CURDIR)" build/goto
build/detect: $(DET_SRC) $(DET_HDR)
	@echo ">> qcc not found, building inside the QNX container"
	"$(REPO)/docker/run.sh" make -C "$(CURDIR)" build/detect ORT_ROOT=$(ORT_ROOT)
endif

detect: build/detect

build:
	mkdir -p build

host: | build
	cc $(CFLAGS) -o build/goto_host $(GOTO_SRC) -lm
	cc $(CFLAGS) -o build/test_core tests/test_core.c $(CORE) $(NAV) -lm
	cc $(CFLAGS) -std=c99 -pedantic -c -o build/nav.o $(NAV)
	cc $(CFLAGS) -o build/test_nav tests/test_nav.c $(NAV) -lm
	cc $(CFLAGS) -o build/nav_cli tests/nav_cli.c $(NAV) -lm
	cc $(CFLAGS) -c -o build/goto_core.o $(CORE)
	c++ $(CXXFLAGS) -o build/test_yolo tests/test_yolo.cpp build/goto_core.o build/nav.o
	c++ $(CXXFLAGS) -o build/detect_host $(DET_SRC)
	@echo ">> goto: ./build/goto_host -v    detect (no ORT): ./build/detect_host --help"

# detect with real inference on the Mac: brew install onnxruntime
host-detect: | build
	c++ $(CXXFLAGS) -DHAVE_ORT -I$(ORT_HOST)/include -I$(ORT_HOST)/include/onnxruntime \
	    -o build/detect_host_ort $(DET_SRC) -L$(ORT_HOST)/lib -lonnxruntime -Wl,-rpath,$(ORT_HOST)/lib

test: host
	./build/test_core
	./build/test_nav
	./build/test_yolo
	python3 tests/test_goto_sim.py build/goto_host
	python3 tests/test_dc_car.py build/goto_host

SAN := -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -g -O1
test-nav-asan: | build
	cc $(CFLAGS) $(SAN) -o build/test_nav_asan tests/test_nav.c $(NAV) -lm
	./build/test_nav_asan 1000000

# real tars_motor_host + goto + tars_voice, fake phone (LDR1) / YOLO / lidar_guard / phone speech
test-fullstack: host
	$(MAKE) -C ../../iter1-wasd/pi host
	python3 tests/test_fullstack.py

test-fullstack-dc: host
	$(MAKE) -C ../../iter1-wasd/pi host
	python3 tests/test_fullstack_dc.py

test-nav-parity: host
	python3 tests/nav_parity.py build/nav_cli --random 200

deploy: build/goto
	ssh $(PI_HOST) 'mkdir -p $(PI_DIR)/models'
	scp build/goto goto.conf objects.txt $(PI_HOST):$(PI_DIR)/
	-test -f build/detect && scp build/detect $(PI_HOST):$(PI_DIR)/
	-test -f models/yolo_world.onnx && scp models/yolo_world.onnx models/labels.txt $(PI_HOST):$(PI_DIR)/models/

clean:
	rm -rf build
