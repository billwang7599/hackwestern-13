# goto/: go-to-object controller (C) + detector (C++), QNX native.
#
#   make               cross-compile goto for QNX aarch64le (inside docker/run.sh
#                      if qcc isn't on PATH)
#   make detect        cross-compile detect (C++, needs ONNX Runtime built for QNX,
#                      see README "Build ONNX Runtime for QNX"; ORT_ROOT=...)
#   make host          build goto + tests with the host cc (macOS clang)
#   make test          host build, C unit tests, Python 2D sim + socket tests
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
GOTO_SRC := src/goto.c $(CORE)
DET_SRC  := src/detect.cpp
DET_HDR  := src/yolo_post.hpp src/camera.hpp

QCC      := qcc -V gcc_ntoaarch64le
QXX      := q++ -V gcc_ntoaarch64le_cxx
REPO     := $(abspath ..)

.PHONY: all detect host host-detect test deploy clean

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
	cc $(CFLAGS) -o build/test_core tests/test_core.c $(CORE) -lm
	cc $(CFLAGS) -c -o build/goto_core.o $(CORE)
	c++ $(CXXFLAGS) -o build/test_yolo tests/test_yolo.cpp build/goto_core.o
	c++ $(CXXFLAGS) -o build/detect_host $(DET_SRC)
	@echo ">> goto: ./build/goto_host -v    detect (no ORT): ./build/detect_host --help"

# detect with real inference on the Mac: brew install onnxruntime
host-detect: | build
	c++ $(CXXFLAGS) -DHAVE_ORT -I$(ORT_HOST)/include -I$(ORT_HOST)/include/onnxruntime \
	    -o build/detect_host_ort $(DET_SRC) -L$(ORT_HOST)/lib -lonnxruntime -Wl,-rpath,$(ORT_HOST)/lib

test: host
	./build/test_core
	./build/test_yolo
	python3 tests/test_goto_sim.py build/goto_host

deploy: build/goto
	ssh $(PI_HOST) 'mkdir -p $(PI_DIR)/models'
	scp build/goto goto.conf objects.txt $(PI_HOST):$(PI_DIR)/
	-test -f build/detect && scp build/detect $(PI_HOST):$(PI_DIR)/
	-test -f models/yolo_world.onnx && scp models/yolo_world.onnx models/labels.txt $(PI_HOST):$(PI_DIR)/models/

clean:
	rm -rf build
