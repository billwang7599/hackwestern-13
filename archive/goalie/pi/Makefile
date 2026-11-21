# goalie_motor: Beat the Goalie real-time server (QNX 8.0, Pi 5).
#
#   make               cross-compile for QNX aarch64 (auto-delegates to
#                      ../../docker/run.sh if qcc isn't on PATH)
#   make host          build with the host cc (macOS clang) for quick testing
#   make test          host build + test_server.py against the stub backend
#   make parity        C predictor/controller vs goalie/sim Python, same event stream
#   make deploy        scp binary to the Pi
#   make run           ssh in and run it (Ctrl-C stops the motor)
#   make compdb        regenerate compile_commands.json for clangd (via bear)
#
# Override: make run PI_HOST=qnxuser@192.168.1.42 BACKEND=stub DEV=/dev/ser1

PI_HOST  ?= qnxuser@goalie.local
PI_DIR   ?= /data/home/qnxuser
BACKEND  ?= arduino
DEV      ?= /dev/serusb1
PORT     ?= 5556

BIN      := goalie_motor
SRCS     := src/main.c src/predict.c src/control.c src/goalie_stub.c src/goalie_arduino.c
CFLAGS   := -Wall -Wextra -O2 -std=gnu11

QCC      := qcc -V gcc_ntoaarch64le
QNX_LIBS := -lsocket -lm

REPO     := $(abspath ../..)

.PHONY: all host test parity deploy run compdb clean

all: build/$(BIN)

ifneq ($(shell command -v qcc 2>/dev/null),)
build/$(BIN): $(SRCS) src/*.h | build
	$(QCC) $(CFLAGS) -o $@ $(SRCS) $(QNX_LIBS)
	@file $@ 2>/dev/null || true
else
build/$(BIN): $(SRCS) src/*.h
	@echo ">> qcc not found, building inside the QNX container"
	"$(REPO)/docker/run.sh" make -C "$(CURDIR)" build/$(BIN)
endif

build:
	mkdir -p build

host: | build
	cc $(CFLAGS) -o build/$(BIN)_host $(SRCS) -lpthread -lm
	@echo ">> run: ./build/$(BIN)_host -b stub"

test: host
	python3 test_server.py

SIM_PY   := $(firstword $(wildcard ../sim/.venv/bin/python) python3)

build/parity_harness: tools/parity_harness.c src/predict.c src/control.c src/*.h | build
	cc $(CFLAGS) -o $@ tools/parity_harness.c src/predict.c src/control.c -lm

parity: build/parity_harness
	"$(SIM_PY)" tools/parity.py
	"$(SIM_PY)" tools/parity.py --speed 2.0 --walls --drop 0.1 --rpm 90

deploy: build/$(BIN)
	scp build/$(BIN) $(PI_HOST):$(PI_DIR)/$(BIN)

run:
	ssh -t $(PI_HOST) '$(PI_DIR)/$(BIN) -b $(BACKEND) -d $(DEV) -p $(PORT)'

compdb:
ifneq ($(shell command -v qcc 2>/dev/null),)
	rm -rf build && bear -- $(MAKE) all
else
	"$(REPO)/docker/run.sh" make -C "$(CURDIR)" compdb
endif

clean:
	rm -rf build
