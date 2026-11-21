# TARS motor server.
#
#   make               cross-compile for QNX aarch64 (inside docker/run.sh; auto-
#                      delegates to the container if qcc isn't on PATH)
#   make host          build with the host cc (macOS clang) for quick testing
#   make test          host build + test_server.py (stub + fake-Uno pty)
#   make deploy        scp binary to the Pi
#   make run           ssh in and run it (Ctrl-C stops motors)
#   make test-bridge   same tests against tars_bridge.py (Python bring-up/debug tool, NOT the demo path)
#   make deploy-bridge / run-bridge   scp / ssh-run the Python bridge on the Pi
#   make compdb        regenerate compile_commands.json for clangd (via bear)
#
# Override: make run PI_HOST=qnxuser@192.168.1.42  BACKEND=stub  UART=/dev/ser1

PI_HOST  ?= qnxuser@tars.local
PI_DIR   ?= /data/home/qnxuser
BACKEND  ?= uno
UART     ?= /dev/serusb1
PORT     ?= 5555

BIN      := tars_motor
SRCS     := src/main.c src/gesture.c src/motor_stub.c src/motor_uno_serial.c \
            src/motor_pico_uart.c src/obstacle.c
CFLAGS   := -Wall -Wextra -O2 -std=gnu11

QCC      := qcc -V gcc_ntoaarch64le
QNX_LIBS := -lsocket -lm

REPO     := $(abspath ..)

.PHONY: all host test test-bridge deploy deploy-bridge run-bridge run compdb clean

all: build/$(BIN)

ifneq ($(shell command -v qcc 2>/dev/null),)
build/$(BIN): $(SRCS) src/*.h | build
	$(QCC) $(CFLAGS) -o $@ $(SRCS) $(QNX_LIBS)
	@file $@ 2>/dev/null || true
else
build/$(BIN): $(SRCS) src/*.h
	@echo ">> qcc not found, building inside the QNX container"
	$(REPO)/docker/run.sh make -C $(CURDIR) build/$(BIN)
endif

build:
	mkdir -p build

host: | build
	cc $(CFLAGS) -o build/$(BIN)_host $(SRCS) -lpthread -lm
	@echo ">> run: ./build/$(BIN)_host -b stub"

# Host build + protocol/watchdog/gesture/tip tests against -b stub, plus the
# real uno backend talking to a fake Uno on a pseudo-terminal. stdlib only.
test: host
	python3 test_server.py build/$(BIN)_host

# Python drop-in for tars_motor (no QNX build needed): same tests + bridge-only tests.
test-bridge:
	python3 test_server.py ./tars_bridge.py
	python3 test_bridge.py

deploy-bridge:
	scp tars_bridge.py $(PI_HOST):$(PI_DIR)/tars_bridge.py

run-bridge:
	ssh -t $(PI_HOST) 'python3 $(PI_DIR)/tars_bridge.py -p $(PORT)'

deploy: build/$(BIN)
	scp build/$(BIN) $(PI_HOST):$(PI_DIR)/$(BIN)

run:
	ssh -t $(PI_HOST) '$(PI_DIR)/$(BIN) -b $(BACKEND) -d $(UART) -p $(PORT)'

compdb:
ifneq ($(shell command -v qcc 2>/dev/null),)
	rm -rf build && bear -- $(MAKE) all
else
	$(REPO)/docker/run.sh make -C $(CURDIR) compdb
endif

clean:
	rm -rf build
