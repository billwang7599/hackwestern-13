# TARS motor server.
#
#   make               cross-compile for QNX aarch64 (inside docker/run.sh; auto-
#                      delegates to the container if qcc isn't on PATH)
#   make host          build with the host cc (macOS clang) for quick testing
#   make deploy        scp binary to the Pi
#   make run           ssh in and run it (Ctrl-C stops motors)
#   make compdb        regenerate compile_commands.json for clangd (via bear)
#
# Override: make deploy PI_HOST=qnxuser@192.168.1.42  BACKEND=stub  UART=/dev/ser2

PI_HOST  ?= qnxuser@tars.local
PI_DIR   ?= /data/home/qnxuser
BACKEND  ?= pico
UART     ?= /dev/ser1
PORT     ?= 5555

BIN      := tars_motor
SRCS     := src/main.c src/motor_stub.c src/motor_pico_uart.c src/obstacle.c
CFLAGS   := -Wall -Wextra -O2 -std=gnu11

QCC      := qcc -V gcc_ntoaarch64le
QNX_LIBS := -lsocket -lm

REPO     := $(abspath ..)

.PHONY: all host deploy run compdb clean

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
