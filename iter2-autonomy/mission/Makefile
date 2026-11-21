# iter2-autonomy/mission/: laptop mission control. Pi side = mission_relay (C, QNX native).
#
#   make               cross-compile mission_relay for QNX aarch64le (docker/run.sh if no qcc)
#   make native        build on the Pi itself with its clang (Andrew's route)
#   make host          build/mission_relay_host with the host cc (macOS clang)
#   make test          host build + relay tests + laptop app tests against the real stubs
#   make deploy        scp mission_relay to the Pi
#
# The laptop side (laptop/mission.py) is Python stdlib; nothing to build.

PI_HOST ?= qnxuser@tars.local
PI_DIR  ?= /data/home/qnxuser/mission
CFLAGS  := -Wall -Wextra -O2 -std=gnu11
QCC     := qcc -V gcc_ntoaarch64le
REPO    := $(abspath ../..)

.PHONY: all native host test test-relay test-laptop deploy clean

all: build/mission_relay

ifneq ($(shell command -v qcc 2>/dev/null),)
build/mission_relay: relay.c | build
	$(QCC) $(CFLAGS) -o $@ relay.c -lsocket
	@file $@ 2>/dev/null || true
else
build/mission_relay: relay.c
	@echo ">> qcc not found, building inside the QNX container"
	"$(REPO)/docker/run.sh" make -C "$(CURDIR)" build/mission_relay
endif

native: | build
	clang $(CFLAGS) -o build/mission_relay relay.c -lsocket

build:
	mkdir -p build

host: | build
	cc $(CFLAGS) -o build/mission_relay_host relay.c

test-relay: host
	$(MAKE) -C ../../iter1-wasd/pi host
	python3 tests/test_relay.py build/mission_relay_host

test-laptop: host
	$(MAKE) -C ../../iter1-wasd/pi host
	$(MAKE) -C ../goto host
	python3 tests/test_laptop.py

test: test-relay test-laptop

deploy: build/mission_relay
	ssh $(PI_HOST) 'mkdir -p $(PI_DIR)'
	scp build/mission_relay $(PI_HOST):$(PI_DIR)/

clean:
	rm -rf build
