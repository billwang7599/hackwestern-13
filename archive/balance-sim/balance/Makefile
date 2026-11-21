# balance/ - portable balance controller.
#   make          shared lib for the simulator (libbalance.dylib / .so) + tests
#   make test     build and run the unit tests
#   make clean
#
# The QNX build does not use this Makefile: add balance.c to SRCS in
# pi/Makefile (qcc compiles it as-is). See docs/balance.md.

CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror=implicit-function-declaration
LDLIBS  := -lm

UNAME := $(shell uname -s)
ifeq ($(UNAME),Darwin)
  LIB := build/libbalance.dylib
  SHFLAGS := -dynamiclib
else
  LIB := build/libbalance.so
  SHFLAGS := -shared
endif

all: $(LIB) build/test_balance

build:
	mkdir -p build

$(LIB): balance.c balance.h | build
	$(CC) $(CFLAGS) -fPIC $(SHFLAGS) -o $@ balance.c $(LDLIBS)

build/test_balance: test_balance.c balance.c balance.h | build
	$(CC) $(CFLAGS) -Wno-double-promotion -o $@ test_balance.c balance.c $(LDLIBS)

test: build/test_balance
	./build/test_balance

clean:
	rm -rf build

.PHONY: all test clean
