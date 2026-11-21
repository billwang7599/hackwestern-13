# TARS voice: whisper.cpp speech -> intent classifier -> tars_motor.
#
#   make host          Mac/Linux build: build/tars_voice_host, build/intent_cli, build/stt_cli (laptop)
#   make test          intent tests + end-to-end (say -> WAV -> whisper -> fake tars_motor)
#   make test-intent   only the classifier tests (no whisper / model needed)
#   make test-stt      whisper repeat-loop filter (no model needed)
#   make test-net      HEAR / AUDIO / WATCH network input (mission control)
#   make test-goto-link  voice moves vs the real goto + tars_motor (CANCEL race)
#   make               QNX aarch64 build (in docker/run.sh): build/qnx/tars_voice, build/qnx/mic_test
#   make mic_test      QNX mic check only (no whisper; small, build this first)
#   make deploy PI_HOST=qnxuser@<pi-ip>
#
# Needs: iter2-autonomy/voice/fetch_whisper.sh (pinned whisper.cpp v1.9.5) and iter2-autonomy/voice/get_model.sh.

PI_HOST ?= qnxuser@tars.local
PI_DIR  ?= /data/home/qnxuser/voice
REPO    := $(abspath ../..)
WSRC    := third_party/whisper.cpp
CFLAGS  := -O2 -Wall -Wextra -std=gnu11
INC     := -Isrc -Iintent -I$(WSRC)/include -I$(WSRC)/ggml/include
SRCS    := src/main.c src/audio.c src/vad.c src/stt.c intent/intent.c

WHOST   := build/whisper-host
WLIBS_H := $(WHOST)/src/libwhisper.a $(WHOST)/ggml/src/libggml.a $(WHOST)/ggml/src/libggml-cpu.a $(WHOST)/ggml/src/libggml-base.a
WQNX    := build/whisper-qnx
WLIBS_Q := $(WQNX)/src/libwhisper.a $(WQNX)/ggml/src/libggml.a $(WQNX)/ggml/src/libggml-cpu.a $(WQNX)/ggml/src/libggml-base.a

WHISPER_CMAKE := -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DWHISPER_BUILD_TESTS=OFF \
    -DWHISPER_BUILD_EXAMPLES=OFF -DWHISPER_BUILD_SERVER=OFF -DGGML_METAL=OFF -DGGML_BLAS=OFF \
    -DGGML_OPENMP=OFF -DGGML_NATIVE=OFF -DWHISPER_SDL2=OFF

UNAME := $(shell uname)
ifeq ($(UNAME),Darwin)
HOST_LIBS := -framework Accelerate -framework Foundation
endif

.PHONY: all host test test-intent test-stt test-net test-goto-link e2e qnx mic_test deploy clean fetch model

all: qnx

fetch:
	./fetch_whisper.sh

model:
	./get_model.sh

# ---------------------------------------------------------------- host
$(WLIBS_H):
	@test -d $(WSRC) || ./fetch_whisper.sh
	cmake -S $(WSRC) -B $(WHOST) $(WHISPER_CMAKE) >/dev/null
	cmake --build $(WHOST) -j 8 --target whisper

build/intent_cli: intent/intent_cli.c intent/intent.c intent/intent.h intent/intent_model.h
	@mkdir -p build
	cc $(CFLAGS) -Iintent -o $@ intent/intent_cli.c intent/intent.c -lm

build/tars_voice_host: $(SRCS) src/*.h intent/*.h $(WLIBS_H)
	cc $(CFLAGS) $(INC) -c src/main.c -o build/main.o
	cc $(CFLAGS) $(INC) -c src/audio.c -o build/audio.o
	cc $(CFLAGS) $(INC) -c src/vad.c -o build/vad.o
	cc $(CFLAGS) $(INC) -c src/stt.c -o build/stt.o
	cc $(CFLAGS) $(INC) -c intent/intent.c -o build/intent.o
	c++ -o $@ build/main.o build/audio.o build/vad.o build/stt.o build/intent.o $(WLIBS_H) $(HOST_LIBS) -lpthread -lm

# laptop-only: transcript of one WAV (mission control --laptop-stt), no classifier
build/stt_cli: src/stt_cli.c src/stt.c src/audio.c src/*.h $(WLIBS_H)
	cc $(CFLAGS) $(INC) -c src/stt_cli.c -o build/stt_cli.o
	c++ -o $@ build/stt_cli.o build/audio.o build/stt.o $(WLIBS_H) $(HOST_LIBS) -lpthread -lm

host: build/intent_cli build/tars_voice_host build/stt_cli

test-intent: build/intent_cli
	python3 tests/test_intent.py

e2e: host
	@test -f models/ggml-tiny.en-q5_1.bin || ./get_model.sh
	python3 tests/test_e2e.py

# network input (HEAR / AUDIO / WATCH, mission control) against the real tars_motor stub
test-net: host
	@test -f models/ggml-tiny.en-q5_1.bin || ./get_model.sh
	python3 tests/test_net.py

# voice manual moves vs the REAL goto + REAL tars_motor (CANCEL/S race, goto stalls)
test-goto-link: host
	@test -f models/ggml-tiny.en-q5_1.bin || ./get_model.sh
	@test -x ../goto/build/goto_host || $(MAKE) -C ../goto host
	@test -x ../../iter1-wasd/pi/build/tars_motor_host || $(MAKE) -C ../../iter1-wasd/pi host
	python3 tests/test_goto_link.py

# whisper repeat-loop filter (stt_dedupe) via stt_cli --dedupe; no model needed
test-stt: build/stt_cli
	python3 tests/test_stt.py

test: test-intent test-stt e2e test-net test-goto-link

# ---------------------------------------------------------------- QNX (aarch64)
ifneq ($(shell command -v qcc 2>/dev/null),)
QCC := qcc -V gcc_ntoaarch64le
$(WLIBS_Q):
	@test -d $(WSRC) || ./fetch_whisper.sh
	cmake -S $(WSRC) -B $(WQNX) $(WHISPER_CMAKE) -DCMAKE_TOOLCHAIN_FILE=$(CURDIR)/cmake/qnx-aarch64.cmake \
	      -DGGML_CPU_ARM_ARCH=armv8.2-a+dotprod+fp16
	cmake --build $(WQNX) -j 8 --target whisper

build/qnx/tars_voice: $(SRCS) src/*.h intent/*.h $(WLIBS_Q)
	@mkdir -p build/qnx
	$(QCC) $(CFLAGS) $(INC) -c src/main.c -o build/qnx/main.o
	$(QCC) $(CFLAGS) $(INC) -c src/audio.c -o build/qnx/audio.o
	$(QCC) $(CFLAGS) $(INC) -c src/vad.c -o build/qnx/vad.o
	$(QCC) $(CFLAGS) $(INC) -c src/stt.c -o build/qnx/stt.o
	$(QCC) $(CFLAGS) $(INC) -c intent/intent.c -o build/qnx/intent.o
	$(QCC) -lang-c++ -o $@ build/qnx/main.o build/qnx/audio.o build/qnx/vad.o build/qnx/stt.o build/qnx/intent.o \
	      $(WLIBS_Q) -lasound -lsocket -lm
	@file $@ 2>/dev/null || true

build/qnx/mic_test: qnx/mic_test.c src/audio.c src/audio.h
	@mkdir -p build/qnx
	$(QCC) $(CFLAGS) -Isrc -o $@ qnx/mic_test.c src/audio.c -lasound -lm

qnx: build/qnx/mic_test build/qnx/tars_voice
mic_test: build/qnx/mic_test
else
qnx mic_test:
	@echo ">> qcc not found, building inside the QNX container"
	$(REPO)/docker/run.sh make -C $(CURDIR) $@
endif

# ---------------------------------------------------------------- deploy
deploy:
	ssh $(PI_HOST) 'mkdir -p $(PI_DIR)/models $(PI_DIR)/logs'
	scp build/qnx/mic_test $(PI_HOST):$(PI_DIR)/
	-scp build/qnx/tars_voice $(PI_HOST):$(PI_DIR)/
	scp voice.conf qnx/start_voice.sh $(PI_HOST):$(PI_DIR)/
	ssh $(PI_HOST) 'test -f $(PI_DIR)/models/ggml-tiny.en-q5_1.bin' || \
	    scp models/ggml-tiny.en-q5_1.bin $(PI_HOST):$(PI_DIR)/models/

clean:
	rm -rf build/main.o build/audio.o build/vad.o build/stt.o build/intent.o build/stt_cli.o build/tars_voice_host build/intent_cli build/stt_cli build/qnx
