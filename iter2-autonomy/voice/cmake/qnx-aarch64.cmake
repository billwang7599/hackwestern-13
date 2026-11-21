# CMake toolchain: QNX SDP 8.0, aarch64 (Raspberry Pi 5). Used by iter2-autonomy/voice/Makefile to
# cross-build whisper.cpp/ggml with qcc. Needs the SDP environment sourced
# (docker/run.sh does that: QNX_HOST / QNX_TARGET set, qcc on PATH).
#
# NOT YET RUN: written without SDP 8.0 on the build Mac. If it fails, the usual
# suspects are listed in iter2-autonomy/voice/README.md ("QNX build").
set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_VERSION 8.0.0)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER qcc)
set(CMAKE_C_COMPILER_TARGET gcc_ntoaarch64le)
set(CMAKE_CXX_COMPILER q++)
set(CMAKE_CXX_COMPILER_TARGET gcc_ntoaarch64le_cxx)

if(DEFINED ENV{QNX_TARGET})
  set(CMAKE_SYSROOT $ENV{QNX_TARGET})
  set(CMAKE_FIND_ROOT_PATH $ENV{QNX_TARGET} $ENV{QNX_TARGET}/aarch64le)
endif()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Pi 5 = Cortex-A76 (Armv8.2-A, dotprod, fp16). -D_QNX_SOURCE exposes POSIX+QNX APIs.
set(CMAKE_C_FLAGS_INIT "-D_QNX_SOURCE")
set(CMAKE_CXX_FLAGS_INIT "-D_QNX_SOURCE")
# QNX: sockets live in libsocket, threads are in libc (Threads::Threads resolves to nothing).
set(CMAKE_EXE_LINKER_FLAGS_INIT "-lsocket")
set(THREADS_PREFER_PTHREAD_FLAG OFF)
