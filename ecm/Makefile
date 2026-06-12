# ECM host build -- compiles the portable control core with the SocketCAN
# backend into a Linux executable (build/ecm_host). For the on-target STM32
# build, see ecm/README.md (use can_stm32.c with -DECM_TARGET_STM32 in your
# CubeMX/Make/CMake project).

CC       ?= cc
CFLAGS   ?= -std=c11 -O2 -Wall -Wextra -D_DEFAULT_SOURCE
CPPFLAGS += -Iinclude
LDLIBS   += -lm

BUILD  := build
SRCS   := src/main_linux.c src/ecm.c src/eng_messages.c src/can_socketcan.c
OBJS   := $(patsubst src/%.c,$(BUILD)/%.o,$(SRCS))
TARGET := $(BUILD)/ecm_host

CHANNEL ?= vcan0

.PHONY: all clean run lib
all: $(TARGET)

# Shared library for the ctypes co-simulation tests (tests/*.py).
lib: | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -fPIC -shared \
	    src/ecm.c src/eng_messages.c tests/ffi_shim.c \
	    -o $(BUILD)/libecm.so $(LDLIBS)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDLIBS)

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

$(BUILD):
	mkdir -p $(BUILD)

# Run against a CAN interface: make run CHANNEL=vcan0
run: $(TARGET)
	$(TARGET) $(CHANNEL)

clean:
	rm -rf $(BUILD)
