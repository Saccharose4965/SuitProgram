CC ?= gcc
CFLAGS ?= -O3
CFLAGS += -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Wpedantic
LDLIBS := $(shell pkg-config --libs alsa sdl2) -lm
CPPFLAGS := $(shell pkg-config --cflags alsa sdl2)

.PHONY: all run clean test

all: latestfft

latestfft: LatestFFT.c include/bpm_core.h include/phase_tracker.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -o $@ $(LDLIBS)

run: latestfft
	./latestfft

clean:
	$(RM) latestfft

# Headless regressions; no microphone or window is opened.
test:
	@set -e; test_bin=$$(mktemp /tmp/localfft-test.XXXXXX); \
	trap 'rm -f "$$test_bin"' EXIT; \
	for source in tests/test_phase_gate.c tests/test_slow_updates.c tests/test_phase_tracker.c tests/test_phase_display.c; do \
	$(CC) $(CPPFLAGS) $(CFLAGS) -Werror -fsanitize=address,undefined "$$source" -o "$$test_bin" $(LDLIBS); \
	"$$test_bin"; \
	done
