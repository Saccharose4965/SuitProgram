"""Compile production diagnostic lifecycle against a bounded fake heap trace."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

STUBS = r'''
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>
#define CONFIG_HEAP_TRACING_STANDALONE 1
#define CONFIG_HEAP_TRACING_STACK_DEPTH 6
#define EXT_RAM_BSS_ATTR
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define ESP_OK 0
#define HEAP_TRACE_LEAKS 1
typedef int esp_err_t;
typedef struct { void *address; bool freed; size_t size; void *alloced_by[6]; } heap_trace_record_t;
typedef struct { size_t high_water_mark, has_overflowed; } heap_trace_summary_t;
static size_t free_bytes = 45000;
static bool tracing;
static int starts, stops, reports, fail_init;
static heap_trace_record_t fake_records[3];
static size_t heap_caps_get_free_size(int c) { (void)c; return free_bytes; }
static const char *esp_err_to_name(int e) { (void)e; return "mock"; }
static void log_mock(const char *tag, const char *fmt, ...) { (void)tag; (void)fmt; ++reports; }
#define ESP_LOGW(...) log_mock(__VA_ARGS__)
static int heap_trace_init_standalone(void *p, size_t n) { (void)p; (void)n; return fail_init; }
static int heap_trace_start(int mode) { (void)mode; assert(!tracing); tracing=true; ++starts; return 0; }
static int heap_trace_stop(void) { assert(tracing); tracing=false; ++stops; return 0; }
static int heap_trace_summary(heap_trace_summary_t *s) { *s=(heap_trace_summary_t){3,1}; return 0; }
static size_t heap_trace_get_count(void) { return 3; }
static int heap_trace_get(size_t i, heap_trace_record_t *r) { *r=fake_records[i]; return 0; }
static bool esp_ptr_internal(void *p) { return p != (void *)2; }
'''

TEST = r'''
#include "playback_diagnostics.c"
int main(void) {
    fake_records[0]=(heap_trace_record_t){.address=(void *)1,.size=100,.alloced_by={(void *)4}};
    fake_records[1]=(heap_trace_record_t){.address=(void *)1,.size=200,.alloced_by={(void *)4}};
    fake_records[2]=(heap_trace_record_t){.address=(void *)2,.size=9000};
    playback_diagnostics_begin();
    playback_diagnostics_sample(false);
    assert(tracing && !captured);
    free_bytes=15000;
    playback_diagnostics_sample(false);
    assert(!tracing && captured && stops==1);
    int logs=reports;
    playback_diagnostics_sample(true);
    assert(reports==logs && stops==1); // snapshot frozen, no repeated work
    free_bytes=47000;
    playback_diagnostics_end();
    assert(sites[0].bytes==300 && sites[0].blocks==2); // external excluded, same stack grouped
    logs=reports;
    playback_diagnostics_end();
    assert(reports==logs);
    playback_diagnostics_begin();
    playback_diagnostics_sample(true); // stall with healthy heap
    assert(captured && stops==2);
    playback_diagnostics_end();
    playback_diagnostics_begin();
    playback_diagnostics_end(); // natural EOF also stops tracing
    assert(stops==3 && starts==3 && !tracing);
    fail_init=1;
    playback_diagnostics_begin();
    playback_diagnostics_sample(true);
    playback_diagnostics_end();
    assert(starts==3 && stops==3);
}
'''


class DiagnosticsTest(unittest.TestCase):
    def test_lifecycle(self):
        with tempfile.TemporaryDirectory(prefix="suit-playback-diag-") as tmp:
            path = Path(tmp)
            (path / "stubs.h").write_text(STUBS)
            for header in ("sdkconfig.h", "esp_attr.h", "esp_heap_caps.h",
                           "esp_heap_trace.h", "esp_log.h", "esp_memory_utils.h"):
                (path / header).write_text('#include "stubs.h"\n')
            (path / "test.c").write_text(TEST)
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-I", str(path),
                            "-I", str(ROOT), "-I", str(ROOT / "include"),
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    unittest.main()
