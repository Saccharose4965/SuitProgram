#include "playback_diagnostics.h"
#include "sdkconfig.h"

#if CONFIG_HEAP_TRACING_STANDALONE
#include <stdlib.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_heap_trace.h"
#include "esp_log.h"
#include "esp_memory_utils.h"

#define TRACE_RECORDS 2048
#define SNAPSHOT_FREE_BYTES (16 * 1024)
#define REPORT_SITES 20

typedef struct {
    void *callers[CONFIG_HEAP_TRACING_STACK_DEPTH];
    size_t bytes, blocks;
} allocation_site_t;

static EXT_RAM_BSS_ATTR heap_trace_record_t records[TRACE_RECORDS];
static EXT_RAM_BSS_ATTR allocation_site_t sites[TRACE_RECORDS];
static const char *TAG = "playback_diag";
static bool active, captured;
static const char *reason;
static size_t start_free, snapshot_free;

static size_t internal_free(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

void playback_diagnostics_begin(void)
{
    active = captured = false;
    esp_err_t err = heap_trace_init_standalone(records, TRACE_RECORDS);
    if (err == ESP_OK) err = heap_trace_start(HEAP_TRACE_LEAKS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "trace unavailable: %s", esp_err_to_name(err));
        return;
    }
    active = true;
    start_free = internal_free();
    ESP_LOGW(TAG, "trace enabled: %u records in PSRAM, task allocations only; timing overhead present",
             TRACE_RECORDS);
}

void playback_diagnostics_sample(bool stalled)
{
    if (!active) return;
    size_t available = internal_free();
    if (!stalled && available >= SNAPSHOT_FREE_BYTES) return;
    heap_trace_stop();
    active = false;
    captured = true;
    snapshot_free = available;
    reason = stalled ? "audio stalled" : "internal heap below 16KB";
    // Do not dump a large trace while the audio/link is already struggling.
    ESP_LOGW(TAG, "snapshot frozen: %s, internal=%u; report after playback", reason,
             (unsigned)available);
}

static int largest_first(const void *a, const void *b)
{
    const allocation_site_t *x = a, *y = b;
    return (x->bytes < y->bytes) - (x->bytes > y->bytes);
}

void playback_diagnostics_end(void)
{
    if (!active && !captured) return;
    if (active) {
        heap_trace_stop();
        active = false;
        snapshot_free = internal_free();
        reason = "playback finished without trigger";
    }
    heap_trace_summary_t summary = {0};
    heap_trace_summary(&summary);
    size_t count = heap_trace_get_count(), used = 0, bytes = 0, blocks = 0;
    memset(sites, 0, sizeof(sites));
    for (size_t i = 0; i < count; ++i) {
        heap_trace_record_t record;
        if (heap_trace_get(i, &record) != ESP_OK || record.freed ||
            !esp_ptr_internal(record.address)) continue;
        size_t j = 0;
        while (j < used && memcmp(sites[j].callers, record.alloced_by,
                                  sizeof(sites[j].callers))) ++j;
        if (j == used) {
            memcpy(sites[j].callers, record.alloced_by, sizeof(sites[j].callers));
            ++used;
        }
        sites[j].bytes += record.size;
        ++sites[j].blocks;
        bytes += record.size;
        ++blocks;
    }
    qsort(sites, used, sizeof(sites[0]), largest_first);
    ESP_LOGW(TAG, "snapshot: %s; internal start=%u snapshot=%u now=%u", reason,
             (unsigned)start_free, (unsigned)snapshot_free, (unsigned)internal_free());
    ESP_LOGW(TAG, "trace records=%u/%u peak=%u overflow=%u; outstanding internal=%u bytes/%u blocks (not proof of leaks)",
             (unsigned)count, TRACE_RECORDS, (unsigned)summary.high_water_mark,
             (unsigned)summary.has_overflowed, (unsigned)bytes, (unsigned)blocks);
    for (size_t i = 0; i < used && i < REPORT_SITES; ++i) {
        ESP_LOGW(TAG, "site %u: %u bytes, %u blocks; allocation backtrace:",
                 (unsigned)i, (unsigned)sites[i].bytes, (unsigned)sites[i].blocks);
        for (size_t j = 0; j < CONFIG_HEAP_TRACING_STACK_DEPTH; ++j)
            ESP_LOGW(TAG, "  #%u %p", (unsigned)j, sites[i].callers[j]);
    }
    heap_trace_init_standalone(NULL, 0);
    captured = false;
}
#else
void playback_diagnostics_begin(void) {}
void playback_diagnostics_sample(bool stalled) { (void)stalled; }
void playback_diagnostics_end(void) {}
#endif
