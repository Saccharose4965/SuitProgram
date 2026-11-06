"""Run the production RX timestamp callbacks/read wrapper against a DMA queue."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / "drivers/audio_i2s/audio_i2s.c").read_text()


def function(name):
    start = re.search(r"^(?:static )?(?:bool IRAM_ATTR|esp_err_t) " + name + r"\(", source, re.M).start()
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


harness = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fft_phase.h"
#include "fft_display.h"
typedef int esp_err_t;
typedef int i2s_chan_handle_t;
typedef struct {size_t size;} i2s_event_data_t;
#define IRAM_ATTR
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_INVALID_STATE 2
#define ESP_ERR_TIMEOUT 3
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define portENTER_CRITICAL_ISR(p) ((void)(p))
#define portEXIT_CRITICAL_ISR(p) ((void)(p))
static int s_rx = 1, s_rx_clock_mux;
static int32_t s_prime_silence[256 * 2];
static uint64_t s_rx_received_bytes, s_rx_read_bytes, s_rx_dropped_bytes;
static int64_t s_rx_end_us;
static uint32_t s_rx_generation, s_rx_overflows;
static int rate = 44100, queued_blocks;
static int extra_after_read, limit_read, restart_during_read;
static double now, physical_rate = 44078, stream_start;
static uint64_t total_frames, last_consumed_frame;
static int64_t esp_timer_get_time(void) {return llround(now * 1e6);}
static int audio_i2s_get_rate(void) {return rate;}
static bool rx_received(i2s_chan_handle_t, i2s_event_data_t *, void *);
static bool rx_overflow(i2s_chan_handle_t, i2s_event_data_t *, void *);
static esp_err_t enable_rx_channel(void);
static int i2s_channel_enable(int channel) {
    (void)channel; queued_blocks = 0; total_frames = last_consumed_frame = 0;
    stream_start = now; return ESP_OK;
}
static void dma(int blocks) {
    while (blocks-- > 0) {
        total_frames += 256;
        now = stream_start + total_frames / physical_rate;
        i2s_event_data_t event = {.size = 2048};
        rx_received(s_rx, &event, NULL);
        if (queued_blocks == 7) {rx_overflow(s_rx, &event, NULL); --queued_blocks;}
        ++queued_blocks;
    }
}
static int i2s_channel_read(int channel, void *dest, size_t size,
                          size_t *bytes, uint32_t timeout) {
    (void)channel; (void)timeout;
    int blocks = size / 2048;
    if (limit_read && blocks > limit_read) blocks = limit_read;
    if (queued_blocks < blocks) dma(blocks - queued_blocks);
    queued_blocks -= blocks;
    *bytes = blocks * 2048;
    memset(dest, 0, *bytes);
    last_consumed_frame = total_frames - queued_blocks * 256;
    if (extra_after_read) dma(extra_after_read);
    if (restart_during_read) enable_rx_channel();
    now += .003; // read-return scheduling delay must not date the samples
    return *bytes < size ? ESP_ERR_TIMEOUT : ESP_OK;
}
@@FUNCTIONS@@

static double wrap(double x) {return x - floor(x);}
static double distance(double x) {return fabs(x - round(x));}
static float bands[DETECTOR_FRAMES * DETECTOR_BANDS];
static fft_history_snapshot_t history;
static uint8_t fb[FFT_DISPLAY_BYTES];

static void test_reads(void) {
    int32_t input[1024 * 2]; size_t bytes; int64_t stamp;
    enable_rx_channel();
    dma(6); // startup backlog
    assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_OK);
    double expected = stream_start + (last_consumed_frame - 1) / physical_rate;
    assert(fabs(stamp / 1e6 - expected) < .000030);
    extra_after_read = 2;
    assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_OK);
    expected = stream_start + (last_consumed_frame - 1) / physical_rate;
    assert(fabs(stamp / 1e6 - expected) < .000030);
    extra_after_read = 0;
    dma(12); // overflow discards queued blocks, not elapsed sample time
    assert(s_rx_overflows > 0);
    assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_OK);
    expected = stream_start + (last_consumed_frame - 1) / physical_rate;
    assert(fabs(stamp / 1e6 - expected) < .000030);
    limit_read = 1;
    assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_ERR_TIMEOUT);
    assert(bytes == 2048 && stamp > 0);
    limit_read = 0;
    restart_during_read = 1;
    assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_OK);
    assert(stamp == 0);
    restart_during_read = 0;
    assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_OK);
    expected = stream_start + (last_consumed_frame - 1) / physical_rate;
    assert(fabs(stamp / 1e6 - expected) < .000030);
    assert(audio_i2s_read_rx(input, 100, &bytes, 100, &stamp) == ESP_ERR_INVALID_ARG);
}

static void test_long_session(bool old_timestamp) {
    now = 1000;
    enable_rx_channel();
    fft_phase_tracker_t tracker = {0};
    fft_beat_clock_t clock = {0};
    fft_phase_evidence_t evidence = {0};
    int32_t input[1024 * 2];
    double last_emit = 0, first_time = 0, worst_audio_error = 0;
    int half_phase_failures = 0, good_frames = 0;
    double bpm = 150;
    // Twelve minutes of audio, with a new tempo midway through the run.
    for (int f = 1; f <= 32000; ++f) {
        size_t bytes; int64_t stamp;
        assert(audio_i2s_read_rx(input, sizeof(input), &bytes, 100, &stamp) == ESP_OK);
        double actual_center = stream_start + (last_consumed_frame - 512.5) / physical_rate;
        if (f == 1) first_time = now - 1.0 / rate;
        double end = old_timestamp ? first_time + (f - 1) * 1024.0 / rate : stamp / 1e6 - 511.5 / rate;
        if (f == 24000) bpm = 126;
        if (f < DETECTOR_FRAMES) continue;
        if (f % 8 == 0) {
            for (int i = 0; i < DETECTOR_FRAMES; ++i) {
                double time = actual_center - (DETECTOR_FRAMES - 1 - i) * 1024.0 / physical_rate;
                double phase = time * bpm / 60;
                phase -= round(phase);
                float energy = 1 + exp(-phase * phase / .001);
                for (int b = 0; b < DETECTOR_BANDS; ++b) bands[i * DETECTOR_BANDS + b] = energy;
            }
            double start = end - (DETECTOR_FRAMES - 1) * 1024.0 / rate;
            assert(fft_phase_evidence(bands, DETECTOR_FRAMES, rate / 1024.0, bpm, start, &evidence));
            fft_phase_decision_t d = fft_phase_update(&tracker, &evidence, bpm, end, true, false);
            if (!clock.initialized || d.acquired || d.restored) fft_beat_clock_init(&clock, bpm, d.anchor, now);
            else fft_beat_clock_schedule(&clock, bpm, d.anchor);
        }
        if (!fft_beat_clock_emit(&clock, now, &last_emit)) continue;
        if (f < 1000 || (f >= 24000 && f < 24500)) continue;
        double audible = distance(now * bpm / 60);
        double origin = wrap(clock.beat_start * bpm / 60);
        double plotted = distance(evidence.raw_best_bin / 128.0 - origin);
        if (audible > .40 && plotted < .04) ++half_phase_failures;
        if (audible > worst_audio_error) worst_audio_error = audible;
        if (!old_timestamp) {
            assert(audible < .09 && plotted < .04);
            history.detector.phase_valid = true;
            history.detector.held_bpm = bpm;
            for (int i = 0; i < 128; ++i) history.detector.phase[i] = evidence.score[i];
            history.latest.status.bpm = history.latest.status.clock_base_bpm = bpm;
            history.latest.clock_origin = origin;
            fft_display_draw(&history, FFT_VIEW_PHASE_COMB, fb);
            // Maximum of the actual lower OLED band must be near an edge.
            bool edge_peak = false;
            for (int x = 0; x < 128; ++x) {
                if ((x < 6 || x >= 122) && (fb[32 * 16 + x / 8] & (0x80 >> (x % 8)))) edge_peak = true;
            }
            assert(edge_peak);
            ++good_frames;
        }
    }
    if (old_timestamp) assert(half_phase_failures > 50);
    else assert(good_frames > 1000 && half_phase_failures == 0);
    printf("%s timestamps: %.3f worst audible phase error; %d graph-zero/offbeat-flash cases; %d aligned OLED frames\n",
           old_timestamp ? "First-read + nominal sample count" : "DMA", worst_audio_error, half_phase_failures, good_frames);
}
int main(void) {
    test_reads();
    test_long_session(true);
    test_long_session(false);
    puts("RX backlog, overflow, partial timeout, restart and long-session phase tests passed.");
}
'''
harness = harness.replace("@@FUNCTIONS@@", "\n".join(function(name) for name in (
    "rx_received", "rx_overflow", "enable_rx_channel", "audio_i2s_read_rx")))
with tempfile.TemporaryDirectory(prefix="suit-rx-time-") as directory:
    path = Path(directory)
    (path / "test.c").write_text(harness)
    subprocess.run([
        "gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-fsanitize=address,undefined", "-Iservices/fft/include",
        "-Iapps/fft_view/include", "-Iservices/fft/tests/host_stubs",
        str(path / "test.c"), "services/fft/fft_phase.c", "apps/fft_view/fft_display.c",
        "-lm", "-o", str(path / "test"),
    ], cwd=root, check=True)
    subprocess.run([str(path / "test")], check=True)
