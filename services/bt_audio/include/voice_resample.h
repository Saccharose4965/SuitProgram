#pragma once
#include <stdint.h>
#include <stdbool.h>

// Streaming mono linear interpolation. Callback supplies input samples; no
// allocation, floating point, or file I/O in the Bluetooth data callback.
typedef struct {
    uint32_t input_rate, output_rate, phase;
    uint64_t remaining;
    int16_t a, b;
    bool primed;
} voice_resample_t;

static inline bool voice_resample_next(voice_resample_t *s,
                                      bool (*read_sample)(int16_t *), int16_t *out)
{
    if (!s->remaining) return false;
    if (!s->primed) {
        if (!read_sample(&s->a)) return false;
        if (!read_sample(&s->b)) s->b = s->a;
        s->primed = true;
    }
    *out = s->a + (int32_t)(((int64_t)(s->b - s->a) * s->phase) / s->output_rate);
    --s->remaining;
    s->phase += s->input_rate;
    while (s->phase >= s->output_rate) {
        s->phase -= s->output_rate;
        s->a = s->b;
        if (!read_sample(&s->b)) s->b = s->a;
    }
    return true;
}
