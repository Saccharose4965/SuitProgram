#pragma once

// Shared constants used by fft logic and rendering
enum {
    SAMPLE_RATE_HZ = 16000,
    HOP_SAMPLES    = 256,     // 16 ms hop
    FFT_SIZE       = 1024,    // 64 ms window
    FPS            = SAMPLE_RATE_HZ / HOP_SAMPLES, // nominal ~62.5 Hz
    NOVELTY_WIN    = 32,      // local mean window for novelty (~0.5 s)
    NOV_RING_FRAMES= 6 * FPS, // ~8 s of novelty history
    TEMP_FFT_SZ    = 2048,    // higher resolution temporal FFT
    BPM_MIN        = 30,
    BPM_MAX        = 300,
    TARGET_BPM_MIN = 64,
    TARGET_BPM_MAX = 127
};
