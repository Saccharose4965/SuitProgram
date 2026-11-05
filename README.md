# Live PC FFT

This microphone harness uses the suit's rhythm-detection rules. The algorithm
is described in the [FFT service README](../SuitProgram/services/fft/README.md).
Firmware lives on the repository's `main` branch; this live PC source and its
tests live on `pc-fft`. Keeping the two checkouts side by side enables parity
tests against their actual implementations.

## Run

```sh
make
./latestfft
```

Build dependencies are a C11 compiler, Make, pkg-config, ALSA and SDL2 development
packages. Capture uses the default ALSA microphone at 44,100 Hz, with saturating
4x gain, Hann window and 1024-point FFT. The window opens up to 2560×1522, fits
the desktop's usable area and supports resizing. Plot rectangles follow actual
renderer dimensions; the spectrogram texture stores 512 frequency rows.

## Controls

- `R`: clear accepted BPM, phase, locks and output offset/ratio; retain audio,
  score evidence and pending analysis, then seek a new hold.
- `F`: force the strongest smoothed BPM or release force; restart challenger support.
- `Space`: toggle phase-family lock and its independent BPM soft lock.
- `D`: after 700 ms idle, toggle on/offbeat. A second quick press selects 2x;
  further quick presses cycle all three choices.
- `O`: toggle offbeat directly. `1` / `2`: select the output ratio directly.
- `Esc`: quit.

## Live pipeline

The cleaned 32-band history covers four seconds. Integer BPM candidates span
80–159. BPM analysis runs every 88 frames, about 2.04 seconds, and phase every
eight frames, about 186 ms. Score history uses elapsed-time weighting over
about 4.09 seconds. Candidate support, trust gates, adjacent-BPM replacements,
phase-family tracking, continuous branch selection and boundary steering use
the same rules as the embedded service.

The BPM worker spreads eight-candidate batches across 75% of its update interval.
Capture, controls and rendering run on each audio frame. Phase analysis uses
its own copy of the latest history in the capture/UI thread. PC spectral math
uses doubles; the embedded implementation uses floats for spectral/BPM work
and separate RTOS workers. These execution choices affect result-arrival time
and precision near decision thresholds.

After microphone warm-up and UI/worker setup, queued startup audio is discarded
and capture is prepared afresh. The first successful read anchors sample time
to monotonic time; subsequent frames advance by sample count. Output emission
and offset/ratio controls follow monotonic real time. The newest crossed beat
boundary triggers once with a half-period spacing guard. A full-brightness frame
marks that event, followed by exponential decay.

An ALSA overrun rebuilds contiguous evidence and invalidates stale worker
results while retaining the held BPM and output clock. Reset retains captured
evidence while clearing accepted state. The held tempo persists through missing
evidence until Reset or a supported replacement.

## Display

- Upper left: scrolling grayscale spectrogram.
- Upper right: 80 mean BPM scores with one bar per integer BPM. The white backing
  marks the exact held BPM.
- Lower left: aligned evidence/readiness history. Cyan is evidence quality;
  white/green is decision confidence, green at threshold one.
- Lower right: folded phase above the full cycle. Their left panel edge is
  phase zero of the emitted clock. The side coordinate follows the selected
  full-cycle timing branch through folded-coordinate wraps.

The spectrogram and confidence panel advance from the same audio-frame history.
Each panel scales its data to available pixels. The phase graph translates
absolute evidence using the output-clock boundary and the held base BPM.

The title reports candidate/held BPM, locks, confidence, evidence, phase error,
family prominence and output ratio, plus worker timing. `rawpeak` gives the
strongest curve position relative to the flash clock; `rawtarget` gives signed
selected-target error; `capture` gives wall time minus sample-progress time.
These are software timing observations. Microphone, OS buffering, display
refresh and audio playback contribute their own physical latencies.

## Verification and source

```sh
make test
cd ../SuitProgram
bash services/fft/tests/run_host_tests.sh
```

Desktop checks cover readiness, history timing, phase tracking, controls,
worker generations/shutdown and rendered pulse alignment
at several window sizes. The firmware suite compares both actual implementations,
including phase replacement across coordinate wraps, emitted boundaries and
both phase renderers. Live music and physical timing are checked on hardware.

`LatestFFT.c` owns ALSA capture, scheduling, controls and SDL drawing.
`include/bpm_core.h` contains portable spectral/BPM math;
`include/phase_tracker.h` contains phase scoring and tracking. `tests/` contains
headless checks. The local workspace's `benchmark/`, `archive/` and audio assets
support offline experiments separately from this live-source checkout.
