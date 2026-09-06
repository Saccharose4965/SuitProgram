# ESP32 microphone detector

This is the embedded port of `../localFFT/LatestFFT.c`, not another parameter
search. The reference file at port time had SHA-256
`975d540db616d5c041aa1970c174382731d866b26c63363b6632a6c80461b515`.
That sibling workbench is not tracked in this firmware repository. Its
`benchmark/EXPERIMENTS.md` records the research; do not restart sweeps without
the user's request. On-board performance and sound tests remain outstanding.

## Responsibilities

- `components/fft/fft_detector.c/.h`: portable band filtering, variance comb,
  normalization, candidate history, held-BPM decisions, loss timer and phase.
- `components/fft/fft.c`: I2S capture, FFT, immutable analysis snapshots, task
  scheduling, controls, beat clock and existing LED/audio-level APIs.
- `components/fft/fft_render.c`: existing OLED views; flux is diagnostic only,
  **not** the input to the BPM detector.
- `main/app_fft_sync.c`: physical-button controls. No new shell application.

LED effects, show playback, show format, radio protocol, manual BPM and pinout
are unchanged. Audio-reactive levels keep their original frequency regions and
envelope time constants, retimed for the new capture hop. Frequency resolution
and physical-mic response still need a visual check on the suit.

## Frozen detector settings

Capture: 44,100 Hz, Hann-windowed 1024-point FFT, 1024-sample hop (~43.07 frames/s).
The first microphone read selects the stronger L/R slot. Keep the existing 4x
PCM gain, now saturating instead of wrapping signed samples on loud impulses.

Features: mean `log(1 + 10 * magnitude)` in 32 linear bands above 100 Hz,
excluding Nyquist. Bands 16–23 receive a sqrt(2) amplitude weight. A broad
positive impulse (upper-half mean rise >=1, at least eight bands rising >=0.75)
holds the previous clean frame. Button suppression also holds/replaces recent
frames; it does not inject zero-valued holes into the band history.

Score: four seconds (173 frames), 80–159 integer BPM, interpolated comb teeth,
eight phase probes per frame (up to 256 per BPM). Average sample variance across
phases/bands, invert, subtract the two-half linear baseline, clamp positive and
normalize to unit sum. Lower original variance is better. No amplitude-scaled
score, onset hybrid, additional harmonic search or learned model was added.

Hold policy:

- Average up to eight score spectra over ~4.09 seconds. Pick the strongest
  adjacent-bin sum, then its stronger bin; no wrap from 159 back to 80.
- Evidence requires pair score >=0.05, summed temporal variance <=0.014, and
  >=95% of recent pair winners within two bins of the average winner.
- An absolute variance change >=0.004 imposes a three-second cooldown.
- Trusted observations build independent support in each candidate bin.
  Support decays with a six-second half-life; decisions sum a ±1-bin neighborhood.
- Acquire at one supported second. Switch only with a trusted challenger,
  challenger support >=1 second and held support <=0.5 seconds.
- Adjacent slides remain blocked except **158 -> 159**, matching the literal
  `BPM_MAX` rule in the PC source. This is not general fractional/±1 tracking.
- Phase lock independently raises challenger support to 2 seconds and lowers
  held release to 0.25 seconds. Hard BPM force prevents automatic changes.
- Drop the clock after 30 accumulated seconds with neither a trusted candidate
  nor a visible held pair (score >=0.03). Good evidence drains that timer at 4x
  elapsed time. Hard force overrides the timeout. No volume gate is used.

`conf` is decision readiness: the minimum of quality, support, and (when needed)
held-release gates, zeroed during shock cooldown. `ev` is just the minimum of
pair-strength, stability and consistency readiness, before cooldown/support
gates. Threshold is 1 for both; neither is a probability of correct BPM.
The public legacy confidence field is capped at 1; evidence can reach 1.1.

## Phase and controls

Phase runs automatically. At the held BPM, 128 offsets measure recurring band
energy; a neighboring peak or one >=20% stronger may replace the held phase.
Stable score history gates clock re-anchoring. Phase lock freezes the accepted
phase; a genuine held-BPM change reinitializes it. Offbeat shifts the output by
half an output pulse; ratio multiplies pulse rate without changing detected BPM.

On **FFT Sync**, A/B select, C applies, D resets detector history and both locks.
Selected controls: phase lock, BPM force, ratio 1x/2x/3x, offbeat, output pause,
phase trim earlier/later by 2%. Reset preserves ratio, offbeat, trim and pause.
BPM force takes the strongest smoothed candidate, like PC F; it does not have
to equal the previous hold. Phase locking is ignored until there is a clock.
Only PRESS is handled; LONG_PRESS does not double-toggle. Shell shortcuts remain.

Phase is not a solved precision clock. Fractional BPM/drift estimation and the
distinction between true timing drift and a beat/offbeat instrumentation change
remain separate research. The port fixes the old phase comparison's mixed
window-local/absolute coordinates, but adds no untested drift controller.

## Scheduling and integration safeguards

Capture/FFT: core 1, priority 4. Analysis: core 1, priority 2. The expensive
80-BPM sweep is requested every 22 audio frames (~511 ms), **not at 30 FPS**.
Between candidates, the worker blocks for one RTOS tick after a ~2 ms work
slice. Capture preempts analysis; LEDs/radio/shell retain their existing tasks.
These budgets are starting points, not measured hardware guarantees.

There is exactly one outstanding immutable snapshot. While busy, requests are
skipped instead of queued. Support credit is capped at one normal analysis hop
per result; slow computation cannot manufacture sustained evidence. History
expires by sample time; stable phase history uses elapsed coverage so missing
alternate jobs does not disable phase updates forever. Support/cooldown/loss
timers use real elapsed sample time. Unlike PC, acquisition needs two distinct
recent observations. These are deliberate conservative scheduling differences.

The float comb shares interpolation positions across all bands and hoists
variance reciprocals outside the band loop. Subtracting each band's fixed first
sample prevents float cancellation on constant, high-offset input. Only the
absolute beat/time calculations use doubles. DSP files use `-O2` even with the
rest of the firmware in debug mode; no fast-math flags or global config changes.
Large history, snapshot and FFT buffers are in PSRAM. Capture/analysis stacks
are 8192/6144 internal bytes. Queue messages are small; no allocation per frame.

Reset/capture gaps increment an epoch; stale results cannot restore an old
clock. Control revisions reject results made with outdated lock settings.
Shutdown is cooperative, using bounded I2S reads and per-BPM cancellation.

Playback and recording share the I2S clock. The audio component exposes rate,
clock revision, RX overflow count and local-playback activity (including boot
tone). Capture discards windows spanning re-clock, failed reads or overflow.
It pauses for recording/foreign sample rates, and restores 44.1 kHz after the
known audio users finish. Starting FFT must not interrupt active playback.
Audio-reactive output is cleared during unavailable capture. This is a targeted
guard, **not** the general audio arbiter/lease system still on the roadmap;
concurrent service start/stop combinations need hardware regression testing.

## Verification and next board test

From `SuitProgram`, host tests (no songs, microphone, SDL, or ESP-IDF needed):

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic \
  -fsanitize=address,undefined -I components/fft \
  components/fft/tests/test_detector.c components/fft/fft_detector.c \
  -lm -o /tmp/suit-test-detector
/tmp/suit-test-detector
python3 -m unittest discover -s tools/show_editor/tests -v
```

Tests cover double-reference comb parity, normalization/constant input, impulse
suppression, acquisition, transient rejection, sustained switching, independent
phase soft-lock, force/release, boundary slides, interruption recovery, loss
reset, skipped jobs and absolute phase across moving windows. A synthetic 159
BPM uniform pulse can prefer near-half-time 80 in both implementations; that
existing octave ambiguity is explicitly tested, not claimed fixed.

Port check: 480 raw-score comparisons across six synthetic tempos differed by
at most 1.44e-6 from the double reference, with identical winning bins after
normalization. Address/undefined-behavior sanitizers and both existing show
export tests passed. These are implementation regressions, not a new musical
accuracy score or proof of microphone performance.

The port was fully built with the installed ESP-IDF 6.1 development checkout,
ESP32 target, 160 MHz CPU and PSRAM enabled. An isolated build/config lives in
`build/fft-port/`; the user's root sdkconfig and previous build were preserved.
No board was connected, so nothing was flashed. To build/flash normally:

```sh
. ../esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Use the board's actual serial port. To flash the already-verified isolated
configuration instead, add `-B build/fft-port` to the idf.py command.

Every five seconds, `fft_vis` logs capture processing wall-time percentage,
worker wall time (including yields), skipped requests, discontinuity errors,
held BPM, evidence, loss time, free internal heap and minimum stack headroom.
One startup/re-clock error may occur while discarding old DMA buffers; growing
errors during steady capture are not acceptable. The build-size report is
static placement, not runtime free-heap proof.

Please test:

1. Known steady song: time to hold, phase alignment, lock/offbeat/ratio/reset.
2. Pause/breakdown for 10 seconds: keep the clock. No rhythm for >30 seconds:
   eventually release; force lock must keep it. Test speech/clicks too: the loss
   criterion is still experimental on real non-music input.
3. Another song: reject brief contestants, then change on persistent evidence.
   Use D if you want immediate permission to forget the previous song.
4. LED audio/beat modes, manual BPM, synchronized shows, radio master/slave,
   speaker playback, message recording, and repeated FFT stop/start.
5. Send 20–30 seconds of `fft_vis` logs with LEDs/radio enabled. If analysis is
   late but capture stays healthy, tune batching/cadence first; do not change
   the sample rate/bands or launch a fresh detector search to fix scheduling.
