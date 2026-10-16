# ESP32 microphone detector

Bluetooth coexistence: entering the Bluetooth app blocks background FFT
requests and synchronously stops capture/BPM/phase tasks before BT initialization.
The block remains while the app is open or BT is starting, scanning, connecting,
connected or streaming. Leaving the app permits pending FFT requests to resume
once BT is idle/disconnected. The FFT viewer also checks this gate before start.

Bluetooth entry also releases the idle I2S TX/RX channels after FFT has stopped
(unless recording or playback is active). Stopping the FFT tasks alone retained
roughly 32 KiB of internal DMA buffers, starving the BT WAV feeder's 4 KiB stack.
The bus configuration is retained and restored by the next local-audio rate or
enable request; microphone recording refreshes its cached RX handle. The entry
log reports internal heap before/after release. Task stack sizes and Bluetooth
heap safety thresholds are unchanged. Hardware playback/reconnect testing remains
required; a successful build does not establish runtime memory headroom.

## Latest parity pass (2026-10-12)

Phase evidence now runs on a separate priority-2 worker on core 0 with one
immutable snapshot outstanding. Capture owns tracker decisions and the beat
clock. Epoch and reset-revision checks reject obsolete phase results. BPM
results preserve the live phase graph only while the held tempo is unchanged.
The graph origin uses the actual beat boundary in nominal-BPM coordinates.
The narrow late-flash rejection window has been removed; delayed processing
emits the latest boundary once rather than discarding it.

The first hold update uses the PC reference interval (22 frames), independently
of the 88-frame BPM sweep cadence. Reset clears accepted state immediately and
preserves audio/score history. On the full-screen FFT viewer, button C performs
this reset (PC R); no footer widget is added. Button D retains its reset shortcut.

Animation selection is a two-column alphabetical catalogue spanning both
renderers. A/B move backwards/forwards, C selects, D moves down a row (on Speed,
C decreases and D increases). Distinct effects formerly named energy, breathe
and random have explicit names. The selected effect is marked with an asterisk
when not focused. Rendering/source selection remains in the existing backends.

Run `components/fft/tests/run_host_tests.sh`: in addition to the spectral tests,
a 1,200-step replay compares phase decisions and beat-clock boundaries directly
against the compiled PC source, including drift, competing peaks, lock/unlock,
uncertain evidence, loss and tempo changes. This verifies those scenarios,
not all possible inputs or hardware scheduling. Flash/runtime validation is
still required after the worker change. Older dated sections below describe
earlier implementations and are superseded by this note.

This is the embedded port of `../localFFT/LatestFFT.c`, not another parameter
search. The reference file at port time had SHA-256
`975d540db616d5c041aa1970c174382731d866b26c63363b6632a6c80461b515`.
That sibling workbench is not tracked in this firmware repository. Its
`benchmark/EXPERIMENTS.md` records the research; do not restart sweeps without
the user's request. On-board performance and sound tests remain outstanding.

## Responsibilities

- `components/fft/fft_detector.c` and `components/fft/include/fft_detector.h`:
  portable band filtering, variance comb, normalization, candidate history,
  held-BPM decisions and loss timer.
- `components/fft/fft_phase.c` and `components/fft/include/fft_phase.h`:
  folded phase-family tracking, fractional tempo correction and the smoothly
  steered beat clock.
- `components/fft/fft.c`: I2S capture, FFT, immutable analysis snapshots, task
  scheduling, controls, beat clock and existing LED/audio-level APIs.
- `components/fft/fft_display.c`: host-testable monochrome adaptation of the
  localFFT dashboard, with the same signal meanings and timeline.
- `components/fft/fft_render.c`: PSRAM display state and mutex adapter. The shell
  draws on demand; there is no separate render task or framebuffer queue.
- `main/app_fft_sync.c`: physical-button controls. No new shell application.

All FFT headers live in `components/fft/include/`. Other components use `fft.h`;
`fft_detector.h` and `fft_render.h` are internal helper interfaces, not new
application entry points. The portable host test includes `fft_detector.h`.

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
- Drop the clock after 40 accumulated seconds while the held pair remains below
  60% of the current winning pair. Hard force overrides the timeout. No volume
  gate is used.

`conf` is decision readiness: the minimum of quality, support, and (when needed)
held-release gates, zeroed during shock cooldown. `ev` is just the minimum of
pair-strength, stability and consistency readiness, before cooldown/support
gates. Threshold is 1 for both; neither is a probability of correct BPM.
The public legacy confidence field is capped at 1; `decision_confidence` and
evidence retain the full display range up to 1.1. Sub-threshold quality does
not zero confidence; only shock cooldown does, matching localFFT.

## Phase and controls

Phase runs independently every eight audio frames (~5.4 Hz). Its 128-bin curve
folds opposite peaks into 64 families. Tracking stays near the predicted family;
a distant family must remain at least 20% stronger for six seconds before a
switch. The chosen side persists, and a recently lost candidate may be restored
for 45 seconds. Filtered phase error supplies bounded anchor and fractional-BPM
corrections. The beat clock applies them only at the next boundary and gently
steers the following cycle, so the active cycle never snaps. Phase lock blocks
family switches but keeps fine drift correction active and soft-locks BPM.

On **FFT Sync**, A/B select, C applies, D resets detector history and both locks.
Selected controls: phase lock, BPM force, ratio 1x/2x/3x, offbeat, output pause,
phase trim earlier/later by 2%. Reset preserves ratio, offbeat, trim and pause.
BPM force takes the strongest smoothed candidate, like PC F; it does not have
to equal the previous hold. Phase locking is ignored until there is a clock.
Only PRESS is handled; LONG_PRESS does not double-toggle. Shell shortcuts remain.

The implementation now follows the PC phase tracker rather than the former
embedded strongest-peak snapper.

## Scheduling and integration safeguards

Capture/FFT: core 1, priority 4. Analysis: core 0, priority 2. The expensive
80-BPM sweep is requested every 88 audio frames (~2.04 s), **not at 30 FPS**.
Phase is refreshed separately every eight frames. Between candidates, the
worker blocks for one RTOS tick after a ~40 ms work
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
Large history, snapshot, FFT, I2S staging and display buffers are in PSRAM. Capture/analysis stacks
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
  -fsanitize=address,undefined -I components/fft/include \
  components/fft/tests/test_detector.c components/fft/fft_detector.c \
  -lm -o /tmp/suit-test-detector
/tmp/suit-test-detector
(cd ../ShowEditor && python3 -m unittest discover -s show_editor/tests -v)
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
ESP32 target, 240 MHz CPU and PSRAM enabled. An isolated build/config lives in
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
2. Pause/breakdown for 10 seconds: keep the clock. No rhythm for >40 seconds:
   eventually release; force lock must keep it. Test speech/clicks too: the loss
   criterion is still experimental on real non-music input.
3. Another song: reject brief contestants, then change on persistent evidence.
   Use D if you want immediate permission to forget the previous song.
4. LED audio/beat modes, manual BPM, synchronized shows, radio master/slave,
   speaker playback, message recording, and repeated FFT stop/start.
5. Send 20–30 seconds of `fft_vis` logs with LEDs/radio enabled. If analysis is
   late but capture stays healthy, tune batching/cadence first; do not change
   the sample rate/bands or launch a fresh detector search to fix scheduling.


## 2026-09-22: board follow-up and localFFT display completion

The source hash above still matches the actual sibling `LatestFFT.c`. The
previous port is the starting point, not an abandoned alternative tracker.
No parameter sweeps or song benchmarks were rerun. Existing conservative
scheduling differences and the absolute-coordinate phase correction described
above remain deliberate.

The first on-board log showed only 2,451 free internal bytes and
`led_modes_start failed: ESP_ERR_NO_MEM`; it also showed analysis wall times of
1.8–2.6 seconds. The 8 KB I2S staging allocation now explicitly uses PSRAM.
The new display model also lives in PSRAM and is drawn by the shell, removing
the old 6 KB renderer task stack, its large queue and internal display history.
Task stacks and actual I2S/RMT DMA buffers stay internal. No global sdkconfig
allocation or CPU-frequency settings were changed by this follow-up.

The existing 2 ms analysis slice slept for a whole 10 ms tick at 100 Hz,
frequently after each BPM candidate. A 12 ms work slice now amortizes that
sleep; capture still preempts the lower-priority worker. This changes only
scheduling, not samples, phase probes, thresholds or score mathematics. New
runtime timing and heap measurements still require flashing and board testing.

### OLED views and controls

The FFT viewer opens on a plain, full-screen spectrum. A/B cycle through
spectrum, spectrogram, BPM scores, confidence and phase. Every graph uses the
entire 128x64 framebuffer: no dashboard, text labels, legends, dither patterns,
stipple, antialiasing or decorative markers. C opens the separate FFT Sync
controls; D resets the tracker, both locks and visual histories. FFT Sync keeps
its functional control labels and numeric status.

- The spectrogram covers 0–22.05 kHz, peak-pooling frequency bins into 32 rows.
  Its binary display threshold is max(0.01, current peak * 0.01), retaining a
  40 dB amplitude range above an absolute floor. This is display-only; detector
  magnitudes and thresholds are unchanged. Moderate-input visibility needs a
  final microphone/OLED check on the suit.
- Spectrogram and readiness share 128 columns, one column per two audio frames
  (~21.5 columns/second, ~5.94 seconds visible). The spectrogram retains either
  frame's peaks; confidence uses the latest sample in the group. No horizontal
  averaging, antialiasing or dithering. Drawing never advances history. Explicit
  reset clears it; capture gaps retain the existing columns.
- Confidence and evidence are two plain, one-pixel solid traces, on a fixed
  0–1.1 vertical scale. Values are held between detector results.
- BPM scores and the instantaneous spectrum use solid vertical bars, scaled
  to their current peaks without additional display-scale smoothing.
- Phase energy uses solid vertical bars and min/max scaling in actual output
  clock coordinates, including the output ratio.

BPM wrapping was discussed but remains unspecified; these presentation changes
do not introduce octave averaging or modulo-128 BPM mapping.

### Repeatable checks

```sh
components/fft/tests/run_host_tests.sh
. ../esp-idf/export.sh
idf.py build
```

The runner uses address/undefined-behavior sanitizers and strict warnings.
Alongside the original detector regressions it checks full-band spectrograms,
aligned histories, rollover, reset, draw bounds and clock-coordinate mapping.
When ALSA/SDL development packages and the sibling reference are available,
`test_localfft_reference.c` compiles the *actual desktop source*, renaming its
main without starting audio or SDL. It compares 480 raw and normalized scores,
768 phase bins, impulse filtering/treble weights, and graded confidence below
threshold. Current raw-score maximum absolute error is about 1.01e-6, with
identical winning bins. These are synthetic implementation checks, not a
claim of musical accuracy or measured ESP32 performance.

The normal project build succeeds. Next board check: flash, activate
`sync:fill`, then open FFT viewer with a known song. Check all views and
locks/ratio/offbeat/reset, and collect 20–30 seconds of heap/capture/worker logs.
The updated firmware has not yet been verified on the suit.

## 2026-10-16: pipeline parity and capture recovery

- Capture discontinuities discard partial audio, band-filter and score history,
  but preserve the accepted BPM/phase clock. They no longer bypass the 40-second
  evidence-based loss rule by resetting the detector. Missing capture time is
  not fabricated into evidence; an indefinitely unavailable microphone is not
  classified as 40 seconds of non-rhythmic audio. Button suppression is unchanged.
- New accepted tempos reset accumulated loss, matching PC `newLock` handling.
- Clean evidence agreeing with the held BPM remains phase-trusted during shock
  cooldown, matching the PC. Cooldown still prevents transitions/support credit.
- A busy BPM worker leaves its deadline pending, retrying on the next audio
  frame rather than waiting another 88 frames. Its core-0 worker yields every
  40 ms. No FFT size, comb resolution or detector thresholds were reduced.
- OLED rendering copies a snapshot under the capture mutex and draws outside
  it. Phase source is compiled with the same -O2 optimization as the detector.
- Diagnostics now report result cadence, audio-result age and overdue frames
  (the old `skipped_jobs` status field now counts these frames).
- Shell LED-source service requests FFT regardless of which viewer is open;
  closing the viewer only hides its display. Shared-audio owners still suspend
  FFT intentionally, and the audio gate retries when they release ownership.

Host tests compare 900 updates against actual desktop BPM history/hold/loss
logic (transients, sustained changes, soft lock, delayed jobs, loss/reacquisition)
and 1,200 phase/clock updates. Capture-gap retention and binary grouped display
are also tested. They establish implementation parity on identical synthetic
inputs, not microphone equivalence or achieved hardware throughput.

Board check: use a steady track, press several buttons, leave/reopen the FFT
viewer, then change tracks. Capture 20–30 seconds of `fft_vis` lines including
`cadence`, `age`, `overdue-frames`, and any `Capture gap` messages. Target BPM
cadence is ~2.04 s (the current PC default); longer cadence identifies remaining
hardware scheduling/compute limits rather than a display refresh setting.
