Pending review

# FFT rhythm service

This background service converts microphone PCM into audio levels, a stable BPM
hold, phase-corrected beat events and diagnostic history. It is the embedded
port of `../localFFT/LatestFFT.c`. The portable detector and phase tests compile
both implementations against identical synthetic inputs to guard that parity.

## Capture and spectral features

Capture runs at 44,100 Hz with non-overlapping 1024-sample frames. The first
read selects the stronger microphone slot. Signed PCM is amplified fourfold
with saturation, multiplied by a Hann window and transformed by an in-place
radix-2 FFT.

The detector reduces log magnitudes into 32 linear bands above 100 Hz. Bands
16 through 23 receive a square-root-of-two weight. A broad upper-band impulse
holds the previous clean frame; button suppression uses the same hold principle
for recent and upcoming frames. This keeps button clicks from becoming rhythmic
evidence without inserting artificial silence into history.

Separate smoothed low, mid, high and overall levels feed LED controls. Their
publication is independent of the BPM analysis cadence.

## BPM score and hold decision

The score uses four seconds of band history and evaluates integer BPM 80–159.
For each candidate, interpolated comb teeth sample multiple phases and measure
the average variance across phases and bands. Lower variance becomes a larger
score. A two-half linear baseline is removed, positive results are retained and
the spectrum is normalized to unit sum.

Up to eight recent spectra are averaged. The strongest adjacent pair chooses a
candidate, with its stronger member becoming the exact bin. Evidence combines
pair strength, low temporal variance and agreement of recent winners. A sudden
variance change starts a three-second shock cooldown.

Trusted observations add elapsed-time support to candidate bins; support has a
six-second half-life. Initial acquisition needs one supported second. Switching
requires a trusted challenger with one second of support and held support below
half a second. Phase lock raises challenger support to two seconds and lowers
the held-release threshold to a quarter second. Hard lock prevents automatic
changes. Adjacent slides are blocked except 158 to 159, the top of the search
range.

Loss accumulates while the held pair is below 60 percent of the current winner.
After 40 seconds the clock is released. Hard lock suspends this release. The
criterion is rhythmic evidence rather than volume.

The public evidence value expresses score quality before support/cooldown gates.
Decision confidence is the minimum readiness across quality, candidate support
and held release, and becomes zero during shock cooldown. Threshold one means
ready; these quantities are decision metrics rather than correctness
probabilities.

## Phase family and beat clock

Phase analysis runs every eight audio frames. Its 128-bin curve folds opposite
peaks into 64 families with equal weight. The tracker follows the predicted
family; a distant family must remain at least 20 percent stronger for six
seconds before replacement. The chosen unfolded side persists while the BPM
hold survives. One recently lost candidate can be restored for 45 seconds.

Filtered phase error supplies a bounded anchor correction and a small
fractional-BPM correction. The beat clock schedules corrections at the next
boundary and steers the following cycle, preserving an already active cycle.
Phase lock freezes family selection while drift correction continues and also
activates the BPM soft-lock thresholds.

Ratio and offbeat controls transform emitted boundaries after tracking. Manual
phase trim shifts that output trigger. Beat events are queued for consumers and
can independently be exported to the LED beat service.

## Tasks and stale-result protection

Capture and FFT run on core 1 at priority 4. BPM analysis and phase analysis use
separate priority-2 workers on core 0. BPM work is requested every 88 audio
frames; phase work every eight. Each worker accepts one immutable snapshot.
Busy requests are skipped instead of queued, and long BPM loops yield after a
bounded work slice.

Epochs invalidate results spanning a DMA gap or reset. Control revisions reject
work computed with obsolete lock settings. Timers use audio timestamps, and
support credit is capped at one normal analysis interval, so delayed computation
cannot manufacture sustained evidence.

The capture loop observes I2S rate, clock revision and RX overflow count. A
window crossing a clock change, overflow or failed read is discarded while the
accepted tempo remains available. Recording and playback resource policy stops
or suspends capture and later restores the 44.1 kHz bus.

## Diagnostic history

The service stores raw FFT magnitudes, grouped spectrogram peaks, detector
scores, phase curve, evidence and confidence in a mutex-protected snapshot.
Two audio frames form one timeline column. Binary frequency peaks use a 40 dB
relative range with an absolute floor. This history advances on the audio clock
whether the viewer is open or closed; it contains no OLED drawing code.

Large histories and FFT buffers live in PSRAM. I2S DMA and task stacks remain
internal. Runtime logs report capture load, analysis wall time, overdue work,
errors, current hold, evidence, loss, heap and stack headroom.

Run `services/fft/tests/run_host_tests.sh` for detector, phase, history and
desktop-reference parity. Hardware acceptance should cover known steady music,
track changes, breaks longer than 40 seconds, clicks/speech, locks and ratios,
then repeated playback/recording/Bluetooth transitions while watching capture
errors and result cadence.
