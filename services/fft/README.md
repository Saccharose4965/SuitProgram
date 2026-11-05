Pending review

# FFT rhythm service

The service turns microphone audio into a held tempo, a phase-aligned output
clock, beat events, audio levels and diagnostic history. Its portable detector
and phase tracker share their decision rules with the PC harness in
`../localFFT`, versioned on the repository's `pc-fft` branch.

## Capture and spectral features

Capture uses 44,100 Hz stereo I2S reads and non-overlapping 1024-sample frames.
The first read selects the stronger microphone slot. Signed PCM receives a
saturating fourfold gain, a Hann window and a radix-2 FFT. The first read anchors
sample time to the monotonic clock; subsequent timestamps advance by sample
count. RX overflow counts and clock revisions protect capture continuity.

Each frame becomes 32 linear bands of log magnitude above 100 Hz. Bands 16–23
receive a square-root-of-two weight. Broad upper-band impulses retain the
previous clean frame. Button suppression applies the same hold to recent and
upcoming frames. This preserves the history's rhythm through short clicks.

Smoothed low, mid, high and overall audio levels feed lighting controls on each
capture frame. Histories and FFT buffers use PSRAM; DMA buffers and task stacks
use internal RAM.

## BPM scores and evidence

Four seconds of cleaned band history supply scores for integer BPM 80–159.
For each BPM, a comb samples interpolated frames at equally spaced beat times
for multiple phase offsets. Average variance across the sampled phases and
bands measures rhythmic mismatch. Lower variance produces a higher score.
Two fitted linear baseline halves remove the broad trend; positive residuals
are normalized to unit sum.

A timestamped score history spans about 4.09 seconds. Observations are weighted
by the elapsed intervals they cover. Its strongest adjacent pair chooses a
candidate, and that pair's stronger member supplies the exact integer BPM.
Candidate quality requires pair score at least 0.05, temporal variance at most
0.014, and winner consistency at least 0.95. A variance change of at least 0.004
per reference interval starts a three-second shock cooldown.

Trusted observations accumulate support at the candidate bin. Support decays
with a six-second half-life; fresh credit is bounded by the observation cadence.
Initial acquisition requires one supported second. A replacement requires one
second of challenger support and held support at most half a second. Adjacent
±1 BPM replacements count each exact bin separately. Larger replacements sum
support over each bin and its neighbours. This lets a persistent neighbouring
candidate earn acceptance while the old candidate's own support decays.

Phase lock raises replacement support to two seconds and lowers the held
release threshold to a quarter second. Hard BPM lock retains the forced tempo.
The accepted tempo continues through missing evidence until Reset or a
supported replacement.

Evidence quality is the minimum readiness of pair score, variance and
consistency, clipped to 1.1. During acquisition and replacement, decision
confidence also includes candidate support, held release and shock cooldown.
For a candidate agreeing with the hold, clean evidence supplies confidence and
phase trust even during transition cooldown. Threshold one means the relevant
requirements are satisfied. These values express decision readiness.

## Phase curve and acquisition

Phase analysis runs every eight audio frames, about 186 ms. The held BPM defines
a period. For each of 128 absolute phase bins, comb teeth sample the four-second
cleaned history at proposed beat positions. The score averages the 32 band
values at each tooth, then averages across up to 64 teeth. Subtracting the
curve's minimum exposes relative peaks.

Opposite bins are summed with equal weight into 64 folded timing families.
The strongest family chooses initial timing; its stronger individual member
chooses the full-cycle occurrence. Prominence compares the family with a runner
more than three family bins away.

Acquisition requires complete score-history coverage, a trusted candidate,
confidence at least one and candidate/held agreement within 0.5 BPM. Confidence
loss pauses correction and clears filtered error while the accepted clock
continues. Returning confidence resumes alignment from current evidence.

## Continuous branch and family replacement

The anchor records the chosen full-cycle beat timing. Each update predicts the
folded family from that anchor and searches a 21-bin neighbourhood. Local
tracking follows gradual phase movement. The numerical side bit identifies the
chosen member in the family's current wrapped coordinates, and is recomputed
from the occurrence nearest the anchor as those coordinates cross 0 or 0.5.

A distant global family can replace the local family after scoring at least
1.20 times the local score for six accumulated seconds. Challenger time decays
when that condition fails. Replacement chooses the full-cycle occurrence
nearest the existing anchor, preserving timing continuity across coordinate
wraps. Changes in relative on-beat/offbeat energy retain the selected branch.
Phase lock holds the current family while allowing local alignment.

A BPM change greater than 0.5 moves active phase state into one previous slot.
It remains available for 45 seconds. A compatible returning BPM restores that
state when local phase evidence lies within 0.10 cycles of its prediction.
Other acquisitions establish a phase state from the current strongest family.

## Fine alignment and emitted beats

The local family's two occurrences are compared with the predicted half-beat
occurrence. Their signed timing error is wrapped over half a period and filtered
with a 0.8-second time constant and a 0.1-bin deadband. Tracking requires
prominence at least 1.05 and positive local evidence.

The anchor moves by 20% of filtered error. Each move is bounded by the remaining
raw error and a 30 ms cap scaled to the update interval. Alignment or a sign
reversal clears filter momentum. The nominal period is 60 divided by held BPM.

Anchor updates become pending output-clock corrections. At the next boundary,
the following cycle length changes by 30% of the requested timing error, capped
at 0.6% of its nominal period. Each completed cycle retains its scheduled end.
Confident updates keep requesting alignment until the clock reaches the target.

Offbeat adds half a base period at output ratio one. Output ratio multiplies
BPM after phase tracking; the service supports ratios one through three. Manual
phase offset adds a base-period fraction to the tracked anchor. These controls
set the output-clock anchor. Controller and viewer timing gestures belong to
[LED Source](../led_source/README.md).

The monotonic real-time clock drives `fft_beat_clock_emit`. It accepts the newest
crossed boundary once, with a half-period spacing guard. Publication updates
cycle phase, queues a beat event and can call the Beat LED renderer. The LED
rendering task, frame submission and wire transmission follow that request.
The selected animation determines how brightness and movement express it.

Reset clears accepted tempo, phase, locks, output controls, clock and pending
beat events while retaining audio and score evidence for the next decision.

## Scheduling and capture continuity

Capture/FFT runs on core 1 at priority 5. BPM and phase use separate priority-2
workers on core 0. BPM requests occur every 88 audio frames, about 2.04 seconds.
Each worker processes one immutable snapshot. Busy requests are skipped, and
long BPM loops yield in bounded work slices.

I2S rate, clock revision and RX overflow observations protect frame continuity.
Clock changes, overflows and failed or empty reads invalidate the audio window
and stale worker results while retaining the accepted tempo and running clock.
Epoch and control-revision checks reject obsolete results. Audio Policy owns
capture availability during playback, recording and Bluetooth use.

## History and diagnostics

A mutex-protected snapshot holds FFT magnitudes, grouped spectrogram peaks,
score means, phase curve, evidence and confidence. Two capture frames form each
spectrogram/history column. The viewer draws the full phase curve below its
folded sum, translating the emitted output-clock boundary to the left border.
[FFT View](../../apps/fft_view/README.md) owns OLED drawing and navigation.

Capture reports warnings for audio discontinuities and errors for failed
operations. Runtime status remains available through the service snapshot for
the viewer and other consumers. Capture processing stays focused on receiving
and analyzing audio.

## Verification

Run `bash services/fft/tests/run_host_tests.sh`. It compiles actual PC and suit
implementations and compares scores, filtering, BPM decisions, phase tracking,
output emissions and both rendered phase bands. Coverage includes sustained
±1 BPM changes, locks, delayed work, confidence recovery, on/offbeat, 2x, and
family replacement after crossing 0 and 0.5 in both directions.

Live acceptance includes a user-confirmed metronome run with the Soft Flash
renderer and pre-gap peaks within approximately eight milliseconds of phase
zero. Capture overflows and physical timing under other workloads remain
hardware acceptance checks.
