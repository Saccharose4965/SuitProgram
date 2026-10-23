> Archived reference. Describes an earlier revision; current explanations live beside the code.

# Message arrival indicators and recording envelope

This is a file-level review of the five files named below. It is not a complete
review of messaging, microphone hardware, telemetry or the message UI. Those
remain independent checklist items. Behaviour described here is implemented;
hardware responsiveness and receiver lifecycle still need suit testing.

## `main/message_notifications.h`

This interface exposes three operations using an unsigned 32-bit arrival
sequence, not a message ID or timestamp:

- `message_arrival_snapshot()` returns the current arrival counter.
- `message_unseen_count()` returns arrivals since the acknowledged snapshot,
  capped at 99 for compact menu display.
- `message_mark_seen(snapshot)` acknowledges exactly that earlier snapshot.

The caller must pass a snapshot obtained from this module. Only the UI should
acknowledge arrivals, in nondecreasing snapshot order. Arbitrary future or old
values are not validated and can yield misleading unsigned differences. This
is a single-consumer arrival indicator, not a general unread-message database.

## `main/message_notifications.c`

Built into the shell through `main/CMakeLists.txt`. Two statically allocated
atomic counters, `s_arrivals` and `s_seen`, start at zero each boot. No allocation,
file access, task creation, audio ownership or radio calls occur in this module.

`telemetry_file_received(path)` is the strong implementation of telemetry's
optional weak application hook. Direct suit reception also calls it. Both
receiver paths invoke it after successfully renaming a completed temporary
file. It accepts only paths beginning `/sdcard/messages/inbox/`, with a filename
of at least five characters, lowercase `.wav` suffix, and no further slash.
Null paths, outbox files, temporary extensions and nested paths are ignored.
It does **not** parse WAV headers, inspect file existence, identify senders or
deduplicate paths. Each accepted callback increments the sequence once.
Repeated delivery callbacks for the same filename would count again; the direct
receiver, not this module, suppresses notifications for repeated final ACKs.

Receiver tasks may increment while the shell reads. C11 atomics avoid torn
cross-task counter access; default sequential consistency is used. There is no
critical section spanning an SD scan. The UI takes an arrival snapshot before
scanning Inbox, then acknowledges only that snapshot after a successful,
non-truncated scan. A concurrent arrival therefore remains indicated. Menu
drawing reads counters only: it does not poll the SD card.

`message_unseen_count()` loads seen first, then arrivals, subtracts using unsigned
arithmetic and caps at 99. Under the single UI consumer contract this tolerates
ordinary counter wrap; losing a full 2^32 arrivals without acknowledgement is
not distinguishable. `message_mark_seen()` is a plain atomic store, not a CAS or
monotonic-max operation: multiple independent readers/acknowledgers would require
a different contract. These constraints matter if another app starts using it.

Reasons for this design: notification should be driven by completed delivery,
not expensive directory polling; acknowledging a snapshot avoids clearing an
arrival that the scan may not have seen. Keeping state in RAM avoids additional
SD writes but deliberately sacrifices persistence. “Seen” means Inbox was opened,
not that a message was listened to. Old files remain on SD after reboot but do
not automatically restore a badge. Receivers currently initialise on first
opening Messages, not at boot. No always-on service was added by this feature.

## `main/tests/test_message_notifications.py`

`NotificationsTest.test_arrivals_and_snapshot` creates a temporary C harness,
compiles the actual notification source with GCC, strict warnings, AddressSanitizer
and UndefinedBehaviorSanitizer, and executes it. Temporary files are removed by
Python's temporary-directory context manager; compilation/run failures fail the
test. It requires Python and a host GCC with sanitizer support, not ESP-IDF.

The harness verifies initial zero, null/wrong-directory/temporary/traversal-like
path rejection, valid arrivals, preservation of an arrival after an earlier scan
snapshot, display saturation at 99, and acknowledgement back to zero. It invokes
the hook directly: this does not prove receiver integration, actual concurrent
task scheduling, filesystem durability, target atomic implementation or OLED
appearance. Direct-transfer integration has a separate test; that file is not
marked reviewed by this document.

## `components/microphone/include/recording_envelope.h`

A dependency-free, header-only accumulator over signed PCM16 samples. It is
included by the recorder and host tests. `recording_envelope_t` owns a 96-byte
column ring plus unsigned counters: `count` is valid columns, `next` is the next
write slot, `frames`/`peak` describe the unfinished window, and `window` is the
required sample count. Zero-initialise the state and set a positive window
before feeding it. The recorder uses sample_rate/20: 800 samples at 16 kHz,
giving 50 ms per column and 4.8 seconds at full capacity.

`recording_envelope_feed(e, pcm, n)` computes absolute magnitude after integer
promotion, so -32768 is representable. It retains the maximum until `window`
samples arrive, then calls `recording_envelope_push`. Chunk boundaries do not
affect window placement. Cost is O(n), with no allocation, floating-point math,
FFT, I/O or clock reads. It does not modify PCM or apply audio gain.

`recording_envelope_push(e)` stores floor(peak/128), saturated at 255, advances
the ring modulo 96, saturates the valid count, and clears the pending frame/peak
accumulators. Thus positive and negative full-scale samples both display 255;
magnitudes below 128 quantise to zero. The recorder explicitly pushes a final
partial window on stop, but only when `frames` is nonzero. Calling push on an
empty window would append a silent column.

`recording_envelope_copy(e, out, capacity)` copies the newest min(count, capacity)
columns in oldest-to-newest order and returns that number. A narrow viewport
gets the newest tail, not a resampled/compressed whole recording. Capacity zero
writes nothing. Nonzero capacities require a valid destination. Functions do
not validate pointers, state corruption or window zero, and are not themselves
thread-safe. The recorder owns the mutable accumulator and publishes snapshots
under its own lock; the UI reads only snapshots through `mic_rec_envelope`.

Why an envelope: it preserves brief transients in a tiny display and requires
only integer comparisons on samples already captured. It is not an oscilloscope
trace, RMS/loudness estimate, clipping detector or playback waveform. The UI's
separate saturating 4x display gain affects pixels only. Recording stops leave
the snapshot frozen; starting a fresh recording clears it. Slow/missing capture
means fewer columns: the horizontal axis measures captured samples, not inserted
wall-clock silence. There is no extra microphone consumer or draw-time SD read.

## `main/tests/test_recording_envelope.py`

`EnvelopeTest.test_pcm_windows` compiles the production header in a temporary
GCC C harness with strict warnings and ASan/UBSan. It compares whole-buffer
feeding against sample-at-a-time feeding, tests -32768 and half-scale peaks,
checks a one-column tail read, wraps 100 values through the 96-column ring,
flushes a final partial silent window, repeats a frozen snapshot and reads with
zero capacity. It uses window=800 for normal 16 kHz timing and window=1 to
exercise ring wrap compactly. It does not exercise I2S, SD writes, lock timing,
actual recorder stop/start failure paths, microphone gain or display legibility.

Run both reviewed test files with `python3 <path>`. Run the message UI and direct
delivery tests too when changing the surrounding integration. On hardware,
check quiet speech, transients, stop/freeze, starting another recording, and
inbox delivery while viewing menus; retain observed limitations in these docs.
