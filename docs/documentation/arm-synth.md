# Arm Synth: gesture control, oscillator and output routing

File-level review of `main/app_synth.c`, `main/synth_voice.h`,
`main/shell_audio.c`, `main/shell_audio.h`, `main/tests/test_synth.py` and
`main/tests/test_synth_bt.py`. Audio/Bluetooth adapter details below explain the
interfaces these files use; they do **not** constitute a review of those entire
components. See [orientation contracts](orientation-and-paint.md) for the shared
coordinate conversion. No implementation changes accompany this review.

## `main/synth_voice.h`

The header-only oscillator is compiled into the synth app and exercised directly
by a host test. `SYNTH_RATE` is 24000 Hz for local output; Bluetooth overrides
the instance rate with the negotiated 44100 or 48000 Hz. `synth_voice_t` owns
phase in cycles, smoothed frequency in Hz, smoothed linear gain, sample rate and
a 257-entry float sine table. There is no heap allocation or external state.

`synth_voice_init` zeroes the whole instance, sets 220 Hz and 24 kHz, and fills
the table with sin(2*pi*i/256), including the extra endpoint for interpolation.
Gain and phase start at zero, so the app opens silent. The table costs 1028 bytes
per instance; the app keeps its one instance in static storage, not task stack.

For each output sample, `synth_voice_render` moves frequency and gain toward
their requested targets with coefficient `0.002*24000/rate`. It then linearly
interpolates table entries at phase*256, multiplies by gain*12000, casts to signed
PCM16, and advances phase by smoothed Hz/rate. Phase wraps by subtracting one
cycle. Target changes do not reset phase. At 24 kHz this smoothing has an
approximately 20.8 ms time constant; scaling the coefficient approximately
preserves that duration at Bluetooth rates. This is a one-pole smoother, not a
fixed-duration ramp or note envelope.

The intended domain is the app's finite positive rate, approximately 78–622 Hz
frequency and gain [0,1]. There is no argument validation, clipping or general
multi-cycle wrap handling: invalid rate, NaNs, excessive frequency/gain or corrupt
phase can produce invalid output or indexing. Callers must own the instance and
serialize rendering. The output pointer must hold n PCM16 samples.

The lookup table avoids a sine call per sample; interpolation reduces coarse
table steps. Smoothing reduces abrupt parameter discontinuities. These are
implementation purposes, not measured proof of ESP32 CPU budget or inaudibility
of all clicks. The output is one continuous sine oscillator: no polyphony,
scale quantisation, MIDI, effects, anti-aliasing engine or audio mixing.

## `main/app_synth.c`

### Entry and control state

Shell registration makes `synth` available under Simulations. Static state owns
the calibrated pose, oscillator, UI flags, target frequency/gain, update time,
last error, worker bookkeeping and a static completion semaphore. A spinlock
protects parameters shared between the UI and audio producer; the oscillator
itself is owned by whichever output path is currently generating samples.

`synth_init` first enables the synth FFT gate, then resets calibration, starts
muted and sets the target to 220 Hz/zero gain. Active recording, audio-player
playback, local output or already-streaming Bluetooth rejects entry with
`ESP_ERR_INVALID_STATE`. It does not interrupt those users or automatically
retry a failed entry. FFT stays gated until the synth app closes even on failure.

The completion semaphore is created once, any old completion token is drained,
the oscillator is reset and an unpinned `synth` task is created with stack argument
6144 and priority 3. Task creation failure is reported as `ESP_ERR_NO_MEM`.
`s_worker` means a task was successfully launched and must be joined on exit;
it may remain true after that task has completed and signalled the semaphore.

Only press events are handled. A requests Simulations (a valid shell context is
assumed); B takes a fresh sample, attempts **full-pose** calibration and mutes;
D toggles mute only after calibration. C is unused. Invalid quaternion rejection
comes from the shared math, but the B handler does not report that return value
and can retain an earlier calibration. Failed/stale sample retrieval leaves
state unchanged. This app hardcodes the left-arm convention; Paint's right-arm
choice and heading-only calibration do not carry over.

### Gesture mapping and freshness

`sample` requests a coherent orientation snapshot and accepts age <=250000 us.
Unsigned age subtraction also rejects future timestamps under ordinary clock
operation. It uses the same monotonic microsecond clock as the sensor service.

Each `synth_tick` computes the locally calibrated finger direction through
`arm_pose_direction(..., true, ...)`. For valid data, target frequency is
`220 * 2^(1.5*clamp(direction.y,-1,1))`, spanning about 77.8 to 622.3 Hz.
Horizontal direction gives 220 Hz. The mapping is exponential in direction's
vertical component, not linear in arm angle; the exponent gives three octaves
over the complete range. If unmuted, gain is `(1+clamp(direction.x,-1,1))/2`.
Sideways +X gives maximum, -X minimum, and straight ahead half gain. Direction
components are coupled by arm geometry; pitch and loudness are not independent
physical controls. Missing/invalid data requests 220 Hz and zero gain.

The tick publishes targets and current update time under the lock; dt is unused.
Muting does not stop frequency updates. `generate` copies that snapshot, requests
zero gain on stop or when no UI update arrived for 250 ms, then renders a block.
Thus both stale IMU samples and a stalled UI can fade output to silence. After
stop is requested and smoothed gain is below 0.0001, it returns zero samples to
terminate local generation. Ordinary mute keeps the stream running, avoiding
repeated audio starts when the user toggles it.

### Route selection and worker lifecycle

On entry to `worker`, an already streamable Bluetooth connection is preferred.
It waits up to two seconds for an earlier media command to finish, checking stop
every 10 ms. Only negotiated 44.1/48 kHz is accepted. It attempts generator
attachment and one start command; no retry/reconnect loop exists.

If start is accepted, it sets the Bluetooth UI flag and polls every 20 ms until
stop, loss of streamability, or streaming=false with no command pending.
`generate_bt` runs on the Bluetooth data-callback path, sets the oscillator rate
and delegates to `generate`. The worker does not simultaneously render audio.

Cleanup detaches the generator, which waits for already-running callback readers
to finish. Only then may the worker mutate oscillator state for local output.
After a successful start it waits up to two seconds for a pending media command,
then requests stream stop once. Detach/stop return values are ignored here; the
app does not claim the speaker drained or that Bluetooth shutdown succeeded.

If no stop was requested, Bluetooth absence, unsupported rate, startup failure
or disconnect falls back once to local speaker output. The worker restores
24 kHz and resets smoothed gain to zero (fade-in on route change), but retains
phase/frequency. It initialises the local bus on demand, then blocks inside
`audio_play_generated`. It never switches back to Bluetooth until the app is
reopened. A connected-but-silent sink or a stalled stream still reported as
streaming is not detected by this worker; it has no audible-progress watchdog.

Local error or final success is published under the lock, completion is signalled
and the task deletes itself. Bluetooth failure details are not preserved if local
fallback succeeds. `synth_deinit` requests stop, waits indefinitely on completion
when a task was launched, then removes the FFT gate. It does not forcibly delete
a task inside I2S or a Bluetooth callback. Individual operations have some bounds,
but the overall join has no timeout: a stuck dependency can block app exit.

### Output interfaces (partial dependency review)

The local generator adapter claims the audio output-user count, rejecting an
existing local output, sets the bus rate and enables TX. It asks for at most
256 mono PCM16 samples per block, applies local volume, duplicates into stereo
32-bit slots and handles partial I2S writes. A zero sample count ends generation;
oversize output, write errors or zero progress return an error. An enabled TX
gets a quiet tail before the output count is released. This is not a general
multi-source mixer; the synth's entry checks remain necessary for other users.

Bluetooth's generator bridge is pull-based: the A2DP callback requests batches
of up to 128 mono samples, applies Bluetooth volume with PCM16 saturation,
duplicates channels and zero-fills unused bytes/short output. It rejects attach
without a streamable peer or while file frames remain queued, and refuses to
replace an existing/in-use generator. A reader count allows detach to wait for
callbacks without holding the spinlock during synthesis. The generator must be
fast and nonblocking; detaching from inside its own callback would deadlock.
This avoids a large application PCM backlog, not codec/radio/speaker latency.
The legacy `bt_audio_feed_pcm` push API is still unsupported.

No synth code writes files, sends PCM over the suit control link, discovers
speakers, configures show delay compensation or changes the microphone hardware.
FFT gating prevents its capture workload from competing with the synth, but is
not a universal resource mutex. Entry checks and application lifecycle assume
other services do not independently start conflicting work afterward.

### Display and remaining checks

`synth_draw` snapshots target Hz/gain, error and route flag under the lock.
It labels Bluetooth only after the start request succeeds; otherwise it says
speaker, including before local initialization completes. Values are **targets**,
not measured output, smoothed instantaneous values or acoustic loudness. Error
text suggests exit/reopen. Fixed text positions ignore w/h and assume a valid
framebuffer/normal content area; clipping and overflow need a separate UI pass.

The local path can execute its release envelope while stopping. The Bluetooth
worker detaches promptly on stop, so callback opportunities to finish that
envelope are not guaranteed. Do not claim identical click-free shutdown for
both routes. Hardware checks still needed: entry busy cases, quiet/strong
gestures, stale IMU, repeated open/close, Bluetooth disconnect/start rejection,
speaker latency, memory/stack headroom and return of normal FFT operation.

## `main/shell_audio.h` and `main/shell_audio.c`

The header exports `bool shell_audio_init_if_needed(void)` with C++ linkage
guards. The implementation's static flag is only a convenience cache, not an
ownership token or synchronization primitive. It returns immediately only if
previous initialization succeeded **and** `audio_tx_handle()` still exists.
If Bluetooth previously released the bus, the missing handle forces restoration.

It constructs I2S config for controller 0 using `PIN_I2S_BCLK`, `PIN_I2S_LRCK`,
`PIN_I2S_DOUT`, `PIN_I2S_DIN`, and no amplifier-enable GPIO (-1), then calls
`audio_init`. Only ESP_OK sets the cache; failure returns false and allows a later
retry. Exact pin numbers live in hardware definitions, not this helper. It does
not set sample rate, volume, start TX, arbitrate capture/playback, expose detailed
errors or serialize concurrent initializers. Callers must establish resource
availability first. The header/source need a component-level bus integration
test; oscillator tests do not exercise this helper.

## `main/tests/test_synth.py`

`SynthTest.test_voice` writes a temporary C harness that includes the actual
oscillator header, compiles using GCC C11, strict warnings, ASan/UBSan and libm,
then executes it. Subprocess failures fail the unittest; temporary files are
cleaned by context-manager lifetime. Requires a host compiler, not ESP-IDF.

It checks a muted one-second buffer is zero; counts 440 Hz positive crossings
after settling at 24 kHz; bounds full-gain PCM to 12000; switches frequency while
fading to zero and checks first-sample discontinuity, gain threshold and final
silence; verifies finite wrapped phase; and counts 440 Hz crossings at both
Bluetooth sample rates after warmup. These checks are coarse numerical tests,
not exhaustive spectral/audio-quality, malformed-input or performance tests.
No app worker, IMU, locks, I2S, Bluetooth stack or real speaker participates.

## `main/tests/test_synth_bt.py`

`SynthBluetoothTest.test_generator_bridge` extracts the production generator
bridge block from `bt_audio_stream_file.c` by source markers, inserts it into a
temporary C harness and compiles/runs with strict GCC warnings and ASan/UBSan.
Marker changes require updating the extraction. Stack/service functions are
stubbed; critical-section macros are no-ops and any unexpected task delay fails.

Tests cover no generator, disconnected attach, queued-file rejection, duplicate
attach rejection, user/rate forwarding, batch size <=128, stereo duplication at
44.1/48 kHz, odd trailing-byte zero fill, saturation with boosted volume,
short-generation silence, unsupported-rate silence without generator calls,
and successful detach. The fake generator emits constant samples, not the synth
oscillator. The test does not exercise concurrent detach, actual callback reader
lifetime, pending-command timeouts, worker fallback, RF transport or speaker
latency. A passing bridge test must not be described as end-to-end Bluetooth
validation.

Run `python3 main/tests/test_synth.py` and `python3 main/tests/test_synth_bt.py`.
The FFT gate test and orientation/paint math test are useful adjacent regressions,
but must be complemented by the hardware cases above.
