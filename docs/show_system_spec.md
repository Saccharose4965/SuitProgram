# Music Show System Spec v1

## Local ESP-IDF log-only patch (2026-10-14)

In ../esp-idf/components/bt/host/bluedroid/stack/l2cap/l2c_link.c, removed only
the L2CAP_TRACE_ERROR("l2cab is_cong_cback_context") call inside the recursive
congestion-callback guard. The condition and return are unchanged; all other
BT errors remain enabled. This is a local SDK modification outside the firmware
repository; preserve/reapply it if replacing/updating ESP-IDF. No broad tag
filter or media/queue behavior change was made.

Latest run completed normally, but the snapshot still showed only 1423 bytes
internal free and 41120 bytes in ten outstanding SBC allocations, with no
trace overflow. Cleanup restored ~50 KB. Silencing the repetitive guard message
is NOT a fix for this remaining buffer pressure. Keep heap/stall diagnostics.

## Best-effort frame pacing (2026-10-14)

Show rendering now targets up to 60 fps but backs off immediately when measured
calculation + submission time rises. The interval is at least twice a cost
estimate (fast attack, 1/16 recovery per rendered frame), with a post-frame
cooldown of that estimate. This leaves approximately half the wall time for
other tasks rather than repeatedly attempting overdue frames. Transport and
audio clock service run before the render gate; skipped frames do not advance
or slow the song. Every actual render samples the current playhead. Identical
paused frames remain skipped. Logs include the adaptive target beside actual
fps. This is pacing, not preemption of an individual expensive calculation;
hardware playback reliability must still be verified.

## Planar torso rendering (2026-10-14)

Editor and firmware now place torso front sections on local Z=+12 and back
sections on Z=-12, preserving their previous X/Y silhouette. Ring, ribs,
abdomen, belts, spine and shoulders follow their respective panel. Arms and
legs retain their existing geometry. Panels rotate/translate with each suit;
circle formation and cylindrical orb trajectories are unchanged.

Orb intersection transforms the center into suit space once per frame and
reuses the squared normal distance for each torso panel; out-of-range firmware
LEDs avoid square roots. World LED coordinates are cached at load/formation
changes for sweeps. The cache is part of the existing PSRAM renderer allocation.
Coplanar radial-ray zero coordinates are snapped consistently on PC/ESP to
avoid floating-point sign flips. No audio/scheduling/BT changes here.

Existing packages use the new geometry after firmware update; no binary format
change or re-export required. Restart the editor to pick up the same geometry.
Visual appearance intentionally changes from curved to flat torso intersections.
Tests compare actual C geometry/rendering against Python for all roles, line/
circle and backward formation seeks; old curved-torso snapshots were updated.
Hardware frame-time improvement remains to be measured, not assumed.

## Playback investigation checkpoint (2026-10-14)

LED-disabled comparison completed the full ~399 s Show at 30 fps. Marker was
confirmed active, submit_max=0, free internal heap stayed ~36–45 KB and returned
to ~50 KB. The failed LED-enabled trace had 28,784 bytes in seven SBC encoder
allocations; no overflow. This implicates the output path but does not prove
CPU contention rather than physical LED power/noise effects.

Output contention fix under test: move LED TX/pulse workers from CPU0 to CPU1,
away from BT/Wi-Fi; retain priorities and bounded/coalesced asynchronous TX.
Increase RMT channel memory from 128 to 256 symbols per output, using all eight
ESP32 RMT memory blocks for the two strips and halving refill frequency. No
other RMT users may be enabled concurrently with this allocation. Shell starts
on CPU1 and normally allocates RMT there; per-output startup logs now confirm
the allocating core (the ISR affinity), separately from worker affinity.
No BT policy/queue changes, no show timing changes, no FFT algorithm changes.
Remove show-no-leds, flash, cold boot, verify RMT init_core=1 and LED workers
core=1, then test full Show playback. Tracing remains enabled for comparison.
Hardware success is not yet established; also check FFT responsiveness after
BT is stopped because LED workers now share CPU1 with the FFT tasks.

Same-firmware hardware comparison: Music played the full ~399 s mono 44.1 kHz
InfectedMushroom track. Show (matching WAV format and data length) stalled at
~376 s after a burst of L2CAP congestion warnings. Free internal memory fell
from ~45 KB to ~10 KB; 64 KB of PCM remained buffered. Disconnect recovered
~47 KB. This implicates an interaction with Show, not proven allocation leaks
or a proven LED/core issue. LED TX/pulse workers are on core 0 with BT, although
Show calculation is on core 1. No core assignment or BT policy was changed.

Temporary diagnostic build:
- Standalone heap tracing, 2048 records, six allocation stack frames, with
  records and lookup hash in PSRAM. Starts immediately before BT WAV startup
  for both Music and Show. Tracks outstanding allocations, not proven leaks.
- Freeze at the first observed internal-free value below 16 KB or an audio
  stall. Dump the top 20 internal allocation stacks by total outstanding bytes
  only after playback cleanup. Normal completion also reports a snapshot.
- Output includes overflow/high-water mark and baseline/snapshot/current heap.
  Overflow means incomplete attribution. PSRAM tracing excludes ISR allocations;
  allocations before tracing began are also absent. Tracing changes timing and
  must be disabled again after diagnosis (HEAP_TRACING_OFF).
- Empty SD-root file `show-no-leds` disables recurring Show LED submissions,
  while still calculating frames and running transport/audio. Read once on
  output acquisition; one blank frame is sent. Remove the file for normal
  output. This does NOT isolate CPU contention from LED power/RF effects.

Next hardware tests: cold boot and play the full same Show normally, retain
the complete log including the post-cleanup `playback_diag` report. Then cold
boot with `show-no-leds` present and repeat under the same speaker/location/
peer conditions. No new BT restart, buffering, or scheduling tweaks. If tracing
changes the failure pattern, compare Music with this same diagnostic build too.
Keep the matching build/SuitProgram.elf for decoding allocation stack addresses.
Do not rebuild before collecting/decoding both runs.

Host checks: audio_player/tests/test_playback_diagnostics.py (freeze, cleanup,
grouping, repeat runs and unavailable trace); test_bt_monitor.py; Show runtime
and LED ownership tests. Hardware cause and fix remain unconfirmed.

## Current integration status (2026-10-12)

The current firmware accepts SWS1 through version 1.11. The standalone editor
is `../ShowEditor`; its exporter selects the minimum required version for
formation cues, cylindrical orbs, parameter automation, baked amplitude,
spacetime color, Perlin color, bounded Fill and Hold/Decay. The dated v1.1
snapshot below describes the original vertical slice, not the current ceiling.
The detailed binary extensions are documented in
`../ShowEditor/docs/format-compatibility.md` relative to the firmware root.

The editor integration tests compile this repository's actual package loader
and renderer and compare exported LED frames with the Python preview, including
seeks and formation changes. Audio packaging now converts to stereo 44.1 kHz
PCM16 WAV and fails on absent/invalid songs.

### Show lobby and audio integration (2026-10-13)

BT stall follow-up: removed the audio player's automatic Suspend/Start kicks
and restart retries. The reported sequence had an initial unexplained stall,
then a pending Suspend, a failed suspend confirmation, and disconnection; the
initial cause is not established. A stall now logs one stream-health snapshot
and waits for natural recovery/user control. Disconnection ends playback and
cleans up once, including when paused. No BT policy/buffer/radio changes were
made for this fix. Reproduce on hardware and retain the `stream health` log.
Host check: `python3 components/audio_player/tests/test_bt_monitor.py`.

Follow-up hardware log: stall with 64 KiB still buffered and only 771 bytes
internal heap free. This rules out an empty ring at that snapshot, but does not
prove the original cause. BT controller/Bluedroid already run on core 0; shell
and Show rendering run on core 1 (audio SD feeder also uses core 1). No BT core,
priority, or policy changes made. Move the music filename catalogue and I2S
zero staging buffers to PSRAM, freeing 14,964 bytes of internal static storage.
SD's initial SPI rate is reduced from 20 to 10 MHz after real CRC/timeouts;
this is a mitigation, not proof of repaired wiring/card integrity. Show rendering
now advances a scheduled deadline instead of resetting it to each shell tick,
avoiding 30-to-20 fps cadence quantization. The host test covers an early 60 Hz
tick. `show_runtime: render ...` reports achieved fps, render+LED-submit cost,
and internal heap every five seconds for hardware profiling.

Long-run follow-up: heap held near 45 KiB for minutes, then fell below 3 KiB
with 46,592 audio bytes still buffered; disconnection followed. This is not
evidence of constant insufficient startup RAM. Congestion/queued allocations
are a hypothesis, not a proven leak; the SDK encoded SBC queue is bounded.
Do not patch BT internals or reintroduce automatic restarts on that evidence.
Show transport/pings now skip periods without live listeners (presence discovery
continues, so late join still works). Unchanged paused/stalled frames reuse the
already submitted output. Render diagnostics now split maximum calculation
and LED-submit times; both include scheduling/preemption, not just CPU time.
Next hardware check: repeat the long run, note single/multiple suits, collect
split render timings and heap. If it still stalls, compare audio-only playback
of the same track before changing coexistence or BT settings.

Subsequent run (658 s uptime) failed within seconds with 1,443 internal bytes
free, no ring underruns, and 65,024 bytes buffered. Whether the heap was already
low before playback is unknown. Added audio-player heap snapshots before file
open and after cleanup (the worker stack still exists at the latter snapshot).
Next comparison must start from a power cycle and play the same WAV in Music,
without entering Shows. No new BT configuration/recovery changes for this run.

Music-only comparison completed EndOfLine normally (~156 seconds), with heap
returning from 49,843 to 49,747 bytes. Congestion warnings occurred without a
failure. Show interaction is therefore a suspect, not yet proven causal.
Confirmed LED contention: the async worker released the pending-frame mutex,
then its flush helper reacquired it across RMT transmission/completion waits.
Flush now owns only the separate TX mutex, covering physical-buffer splitting,
submission, drain, and retry. Pending frames can be replaced during transmission
without modifying in-flight data; synchronous diagnostic/init paths still wait
as before. All outputs are drained even after partial submission failures, and
leftover transactions must drain before buffers can be reused. RMT wait timeouts
use milliseconds per the IDF API. No BT changes in this pass.
Test: `python3 components/led/tests/test_tx_ownership.py`. Firmware build plus
Show/runtime and BT-monitor tests pass. Next board run: same EndOfLine show,
check `submit_max`, heap, and completion; do not assume the BT failure is solved.

- Open Shows on each suit, on the same radio channel. Presence is advertised
  every 500 ms and expires after 2 seconds. Only suits in Shows (or already
  playing a show) accept remote transport. On entry, query peers and allow a
  700 ms discovery window before allocating a free A/B/C role. Existing claims
  stay in place when another suit enters. The lowest live MAC coordinates a
  three-entry assignment table; this does NOT make it the playback leader.
  Selecting an occupied role swaps the two suits. HELLO carries numbered role
  requests; periodic revisioned tables acknowledge them, so packet retries do
  not repeat swaps. Coordinator departure triggers re-election. Radio partitions
  can temporarily disagree; this is eventual coordination, not a guarantee
  across disconnected groups. Show wire protocol is now 0x52: flash all suits.
- Header: your role and ready/lead/follow, then A/B/C (`+` present, `*` leader,
  `-` absent). List rows are calculated from available shell content height;
  verbose package metadata has been removed. A/B navigate, D plays/pauses,
  C stops a running leader or changes idle role, Back leaves the menu. On the
  Role row C or D requests the next role, including during playback. The header
  shows `?` during discovery; pending requests display "Finding roles...".
  On the BT delay row, C/D decrease/increase by 25 ms (0–1000 ms).
  Delay and assignments are RAM settings, not persisted across reboot.
- Browsing uses the in-memory directory list only. No package reads/CRC checks
  on highlight changes or initial selection; D loads/validates the chosen
  package once. Opening the menu and long-D refresh scan directory entries.
  Idle role changes do not load/configure any show, even if one was previously
  loaded. An active role change loads only the necessary local role program,
  preserving audio ownership, session and song clock. Failed role loads retry
  at most once per second. SD work during an active change is still synchronous;
  brief local rendering delay is possible, not a restart of the song.
- All suits need identical `/shows/<slug>/show.bin`; the initiating suit also
  needs `/shows/<slug>/track.wav`. The starter owns audio and transport. A one
  second audio preroll gives followers time to load; late followers catch up.
  Opening Shows mid-song automatically joins after role negotiation. The coarse
  anchor uses packet receive time, not the start/end of loading; the first
  render catches up with time spent on SD. Clock-corrected unicasts refine it.
  Near-simultaneous starts use MAC order to break the tie before PCM starts.
- Show audio prefers an already-connected Bluetooth sink, falling back to the
  built-in speaker if BT startup fails. Both accept PCM16 mono/stereo; local
  stereo is downmixed. Unsupported BT sample rates are rejected rather than
  played at the wrong speed. A disconnect after playback starts currently stops
  the show through the existing disconnect handler; seamless mid-song speaker
  handover is not implemented.
- The leader's timeline follows PCM handed to A2DP / I2S, not the button press
  or an independent wall clock. BT compensation starts at an **unmeasured 100 ms**;
  local output estimates the I2S DMA queue delay. These are estimates of audible
  time, not measurement of the speaker DAC. Speaker-specific calibration is
  required. A sustained PCM stall pauses transport until delivery resumes.
- Radio RTT/clock offset uses callback receive timestamps; corrected unicast
  anchors are not replaced by coarse same-generation broadcasts. This does not
  guarantee sample-accurate sync under Wi-Fi/BT coexistence or packet loss.
- Entering Shows stops FFT and releases idle I2S buffers. Local speaker playback
  restores I2S on demand; followers do not start audio.

Validation: `python3 components/show/tests/test_runtime.py` compiles the actual
runtime with host mocks and checks discovery/roles, audio timing, stalls,
pause/resume, stale peers, sync-anchor preservation, and exhaustive UI bounds.
`python3 components/show/tests/test_roles.py` runs three simulated runtime
instances through sequential/simultaneous entry, occupied-role swaps, lost
snapshots/retries, concurrent requests, stale/invalid tables, coordinator
departure/rejoin, independent playback leadership, and a 1.2-second late load.
These checks and a firmware build do not replace the three-board/audio test:
open all menus, check A/B/C, start from each suit, try pause/stop, disconnect a
follower, test missing packages, then calibrate BT delay and verify local audio.

## Status

This document turns the requirements in
`docs/show_system_requirements.md` into an implementable first spec.

It defines:

- the runtime package structure
- the compiled show file format
- the editor data model
- the playback model on ESP32
- the multi-costume synchronization model
- the implementation plan for this repository

This is a v1 engineering spec. It should optimize for:

- a strong editor
- a simple, robust runtime
- no live frame streaming between costumes
- compact SD-backed playback

Implementation snapshot (2026-08-17): the silent LED vertical slice is now
implemented. The exporter writes format `1.1`; firmware performs full bounds
and CRC validation, loads the selected role and buckets, resolves local layout
semantics, renders the compiled effect catalog and per-clip color programs,
arbitrates LED ownership, and
synchronizes play/pause/stop with ping-derived peer clock offsets. Audio,
authenticated control, seek UI, long-duration drift tests, and extraction of a
shared clock-sync component remain open.

## Guiding Decisions

### 1. Compile convenience into runtime simplicity

The editor should support high-level authoring features:

- repeat every N beats
- repeat every N seconds
- interlaced role patterns
- global clips
- per-role clips
- reusable presets
- beat/bar snapping

The runtime should **not** interpret those abstractions directly.

Instead, export should compile them into:

- absolute-time clips
- already-expanded role-specific clip streams
- fixed target IDs
- simple effect parameter blocks

This keeps the ESP32 runtime small, predictable, and seek-friendly.

### 2. Runtime uses absolute time only

The editor can show beat and bar helpers, but the runtime executes by
milliseconds from show start.

### 3. Runtime packages target named body semantics

Shows should target:

- canonical section IDs
- canonical group IDs

They should not target hardcoded LED indices from one specific costume.

### 4. One runtime package per show

The clean v1 package layout is:

- one audio file
- one compiled show binary

The binary may contain separate compiled programs for role A, B, and C.

This keeps SD access simple and avoids having to manage multiple role files
manually.

## SD Package Layout

Each show lives in its own directory:

```text
/sdcard/shows/<show_slug>/
  track.wav
  show.bin
```

Where:

- `track.wav` is the runtime audio asset
- `show.bin` is the compiled choreography package

Future optional files can be added later without affecting playback:

- preview thumbnail
- author notes
- analysis cache

The runtime should ignore unknown extra files.

## Runtime Audio Asset

The v1 runtime audio asset is always:

- `track.wav`

The editor/exporter should normalize source audio into a playback-safe WAV
profile for the firmware.

The exact encoded WAV constraints can track the existing audio stack, but the
important design point is:

- the show player only needs to know the path to `track.wav`
- the editor/exporter is responsible for preparing a compatible file

## Canonical Target Model

The runtime must not depend on the local section order from one layout file.

Instead, the show system uses canonical section IDs.

### Canonical section IDs

The initial canonical section list should be name-based and stable across
layout revisions.

Current torso and arm IDs:

- `front_left_top`
- `front_left_ring`
- `front_left_rib`
- `front_left_abs`
- `front_left_belt`
- `back_left_belt`
- `back_left_vertebra`
- `back_left_rib`
- `back_left_top`
- `left_upper_arm`
- `left_forearm`
- `front_right_top`
- `front_right_rib`
- `front_right_abs`
- `front_right_belt`
- `back_right_belt`
- `back_right_vertebra`
- `back_right_rib`
- `back_right_top`
- `right_upper_arm`
- `right_forearm`

Implemented leg section IDs (renderer and editor). Physical firmware layout
support is incomplete: defaults contain only torso/arms, and the current
24-section capacity cannot hold the full 33-section suit. These IDs do not
automatically configure LED counts, wiring order or output pins:

- `left_thigh_front`
- `left_thigh_back`
- `left_shin_f_in`
- `left_shin_f_out`
- `left_shin_b_in`
- `left_shin_b_out`
- `right_thigh_front`
- `right_thigh_back`
- `right_shin_f_in`
- `right_shin_f_out`
- `right_shin_b_in`
- `right_shin_b_out`

### Canonical group IDs

The editor and runtime should also support canonical groups such as:

- `all`
- `front_all`
- `back_all`
- `left_all`
- `right_all`
- `torso_all`
- `arms_all`
- `left_arm_all`
- `right_arm_all`
- `ring`
- `belt_all`
- `spine`
- `legs_all`
- `left_leg_all`
- `right_leg_all`

The exact group catalog can grow over time.

### Local layout resolution

On load, the runtime resolves canonical section IDs against the local
`led_layout.txt` data and builds a local section mapping.

Because the final system is expected to make layouts consistent, v1 does not
need complicated missing-section behavior beyond basic validation.

## Runtime Coordinate Spaces

Every effect should be able to work from one or more of these spaces:

- `time`: absolute show time in milliseconds
- `clip_t`: normalized time inside a clip, `0..1`
- `section_u`: normalized position inside a section, `0..1`
- `target_u`: normalized position inside the current target bounds, `0..1`
- `x`, `y`, `z`: spatial coordinates from layout
- `radial`: distance from body center or group center
- `angle`: angular coordinate for ring-like regions

This is what lets authored effects stay semantic and layout-aware even when
section LED counts differ between costumes.

## Package Format

`show.bin` is a single compiled container holding:

- show metadata
- palettes
- group definitions
- three role programs
- clip time-bucket indices
- clip records
- effect parameter blobs

The container is binary and read-optimized.

## `show.bin` File Structure

### Header

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;              // 'SWS1'
    uint16_t version_major;      // 1
    uint16_t version_minor;      // 1 legacy, 2 static formation, 3 cues/ray space, 4 cylindrical orb
    uint32_t header_bytes;
    uint32_t file_bytes;
    uint32_t crc32;
    uint64_t show_uid;
    uint32_t duration_ms;
    uint16_t role_count;         // always 3 in v1
    uint16_t fps_hint;           // editor preview/export hint, e.g. 30
    uint16_t bucket_ms;          // e.g. 1000
    uint16_t palette_count;
    uint16_t group_count;
    uint16_t formation;          // v1.2: 0=line, 1=inward circle; previously reserved zero
    uint32_t role_table_offset;
    uint32_t palette_table_offset;
    uint32_t group_table_offset;
    uint32_t bucket_table_offset;
    uint32_t clip_table_offset;
    uint32_t param_blob_offset;
    uint32_t string_table_offset;
} sws_header_v1_t;
```

Notes:

- `show_uid` is a stable hash of the complete editable show content.
- `crc32` validates the compiled package.
- `bucket_ms` defines the seek/index granularity.

Format `1.1` appends this timing extension between the base header and role
table while retaining the v1 base header:

```c
typedef struct __attribute__((packed)) {
    uint32_t tempo_millibpm;
    int32_t beat_offset_ms;
} sws_timing_v1_1_t;
```

### Role table

Formation support (v1.2): the former base-header reserved uint16 at byte 42
stores `formation` (0=line, 1=inward circle). Header size, offsets and clip
records stay unchanged. v1.0/v1.1 imply line; unsupported versions/formations
are rejected. Static legacy line exports stay v1.1; circle exports use v1.2.

Neighbour spacing S=max(1, 1.5 × posed suit width), R=S/sqrt(3). In X,Z:
A=(-S/2,-R/2), yaw +60 degrees; B=(0,R), yaw 180; C=(S/2,-R/2), yaw -60.
Local +Z faces forward. Global effects use transformed coordinates and bounds.
Formation is saved per project, not a camera change; 2D stays an unfolded map.
v1.2 also poses flat torso/arm geometry once at renderer configuration;
v1.0/v1.1 retain their existing raw-layout behavior for compatibility.

v1.3 adds timed formation changes: after the timing extension, the header stores
a uint32 cue count followed by `{uint32 time_ms; uint16 formation; uint16 reserved;}`
records. At most 256 cues, strictly increasing times inside (0,duration), valid
formation IDs and zero reserved fields are required. Header size and subsequent
table offsets include these records; CRC includes them. The base-header formation
is used before the first cue. A boundary changes geometry/bounds, not clip time,
and backward seeking re-resolves the formation. No physical sensing/interpolation.

Each v1.3 radial-ray clip appends a packed 16-byte record after its spatial params
and before color animation: `uint8 space, plane, origin, reserved; float x,y,z;`.
Space: 0 stage, 1 suit. Plane: 0 XY, 1 XZ, 2 YZ. Origin: 0 average chest centers,
1 target LEDs' bounding-box center, 2 chest (B in stage), 3/4/5 A/B/C chest,
6 custom XYZ. Local space resolves suit/group origins to its local chest.
Stage target bounds include all three suits. Coordinates must be finite within
±10000 layout units; reserved is zero. XY/YZ phase zero is -Y, XZ is +Z.
Old ray defaults remain Stage/XY/B-chest. New cues or explicit ray spatial options
select v1.3 for the whole file; older firmware rejects it before playback.

v1.4 keeps the v1.3 binary layout and adds orb axis ID 6 (fixed XY diagonal).
For circle formations, X/Y/diagonal/random-XY orb centers follow straight paths
on an unwrapped cylinder, wrapped back into XYZ; the sphere and its Euclidean
LED distance tests are unchanged. Horizontal motion closes after one revolution;
vertical/diagonal motion enters/exits beyond height bounds. Radius/softness use
the Line-layout projected extent, not circumference. Diagonal paths have a constant
slope constrained to 0.5–1.5 rotations per full vertical pass (including margins);
pure vertical and horizontal paths remain unchanged. Random-cross-X becomes a
deterministic angular start offset. Legacy Z/radial/section paths remain straight.
See `../ShowEditor/docs/format-compatibility.md` (from the firmware root) for exact
trajectory equations and regression checks. Orb poses are computed once per
clip/frame. Existing <=v1.3 binaries keep their old trajectories. New circle-orb
exports and fixed diagonals require v1.4 firmware; nothing is automatically flashed.

### Generalized effects (v1.5)

Fill (ID 1) also implements the editor's Blink recipe: brightness cycle shape
Hold blinks, Smooth pulses, Linear gives a triangular fade. Retired standalone
Blink/Pulse/Mirror Sweep IDs are rejected. Tesla ID 18 has since been retired.

After the base parameters (Fill 8 bytes; spatial 30, plus radial backend 16), v1.5 adds
an automation header `<HH>` (count, zero), count records `<BBBBffff>` (key, mode,
BPM, shape, From, To, rate, duty), then an 8-byte spatial gradient (enabled,
mapping, reverse, space, RGBA), before any existing color-animation suffix.
There are nine numeric keys: intensity, width, softness, rate, phase, repeats,
center X/Y/Z. Evaluators are stateless; animated speed is analytically integrated.
Clip flags 1/2/4 mean BPM / explicit cycle motion / fit cycles to clip.
The loader validates lengths, unique keys, numeric bounds and CRC.

The PC contract is detailed in `../ShowEditor/docs/format-compatibility.md`
(relative to the firmware root). `show_effect_presets.h/.c` provides an empty
named registry of immutable program recipes, to populate and connect to a suit
chooser later. Existing standalone LED modes, input handling and FFT are untouched.

### Baked song amplitude (v1.6)

Animation mode 4 controls intensity, width, softness or repeats (keys 0/1/2/5).
Immediately after that 20-byte animation record comes `<HH>` count, interval_ms,
then count normalized uint16 samples. Count is 2–6001, interval at least 20 ms;
the curve must cover exactly ceil(clip_duration/interval)+1 samples. The loader
checks sizes, coverage and valid keys before playback. Other animation records
and the gradient/color suffix follow as usual.

The PC bakes per-channel-energy RMS, gain and smoothing from the project song.
Firmware interpolates two samples per parameter/frame and maps through From/To;
no microphone measurement or new FFT work is involved. Thus each costume gets
the same response. This requires re-exporting when audio or amplitude settings
change. Existing <=1.5 exports keep their previous layout.

### Independent Blink controls (v1.7)

Animation key 9 is the Blink level multiplier (cycle only, range 0–1). It uses
the existing record structure, is allowed only on Fill records, and multiplies
the independent intensity track. Thus Color brightness can follow amplitude
while Blink keeps its rhythm. Key 9 requires v1.7; older versions reject it.
BPM timing anchors to the project beat offset; unsynced timing starts at the clip.
Hold starts High for the duty fraction; Smooth/Linear peak High at phase zero.
New editor timing defaults to one cycle per beat; saved choices are preserved.

### Spacetime color and generalized Sweep (v1.8)

Color suffix marker 4 introduces a `<BBBBBBBBf>` grid header: marker, rows,
columns, loop, time blend, space blend, BPM, zero, rate; then row-major RGBA8.
Rows are 1–8, columns 1–16, interpolation Hold/Linear/Smooth=0/1/2. Existing
gradient metadata supplies spatial mapping. Once spans the clip; Loop includes
the final-to-first interval. Time colors are prepared once per clip, then sampled
along the LED's gradient coordinate. The loader checks dimensions, enums, rate
and exact size before rendering.

Cycle duty applies to all shapes with a Low interval outside the active window.
Decay shape 3 adds an instantaneous onset and exponential fall, no rise; Blink
anchors that onset to phase zero. Old full-period curves remain version-gated.
Sweep combines repeats and wrapping (axis_mix_1024 bit 14); old Chase ID 8 is
retired. Re-export old Chase JSON with the editor. See the PC format notes for
equations. New color-grid/cycle exports require v1.8 firmware; nothing is flashed
automatically.

### Bounded Fill and evolving Perlin color (v1.10)

Current firmware accepts through v1.11. Every Fill/Blink backend block in a
v1.10 file adds 32 bytes `<B3x7f>` after its 8-byte base: space, three zeros,
softness, X min/max, Y min/max, Z min/max. Then come automation and color.
Bounds use normalized 0–1 coordinates; crossed/equal bounds are empty.
Fill Y is upward-positive (feet=0, head=1). Animation keys
10–15 animate limits with Hold/Linear/Smooth/Cycle/Amplitude; key 2 can animate
Fill softness. Default full bounds retain the uniform fast path.

Color mapping 6 uses suffix marker 5: grid header, `<fff>` noise scale/width/speed,
then cells. The field is fixed-plane, evolving 3D gradient Perlin, not translation.
XY matches the unwrapped 2D costume with front/back overlaid, cached at load.
Time follows project beats; zero speed freezes the field. Core → edge for new
gradients reaches its final color at the solid boundary independently of brightness.
Format/float/length validation precedes rendering.

Global Sweep / Radial exports existing backend ID 17. Ground Energy authoring
is replaced by animated Fill limits; Hold has optional Decay. Tesla is removed.
Source clips are preserved but retired Tesla clips must be replaced/deleted before
export. See ShowEditor/docs/format-compatibility.md for migration caveats and tests.
Nothing is flashed automatically.

### Hold with trailing decay (v1.11)

Animation shape 4 keeps High for the full duty, then decays to Low during the
remaining cycle. Duty 1 stays High; duty 0 gives pure decay. The integrated curve
includes the plateau and tail. Legacy shape 3 retains its immediate decay.
New optional-Decay exports require v1.11, with the existing v1.10 Fill prefix
and unchanged animation record size. Nothing is automatically flashed.

Each package contains three compiled role programs:

- role A
- role B
- role C

```c
typedef struct __attribute__((packed)) {
    uint8_t  role_id;            // 0=A, 1=B, 2=C
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t bucket_first;
    uint32_t bucket_count;
    uint32_t clip_first;
    uint32_t clip_count;
} sws_role_program_v1_t;
```

### Bucket table

Clips are sorted by `start_ms`. A time-bucket index allows fast seek and
catch-up without scanning the entire show.

```c
typedef struct __attribute__((packed)) {
    uint32_t bucket_start_ms;
    uint32_t clip_first;
    uint16_t clip_count;
    uint16_t reserved0;
} sws_bucket_v1_t;
```

Behavior:

- a bucket points to the first clip that can become active in that window
- bucket size should be coarse enough for compactness and fine enough for fast
  seeks
- `1000 ms` is a good v1 default

### Clip table

Each role program compiles down to clips.

```c
typedef struct __attribute__((packed)) {
    uint32_t start_ms;
    uint32_t end_ms;
    uint16_t layer;
    uint8_t  effect_kind;
    uint8_t  blend_mode;
    uint8_t  target_kind;        // section, group, all
    uint8_t  reserved0;
    uint32_t target_id;
    uint16_t palette_id;
    uint16_t flags;
    uint16_t fade_in_ms_div10;
    uint16_t fade_out_ms_div10;
    uint32_t param_offset;
    uint16_t param_bytes;
    uint16_t seed;
} sws_clip_v1_t;
```

Notes:

- `layer` controls evaluation order
- `effect_kind` selects the evaluation function
- `blend_mode` controls compositing into the framebuffer
- `target_kind` + `target_id` resolve the clip target
- `param_offset` points into the parameter blob
- `seed` keeps pseudo-random effects deterministic

### Parameter blob

Effect-specific parameters are stored in a packed blob.

This avoids inflating the fixed clip structure while keeping export flexible.

Each effect kind owns the meaning of its parameter block.

In format `1.1`, a solid or spatial parameter block may carry an optional
compact color-program suffix: mode (`linear`, `smooth`, or `cycle`), timing
flags/rate, and up to 16 RGBA stops. No suffix means a static hold color. This
keeps common clips small while making exported editor color behavior explicit
instead of silently flattening it.

## Effect Model

### Initial blend modes

The v1 runtime should support:

- `replace`
- `alpha`
- `add`
- `max`

This is enough to build layered shows without overcomplicating the first pass.

### Initial effect kinds

The effect system should be extensible, but the first supported set should be:

- `solid`
- `blink`
- `pulse`
- `sweep`
- `mirror_sweep`
- `chase`
- `sparkle`
- `fanout`
- `global_sweep`
- `traveling_orb`
- `radialray`
- `ground_energy`

These cover the major use cases already discussed:

- blinks
- sweeps
- section light-up
- color effects
- interlaced role behavior
- repeated musical motifs

### Effect parameter examples

The editor should define effect-specific parameter blocks such as:

#### `solid`

- base color
- intensity

#### `blink`

- on color
- off color or transparency
- period_ms
- duty_cycle
- phase_ms

#### `pulse`

- color
- envelope curve
- amplitude

#### `sweep`

- color or palette reference
- axis mode: `section_u`, `x`, `y`, `z`, `radial`, `angle`
- width
- speed
- direction
- feather

#### `chase`

- head color
- tail length
- speed
- spacing
- repeat mode

#### `fanout`

- trigger order mode: by section order, by distance, by explicit list
- section delay
- section-local effect reference

### Compile-time helpers

The editor may expose richer helpers than the runtime.

Examples:

- repeat every 2 beats
- alternate every bar
- play only on role B
- interlace between A/B/C
- apply to torso but skip ring

These should be compiled into explicit clips before export.

## Editor Source Model

The editor project model is separate from the runtime package.

The editor should save a source project containing:

- audio asset reference
- layout references
- palettes
- named targets and groups
- analysis markers
- timeline tracks
- clip definitions
- preset library references

The source format can stay editor-friendly. It does not need to match
`show.bin`.

### Suggested project structure

At the editor level, a show project should contain:

- show metadata
- one main audio track
- marker lanes
- three role timelines
- optional global timeline
- clip library / presets

### Global vs role-local authoring

The editor should allow:

- global clips applied to all roles
- global clips with role modifiers
- explicitly separate role-local clips

Export then resolves this into three independent role programs.

### Marker lanes

The editor should support helper marker lanes such as:

- beats
- bars
- drops
- chorus
- custom labels

These are authoring aids. The runtime does not need them except if explicitly
exported into clips.

## 3D Preview Model

The editor should provide:

- LED-precise simulation
- 3D preview

The 3D body model does not need to be anatomically perfect in v1. It only
needs to be useful for:

- checking spatial sweeps
- checking left/right balance
- checking torso-to-arm transitions
- validating approximated curvature

The current Python layout tooling and real layout files should be reused as the
base input model.

## Runtime Playback Model

### Loading

When a show is selected, the runtime should:

1. mount the SD card
2. open `/sdcard/shows/<show_slug>/show.bin`
3. validate header and CRC
4. resolve the selected role program
5. load or map the role clip stream into RAM or PSRAM
6. open `track.wav` path for optional local audio playback (not implemented yet)
7. build local section/group resolution tables from `led_layout`

### Memory strategy

For v1, the simplest acceptable strategy is:

- load the compiled role program and supporting tables into RAM/PSRAM at show
  selection time

Because the runtime data is clip-based rather than frame-baked, this should be
practical for large shows.

If future shows become too large, the bucketed clip layout already provides a
path toward partial streaming.

### Playhead

The runtime playhead is defined by:

- `transport_state`
- `anchor_local_us`
- `anchor_playhead_ms`

Current playhead:

```text
if paused:
  playhead_ms = anchor_playhead_ms
else:
  playhead_ms = anchor_playhead_ms + elapsed_ms_since(anchor_local_us)
```

This is what makes:

- catch-up
- pause
- resume
- seek
- synchronized restart

all straightforward.

### Rendering loop

Each LED frame:

1. compute current `playhead_ms`
2. find candidate active clips from the current time bucket
3. filter clips where `start_ms <= playhead_ms < end_ms`
4. sort/evaluate them by layer
5. render into the LED framebuffer using local layout resolution
6. submit through the show owner with `led_show_pixels_owned`

### Clip evaluation model

For each active clip:

1. resolve target pixels from section/group ID
2. compute clip-local normalized time
3. evaluate effect for each targeted LED
4. blend into the framebuffer using the clip blend mode

### Audio ownership

The show system should support optional on-device audio playback.

For v1:

- only one suit should own audio playback at a time
- that suit can be the show master/controller
- all other suits remain silent

If audio playback is disabled or unavailable:

- the light show still runs from the synchronized timeline

### Audio delay

Because Bluetooth output may be delayed relative to show start, the transport
state should include:

- `audio_delay_ms`

Meaning:

- timeline zero is show time zero
- the local audio owner may start playback with a known offset
- the visual timeline may also be phase-adjusted against expected heard audio

The exact tuning UI can come later. The spec only requires that the field
exists in the transport model.

## Sync Architecture

### Clock sync

Long term, show sync should not keep a second independent clock-sync mechanism.

Instead, the existing master/slave clock alignment logic should be extracted
into a reusable service, for example:

- `components/clock_sync/`

Responsibilities:

- peer discovery
- ping/pong timestamp exchange
- offset estimate
- one-way latency estimate

The current vertical slice keeps ping/pong offset estimation inside
`show_runtime.c`; extracting the common math used by Master Control is a named
refactor, not a protocol behavior change.

### Transport authority

At any time, one device is the transport authority for the session.

That device controls:

- selected show
- current session ID
- current transport generation
- play/pause state
- playhead anchor
- audio delay

### Sync packet family

Show sync should use `LINK_MSG_CONTROL` with a dedicated show-player protocol
kind distinct from the current master-control packets.

### Packet kinds

Suggested v1 packet kinds:

- `SHOW_PKT_HELLO`
- `SHOW_PKT_ARM`
- `SHOW_PKT_TRANSPORT`
- `SHOW_PKT_RELEASE`

### `SHOW_PKT_ARM`

Sent when preparing a synchronized run.

Fields should include:

- protocol version
- session ID
- show UID
- role assignment
- transport generation
- desired initial state

### `SHOW_PKT_TRANSPORT`

This is the main periodic state packet.

Fields should include:

- protocol version
- session ID
- show UID
- transport generation
- state: stopped, armed, playing, paused
- anchor_playhead_ms
- anchor_master_us
- audio_delay_ms
- flags

Every receiver computes its local current playhead from:

- `anchor_master_us`
- clock offset estimate
- local current time

This allows late joiners to catch up.

### `SHOW_PKT_RELEASE`

Ends the active session and clears transport ownership.

### Catch-up behavior

Late joiners or temporarily desynced suits should:

1. receive the latest transport packet
2. compute the current playhead
3. seek local playback state to that playhead
4. render from the current position

This is preferred over waiting for a manual restart.

## Runtime State Machine

Suggested runtime states:

- `SHOW_IDLE`
- `SHOW_BROWSING`
- `SHOW_LOADED`
- `SHOW_ROLE_SELECTED`
- `SHOW_ARMED`
- `SHOW_WAIT_START`
- `SHOW_PLAYING`
- `SHOW_PAUSED`
- `SHOW_ERROR`

### Key transitions

- browse -> loaded: show package validated
- loaded -> role selected: local role chosen
- role selected -> armed: transport prepared
- armed -> wait start: synchronized start pending
- wait start -> playing: anchor time reached
- playing -> paused: transport pause
- paused -> playing: transport resume
- any -> loaded: seek/restart without reloading package if same show
- any -> error: file/sync/runtime failure

## Recommended Firmware Modules

The show system should be split into dedicated modules rather than folded into
existing apps.

### Firmware additions

Recommended new modules:

- `components/show/`
  - package loading
  - clip tables
  - target resolution
  - render engine
- `components/clock_sync/`
  - reusable peer time synchronization
- `main/app_show_player.c`
  - shell UI app for show browse/load/play

### Existing modules to reuse

- `components/led_layout/`
- `components/led/`
- `components/link/`
- `components/storage_sd/`
- `components/audio_player/`
- `components/bt_audio/`

## Recommended Editor Architecture

Because the project is Linux-first and already has Python tooling, the simplest
strong direction is:

- Python
- PySide6

Suggested editor package:

- `../ShowEditor/show_editor/`

Suggested major pieces:

- `project_model.py`
- `timeline_model.py`
- `layout_scene.py`
- `preview_2d.py`
- `preview_3d.py`
- `audio_analysis.py`
- `export_show.py`

### Why this direction

- matches current repo tooling
- fast to iterate locally
- suitable for waveform/timeline UI
- suitable for 2D + 3D preview
- easy to integrate export scripts

## Export Pipeline

The export pipeline should do this:

1. validate source project
2. resolve global clips into role-specific clips
3. expand repeats and interlace helpers into explicit absolute-time clips
4. resolve target IDs and palette IDs
5. pack effect parameter blocks
6. sort clips by time and layer
7. build time-bucket index
8. write `show.bin`
9. convert/export `track.wav`

The runtime should never need to know about the source abstractions that were
expanded during export.

## Implementation Phases

### Phase 1: Editor foundation

Deliver:

- show project save/load
- audio waveform
- timeline with clips and layers
- layout import from existing layout model
- LED-precise 2D simulation
- basic 3D preview
- export skeleton that writes a valid `show.bin`

### Phase 2: Firmware show loader — implemented

Deliver:

- `show.bin` parser
- role program loading
- target resolution via `led_layout`
- basic clip playback for a small effect set
- dedicated `show_player` shell app

### Phase 3: Multi-suit sync — core transport implemented

Deliver:

- reusable clock sync service (private implementation exists; extraction open)
- show transport packets
- play/pause/restart/stop (seek UI remains open)
- catch-up for late joiners

### Phase 4: Audio ownership

Deliver:

- show-linked `track.wav` playback
- optional Bluetooth audio ownership on one device
- audio delay offset support

### Phase 5: Rich editor and effect library

Deliver:

- reusable presets
- marker helpers
- advanced sweep and palette tools
- stronger 3D preview and layout refinement workflow

## Next Build Target

Uniform rendering optimization (2026-10-16): uniform fill/blink prepares a
REPLACE color once, or reuses a blend result while the targeted destination
color remains identical. Final RGB packing reuses conversion across identical
adjacent RGB values. This preserves zero-level REPLACE, alpha, layering,
brightness rounding, spatial colors and absolute song timing; no pulse widening
or audio-protection/scheduler changes. No additional persistent buffers.
`components/show/tests/test_uniform_render.py` compares 2,000 cases against the
previous calculation and checks flat spans use one blend and three roundings.
Runtime and editor/firmware spatial tests pass. ESP build passes; actual FPS and
blink improvement still require an EndOfLine hardware replay. Frame submission
timing measures enqueue cost, not physical LED latency.

The editor/export/SD/renderer/sync vertical slice exists. The next target is a
physical three-suit rehearsal with the same exported packages, followed by
golden/corrupt package tests, measured long-song drift, seek controls, and
single-owner `track.wav` playback with audio-delay calibration.
