# Music Show System Spec v1

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

Reserved future IDs for legs:

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
    uint16_t version_minor;      // 1
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
    uint16_t reserved0;
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

- `tools/show_editor/`

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

The editor/export/SD/renderer/sync vertical slice exists. The next target is a
physical three-suit rehearsal with the same exported packages, followed by
golden/corrupt package tests, measured long-song drift, seek controls, and
single-owner `track.wav` playback with audio-delay calibration.
