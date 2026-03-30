# Music Show System Requirements

## Purpose

Build a dedicated music-driven LED show system for the costumes.

This system is separate from the existing custom LED modes and audio-reactive
LED modes. It is a long-term authoring and playback workflow for prebuilt
shows tied to chosen audio tracks.

The target outcome is:

- choose a song
- author a show against that song in a desktop editor
- export a compact runtime package
- copy it to the SD cards
- have multiple costumes play the same show in sync, with role-specific output

## Core Product Goals

- Prebuild animations against a specific audio track chosen in advance.
- Avoid manual per-pixel authoring for normal workflow.
- Author at the level of sections, groups, lanes, layers, and reusable effects.
- Support three coordinated costumes that can do different things while still
  being synchronized as one show.
- Keep playback robust by avoiding live frame streaming between costumes.
- Make the editor strong enough that long-term show creation is comfortable.
- Support simulation and visual feedback, including layout refinement.
- Keep the runtime file compact and easy to read from SD.

## Non-Goals for the First Pass

- Do not merge this system into the existing custom or FFT/reactive modes.
- Do not depend on live editor-to-suit preview.
- Do not depend on continuous live frame communication between suits.
- Do not require per-costume calibration tools for brightness or color.
- Do not optimize around manual pixel-by-pixel choreography as the main method.

## High-Level System Shape

The show system consists of:

1. A desktop authoring editor.
2. A show export format for runtime playback.
3. A dedicated on-device `show_player` runtime.
4. Multi-costume synchronization based on a shared show timeline.

## Timeline Decision

### Primary timebase

The primary authoring and runtime timebase should be **absolute time** in
milliseconds.

### Why absolute time

Pros:

- precise alignment to the actual audio track
- works for dubstep, rock, classical, ambient, tempo drift, rubato, and
  sections with no strong beat
- easier to guarantee audio/show sync at runtime
- better fit for prebuilt choreography tied to one exact track

Cons:

- repetitive beat-based edits are less convenient by default
- musical structure like bars and beats must be added as helper overlays

### Supporting musical helpers

Even though absolute time is primary, the editor should support helper data and
tools based on musical structure, such as:

- beat markers
- bar markers
- snapping to beats or bars
- repeat every N beats
- repeat every N seconds
- event ranges defined over beat intervals

The runtime should still execute by absolute timeline position.

## Authoring Philosophy

- Manual authoring is primary.
- The tool should help build the show.
- The tool should not force the workflow into manual pixel editing.
- Audio-derived helpers are allowed and useful, but they support authoring
  rather than replacing it.

## Editor Requirements

### Platform

- Linux desktop first.
- Prefer the simplest implementation path that still yields a strong editor.
- A web app is not required for the first version.

### Editor capabilities

The editor must include:

- audio playback inside the editor
- timeline-based show authoring
- visual animation authoring against the track
- export to runtime files for the ESP32 system

### Visualization requirements

- Animation authoring can be 2D-oriented.
- The editor preview/simulator should support a 3D render.
- LED-precise simulation is desired, not just section boxes.
- The simulation should be able to use real section LED counts.
- Existing example layouts such as the `tom` and `jonas` folders should be used
  as references for real-world variation.

### Layout editing

- Layout editing should be local to the editor/tooling workflow.
- Layout approximation is acceptable for animation authoring.
- The approximation should still be good enough to make sweeps and section
  choreography look believable.
- Front and back panels have some side curvature toward each other.
- Arms are meaningfully 3D and should benefit from 3D preview.
- A strong 2D workflow is acceptable for authoring if the preview includes 3D.

### Preview and rehearsal

- No live editor-to-suit preview is required for the first version.
- The workflow is either:
  - simulate in the editor, or
  - export to SD and run on the actual suits

### Reusability

The editor should support abstraction and reuse:

- reusable effect presets
- reusable animation patterns
- reusable role/group behaviors
- higher-level authoring tools rather than repeated low-level edits

## Animation Model Requirements

### Desired authoring level

Authoring should happen mainly at these levels:

- global show lanes
- per-costume lanes
- section groups
- named sections
- layered effects

The system should be good enough that manual pixel editing is rarely needed.

### Effect support

The supported effect family should grow over time and aim to cover as many
practical, visually effective cases as possible.

At minimum, the system should be designed to comfortably support effects such
as:

- blink
- hold / section on
- fade in / fade out
- strobe
- pulse
- section light-up
- sweep
- chase
- gradient / palette-driven fills
- color changes
- brightness envelopes
- interlaced role patterns
- repeated patterns over time

The effect system should be extensible rather than fixed to a tiny set.

### Layering

Layering is desired.

That means:

- multiple effects may overlap in time
- composition rules will matter
- the show format should be designed for layered playback, not single exclusive
  cues only

Exact blend/priority rules can be specified later, but the system should be
built assuming layers exist.

### Role structure

Per-costume behavior should be authorable as separate lanes or role-specific
data, while still allowing shared/global animations.

This implies the editor should support:

- global animations applied to all costumes
- role-local animations
- export of role-specific runtime data

## Costume and Layout Requirements

### Current reality

- The costumes share the same semantic layout.
- The number of LEDs per section differs between costumes.
- The strips are cut and distributed into named pieces around the body.

### What must stay stable

- Section names are the canonical abstraction.
- Animations should target layout semantics, not hardcoded pixel addresses.
- Variation in LED count per section must not break authored shows.

### Layout consistency

- Missing-section fallback logic does not need to be a first-class concern.
- The expectation is that the final system will make the layouts consistent.

### Known future layout changes

Legs will be added.

Expected logical ordering:

- front
- leg
- back
- arm

Placeholder names for now:

- `upper_leg`
- `lower_leg`

This naming may be refined later.

### Asymmetry

- The right forearm may have a different LED count than the left forearm.
- This should remain the same semantic section naming model.
- No extra right-forearm naming split is currently required.

### Layout storage preference

The preferred implementation direction is:

- geometry/layout logic can live in code and editor tooling
- per-costume LED counts on SD are acceptable and useful
- the exact split can be chosen pragmatically as long as the system works well

### Calibration

- No dedicated per-costume calibration workflow is required for the first pass.

## Runtime Playback Requirements

### Dedicated mode

The show system must have its own runtime/player path.

It must not be treated as:

- just another custom LED mode
- just another audio-reactive mode

This should be a dedicated `show_player` application/state in the firmware.

### Multi-costume runtime model

- Each costume should hold its own local copy of the required show data.
- Playback should not depend on live frame streaming from one costume to the
  others.
- Synchronization should be based on shared show state and timeline, not raw
  LED frame transport.

### Control needs

The runtime should support:

- select show
- choose role / identity
- play
- pause
- seek
- restart
- synchronized start across costumes

The exact user flow for role assignment and arming can be designed later.

### Catch-up behavior

If a costume joins late or falls behind, catch-up behavior is preferred.

That means the runtime should be able to:

- determine current show timeline position
- jump to the current point in the show
- continue in sync rather than refusing until restart

## Audio Playback Requirements

### Editor audio

The editor must be able to play the audio track while authoring.

### Suit-side audio

Bluetooth audio playback is acceptable for the first version if wiring it into
the dedicated show player is straightforward.

However:

- the editor is still the first thing to build
- local speaker fallback is useful, but it is lower priority

### Sync and latency

It is expected that Bluetooth audio output may have a relatively consistent
latency offset.

Therefore the show system should be designed with the assumption that runtime
audio/show alignment may need:

- a configurable playback offset, or
- a known calibrated delay

This should be treated as normal runtime behavior rather than a hack.

## File Format and Packaging Requirements

### Packaging flexibility

The original idea was:

- one audio file
- three show files, one per costume

But this is not mandatory if a cleaner package structure is better.

Acceptable directions include:

- one audio file plus one runtime manifest/package with role-specific data
- one audio file plus three role binaries
- one container that internally stores all role data

The exact packaging should prioritize:

- simple SD access
- robustness
- compact runtime data

### Runtime data format

- Binary runtime data is preferred.
- Ease of SD loading matters more than human editability.
- The editor may use any internal/source format needed.

### Capacity expectations

- Shows may be long, potentially up to about one hour.
- The system should avoid artificial small limits on track count or show count.
- SD-backed storage means the design should scale without assuming only a few
  tracks exist.

## Sync Model Requirements

### Core sync principle

All costumes should agree on:

- which show is active
- which role each costume has
- the current timeline position
- play/pause/seek state

### Identity / roles

- Roles should be manually assignable at runtime.
- This can be handled later in the UI/app flow.
- The system should be built assuming role choice exists.

### Start flow

The final exact flow is deferred, but the desired behavior is:

- choose roles
- choose show
- start in sync automatically

## Practical Requirements Confirmed by Existing Layout Data

Existing real-world layout examples already show that section counts vary by
costume while semantics stay stable.

This confirms that the animation system must target named sections/groups rather
than raw LED indices.

Examples already observed in local layout data include:

- different torso section counts across costumes
- arm sections present in one layout and absent in another draft
- differing forearm and upper-arm counts

This is a normal production condition and should be treated as expected.

## Deferred / To Be Designed Later

The following are intentionally deferred and do not block the first real design
pass:

- exact role-assignment UI flow on device
- exact show selection UX on device
- exact Bluetooth calibration UX
- exact live network protocol details
- exact layer blend modes and priority rules
- exact leg section naming refinement
- whether the runtime package is one file, several files, or a container

## Requirements Summary

The first version should optimize for:

- a strong editor
- a durable long-term architecture
- section/group-based choreography
- role-specific synchronized playback
- compact local runtime data on each costume
- simulation good enough to trust before copying to SD

The system should not optimize for:

- one-off hacks
- manual per-pixel authoring as the default workflow
- fragile live frame transport between suits

## Immediate Next Step

The next document to create after this one should define:

1. the show package/runtime file format
2. the runtime `show_player` state machine
3. the synchronization model for three costumes
4. the editor data model and export path
