# Show Editor UX Plan

## Purpose

Define the editor workflow and controls needed to author shows efficiently.

This document focuses on:

- main window layout
- required buttons and tools
- preview behavior
- timeline interactions
- audio transport
- authoring workflow

It does not redefine the runtime show format. That remains in
`docs/show_system_spec.md`.

## Core UX Decisions

### 1. One main authoring window

The editor should feel like one integrated tool, not a set of disconnected
tabs.

The timeline and render preview must be visible in the same main working area.

Recommended layout:

- top: transport and project toolbar
- center-left: preview viewport
- center-right: inspector and library panels
- bottom: waveform and clip timeline

The preview can switch between 2D and 3D, but it should stay in the same
viewport area.

### 2. Preview is always part of the authoring loop

The user should be able to:

- scrub the timeline
- hear the audio
- see the current light result
- inspect the active target/section/effect

without switching to a different screen.

### 3. 3D preview must be navigable

The 3D preview should not be a static render.

It needs:

- orbit
- pan
- zoom
- camera reset
- fixed camera presets
- focus selected section/group

The preview should support ghosting or dimming unselected roles so the active
work stays readable.

### 4. The editor is clip-driven, not frame-driven

The primary authoring object is a clip:

- start time
- end time
- role or global lane
- target
- effect
- parameters
- blend/layer behavior

The workflow should avoid manual frame editing except for future special cases.

## Main Window Layout

## Top Toolbar

The top toolbar should contain:

- `New Project`
- `Open`
- `Save`
- `Save As`
- `Export`
- `Set Audio`
- `Load Layout`
- `Undo`
- `Redo`
- current show title
- current timeline time

It should also include global editing toggles:

- `Snap`
- `Snap To Beats`
- `Snap To Bars`
- `Snap To Grid`
- `Solo Role`
- `Loop Selection`

## Transport Bar

The transport controls should be always visible near the top.

Required buttons:

- `Play`
- `Pause`
- `Stop`
- `Restart`
- `Previous Marker`
- `Next Marker`
- `Step Back`
- `Step Forward`
- `Set Loop In`
- `Set Loop Out`
- `Clear Loop`

Required readouts:

- current time in `mm:ss.mmm`
- project duration
- zoom scale
- playback speed, if supported later

Useful later:

- `metronome`
- `beat count-in`
- `play selected range only`

## Preview Viewport

The preview viewport is the main visual workspace.

Required controls:

- `2D`
- `3D`
- `Front`
- `Back`
- `Left`
- `Right`
- `Reset Camera`
- `Focus Selection`
- `Show LED Dots`
- `Show Section Labels`
- `Show Section Bounds`
- `Show Body Wireframe`
- `Ghost Other Roles`
- `Solo Active Role`

3D mouse controls:

- left drag: orbit
- middle drag or shift-left drag: pan
- wheel: zoom
- double click target: focus

2D mouse controls:

- drag: pan
- wheel: zoom
- click: select section or LED cluster

The viewport should support:

- one-role preview
- all-roles preview
- split or ghosted role comparison

## Timeline Area

The bottom timeline area should combine:

- waveform
- markers
- lane headers
- clips
- playhead

Required lanes:

- `Global`
- `Role A`
- `Role B`
- `Role C`

Each role lane should support multiple layers.

Required timeline controls:

- horizontal zoom
- vertical zoom
- scroll
- playhead drag
- marker add/remove
- clip select
- clip drag
- clip resize left
- clip resize right
- clip duplicate
- clip split at playhead
- clip mute/disable
- clip delete

Required context actions on clips:

- `Edit Effect`
- `Change Target`
- `Change Role`
- `Duplicate`
- `Copy`
- `Paste`
- `Split`
- `Delete`

## Inspector Panel

The right-side inspector should edit the selected clip or selected target.

When a clip is selected, show:

- clip name
- start time
- end time
- duration
- lane
- layer
- role or global
- target kind
- target id
- blend mode
- effect type
- effect parameters

Effect parameter controls should be direct and fast:

- color pickers
- numeric sliders
- dropdowns
- toggles

## Preset / Library Panel

The editor should have a reusable preset area.

Required content:

- effect presets
- saved color palettes
- reusable target groups
- reusable animation templates

Required actions:

- `Save Preset`
- `Apply Preset`
- `Duplicate Preset`
- `Delete Preset`

This matters because the user wants a long-term workflow and repeated patterns.

## Layout / Target Panel

The editor also needs a panel for target selection and layout context.

Required content:

- canonical section list
- canonical group list
- role visibility toggles
- selected layout file/profile

Useful controls:

- `Select Section`
- `Select Group`
- `Highlight In Viewport`
- `Show LED Count`
- `Show Direction`

Later, when layout editing grows:

- reverse section
- move control point
- edit path
- assign LED count

## Must-Have Authoring Actions

These are the actions that should feel fast enough for real use.

### Clip creation

- add a clip by double-clicking a lane
- add a clip from toolbar
- drag a preset onto a lane
- create clip at playhead

### Clip editing

- drag clip in time
- resize clip edges
- duplicate by modifier-drag
- copy/paste
- change role/layer/target quickly

### Target editing

- click a section in the viewport to retarget a selected clip
- click a group in the list to retarget
- preview target highlight before applying

### Time navigation

- drag playhead
- click waveform to seek
- use keyboard to step left/right
- jump to next/previous marker
- jump to clip boundaries

## Keyboard Shortcuts

These matter for efficiency and should be built early.

Recommended first set:

- `Space`: play/pause
- `S`: stop
- `Home`: go to start
- `Left` / `Right`: nudge playhead
- `Shift+Left` / `Shift+Right`: larger nudge
- `Ctrl+S`: save
- `Ctrl+Z`: undo
- `Ctrl+Shift+Z`: redo
- `Delete`: delete selected clip
- `D`: duplicate clip
- `M`: add marker
- `L`: set loop from selection
- `F`: focus selected target in viewport
- `1`: front camera
- `2`: back camera
- `3`: left camera
- `4`: right camera

## Effect Authoring Surface

The first useful effect list should be available directly from an `Add Clip`
menu or preset browser.

Recommended first-class effect entries:

- `Solid`
- `Blink`
- `Fade`
- `Pulse`
- `Strobe`
- `Sweep`
- `Mirror Sweep`
- `Chase`
- `Gradient`
- `Palette Cycle`

Each effect should have a small, predictable parameter set.

The UI should prefer clarity over exposing every possible option at once.

## Audio Workflow

The editor must support local playback of the chosen track.

Required audio behavior:

- load local audio file
- show waveform
- play and pause from playhead
- seek from timeline
- loop selected range
- keep preview synced to audio time

For the first useful pass, WAV support is enough.

Later, import can accept more source formats and convert them to a working WAV.

## Viewport and Timeline Relationship

The timeline and viewport should stay synchronized at all times.

When the user:

- scrubs the timeline
- plays audio
- selects a clip
- selects a target in the viewport

the rest of the interface should react immediately.

The preview should not live in a separate tab during normal use.

## Recommended Build Order

### Phase 1

- integrated viewport + timeline layout
- audio play/pause/seek
- waveform display
- real clip creation
- clip drag/resize
- inspector for `Solid`, `Blink`, `Fade`, `Sweep`

### Phase 2

- orbitable 3D camera
- target picking in viewport
- presets and reusable effect templates
- markers and loop ranges
- undo/redo support

### Phase 3

- beat/bar helper tracks
- richer effect catalog
- layout editing tools
- packaged audio conversion pipeline

## Immediate Decisions For Implementation

The next coding step should assume:

- the preview stays visible while editing the timeline
- the 3D view becomes navigable, not a static tab
- audio playback is a first-class transport feature
- clip editing is the next real authoring feature after transport

