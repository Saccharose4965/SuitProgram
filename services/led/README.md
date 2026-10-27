Pending review

# LED output and beat service

This service is the shared logical-frame boundary for every suit renderer. It
maps canonical LED indices to the two wired strips, arbitrates which renderer
may publish, coalesces pending frames, limits wire cadence and maintains the
FFT-triggered beat animation state. Electrical transmission is handled by the
[LED strip driver](../../drivers/led_strip/README.md); persisted wiring is
handled by [LED layout](../led_layout/README.md).

## Logical frames and ownership

Callers produce RGB triples in canonical suit order. The service snapshots the
layout and caches each logical index's strip and physical index. Before a wire
write it clears both physical buffers, copies valid mapped pixels and derives
the actual length required on each strip. Changing the layout invalidates the
cache through a full configuration comparison.

Base animations own output by default. Show and Paint claim an explicit owner
token, preventing the beat worker or another high-level renderer from replacing
their frames. Releasing the token returns control to base animations. Ownership
protects frame submission; each renderer remains responsible for stopping or
pausing its own worker when appropriate.

## Submission and wire cadence

Submission copies the newest logical frame into one pending buffer and wakes a
core-1 priority-5 worker. Multiple notifications collapse to the most recent
frame. The worker copies that pending frame to a separate shadow, leaving
producers free while layout mapping and physical transmission block.

Cadence is bounded by both 60 frames per second and the estimated WS2815 wire
time: 30 microseconds per pixel plus latch time. The larger bound wins. This
keeps frame production from building a queue the strip cannot physically send.
The driver call is synchronous, so mapped physical buffers remain immutable
until both strips finish.

## Output-wide audio brightness

Optional FFT volume scaling is applied once when a logical frame is accepted.
The raw volume is clamped to zero through one. Full, mid and high ranges select
different minimum brightness floors; values above the noise threshold expand
linearly to full scale. This modulation affects the submitted renderer,
including non-beat owners, while enabled.

## Beat renderer

Beat effects run in a core-1 priority-4 worker at roughly 62.5 updates per
second. A trigger updates a smoothed beat period and creates state for the
selected effect. The renderer combines short flashes, soft waves, sparks,
linear pulses, comets and geometry-based ring/plane waves into one frame.
Geometry effects cache posed suit coordinates and layout statistics. Their
distance calculations use the same suit coordinate service as Shows and Paint.

Primary/secondary colors can alternate or follow a palette clock. Random mode
chooses from the concrete effects every ten seconds and clears incompatible
state when it changes. Breathe and audio-energy modes remain continuously
active between beat triggers. Background options add low-level ring, plane or
comet fields before brighter moving features.

Colour, style, backgrounds, brightness and audio-response choices form one
protected configuration. A caller can replace that complete configuration in
one operation, and the beat worker copies it once before constructing a frame.
Every helper used for that frame reads the immutable copy, so one frame cannot
combine values from two presets. The older narrow setters remain available for
single-setting changes and update the same protected configuration.

## Hardware test pattern

The blocking diagnostic claims base output, displays red, green, blue and
white prefixes, walks a white pixel, then clears the active layout. It exercises
mapping and both physical outputs through the normal driver boundary.

The host ownership test verifies that a newly submitted logical frame cannot
modify buffers already handed to the driver. Hardware acceptance still needs
every beat mode, Show/Paint ownership transitions, live layout edits and both
strip outputs.
