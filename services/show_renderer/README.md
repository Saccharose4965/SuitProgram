Pending review

# Show renderer service

This service turns one validated role program and an absolute song time into an
RGB frame. It owns effect evaluation, targeting, formation-aware coordinates,
layer composition and brightness conversion. It has no radio, audio, SD-card,
button or OLED behavior.

## Configuration and coordinate cache

Configuration snapshots the current LED layout once. Each logical LED receives
its canonical section ID, position along that section and posed suit coordinate
from `suit_geometry`. The renderer also precomputes radial distance, flattened
front/back noise-plane coordinates, suit bounds, ring centre and world-space
coordinates for its A/B/C role. This makes every show effect use the same
coordinate model as the layout and Paint services while avoiding repeated
geometry work per frame.

Line formation offsets the three roles horizontally. Circle formation transforms
each role into the shared circular stage. Formation cues rebuild the cached
world points only when the playhead crosses a cue. Global axes and centres are
calculated across all three role placements, so an effect can travel continuously
through the group while each suit renders only its own LEDs.

The renderer object is large and is allocated in PSRAM when available. It holds
float composition colors and geometry for the maximum logical pixel count. Its
program pointer is borrowed; the package must remain loaded until the renderer
is destroyed or configured with another program.

## Finding active clips

Song time selects the role's bucket, whose range narrows the clip scan. Clips
outside their start/end interval are skipped. At most 96 active clips are kept
for one frame and are ordered by layer before evaluation. Equal-layer ordering
remains the exported table order, making the result deterministic on every
costume.

Targets are tested against the cached semantic section. `all`, one section and
the named groups share this path. A fade-in/fade-out envelope multiplies the
effect output at its edges.

## Time and parameter evaluation

Motion may use normalized clip progress, wall-clock seconds from clip start, or
the exported tempo and beat offset. Phase is wrapped to one cycle; a separate
cycle index seeds deterministic random choices. Thus Sparkle and randomized
axes change reproducibly at the same musical point on all suits.

Each automated parameter is evaluated once per clip, then reused for its LEDs.
Hold, linear, smooth, cycle and baked-amplitude modes share the exported record.
Blink is represented as Fill intensity automation, including high/low level,
duty, shape and optional decay at the end of the hold. Baked amplitude curves
carry editor-processed song data, so playback does not run a second audio
analysis pipeline.

Color can be a constant, a time sequence, a two-dimensional time/spatial grid,
or an evolving Perlin mapping over the flattened suit plane. Spatial mapping
uses the same axis normalization as the effect. Interpolation and looping are
encoded in the package, keeping costumes deterministic.

## Spatial effects and composition

Fill applies optional normalized X/Y/Z bounds and softness. The spatial family
computes a normalized coordinate, then specializes its mask for sweeps,
repeats, fanout, sparkles, orbs, ground energy and rays. Suit-space effects use
the local posed body. Stage-space effects use formation-transformed points.
Orb motion remains a straight path in the selected 2D plane even in circle
formation, preserving collision-like motion rather than orbiting a costume.

Each source is composed into a float RGBA buffer using Replace, Alpha, Add or
Max. Replace still writes a zero-valued blink phase so a higher layer can erase
lower layers intentionally. Uniform spans reuse an identical blend result;
packing similarly reuses RGB conversion across equal neighboring colors.
Brightness is applied once while converting the completed float buffer to the
LED service's RGB byte layout.

## Determinism and verification

All visible state is derived from the role program, role ID, layout and absolute
playhead. There is no accumulated frame simulation, so a skipped frame or late
join renders the current state directly. Hash-based randomness includes show
seed, role, LED, cycle and a purpose-specific salt.

`tests/test_uniform_render.py` extracts the production blend/packing functions
and compares optimized uniform rendering against per-pixel reference work under
sanitizers. Visual hardware acceptance is still required for every effect,
both formations, all role positions and layouts with legs.
