Pending review

# Standalone LED animation service

## Role

This service renders the named animations selected from the Animation screen
when the standalone backend is active. It owns their catalogue, common colour
settings, animation clock, geometry caches and one background render task. The
`led` component accepts the completed logical RGB frame and maps it to physical
outputs.

The catalogue index is part of runtime control and suit-to-suit Live Sync
state. Its order therefore remains stable. The final `random` entry is a
shuffle controller; it chooses from the animated entries and never selects
itself, off or static fill.

## Time model

The worker normally advances a free-running clock by its scheduled frame
period multiplied by the speed setting. A published sync clock replaces that
time when sync is enabled. This makes the rendered result a function of shared
show time, allowing suits to converge after delayed packets instead of
integrating independent frame histories.

Synchronized shuffle hashes the ten-second epoch into a mode and prevents the
same mode in consecutive epochs. Free-running shuffle uses local randomness
and resets its mode-local clock on each change.

The target period starts at 33 ms. It is raised when necessary to cover the
WS2812 wire time for the configured maximum strip plus a small margin. Disabled
rendering sleeps for 80 ms. The first disabled iteration clears the physical
frame once; later idle iterations perform no LED submission.

## Frame construction

Every frame starts black and is rendered into a fixed logical RGB buffer.
Linear modes use logical LED order. Spatial modes snapshot `led_layout`, cap the
working count to the layout and build coordinates through `suit_geometry`.
Cached positions, radial distance, angle, ring distance and helix terms avoid
repeating geometric work when the layout is unchanged.

The service derives layout bounds, centre and ring measurements once per cache
refresh. Plane, prism, ring, contour, orbit, aurora, vortex, helix, sweep and
fire render from those shared measurements. Mirror uses section placement.
Fill, breathe, scanner, energy, blink, chase, twinkle and glitch operate on
logical order.

Primary and secondary colours feed mono, duo and palette styles. Palette modes
generate deterministic rainbow, siren, arcade, inferno or biohazard sequences.
Optional plane/ring backgrounds and peak highlights are interpreted by the
effects that support them. Global brightness is applied after rendering so all
modes share the same output scale.

## Ownership and lifecycle

`led_modes_start` initializes LED output and creates the pinned render task once.
Enabling and disabling only changes service state; the task persists for the
boot. Shows and physical layout preview temporarily disable this renderer while
they own LED submission, then restore the previous state.

The sync clock pair has its own protected state. Colour, style, backgrounds,
brightness and speed form a second protected configuration. Whole presets are
published in one operation, and the render worker copies the configuration
once at the start of each frame. All rendering and final brightness work from
that immutable frame copy.

## Verification

The full firmware build checks every effect and dependency. Hardware review
should run every catalogue entry, compare two suits on a shared sync clock,
cross a shuffle boundary, vary speed and brightness, and confirm that disabling
the service clears once without disturbing the next LED owner.
