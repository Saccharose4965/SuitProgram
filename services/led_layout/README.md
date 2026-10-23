Pending review

# LED layout service

## Owned model

The service owns the costume's wiring description and raw measured geometry.
A layout contains two physical strips and an ordered list of named sections.
Each section records its strip, LED count, wiring direction and either a
polyline or planar arc in centimetres. Section names are stable identities used
by shows, effects and pose rules.

Logical LEDs follow canonical section order. Finalization derives each
section's logical start, its physical start within its strip, total LED count
and strip lengths. Reversal changes the logical-to-physical index within one
section. It does not reverse geometry: geometry always follows logical section
order, which lets animation direction remain meaningful when wiring direction
changes.

The service exposes raw authoring coordinates. `suit_geometry` applies the
worn body pose and is the coordinate source for spatial animation, Paint and
Shows.

## Geometry sampling

Polyline LEDs are spaced by accumulated path length, giving equal physical
spacing across unequal source segments. Open paths include both endpoints. A
closed 360-degree arc divides by LED count so its final LED does not duplicate
the first. A one-LED section samples its midpoint.

The compiled measurements share the editor coordinate convention: +X points
toward the wearer's left as viewed from the front, +Y points down and +Z points
forward.

## Default and leg insertion

The default contains 33 named sections ordered front, legs, back and arms for
each side. The twelve leg sections carry measured geometry and zero LEDs until
the physical legs exist. Zero-length sections reserve no address, so the back
remains wired directly after the front.

When an older chestplate file is loaded, missing leg sections are inserted only
when its matching front and back belt sections exist. Their zero counts retain
every existing physical address. Saving then persists the expanded model.

The output ceiling is 721 logical LEDs. The current chestplate and arms already
use that count. Installing legs therefore requires deliberate output-capacity
and power review as well as entering their real counts.

## SD representation and upgrades

`led_layout.txt` is a line-oriented key/value file. It stores profile and strip
names followed by every section's wiring fields and geometry. Unknown keys are
ignored. Counts, indexes and floats are range checked; non-finite coordinates
and angles are rejected. Finalization repairs incomplete geometry and clamps
the model to the supported section and pixel limits.

Loading also upgrades historical section names, placeholder-scale geometry,
the former left/right handedness, old arm measurements and ring sweep
direction. These upgrades are selected from recognizable stored values so
custom measured geometry is retained.

Saving writes a temporary file, flushes and syncs it, then renames it. FAT may
reject replacement of an existing name, in which case the service removes the
old entry and retries the rename. A power loss in that fallback interval can
leave only the temporary file; recovery of that case remains future work.

## Runtime ownership

A static mutex protects initialization, snapshots, mapping and live edits.
The large current layout and parse workspace live in external RAM. Consumers
take a complete snapshot before multi-LED work, keeping one coherent layout
through a rendered frame and avoiding a lock for every LED.

Initialization starts with the compiled default. It then mounts the SD card,
loads the stored file, or persists the default when the file is absent. The
default remains usable when mounting fails and the mount error is returned to
the caller.

Live setters finalize all derived indexes before releasing the mutex. Length
edits are capped by remaining pixel capacity. Persistence is explicit, so an
editor can preview several changes and commit them together.

## Verification

`python3 services/led_layout/tests/test_legs.py` compiles the real service with
host stubs and sanitizers. It checks the 721-pixel default, legacy leg
insertion, address preservation, save/load geometry, adding and removing leg
LEDs, and rejection of overflowing or non-finite stored values. Hardware review
must still walk the physical preview across every section and reboot after a
save.
