Pending review

# Suit geometry service

## Define the body

The service converts stored authoring geometry into LED positions on the worn
costume. Section identity selects the pose: torso panels acquire their front/back
depth, and arm sections acquire their profiles. Sections without special posing,
including the existing legs, retain their authored coordinates.

Positions are centimetres: +X is viewer-right (wearer's left), +Y is down and
+Z is forward. The waist centreline is (0,33.5,0). Torso panels lie at Z=+12 and
-12. Keeping this established convention preserves current authored show geometry.

Zero-length sections contribute no LEDs. Wiring order and LED counts remain
properties of the stored layout.

## Resolve and cache positions

`suit_geometry_get` finds a logical LED's section and its position along that
section, obtains its raw coordinates, then applies the pose. Invalid coordinates
are rejected. Callers already iterating sections can call the pose directly.

Spatial consumers cache these results when geometry changes. This keeps section
lookup and pose calculation outside their per-frame rendering loops.

Ring measurements come from the posed ring LEDs: their average gives the centre,
and the average planar distance gives a usable radius. That scalar approximates
the ring for radial effects when posing makes its shape elliptical.

## Derived mappings

Formation placement rotates and translates body coordinates into stage space.
Brush projection and screen projection likewise start from body coordinates.
Noise colouring has a dedicated flattened texture mapping. The service exposes
that mapping alongside the physical pose so its definition stays centralized.

The existing section IDs and `show_geometry_*` symbols remain stable. Their code
now belongs to this service; retaining the IDs preserves show package encoding.

## Visual consequences and verification

Normal spatial animations now use posed coordinates and ring measurements.
Their sweep bounds and radial appearance can change. Older show packages also
use the posed space and may require visual review or re-export.

Host tests compare logical lookup with direct posing and cover front/back/arm
positions, ring statistics and invalid input. Hardware acceptance should compare
a ring effect and a sweep across the torso and arms, followed by a current show
and a brush stroke.
