# Leg placeholders

Each strip now follows front → leg → back → arm. The six leg sections, in
order, are thigh_front, shin_f_in, shin_f_out, thigh_back, shin_b_in, shin_b_out.
The right strip mirrors the left. Leg counts default to **zero**: the back is
still wired directly after the front and no physical addresses are reserved.
The eventual within-leg strip directions can be adjusted independently when
the actual legs are built; do not guess counts before then.

Existing chestplate SD layouts acquire missing zero-length leg sections in
memory when loaded. Existing lengths, reversal, geometry and physical pixel
addresses are preserved. Saving from the layout editor persists the expanded
layout. Reset is not required. Custom layouts without the matching front/back
belt sections are not automatically expanded.

The section capacity is 33. Geometry matches the existing ShowEditor leg
polylines, with stable show section IDs. Zero sections consume neither pixels
nor render work. The layout editor supports returning a section length to zero.
Large parsing/UI snapshots no longer occupy task stacks; show setup allocates
its snapshot in PSRAM only while configuring, not per frame.

The total output limit remains **721 LEDs**. The current default already uses
that budget for chestplate/arms. Before installing legs, confirm actual counts
and deliberately resize the driver/output budget if needed; this patch does
not increase transmitted strip lengths or reserve nonexistent LEDs.

Regression: `python3 components/led_layout/tests/test_legs.py` verifies all 721
existing pixel addresses survive migration, zero counts round-trip through SD,
and adding/removing a leg's LEDs shifts only the following physical sections.
Editor/firmware spatial rendering parity is also tested separately.
