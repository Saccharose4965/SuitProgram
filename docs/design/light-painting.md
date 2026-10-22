# How light painting works, and why

This is the first sample of the new explanation style. It describes current
behaviour, then proposes where that behaviour belongs in the agreed architecture.
It does not claim that the proposed split is already implemented.

## We measure a direction, not an arm position

The desired gesture is simple: hold the forearm folded horizontally, calibrate,
then unfold and point at another suit to paint it.

But the forearm sensor does not know where either person is. It measures rotation
and acceleration, not the shoulder position, distance to the target or which
person is in front. Trying to reconstruct all of that from this sensor would
promise information we do not have.

The implemented model is therefore a pointing ray aimed at an imaginary rectangle
in front of the painter. The other suit maps that rectangle onto its own LEDs.
It feels like pointing at a person when the people stand approximately where the
model assumes. It is not spatial tracking of that person.

## First establish what “forward” means

The gyro tracks changes in orientation; gravity from the accelerometer helps
correct tilt. There is no absolute compass heading in this pipeline. We need
the user's calibration to establish the horizontal reference.

In the folded reference, the light comes out along the sensor's finger axis
(+Y), while the target is towards -X. Unfolding rotates that finger direction
towards the target. This is device pointing, not the inverse rotation used to
make scenery move around a camera.

For painting, calibration keeps only the horizontal heading. It deliberately
does not declare the current wrist roll or arm tilt to be the new vertical.
Otherwise a slightly rolled forearm at calibration would tilt the entire target:
an intended horizontal gesture could paint diagonally. Gravity remains “up”.
If the forearm points nearly vertically, its horizontal direction is ambiguous,
so calibration is rejected rather than inventing a heading.

Left/right selection mirrors the lateral mapping. It does not discover which
arm is wearing the sensor or read another sensor. It assumes the supported
mounting. Changing arms invalidates calibration because the reference no longer
means the same thing.

Arm Pose and Synth currently use full-orientation calibration instead. That
removes the complete starting orientation. This is a real difference to preserve
or explicitly revisit—not something to erase merely to make APIs look uniform.
The Arm Pose display also guesses the upper-arm position: only forearm rotation
is measured, so its skeleton is an illustration, not ground truth.

## Turn the direction into a place on the suit

We agreed on a virtual target 1.5 m ahead, 0.6 m wide and 1.5 m tall. Intersecting
the pointing ray with that plane gives a horizontal and vertical position.
Pointing away, almost parallel to the plane, or outside the rectangle means
“not on the canvas”, not “paint the nearest edge”.

Those dimensions determine sensitivity. A farther virtual plane makes the same
angular movement travel farther across the canvas. Actual distance is not
measured, so changing how far apart people stand changes how natural aiming feels.

The receiver anchors the rectangle to the suit's waist/body centreline, not the
centre of whichever LEDs happen to exist. Otherwise adding legs would shift the
whole painting reference even if the torso had not moved. Eligible front/leg
sections and recognized arm sections receive strokes; this is a layout rule,
not visibility detection. It does not know whether a real arm is behind the body.

## Only paint suits that volunteered

Opening the app does not overwrite anyone's animation. The user chooses Painter
or Canvas. Canvas must acquire LED ownership; if another feature owns the output,
entry fails. Leaving Canvas clears its output and releases that ownership.

The painter broadcasts rather than selecting a named target. Every opted-in
Canvas can receive the same stroke. There is no directional radio targeting:
the pointing calculation chooses a position within the virtual rectangle, not
which suit should hear the message.

The receiver accepts one painter at a time, retaining that painter while its
messages continue. This avoids two sources alternating the brush position and
drawing a line between unrelated people's gestures. The current lease is two
seconds, and even inactive messages renew it. Consequently, putting down the
brush without leaving Painter can still prevent another painter taking over.
That is current behaviour, not a claim that this interaction is ideal.

## Send gestures, not LED frames

The painter sends position, colour and whether painting is active, at most
20 times per second. Each receiving suit renders those instructions against its
own layout. This keeps radio traffic independent of the number of LEDs and avoids
requiring the painter to know every target's wiring.

Messages distinguish a session, an ordered sample and a stroke. These answer
different questions: “is this the same run?”, “have I already received this?”
and “should these positions be joined?”. Calibration, colour changes and
paint toggles start a new stroke so they do not draw an accidental connecting line.

There is no acknowledgement/retry for each brush update. The implementation
favours current gestures over guaranteed delivery of old positions: a delayed
stroke is often worse than a missed point. A small receive queue keeps network
callbacks short; stale queued data is discarded. Local send success means the
send was accepted, not that a canvas definitely displayed it.

## Make motion continuous without inventing long jumps

Drawing only at received positions would leave dots when the hand moves quickly.
The receiver therefore paints the segment between consecutive points—but only
for the same painter, session and stroke, with a short enough gap. After an
interruption it paints a new dab instead of sweeping through everything between
an old point and the current one.

LEDs near that segment receive more colour than LEDs near the brush edge. The
brush calculation uses the rectangle's normalized coordinates, so the current
physical footprint is taller than it is wide. A circular-looking mathematical
brush is not necessarily circular on a differently scaled suit; that is an
existing consequence worth understanding before tuning it.

Paint fades continuously. Using elapsed time rather than subtracting a fixed
amount per frame keeps the intended fade similar when frame rate varies.
New paint takes the brighter value of each colour channel rather than summing
brightness without limit. Repeated painting refreshes a trail; overlapping
colours can combine. These are fading light trails, not opaque paint layers.

## What happens when data or resources disappear?

A missing/stale orientation sample makes the painter send inactive updates;
it does not continue painting from the last known aim. The paint toggle can
remain armed, so valid aim can resume painting when data returns. Inactive
updates break the receiving stroke's continuity.

If the radio disappears, the receiver keeps fading the existing trail. It does
not need a reliable final “stop” packet to avoid permanent light. When a later
point arrives after too long, it begins a new dab. Old or duplicate samples are
not allowed to repaint history.

FFT is suspended while the Paint app is open, including its menu, to avoid
competing work. On exit the ordinary resource logic can resume pending FFT use.
Painting does not own that detector and should not decide its entire lifecycle.
Likewise, it shares the orientation service rather than starting a second sensor
reader. The exact resource decision currently lives in UI-named support code;
that placement is one of the architectural problems, not the intended model.

## How this becomes apps, services and drivers

The **Paint app** should own controls and presentation: choose a role/arm,
calibrate, enable paint, select colour, clear, show status, leave.

The **Paint service** should own the session and the decisions above: validate
aim, project it, send/accept strokes, handle painter ownership, join samples,
fade trails and request LED output. Being a service does not make Canvas always
active. The app still explicitly starts and stops it.

Other **services** supply orientation/arm maths, suit geometry, connectivity
and resource policy. Geometry belongs to the suit, not specifically to Shows.
The paint maths remain usable in a host test without a display or radio.

The **drivers** read the sensor and transmit pixels/packets through hardware.
They should not know what a brush, a painter or a show is.

Most of this behaviour already exists. The cleanup is primarily to give it
clear owners and boundaries, preserving the coordinate mapping and timing—not
to replace a working brush with a new algorithm.

## What we know, and what still needs checking

The existing host test checks coordinate transformations, calibration, stale and
duplicate packets, stroke handling, fading and ownership against fake hardware.
It does not establish physical mounting, radio reliability or natural aiming.
Those need the two-suit test: calibrate folded, unfold towards the waist, trace
horizontal/vertical strokes, roll the wrist, interrupt the link, and leave Canvas.

Implementation references: `main/app_light_paint.c`,
`components/orientation/arm_pose.c`, `main/tests/test_light_paint.py`.
The older [technical review](../documentation/orientation-and-paint.md) remains
available for exact constants and mechanics; it is not the preferred introduction.
