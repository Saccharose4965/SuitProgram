Pending review

# Paint service

Controls and display are described in [Paint app](../../apps/paint_app/README.md).

## Model the target

Painting uses a pointing ray and an imaginary rectangle 1.5 m ahead, 0.6 m wide
and 1.5 m tall. This gives a usable aiming model from forearm orientation alone.
The actual distance and relative position of the people must approximately fit
that model.

Calibration and direction conversion follow the
[orientation service](../orientation/README.md). Arm selection chooses the lateral mapping;
changing it invalidates calibration and ends the current stroke.

The ray intersects the rectangle to produce brush coordinates. Backward,
near-parallel or out-of-bounds rays deactivate the outgoing mark. Distance
controls sensitivity: a farther virtual target makes an equal angular movement
travel farther across the canvas.

The receiver maps [posed LED positions](../suit_geometry/README.md) onto the rectangle,
anchored at the waist centreline. This fixed anatomical anchor keeps the torso's
brush mapping stable when leg sections are added. Eligible sections include the
front, front legs and recognized arms. Eligibility follows section identity;
physical arm occlusion is outside the model.

## Enter and leave a session

Opening creates an idle session with a new session ID. Painter begins uncalibrated
and disarmed. Canvas prepares its geometry cache and claims LED output; failure
keeps the session idle and reports the reason.

Leaving Canvas clears its output, releases ownership and disables reception.
Buffers remain allocated for reuse. Left/right preference persists between
openings; calibration and active painting reset.

## Deliver gestures

Painter broadcasts brush position, colour and activity at most 20 times per
second. Each Canvas renders against its own layout, keeping radio traffic
independent of LED count. Every opted-in receiver can display the broadcast.

Packets carry three distinct identities:

- Session identifies a run.
- Sequence orders samples and detects duplicates.
- Stroke determines whether consecutive points should connect.

Calibration, colour changes and brush toggles advance the stroke identity.
This prevents connecting gestures across a change in meaning.

Brush updates use unacknowledged delivery. Prioritizing recent positions keeps
interaction responsive under loss. The receive callback validates packets and
adds them to a bounded queue; the regular tick processes them. Full queues drop
incoming samples, and samples older than 250 ms are discarded.

## Keep one painter's stroke coherent

A Canvas retains the current painter for two seconds after its latest accepted
message. During that lease, other painters are ignored. This prevents alternating
sources from drawing connections between unrelated gestures.

Inactive messages renew the lease too. A painter can therefore keep the Canvas
occupied while the brush is disarmed. Session/sequence checks reject duplicate
or out-of-order samples.

Two points connect only when they belong to the same painter, session and stroke,
with less than 200 ms between receptions. Otherwise the new point produces a dab.
The gap rule limits invented motion after interruption.

## Render and fade

Each eligible LED receives strength according to its distance from the stroke
segment, decreasing to zero at the brush edge. Coordinates are normalized to the
target rectangle, so the current brush footprint is physically taller than wide.

New colour takes the maximum of the existing and incoming value for each channel.
Repeated strokes refresh brightness; different colours can combine.

Every tick applies exponential fading using elapsed time. This keeps the fade
duration consistent as frame rate varies. Once incoming updates stop, the existing
trail continues fading naturally.

## Handle interrupted input

A stale orientation sample makes outgoing updates inactive. Brush activation
remains armed, allowing painting to resume when valid aim returns. Inactive
updates break segment continuity at the receiver.

After radio interruption, the gap rule starts a new dab. Leaving the receiver
ends its session through the same cleanup path as normal role changes.

Commands, status reads and ticks share one caller task. Reception may run
concurrently and only queues data. Status includes role, arm, calibration,
aim, brush activity, colour, peer freshness and error. Successful sends currently
clear earlier error status; this can shorten the visibility of a calibration error.

## Verification

Host tests exercise the service and app with fake sensor, radio and LED output:
calibration, target bounds, packet validation, fading, stale/duplicate handling,
painter exclusion, ownership and cleanup.

Hardware acceptance requires two suits: trace horizontal and vertical strokes,
interrupt the connection, then leave Canvas. Confirm that aiming, fading and
ownership behave as described.
