> Archived reference. Describes an earlier revision; current explanations live beside the code.

# Design reviews

One main explanation per app, service and driver. File-inspection checkmarks
are not design approval. States: **Draft → Feedback → Accepted**. Material
changes reopen review. Give feedback here, in the document, or in chat; only
explicit user approval closes a review.

| Document | State | Feedback |
| --- | --- | --- |
| [Architecture](../architecture-proposal.md) | Feedback | Three layers accepted; shared UI helpers requested. |
| [Light painting](../../services/paint/README.md) | Feedback | Writing style accepted; calibration implemented, hardware check pending. |
| [Orientation service](../../services/orientation/README.md) | Draft | Shared policy implemented; user review and hardware validation pending. |
| [Paint app](../../apps/paint/README.md) | Draft | Extracted controls/display; layout unchanged. |
| [Suit geometry](../../services/suit_geometry/README.md) | Draft | Canonical posed coordinates shared; normal animation/legacy-show visual changes need hardware review. |
| [Gyro calibration app](../../apps/gyro_calibration/README.md) | Draft | Stationary bias measurement and SD persistence; hardware check pending. |
| Earlier technical reviews | Reference | Not accepted design explanations. |
| [Input driver](../../drivers/input/README.md) | Draft | Decoding/activity boundary extracted; hardware regression pending. |
| [Input audio policy](../../services/input_audio_policy/README.md) | Draft | Existing suppression timing preserved and host-tested. |

## ORI-01: consistent calibration

User decision: calibration belongs in the orientation service and must preserve
gravity rather than absorbing initial tilt across orientation consumers.

Implemented: shared heading-only policy for Paint, Arm Pose, Synth and filter
recenter; 3D now uses coherent snapshots and a private reference. Fusion tuning
and raw measurements are unchanged. Source search found no other active recenter
callers in main/components. Physical mounting and user acceptance remain pending.

## UI-01: common layouts

User request: reduce repeated layout code; exact visual redesign undecided.
Proposed helpers under app infrastructure: viewport bounds, lists/scrolling,
title/status and button legends. Not a separate display app, extra UI task or
mandatory widget framework. Games and plots retain custom drawing. Compare
existing screens and choose a pilot before changing appearance or controls.

Status: recorded; implementation pending.

Sequencing update: user requested UI rework wait until visual code is in apps.
First moved screens are Arm Pose and 3D; no layout redesign in that move.
