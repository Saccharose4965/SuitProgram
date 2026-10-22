# Orientation service, calibrated arm math and Light Paint

File-level review of the six files named in this document. For operating
instructions see [Light painting](../light-painting.md). The fusion implementation,
MPU driver, Arm Pose renderer, synth, LED driver and link implementation have
separate responsibilities and are **not** checked off by this review. Fusion and
MPU acquisition now have a separate [filter/driver review](imu-filter-and-driver.md).

## `components/orientation/CMakeLists.txt`

Registers `orientation.c`, `orientation_service.c` and `arm_pose.c`, exports
`include`, and declares public dependencies on `hw` and `mpu6500`. The component
combines fusion, the continuously running service and pure calibrated-vector
math. Public dependencies allow its exported headers to include driver types.
There are no conditional source selections in this file; linking an app does
not by itself start the service. Host math tests compile `arm_pose.c` directly
with stubs instead of starting ESP-IDF or the hardware service.

## `components/orientation/include/orientation_service.h`

Public C/C++ interface to the singleton service:

| API | Contract |
| --- | --- |
| `orientation_service_start` | Initialise the sensor/sampler/filter once; return an ESP error on failure. Call from serialised startup, not concurrent callers. |
| `orientation_service_ready` | Started and filter ready; does not check sample age. |
| `orientation_service_snapshot` | Require both output pointers; atomically copy absolute body-to-world quaternion and last published sample timestamp in microseconds. Return false when timestamp is zero. |
| `orientation_service_ctx` | Return a borrowed mutable pointer to service-owned filter state, even before startup. Not a coherent snapshot. |
| `orientation_service_read_sample` | Copy latest raw sample through the MPU driver; false for null output, service not started, or zero sample timestamp. |
| `orientation_service_recenter` | Change the shared filter reference, not the physical sensor frame or each app's local calibration. |

The header includes ESP error, MPU and orientation types and exposes no stop
or destructor API. Snapshot consumers own their copied values and must enforce
freshness themselves; the service does not guarantee ongoing sensor health.

## `components/orientation/orientation_service.c`

### Startup and ownership

Static state owns the MPU object, filter, task handle and published snapshot.
The shell starts it during sensor setup; Fluid can also request startup. A
successful repeated `start` returns immediately via `s_started`. This boolean
is not a mutex: concurrent initialisation is not supported.

Startup initialises shared SPI2 and drives chip selects idle high. It tries
`PIN_CS_IMU1` at 8 MHz, then 1 MHz, using SPI mode 0 and queue size 4. A device-add
failure skips that attempt. MPU initialisation failure removes that SPI device
before retrying. This service does not own or stop unrelated SPI devices.

After MPU initialisation, it initialises the orientation filter with beta=0.05
and creates `imu_orient`, stack argument 3072, priority 2, pinned to core 1.
Task creation failure removes the device and returns `ESP_ERR_NO_MEM`.
The MPU sampler is requested at 200 Hz, priority 2, core 1, with stack parameter
zero (driver-selected default). Sampler failure deletes the orientation task,
removes the SPI device and tries the next speed. Exhausting attempts returns
`ESP_ERR_NOT_FOUND`; the exact driver failure is not returned through this path.
Success retains the device and tasks for process lifetime. Core 1 is a fixed
dual-core target assumption, not a portable single-core configuration.

The fallback speed is visibly a robustness provision. The historical tuning
reason for beta=0.05, priority 2 and the exact stack size is not established by
this file; do not interpret them as analytically optimal values.

### Continuous update and publication

`orientation_task` uses `vTaskDelayUntil` with a nominal 5 ms period, clamped to
at least one scheduler tick. Thus 200 Hz is a request; FreeRTOS tick granularity,
SPI contention and scheduling can lower actual frequency. The service reads
the latest driver sample, not a queue of all samples. Zero/repeated timestamps
are skipped; intermediate samples can be missed if the task runs late.

Fresh samples pass acceleration in g, angular velocity in degrees/second and
the sensor timestamp to `imu_orientation_update`. Afterward, a short critical
section publishes `q_BW` together with that timestamp, or timestamp zero while
the filter is not ready. This pairs orientation with its sample time, avoiding
cross-core readers seeing torn publications. However, a ready filter rejecting
an interval above 50 ms still gets its unchanged quaternion published with the
new sample timestamp; publication freshness is not proof of integration.
The loop never exits.

The snapshot API uses the same lock and reports availability by nonzero time.
It does not require `s_started` or test age; a stalled sampler can leave the last
snapshot available indefinitely. Paint/Arm Pose/Synth currently apply their own
250 ms freshness checks. Raw sample reads rely on the MPU driver's snapshot
function. `ready`, `ctx` and `recenter` access shared filter state without the
snapshot lock; the legacy pointer/recenter path is not made thread-safe by the
new snapshot API. Recentring does not rewrite the absolute published `q_BW`.

No watchdog recovery, sensor restart, dynamic core selection or app-lifetime
shutdown exists here. Hardware tests must exercise SPI contention, initial
sensor absence, stale samples and long-running yaw drift. Math-only host tests
do not establish service timing or concurrency safety for legacy pointer users.

## `components/orientation/include/arm_pose.h`

Defines a three-float vector and a pose consisting of a stored zero quaternion
plus a calibrated flag. Callers own these objects; zero-initialise before use.
The API is allocation-free and does no I/O, locking, scheduling or persistence.
Do not mutate a pose concurrently with its readers without external protection.

- Full calibration stores a normalised sensor quaternion.
- Heading calibration stores only horizontal heading, preserving world vertical.
- Vector conversion accepts an arbitrary local MPU vector; direction conversion
  is the convenience case for finger-axis +Y.
- Canvas projection maps a direction to normalised target coordinates, returning
  false outside the target. Outputs should only be used when true is returned.
- Brush distance returns point-to-segment distance in the supplied coordinate
  units, normally the normalised paint plane.

`left` selects lateral mirroring; it does not select a second IMU. Vectors are
dimensionless directions until projected into the assumed target plane. Full
calibration is used by Arm Pose/Synth; Paint uses heading-only calibration.

## `components/orientation/arm_pose.c`

### Quaternion and calibration rules

`mul` implements Hamilton quaternion multiplication in w,x,y,z order; `conjq`
negates its vector part. `normalize` divides by the Euclidean norm, rejecting
non-finite norms and norms below 1e-6. For normalised rotations, conjugation is
the inverse. Both calibration functions check the pose pointer and quaternion.
Failure leaves any previous calibration intact; callers must decide whether
to clear it. The helper does not silently erase a working reference.

`arm_pose_calibrate` stores the entire normalised orientation. Heading-only
calibration first rotates raw +Y through the absolute quaternion, projects that
finger axis onto world XY, and rejects horizontal magnitude below 0.2 because
heading is unreliable close to vertical. It takes
`heading = atan2(beam.y, beam.x) - pi/2` and stores the pure Z quaternion for that
heading. Rotating around local +Y does not change the finger axis, so calibration
roll cannot tilt the canvas. World Z is assumed gravity-up from upstream fusion;
this helper does not estimate gravity or correct the fusion itself.

### Direction and target mapping

`arm_pose_vector` checks calibrated state/output pointer and normalises the
current sensor quaternion. It computes `relative = conjugate(zero) * sensor`,
then `v = relative * local * conjugate(relative)`. The output is
`(left ? v.y : -v.y, v.z, -v.x)`: lateral, up, forward. It does not validate the
local vector or re-normalise stored zero; these must come from valid caller
state. `arm_pose_direction` supplies local `(0,1,0)`.

This is device pointing, not the inverse transform used to move a scene around
a camera. At identity calibration, +Y points sideways; +90 degrees around MPU Z
unfolds it toward -X (forward). Pure forearm +Y roll changes hand orientation,
not beam direction. Mirroring the right-arm lateral axis assumes compatible
physical mounting; it is not a complete arbitrary mounting calibration.

`arm_pose_canvas` requires finite direction components and forward >=0.15. It
projects to `u = 1.5*x/(0.30*z)`, `v = -1.5*y/(0.75*z)` and accepts only |u|,|v|
<=1. The assumed distance is 1.5 m, width 0.6 m and height 1.5 m; +v is downward.
The forward threshold avoids backward/near-parallel rays. The app maps target
zero to the receiving suit's waist. There is no measured distance, painter
height, absolute position, torso-turn compensation or identification of a person.

`arm_brush_distance` projects onto a segment, clamps its interpolation parameter
to [0,1], then returns Euclidean distance. Squared segment length <=1e-9 is
treated as a point. Inputs are assumed finite. This bridges successive brush
positions without generating intermediate packets or processing 3D ray meshes.

## `main/app_light_paint.c`

### Shell lifecycle and controls

Registered as `light_paint` under Simulations, with init/deinit/input/tick/draw
callbacks and no shell legend. `LIGHT_PAINT_LEGEND` is defined but the registry
does not use it. All modes suspend background FFT through the existing gate.
Init creates an eight-item static FreeRTOS queue once, resets to menu/unarmed,
chooses cyan, invalidates local calibration and creates a random nonzero session.
Left/right selection is static RAM state retained across openings, default left
after boot; no settings file is written.

Menu A/B select Painter, Canvas, Arm, Back; D activates. Arm toggle invalidates
calibration and advances the stroke ID. Painter B attempts heading calibration
from a snapshot newer than 250 ms, always disarming paint and advancing stroke;
C changes among five colours and advances stroke; D toggles paint only if
calibrated and advances stroke. A returns to menu. Calibration failure can leave
the previous pose valid, as specified by the math API. Canvas B clears colours
and breaks stroke continuity; A returns. Input only handles press events.

`leave_canvas` disables receive acceptance under the queue-state lock. When
leaving Canvas it submits a black owned frame and releases paint's LED claim;
it then disarms painting. Deinit also clears the FFT gate, letting normal service
resume if otherwise requested/allowed. The static queue and PSRAM buffers remain
allocated. No file is saved, no radio connection is torn down, and no explicit
final inactive packet is sent; receivers rely on their fade/timeout behaviour.

### Geometry preparation and ownership

`prepare_canvas` allocates one layout snapshot in PSRAM, builds cached positions,
then frees the snapshot. Static PSRAM arrays hold XY, eligibility, float RGB and
packed output for `LED_LAYOUT_MAX_PIXELS`. There is no geometry allocation during
ticks. Eligible names start `front_`, contain `thigh_front`, `shin_f_` or `arm`,
and must resolve to a known show section. Thus all recognised arm LEDs are
eligible, not a measured visibility/occlusion mask. Each valid LED is posed by
show geometry using its normalised section index (0.5 for a one-LED section).
Invalid indices/lookups and non-finite XY are skipped.

Total count must be positive and within capacity, and valid geometry must span
both axes. Geometry bounds are used only for validation, **not** normalisation.
Posed centimetres map as `x/30`, `(y-33.5)/75`, fixing zero at the waist/belt
centreline regardless of missing legs. Coordinates are not clamped; off-plane
LEDs receive paint only if within brush range of an accepted stroke.

Paint claims LED output only after preparation; failure leaves the app in menu
with an error instead of overwriting a show. Success resets queued input and
stroke history, records entry time and enables receive acceptance. Clear does
not release ownership; normal animation resumes only after leaving Canvas.

### Wire format, queuing and painter

`paint_packet_t` is the native C struct: version, active, colour, reserved (four
bytes), three uint32 fields session/sequence/stroke, then signed int16 x/y.
On the current ABI this is 20 bytes; it is not explicit endian-independent
serialisation. Coordinates encode normalised [-1,1] as [-10000,10000]. Protocol
0x51 distinguishes this mapping from the old paint packet. A different ABI or
endianness would require a deliberate protocol change.

`light_paint_receive` handles only CONTROL packets of exact struct size with
non-null arguments. It copies the payload, MAC and local receive timestamp,
rejects wrong version, active >1, colour >=5, nonzero reserved, zero session or
out-of-range coordinates. The lock protects acceptance/queue-pointer reads,
not the render state. It enqueues with zero wait: full queues drop incoming
packets silently, avoiding radio-callback blocking. Entry timestamps filter
old queued packets; the persistent queue is not freed while callbacks run.

Painter ticks obtain a fresh snapshot, transform +Y and test the target. At
most every 50 ms they broadcast a packet with no requested ACK, incrementing
sequence. Active means both armed and on-canvas; inactive packets still send
and may carry the last valid coordinates. Successful send clears the displayed
link error, but does not prove delivery. Broadcast deliberately paints every
opted-in canvas rather than selecting a person. Sample freshness compares the
local clock against the service timestamp; it assumes a common monotonic clock
and does not explicitly reject timestamps in the future.

### Canvas tick and blending

Each tick first multiplies every RGB channel by `exp(-max(0,dt)/1.5)`, then drains
the queue. Receive age over 250 ms or timestamps before Canvas entry are rejected.
An accepted painter owns a two-second lease keyed by MAC and session. During
that lease other painters/sessions are ignored, even if the current painter is
sending inactive packets. Same-session duplicate/older sequences are rejected
using a signed difference, assuming sequence gaps stay below half the uint32
space. Inactive packets renew the lease but break stroke continuity.

A segment joins only for the same peer/session and stroke ID, a previous active
point, and receive gap below 200 ms. Otherwise it is a point dab. Per eligible
LED, strength is `max(0, 1-distance/0.12)`. RGB takes the per-channel maximum of
existing colour and the selected colour times strength: overlapping colours can
mix; this is not alpha-over or additive blending. Palette values are cyan
(0,180,255), orange (255,50,10), violet (160,30,255), green (20,255,70), white.
Because normalised axes have different physical scales, a radius of 0.12 spans
about 3.6 cm horizontally and 9 cm vertically: the physical brush is elliptical.

Packet samples are applied at processing time after the common fade, without
additional age compensation. Up to eight queued segments can each touch every
eligible LED, so worst-case work is proportional to queue depth times LED count.
Finally channels are truncated to bytes and submitted through owned LED output.
Return errors from frame submission/release are not surfaced in the UI. There
is no renderer deadline/budget inside this app and no ACK/retry queue.

### UI and known limits

`draw` shows mode, selected arm, calibration/on-target/armed state, colour or
lease freshness; error text occupies the bottom. It uses fixed pixel offsets
and ignores supplied width/height, unlike the newly bounded Message UI. It
assumes a valid framebuffer and enough content space; clipping/overlap on small
regions needs a separate fix. Send success can overwrite calibration error text
on a later tick. These are documented limitations, not guarantees of polished UI.

The tests in `main/tests/test_light_paint.py` exercise math and the production app
against mocked hardware, including folded/unfolded axes, roll invariance,
heading calibration, target bounds, fading, queue freshness/duplicates, painter
isolation, LED ownership, controls and exit. Run that test plus the FFT audio-gate
test after changes. They do not establish real mounting, radio range, two-suit
latency, dead-reckoning accuracy, draw clipping or CPU headroom with live traffic.
The service needs independent hardware timing/concurrency validation. No firmware
behaviour was changed as part of this documentation review.
