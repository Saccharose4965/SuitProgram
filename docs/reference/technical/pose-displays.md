> Archived reference. Describes an earlier revision; current explanations live beside the code.

# Arm Pose and 3D Render

This review covers `main/app_stickman.c`, `main/app_threedee.c`,
`components/threedee/threedee.c`, its public header and CMake file, and
`main/tests/test_light_paint.py`. Shared sensor/brush maths are described in
[orientation and painting](orientation-and-paint.md). Shell registration,
FFT arbitration and the orientation filter were inspected at relevant call
sites only; this document does not certify those entire files.

## Purpose and execution

Both apps are active firmware features, compiled by `main/CMakeLists.txt`.
The shell registers `stickman` as **Arm Pose**, with a button legend and no
tick callback, and `threedee` as **3D Render**, with tick and draw callbacks
but no legend. They draw into the shell's framebuffer, not directly to OLED
SPI. They create no tasks, allocate no heap memory, claim no suit LEDs, and
do not read the MPU directly. Their callbacks run in the shell's execution
context; they have no independent frame-rate guarantee or core assignment.

Entry/exit set the corresponding `led_ui_fft_set_*_app` flag. Those setters
only change a flag: the next ordinary FFT service request enforces the gate.
They do not synchronously stop FFT inside these app initializers. Exiting
removes this gate, allowing the shared service to honour pending FFT demand;
it does not unconditionally start FFT.

## `main/app_stickman.c`: Arm Pose

### Calibration, coordinates and controls

`pose_sample` uses the service's coherent quaternion/timestamp snapshot and
accepts samples at most 250 ms old. Invalid or stale samples cause drawing
to return without an error label. This function does not clear the screen;
the visible result depends on the shell's framebuffer clearing.

The app starts with the left arm selected. The arm choice persists across
app openings in RAM, but calibration and view yaw reset on every entry.
Only press events are processed:

| Button | Action |
| --- | --- |
| A | Decrease view yaw by 0.20 radians, wrapping at -pi. |
| B | Increase view yaw by 0.20 radians, wrapping at +pi. |
| C | Toggle tracked arm and invalidate calibration. |
| D | Calibrate from a fresh sample using `arm_pose_calibrate`. |

Calibration is **full orientation**, not Paint's heading-only calibration.
The stored reference removes the initial roll and tilt as well as heading.
Until calibrated, a fresh sample produces `Fold arm; press D`. D silently
does nothing if the snapshot is unavailable/stale; the calibration function's
return value is not displayed. Calibration is local to this app's `arm_pose_t`;
it does not call the filter's global recenter operation.

The shared maths rotate raw MPU +Y (towards fingers) into calibrated body
coordinates: +X right, +Y up, +Z forward. Raw +X is transformed separately
to draw the hand's span. Thus forearm-axis roll can rotate the hand bar
without steering the pointing ray. Left/right changes the shared mapping
and the shoulder used; it does not select a second physical sensor.

### Skeleton and inference

Only the forearm orientation is measured. `guess_upper_arm_dir` explicitly
infers a plausible shoulder-to-elbow direction, normalizing this sum:

- 0.78 times a normalized hanging direction `(side*0.58, -0.80, 0.14)`;
- 0.34 times the normalized measured forearm direction;
- 0.20 times a normalized outward direction `(side*0.30, -0.12, 0.52)`.

Here side is -1 for left, +1 for right. These are visual tuning constants,
not anatomical constraints or inverse-kinematics measurements. Upper arm,
forearm and wrist-to-hand-centre lengths are 14, 16 and 3.5 drawing units;
the hand bar extends 4 units either side. The other arm hangs in two
14-unit links. Torso, hips, legs and feet are fixed points. Head centre is
`(0,31,0)` with radius 6. The skeleton is illustrative, not a physical suit
layout or a calibrated distance model.

Vector helpers implement construction, addition, subtraction, scaling, dot
product, length and normalization. A vector shorter than 1e-6 normalizes
to `(0,-1,0)`, supplying a hanging fallback rather than dividing by zero.
The shared pose API is responsible for rejecting invalid quaternion data.

### Projection and rasterization

The view rotates each point by user yaw, then fixed Y rotation -0.68 rad,
then X rotation +0.44 rad, and translates camera Z by +140. Projection uses
focal length 120 pixels and centre `(x+w/2, y+h/2+8)`. Screen Y is inverted:
`screen_y = centre_y - focal*camera_y/camera_z`. Points with camera Z <=8
are rejected. A line with either rejected endpoint is omitted altogether;
there is no near-plane intersection clipping.

Fourteen skeleton segments are insertion-sorted by increasing midpoint
camera depth. The head is drawn first as a closed 12-edge ring. All pixels
are ORed into a monochrome framebuffer, so this sort does **not** provide
hidden-line removal: crossing lines remain visible regardless of order.
`draw_line` is integer Bresenham; `fb_pset` clips each pixel to both the
content rectangle and physical panel dimensions. Storage is row-major,
one bit per pixel, most-significant bit first. The prompt uses the shared
text renderer rather than this local clipping helper.

Draw rejects a null framebuffer or nonpositive viewport dimensions. It
recomputes the small skeleton, depth ordering and projection each draw;
there is no interpolation, smoothing, cached geometry, anti-aliasing or
allocation. View yaw changes presentation only, never calibration/aim.

## `main/app_threedee.c`: shell adapter

Init sets the FFT gate before checking for a null shell context. With a
valid context it stores the shell switch callback/user data, attaches the
orientation service's legacy context if available, and initializes the
renderer. A missing context does not get explicitly detached: if a pointer
was previously installed it stays installed. Normal use assumes the
service context has process lifetime.

Deinit clears the FFT gate and calls renderer deinit. Input, tick and draw
wrappers forward arguments unchanged. `THREEDEE_LEGEND` contains four empty
icons and is not used by the current shell registration. Although the
renderer supports D recenter, the registered screen has no button legend.

## `components/threedee/include/threedee.h`: API contract

The C/C++ compatible header exposes shell-style init/deinit/input/tick/draw
functions, a borrowed `imu_orientation_t *`, mounting-quaternion setter,
and optional app-switch callback with opaque user data. No ownership is
transferred. There is no synchronization around these setters or renderer
state; callers must serialize renderer use and keep borrowed objects alive.

`dt_sec` is seconds; mount quaternion uses `(w,x,y,z)` fields. Draw expects
a valid panel-sized framebuffer plus a content viewport. Unlike Arm Pose,
it has no explicit null-framebuffer or positive-size guard. The switch
callback is stored but never called by the current implementation.

## `components/threedee/threedee.c`: moving camera in a wire scene

### Quaternion convention: do not confuse camera and pointing ray

The default mount is `(sqrt(0.5),0,0,sqrt(0.5))`, a +90-degree Z rotation.
The implemented frame change is **conjugation**:
`mounted = normalize(mount * relative * conjugate(mount))`.
It is not simply a one-sided mounting multiplication. The view then uses
`conjugate(mounted)`: rotating the camera makes the world appear to rotate
in the opposite direction. This is why copying the final camera matrix
directly into a brush-pointing calculation would reverse its behaviour.

When the attached filter is ready, `imu_orientation_get` supplies
`normalize(conjugate(q_zero) * q_BW)`. Despite some local comments calling
this sensor-to-world, it is the **recentered relative** orientation. Without
a ready context the renderer uses identity. It has no timestamp freshness
check; a stopped filter can leave a frozen camera while shapes keep spinning.

Quaternion helpers implement identity, conjugation, Hamilton product,
normalization, mounting and a row-major rotation matrix. Norm below 1e-9
falls back to identity; NaN inputs are not explicitly rejected. The mount
setter normalizes its argument. The matrix conversion normalizes again.

D press directly calls `imu_orientation_recenter` on the borrowed context;
other input is ignored. This is the legacy shared-context path, not the
snapshot/private-reference path used by Arm Pose/Paint. Recenter and reads
are not protected here against the service's concurrent updates. The
service's raw published snapshot is distinct from this zeroed getter, so
do not assume D recalibrates the private Arm Pose/Paint references.

### Scene, time and projection

Static tables define four wire meshes: cube (8 vertices/12 edges), octahedron
(6/12), pyramid (5/8), and triangular prism (6/9). Eight static instances
provide centre, per-axis scale, spin axis, angular rate and initial phase;
each mesh occurs twice. Geometry is in arbitrary scene units, rates in
radians/second and phases in radians. These tables are authored in source;
there is no asset loader, editor export, SD dependency or persistence.

Init resets scene time only. Deinit is empty: attached context, callback and
mount survive reopening. Tick clamps negative dt to zero, accumulates time,
and applies modulo 1000 when time exceeds 1000 seconds. Because rates do
not generally complete whole turns in 1000 seconds, that wrap can cause a
visible spin discontinuity. No finite-value validation is performed on dt.

Per instance, draw scales each mesh vertex, spins it using Rodrigues' axis
rotation, adds the instance centre, and applies the inverse camera matrix.
There is **no camera translation**. Objects surround the origin; points
behind or too near the camera are expected to disappear. Projection has
focal length 100 pixels, viewport-centred origin and inverted screen Y;
camera Z <=20 is rejected. Edges are drawn only when both endpoints pass,
not clipped against the near plane. Pixel clipping/Bresenham/bit layout
match the Arm Pose approach. No fill, depth buffer, occlusion or anti-aliasing.

Temporary arrays hold eight vertices, projected positions and validity
flags on the caller's stack. Adding a mesh with more than eight vertices
requires enlarging those arrays. Edges must index valid vertices. Vector
helpers supply component scaling, dot/cross and normalized-axis rotation;
zero-length axis normalization returns zero, not a replacement unit axis.
Current static axes are nonzero. Trigonometry is repeated per vertex; this
is a small scene, not an optimized reusable graphics engine. An unused
vector subtraction helper and unused switch callback storage remain.

## `components/threedee/CMakeLists.txt`

Registers `threedee.c`, exports `include`, and declares dependencies on
`hw`, `mpu6500`, `oled`, `input`, `log` and `orientation`. It has no Kconfig
switch or conditional source selection. Some dependencies are broader than
the renderer's direct calls; this review preserves the build declaration
rather than claiming each is minimal or removing them without a build audit.

## `main/tests/test_light_paint.py`: host regression coverage

One Python unittest builds a temporary C harness using GCC C11, warnings as
errors, AddressSanitizer and UndefinedBehaviorSanitizer. It compiles the
actual `arm_pose.c`, injects the production Paint app after removing include
lines, and extracts actual quaternion/matrix helpers from `threedee.c`.
Fixtures emulate clock, IMU snapshot, bounded queue, link, three layout LEDs,
LED ownership, memory allocation, text drawing and FFT gate. Temporary files
are removed by `TemporaryDirectory`; subprocess failures fail the test.

Assertions cover:

- 100 heading calibrations retaining yaw despite finger-axis roll; rejection
  of vertical calibration and canvas boundary checks at 1.5 m;
- 1000 deterministic quaternion pairs, both arm mappings, comparing brush
  direction to production renderer **device rotation**, not its inverse view;
- uncalibrated/NaN rejection, folded and unfolded rays, finger-axis roll,
  lift, visible wrist-span rotation, backwards/nonfinite canvas rejection
  and a point-on-segment brush distance;
- FFT gating, busy LED ownership refusal, canvas claim, waist-centred mapping,
  packet size/range rejection, painting, fading, duplicate/stale suppression,
  exclusion of a second painter during the active lease, clearing and exit;
- painter calibration, active sending, stale samples, arm toggles invalidating
  calibration, retained arm preference on reopening, and bounded menu controls.

The radio stub checks no-ACK sending and counts sends, but does not validate
every transmitted field. Locks are no-ops; the queue is synchronous. Layout
and geometry are simplified to three LEDs, and samples are artificial. This
does not test MPU mounting, drift, radio loss, concurrent callbacks, service
scheduling, real LED output, frame timing or the physical OLED. It does not
execute Arm Pose or the full 3D renderer; extracting shared renderer maths
is not a visual rendering test. Source substring extraction also depends on
helper names/order, so refactoring may require harness maintenance.

## Hardware checks still needed

With the actual mounted MPU, verify folded calibration, unfolding towards
the target, up/down pointing, and hand-bar roll for each arm selection.
Compare Paint's gravity-retaining calibration with Arm Pose's full-pose
calibration deliberately, rather than assuming they should look identical.
For 3D Render, verify camera direction and D recenter, and check stale-sensor
behaviour separately from continuing scene spin. No hardware correctness or
frame-rate claim is implied by the host test.
