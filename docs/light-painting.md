# Light painting and Arm Pose

Open **Simulations → Light Paint** on each participating suit. Select **Painter**
on the suit with the forearm IMU; select **Canvas** on the receiving suits.
Only open Canvas apps accept paint. All connected canvases receive the same
stroke; this is not automatic identification of the person you point at.

## Controls

- Menu: B/C navigate Painter, Canvas and Arm; D selects or toggles left/right;
  A returns. Arm defaults to left and is remembered until reboot. Switching
  arms requires recalibration. This selects the pose convention, not another IMU.
- Painter: fold the **selected forearm horizontally across the chest**, then B
  calibrates. Unfold toward the target in front. D toggles painting; C changes
  colour; A returns. Painting starts OFF. B recalibrates and stops painting.
- Canvas: B clears; A leaves and releases LED output.

Trails fade with a 1.5-second time constant. The front torso/arms and enabled
front leg sections form a virtual canvas. The receiver uses its own configured
LED positions, so the sender transmits only normalized brush coordinates, not
pixels. The virtual target is 0.6 m wide by 1.5 m tall, 1.5 m ahead
(half-angles about 11.3 degrees horizontal and 26.6 degrees vertical).
Its fixed anatomical anchor is the waist/belt centreline: posed layout
x=0, y=33.5 cm. Geometry units are interpreted as centimetres; this is a
nominal body model, not a measurement of the wearer. Missing LEDs/legs never
shift or resize the target. Geometry is prepared once on entry. Strokes interpolate between recent
points; gaps, separate strokes and colour changes do not get connected.

Canvas claims exclusive LED ownership and refuses to replace an active show.
Leaving clears its frame and returns output to the normal LED renderer.
FFT capture/analysis is suspended for the entire Light Paint app (including
its menu, Painter and Canvas) to free processing time. Background requests
cannot restart it while the app is open. On exit, normal FFT service resumes
if requested and not blocked by another audio owner. Detector settings,
Bluetooth and show timing are unchanged.
Each canvas follows one painter until its packets have been absent for two
seconds. Stale, duplicate, malformed and out-of-order packets are ignored.
If a sender disconnects, existing paint simply fades; it is never held on.

## Motion calculation

Both apps use the same sensor-to-reference vector rotation. The beam is
raw MPU **+Y** (fingers); the target is reference **-X**, and up is **+Z**.
The board screen axes (+Y, -X) are not the beam axis or the target plane.
No inverse scene/camera transform is applied to the pointing vector.
Paint calibration projects the finger axis onto the gravity-level horizontal
plane and saves heading only. Arm roll around local Y is ignored; calibration
pitch does not tilt the canvas. Near-vertical calibration is rejected because
heading is undefined. Arm Pose and Synth retain their full-pose calibration.
Right-arm selection mirrors lateral placement, assuming the same sensor axes.

Arm Pose: fold the arm horizontally and press D to calibrate; C switches arms
and requires recalibration. A/B rotate the viewing angle. Hand span follows
raw X, so wrist roll remains visible even when the beam direction is unchanged.
The shoulder/elbow reconstruction is still an estimate, not a measurement.
Both apps read coherent timestamped snapshots and reject samples older than
250 ms. Calibration is local: the 3D viewer reference and IMU fusion are unchanged.

At the folded reference, +90 degrees around MPU Z unfolds +Y toward -X;
+90 degrees around MPU X lifts +Y toward +Z. Pure rotation about local MPU Y
rolls the hand without steering its +Y beam. Movement of the upper arm with a
bent elbow is not necessarily a pure rotation about the forearm's local Y.

This uses the existing accelerometer **and gyroscope** fusion. It does not
integrate acceleration into room position, locate target suits, or compensate
for walking/turning the torso. With no absolute heading sensor, yaw can drift;
recalibrate with B when needed. The physical mounting still needs a hardware check.
Straight-ahead horizontal aim maps to the target waist; there is no measured
origin height, distance or target location. Both Painter and Canvas must be
updated for this mapping (packet version 0x51 rejects the old bounds mapping).

## Verification

`python3 main/tests/test_light_paint.py` exercises the production app and shared
math with simulated sensor, link, clock and LED output: folded/unfolded poses,
invalid input, LED ownership conflict, fading, duplicates, stale packets,
peer isolation, clear and exit. It also compares 2,000 left/right orientations
against the actual unchanged 3D renderer's quaternion/matrix functions, with
non-identity calibration poses, and checks that wrist roll does not steer.
Firmware compiles, but real two-suit pointing,
mounting direction and radio behavior still need testing.

First physical test: one Painter and one Canvas, calibrate folded, unfold and
toggle paint. Check centre, left/right and up/down, then clear and exit. Report
which direction is wrong if the sensor mounting differs; do not change show
coordinates to compensate for an IMU mounting issue.
