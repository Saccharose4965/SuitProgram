Pending review

# Orientation service

## Upper-arm estimate

The forearm sensor cannot measure the shoulder joint. arm_pose_estimate_upper_arm
combines normalized resting, forearm-following and outward-flare directions with
weights 0.78, 0.34 and 0.20, then normalizes the result. Left/right selection
mirrors the resting and flare directions. This is a visual pose estimate, not
an additional sensor measurement. Arm Pose uses it to place the elbow; screen
projection and drawing remain in the app.

## Estimate orientation

The service combines gyro rotation with accelerometer gravity feedback. Gyro
integration follows motion quickly; gravity correction limits tilt drift.
Horizontal heading can drift, so consumers establish a reference through user
calibration.

The service publishes a coherent quaternion and sample timestamp. Raw acceleration
also remains available. The MPU driver supplies the physical measurements.

## Establish the reference

Calibration projects the finger axis onto the horizontal plane and stores its
heading. Keeping vertical aligned with gravity makes later gestures independent
of wrist roll or slight tilt during calibration.

The forearm should be folded horizontally. Near-vertical calibration is rejected
because its horizontal projection gives an unreliable heading. Invalid input
preserves the previous valid reference.

This policy lives in `orientation_reference.h`. Each consumer owns a reference
value, giving each feature an independent calibration lifetime.

## Convert a direction

Arm mapping rotates the sensor's finger axis through the stored reference, then
expresses it as right/up/forward relative to the body. Arm selection mirrors the
lateral component. An arbitrary sensor vector can use the same mapping, which
allows a display to represent hand rotation as well as pointing.

## Publication and freshness

Consumers receive the orientation together with its timestamp and choose how to
respond to stale samples. The current consumers use a 250 ms limit.

A timing gap rejected by the filter can still result in an unchanged quaternion
being published with the latest sample timestamp. Consequently, publication age
alone cannot establish that integration advanced. This limitation remains open.

Legacy filter getter/recenter functions remain available; new consumers use the
coherent snapshot and their own reference.

## Gyro bias calibration

The gyro has a stationary offset on each axis. `gyro_calibration.c` estimates it
from raw measurements, and the filter subtracts it before integrating rotation.
The heading reference remains independent of this physical sensor correction.

A session settles for two seconds and samples for five seconds. It counts unique
timestamps and requires at least 100 samples. Acceleration must stay near 1 g and
near its initial direction; gyro magnitude must stay within 10 degrees/second.
Per-axis variance must stay below 0.25 (degrees/second)². These conservative
checks reject handling movement. Slow constant rotation can still resemble bias,
so the user must keep the sensor stationary.

The mean becomes a candidate. Confirmation writes `gyro_bias.tmp`, flushes it,
keeps the previous valid reference as `gyro_bias.bak`, and installs `gyro_bias.txt`.
This replacement sequence accommodates FAT's rename rules. Startup loads the
primary file or its backup after SD initialization, validates its version and
finite bounded values, and retains existing defaults if both fail validation.
Power-loss recovery still depends on the SD filesystem remaining readable.

After successful save, a synchronized bias publication makes the new value
available to the filter task. The filter applies it on its next sample. Until
a valid saved reference is available, defaults remain X=-1.9, Y=0.5, Z=0 d/s.
Each suit's SD card therefore carries its own sensor correction.

## Verification

Host tests cover heading extraction under tilt/roll, vertical and invalid-input
rejection, reference preservation and filter recentering. Hardware validation
checks the actual mounting: calibrate with slight wrist roll, unfold, and confirm
that upward motion remains upward.

Migration history and outstanding integration work live in
[Progress](../../PROGRESS.md).
