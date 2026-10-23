Pending review

# Arm Pose

This screen draws a stick figure whose selected forearm follows the sensor.
The screen owns the camera angle, selected arm and a session-local reference.
Direction mapping and upper-arm estimation belong to the
[orientation service](../../services/orientation/README.md).

Opening resets the camera and reference while retaining the selected arm.
A/B rotate the view, C changes arms and invalidates calibration, and D captures
the folded-arm reference. Samples older than 250 ms are ignored. With a fresh
sample but no reference, the screen asks for calibration.

Drawing places the elbow from the estimated upper-arm direction and the wrist
from the measured forearm direction. A second mapped sensor axis draws the
hand's width, making wrist roll visible. The remaining body is a fixed diagram;
these display dimensions are illustrative, separate from LED placement geometry.

The camera projects body segments onto the content rectangle, clips lines and
orders them by depth. Camera rotations and projection are presentation logic.
The app requests FFT suspension for its lifetime through
[audio policy](../../services/audio_policy/README.md), releasing that request
on exit. It creates no task and writes no persistent calibration.
