Pending review

# 3D view

This app draws a small animated wireframe scene. Sensor movement rotates the
camera; each object also has its own visual spin. The inverse camera rotation
makes the scene move as the viewer turns.

Opening resets scene time and the local heading reference. The first suitable
fresh sample establishes that reference; D requests another. Calibration uses
the shared [orientation policy](../../services/orientation/README.md).
Each sample must be at most 250 ms old. Stale input keeps the previous camera
orientation while scene animation continues.

The mounting correction aligns the sensor with the screen. The renderer combines
that correction with the relative sensor rotation, inverts it for the camera,
then projects the scene vertices and draws clipped edges in the content area.
These transformations control presentation, so they live with the renderer.

The shell ticks scene time and supplies the drawing rectangle. Negative time
steps are clamped to zero and accumulated time wraps after 1000 seconds.
The app blocks FFT for its session through
[audio policy](../../services/audio_policy/README.md).
There is one implementation file containing lifecycle, input and rendering.
