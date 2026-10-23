Pending review

# Fluid screen

This app turns the fluid service's 64 by 32 binary mask into OLED pixels. It
fits the complete simulation into the content rectangle with an integer scale,
centres the result, and expands each set mask cell into a square of OLED pixels.
The integer transform keeps edges crisp and makes the same mask usable in a
different screen rectangle without changing the simulation.

Opening the screen starts a fresh simulation and its worker. Closing it stops
the worker and resets the particles. The shell tick only advances the model
when worker creation failed. Drawing copies one published snapshot, so a fluid
step cannot change half of a rendered frame.

There are no controls inside the screen. The former blocking runner duplicated
button polling and direct OLED output and had no caller; it was removed during
the layer split.
