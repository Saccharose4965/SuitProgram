Pending review

# Arm Synth app

The Arm Synth app presents the controls and status of the
[Synth service](../../services/synth_service/README.md). Opening and closing the
screen start and stop that service.

B calibrates the folded left-arm reference and returns the output to mute. D
toggles mute after calibration. The app ticks motion interpretation at the shell
rate and displays the requested pitch, gain, selected audio route and startup
error. A returns to Simulations.

All sensor conversion, sound generation, route fallback and audio ownership live
in the service. The app performs no I2S, Bluetooth or orientation work directly.
