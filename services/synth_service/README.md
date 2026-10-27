Pending review

# Synth service

The Synth service owns gesture interpretation, oscillator state and the active
audio route for Arm Synth. The app sends calibration and mute commands, ticks
fresh sensor interpretation, and reads one status snapshot.

Calibration records a Synth-owned left-arm heading through the shared
Orientation pipeline. Vertical direction controls pitch exponentially over three octaves;
horizontal direction controls gain. Missing or stale orientation data requests
silence. Frequency and gain are smoothed while oscillator phase remains
continuous, avoiding a new waveform edge for every sensor update.

A streamable Bluetooth connection is preferred. Its callback pulls oscillator
samples at the negotiated rate. Failure to start or a later disconnect falls
back once to the integrated speaker. The service owns FFT exclusion for its full
lifetime and refuses to start over recording, playback, local output or an
already-streaming Bluetooth source.

Closing requests a short release envelope, detaches the Bluetooth generator or
finishes local generation, joins the worker, then releases FFT exclusion. Task
placement, stack size, wait bounds and route behavior are preserved from the
working implementation.

Host tests cover silence, frequency, gain bounds, phase continuity, output sample
rates and the Bluetooth generator bridge. Hardware review still needs gesture
feel, latency, disconnect fallback, repeated entry and FFT recovery.
