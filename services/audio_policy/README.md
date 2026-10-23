Pending review

# Audio resource policy

This service decides when background beat analysis can use the audio resources.
Apps report active sessions. The shell regularly reevaluates pending requests,
so leaving a screen does not discard a lighting request. There is no new task.

## Keeping requests while resources are busy

Each requesting screen has a bit in a request mask. Beat-driven lighting and
audio-driven brightness also request analysis. Clearing one bit preserves the
other clients.

Paint, Synth, Call, Show, Arm Pose and 3D sessions block FFT. A show owning the
LEDs also blocks it. Bluetooth blocks analysis while its app is open or its
connection is starting, discovering, connecting, connected or streaming.
On evaluation, a blocker stops FFT and clears this service's ownership flag;
the request mask survives.

Paint and Synth evaluate immediately on entry. Show and Bluetooth explicitly
stop FFT on entry and release an idle audio bus to recover DMA memory.
Arm Pose, 3D and Call update their flags for the next evaluation. These existing
differences preserve the callers' lifecycle timing.

Recording, playback and generated audio postpone an FFT start. Once they finish,
the next evaluation can retry. Start failures retain the request and are logged
at most once every five seconds. Audio initialization failures have a separate
warning on each attempted evaluation.

When the final request disappears, the service stops FFT only if it started
that running instance. Explicit blocking sessions can stop any running instance.
Screen visibility is disabled before background startup and blocking stops.

## Audio bus setup

audio_bus.c supplies the board's I2S pins and lazily initializes the bus.
Its cached success is accepted only while a transmit handle exists. Releasing
the bus causes the next request to initialize it again.

Calls retain the existing shell/app scheduling. This service adds no locking.
Broader resource coordination remains in the work queue.
