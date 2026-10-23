Pending review

# FFT viewer

This app presents the background FFT service on the 128 by 64 OLED. Opening it
ensures the shared audio bus and detector are available, then displays the
service's accumulated history. Leaving the app only removes the screen; audio
analysis continues when another feature still requests it.

## Snapshot drawing

Each draw copies one coherent history snapshot under the service mutex. Pixel
generation happens afterward in app context. A cached framebuffer is reused
until the service generation or selected view changes, keeping capture locks
short and avoiding a render task or framebuffer queue.

A and B cycle five full-screen views:

- spectrum: current log magnitudes, four FFT bins per horizontal pixel;
- spectrogram: the latest 128 grouped time columns across the full frequency
  range, with frequency peaks retained inside each two-frame group;
- BPM scores: the normalized candidate spectrum over 80–159 BPM;
- confidence: decision readiness and ungated evidence on a fixed 0–1.1 scale;
- phase: folded phase energy transformed into the actual output-clock
  coordinates, including ratio.

The monochrome renderer uses solid one-pixel lines and bars. Timeline views
share the same ring index, keeping confidence and spectrogram events aligned.
Drawing is pure: it cannot advance history or alter detector state.

C and D reset the detector from this screen. Startup failures display the
audio error, including the Bluetooth resource gate, in place of a graph.

The host display regression checks the full frequency range, two-frame grouping,
timeline rollover/alignment, phase-coordinate mapping, framebuffer bounds and
reset behavior. Visual scale and legibility still require suit OLED testing.
