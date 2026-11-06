Pending review

# FFT viewer

This app presents the background FFT service on the 128 by 64 OLED. Opening it
registers one demand with Audio Policy, then displays the service's accumulated
history. Leaving clears only that demand, so analysis continues when lighting
or another client still requires it. Bus and detector lifecycle remain owned by
Audio Policy and the FFT service.

## Snapshot drawing

Each draw copies one coherent history snapshot under the service mutex. Pixel
generation happens afterward in app context. A cached framebuffer is reused
until the service generation or selected view changes, keeping capture locks
short and avoiding a render task or framebuffer queue.

A and B cycle five full-screen views:

- spectrum: current log magnitudes, four FFT bins per horizontal pixel;
- spectrogram: the latest 128 grouped time columns across the full frequency
  range, with frequency peaks retained inside each two-frame group;
- BPM scores: 80 normalized candidates over 80–159 BPM, one pixel column each,
  centred with 24 empty columns on each side;
- confidence: decision readiness and ungated evidence on a fixed 0–1.1 scale;
- phase: the folded half-cycle evidence in the top band and the full 128-bin
  phase-energy curve in the bottom band. Both use the emitted beat clock as
  phase 0 at the left screen border.

The folded band sums each phase bin with its opposite half-cycle bin, exposing
the timing family used by the tracker while retaining both individual sides in
the full band. The graph origin comes from the applied output snapshot published before blink
dispatch. A curve is withheld during a handover if its tempo differs from the
applied clock. The graph has no moving playhead: the screen border is the
actual clock reference. Evidence moving away from the border exposes clock
drift, while gradual steering visibly returns it. The monochrome renderer uses
solid one-pixel lines and bars. Timeline
views share the same ring index, keeping confidence and spectrogram events
aligned. Drawing is pure: it cannot advance history or alter detector state.

Opening the viewer preserves the selected lighting source. In the phase view,
a small `LED: manual` or `LED: follow A/B/C` label identifies output using a
different timing authority from the local FFT graphs.

C resets the detector. D uses the shared Controller timing gesture: isolated
presses alternate on-beat/offbeat; two presses within 700 ms select 2x, and
further rapid presses cycle the choices. While analysis is unavailable, the
screen distinguishes Bluetooth suspension from waiting for another audio owner.
Its request remains pending and resumes through the regular policy evaluation.

The host display regression checks the full frequency range, two-frame grouping,
timeline rollover/alignment, phase-coordinate mapping, framebuffer bounds and
reset behavior. The FFT service's desktop parity suite also draws identical
phase evidence through the actual SDL and OLED renderers with on/offbeat and
2x output clocks, checking horizontal peak agreement after scaling. The OLED
keeps its full-screen phase layout; its column zero corresponds to the left
edge of the PC phase panel. Visual scale and legibility still require suit OLED testing.
