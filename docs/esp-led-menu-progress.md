# ESP LED menu progress

## Done

- LED menu order: Controller, Animation, Color, Source, Editor.
- Beat and synchronized animations share one Animation list; backend prefixes are hidden.
- Both animation backends use the selected source clock (FFT by default, or manual).
- Controller contains Reset, Force, and Offset controls.
- Offset supports on-beat, offbeat, and deliberate double-press 2x selection.
- Source menu scroll animation is active and the fourth row is reachable visibly.
- Source menu includes the future show/master source as an explicit TODO.
- ESP BPM decisions now mirror `localFFT`: time-weighted score history, support credit/decay,
  shock scaling, switch gates, relative held-score loss, 40 s stop, and history-preserving reset.
- Host parity tests compile the actual `LatestFFT.c` and compare filter, comb scores,
  normalization, readiness, and phase-evidence curves against the ESP implementation.

## Next

- Implement show/master control as a source once the show transport contract is defined.
- Test controller gestures and every animation backend on the physical buttons and LEDs.
- Decide whether same-named animations from the two internal engines should be consolidated.
