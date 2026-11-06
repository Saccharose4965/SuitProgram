Pending review

# LED control apps

This group contains the five screens used to configure live lighting: animation
selection, colour, source selection, manual BPM and FFT control. Each screen
owns its cursor, labels, button interpretation and OLED drawing. Runtime
animation generation remains in the LED services.

## Beat, ambient and colour

Beat and Ambient are separate three-row grids. Beat contains effects whose state
is advanced by BPM events. Ambient contains continuously rendered modes, whose
motion follows their own or the published synchronization clock. Audio Energy is
also Ambient because it follows live frequency-band levels and ignores beat
triggers, even though its implementation currently shares the beat renderer.
The active three-item column is centred while cropped portions of its horizontal
neighbours remain visible.
Opening a catalogue only positions its cursor; D selects a mode, disables the
other engine and publishes the choice. A/C moves between columns and B cycles
within one column. Ambient speed is edited in ten-percent steps between 10% and
250%; plane and ring background choices are applied immediately in both
catalogues. Mode and background choices are saved immediately. Speed previews
live while it is edited and is saved when editing finishes or the app closes.

The colour screen edits the shared LED colour configuration. Presets establish
a coherent style, highlight mode and two colours. Editing an individual channel
creates a manual configuration where required. Audio brightness cycles through
disabled and the available frequency ranges. Every edit is applied immediately,
making the suit a live preview. One edit publishes a complete configuration to
each renderer, so the preview cannot expose a partially applied preset.
Every completed edit also saves that complete profile to the SD card.

## Timing screens

Source selection chooses FFT, the manual clock or Follow A/B/C. A selected
remote source is marked active when packets are current and `lost` after its
link expires. The Source title shows this suit's settled A/B/C identity so the
wearer knows which role another suit should follow. The manual screen adjusts
BPM and phase, displays the current
cycle and briefly marks beat crossings. The FFT
controller resets the detector, toggles forced lock and selects automatic phase,
offbeat or double-rate timing. Two offset presses within 700 ms select double
rate; isolated presses alternate on-beat and offbeat.

These screens call the [LED source service](../../services/led_source/README.md)
for local clock state and [Live Sync](../../services/live_sync/README.md) for
remote state. They contain no background clock generator.

## Verification

The host regression compiles the production animation input handler and checks
three-row grid navigation, selection, speed editing and limits. Hardware review should
exercise every screen while LEDs are visible, including manual-BPM long presses
and the controller's double-press timing.

## Controller phase alignment

The Controller lists Back, Reset, Force, Set and Phase. A/B browse. D invokes
the selected action. Set uses the shared timing gesture. Phase uses C/D for
0.01-cycle earlier/later changes and long presses for 0.05-cycle changes.
Its signed value and button legend update live. Follow displays `Phase: follow`
and leaves alignment with the remote source.

## Controller lifecycle

Opening, drawing and closing Controller update only UI state. The shell's
background services own audio setup, source policy and clock advancement.
Entry preserves source, tempo, phase and output controls.

`tests/test_fft_controller.py` compiles the production callbacks and checks
100 entry/draw/exit cycles, a held entry button and explicit timing actions.
User long-duration testing confirms phase alignment remains stable after opening
Controller.
