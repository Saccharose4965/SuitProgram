Pending review

# LED source service

This service selects FFT, a manual BPM clock or a remote A/B/C suit as the LED
timing source. It translates local sources into the continuous timeline used by
standalone LED animations and requests FFT ownership only while FFT is selected.

## Manual clock

Manual BPM is clamped to 32–255 BPM. The service advances normalized phase from
elapsed frame time, detects every trigger crossing and emits one beat event per
crossing. Completed beats and aligned phase form a monotonic animation time,
keeping motion continuous across phase wrapping.

Offbeat timing places the trigger halfway through the cycle. Double timing runs
the manual clock at twice its configured BPM. The timing selection also updates
the FFT detector's offbeat and ratio controls.

## FFT clock

With a running detector, enabled beats and a valid BPM, detector phase is aligned
to its trigger phase. A backwards phase crossing advances the local beat count.
Beat count and aligned phase form the animation timeline. Invalid detector state
disables the clock until valid FFT state returns.

## Follow sources and ownership

Follow A/B/C disables local FFT demand, FFT LED export and the locally generated
sync clock. The [Live Sync service](../live_sync/README.md) then drives timing
and configuration from the selected suit. Selecting FFT or Manual returns
clock ownership here.

The service owns source choice, manual state, timing choice and timeline
continuity. Menus and drawing belong to
[LED control](../../apps/led_control/README.md).

## Timing button

Controller and FFT View share `led_source_timing_press`. After 700 ms idle, a
press toggles on/offbeat; a second quick press selects 2x. Further quick presses
cycle the three choices. The gesture sets the FFT output controls through the
same timing selector used by Manual.

## FFT output controls and phase trim

The timing getter reads requested FFT offbeat/ratio controls, so a button edit
is immediately visible to navigation. The FFT sync snapshot separately reports
the applied clock state used for beat export and the phase graph. Accepted BPM
replacements reset offbeat, ratio and trim; timing gestures restart when the
held BPM changes.

The phase-offset getter and adjustment route to the selected local source.
Positive trim moves the trigger later. Manual uses its stored trigger phase;
FFT uses its requested base-period offset. Follow output retains remote control.
