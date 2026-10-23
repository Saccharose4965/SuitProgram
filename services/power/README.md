Pending review

# Battery monitor

The monitor publishes a smoothed battery percentage for the shell. It starts
one background task on core 0, priority 4, with a 2048-byte stack. Repeated start
requests reuse that task. It runs until reboot.

## Measurement and estimate

The existing board ADC driver reads the channel named Hall B, repurposed for
the battery divider. The service converts millivolts back to pack voltage using
the configured divider ratio. It assumes three cells in series and maps per-cell
voltage linearly between the configured empty and full endpoints.

Defaults are ratio 6.0, empty 3.30 V and full 4.15 V. These settings belong to
this service's Power sensing menu. The divider default still needs confirmation
against the physical wiring. This is a voltage-based display estimate.

The estimate is clamped to 0–100 and rounded before smoothing. Every successful
reading contributes 20% to the filtered value; the previous value contributes
80%. The filter starts at zero, so the display ramps up after startup.
The published value truncates the filtered percentage to an integer.

One measured percentage is copied into all three state slots. Those slots
currently do not represent three independent measurements.

## Timing and failures

After each iteration the task waits one second. A failed read logs a warning
and retains the previous state and filter value. ADC initialization is attempted
on task entry; later read failures use the same warning path.

Configuration strings are parsed once at startup. Positive numeric prefixes
above 0.1 are accepted for ratio and empty voltage; full must exceed empty.
The conversion enforces a minimum voltage span of 0.05 V. These existing rules
remain unchanged; stricter configuration validation can be considered separately.

Acquisition stays in the board driver, while voltage interpretation, filtering,
publication and task lifetime belong here. No UI or hardware protection cutoff
is implemented by this service.
