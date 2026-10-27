Pending review

# Input driver

## Decode a physical press

Buttons share an ADC resistor ladder. Each sampled voltage is compared with the
expected single-button and supported combination levels. The nearest level within
tolerance wins; unmatched voltages mean released. This accommodates ordinary
measurement variation while keeping decoding deterministic.

A new decoded level must persist for 25 ms before becoming the logical state.
The driver remembers when that candidate first appeared. Brief excursions are
discarded, and stable direct changes between buttons are accepted.

Every accepted change restarts hold timing. A pressed level produces a press
event, followed by one long-press event after 1200 ms. Release updates state
without producing a UI event. Failed ADC reads leave the existing state intact.
Continuous controls can read the cached debounced level. This performs no ADC
transaction and stays consistent with the event stream.

## Report activity

A registered callback runs synchronously for each accepted level change,
including releases. Its argument is the elapsed time since the candidate first
appeared. Consumers can therefore account for debounce and polling latency.
Long-press events represent a held level and produce no new activity callback.

The polling task registers the callback before polling begins. Registration
persists through input reinitialization; a null callback detaches it. Callbacks
must remain short because they run inside the poll operation.

Initialization resets button/debounce state and requests ADC setup. The driver
owns decoding and event timing. The [audio policy](../../services/input_audio_policy/README.md) consumes
the activity notification through startup wiring in the shell.

## Verification

The host test runs production decoding and policy with synthetic voltages. It
covers bounce, direct button changes, delayed polls, long press, release, ADC
failure and callback detachment. Actual ladder tolerances require hardware checks.
