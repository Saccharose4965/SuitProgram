Pending review

# Settings apps

This component contains four small screens under the Settings menu: Volume,
Preferences, Service Management and ADC Debug. They share the shell lifecycle
and display conventions while keeping their independent screen state.

Volume edits speaker and Bluetooth gain/mute and persists each accepted change
through the Settings service. Preferences browses the two global direction-icon
styles with A/B and applies the highlighted choice with D using the shared menu
renderer.

Service Management exposes explicit recovery actions for SD, OLED, audio,
Bluetooth, FFT and file reception. Each selection calls the owning service API
with D and displays its result; TODO-only entries are labelled rather than pretending
to restart unsupported services. ADC Debug samples the input driver's measured
voltage/button result and draws a bounded history for hardware diagnosis.

The component owns no persistent configuration or hardware driver state.
