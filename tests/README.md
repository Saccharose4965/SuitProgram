# Focused host checks

Project-owned checks move here as their modules are cleaned up. They are
independent of the firmware build and run individually:

    python3 tests/services/test_fft_audio_gate.py
    python3 tests/apps/test_light_paint.py
    python3 tests/services/test_calculator.py

These check pending FFT requests across audio conflicts and Paint's service
boundary, brush mathematics and rendering. They compile small host harnesses
with fake peripherals and use temporary build directories.

Tests elsewhere will move with their next cleanup batch. Temporary migration
comparisons can be removed after validation; retain small regressions that
protect delicate behaviour. Physical suit checks remain necessary.
