# Documentation

The documentation follows the firmware structure. Every app, service and driver
keeps its design explanation beside its source as `README.md`. This folder holds
only project-wide design and the source-review checklist.

## Project-wide design

- [Firmware architecture](architecture.md)
- [Show system](show-system.md)
- [Work queue](../TODO.md)
- [Completed work and hardware checks](../PROGRESS.md)

## Apps

[Arm Pose](../apps/arm_pose/README.md) ·
[Bad Apple](../apps/bad_apple/README.md) ·
[Bluetooth](../apps/bluetooth/README.md) ·
[Calculator](../apps/calculator/README.md) ·
[Call](../apps/call_app/README.md) ·
[FFT View](../apps/fft_view/README.md) ·
[File Receive](../apps/file_rx/README.md) ·
[Flappy](../apps/flappy/README.md) ·
[Fluid](../apps/fluid/README.md) ·
[Gyro Calibration](../apps/gyro_calibration/README.md) ·
[Keyboard](../apps/keyboard/README.md) ·
[LED Layout](../apps/led_layout_app/README.md) ·
[LED Control](../apps/led_control/README.md) ·
[Remote Control](../apps/remote_control_app/README.md) ·
[Shell](../apps/shell/README.md) ·
[Light Paint](../apps/paint_app/README.md) ·
[Messages](../apps/message_app/README.md) ·
[Music](../apps/music/README.md) ·
[Pong](../apps/pong_app/README.md) ·
[Show Player](../apps/show_player/README.md) ·
[Snake](../apps/snake/README.md) ·
[Arm Synth](../apps/synth/README.md) ·
[Settings](../apps/settings_app/README.md) ·
[Tetris](../apps/tetris/README.md) ·
[3D View](../apps/threedee/README.md) ·
[2048](../apps/twenty48/README.md) ·
[UI Shell](../apps/ui_shell/README.md)

## Services

[Audio](../services/audio/README.md) ·
[Audio Player](../services/audio_player/README.md) ·
[Audio Policy](../services/audio_policy/README.md) ·
[Audio Receive](../services/audio_rx/README.md) ·
[Bad Apple Decoder](../services/bad_apple_decoder/README.md) ·
[Bluetooth Audio](../services/bt_audio/README.md) ·
[Calculator Math](../services/calculator_math/README.md) ·
[Call](../services/call/README.md) ·
[FFT](../services/fft/README.md) ·
[Flappy Game](../services/flappy_game/README.md) ·
[Fluid Simulation](../services/fluid_sim/README.md) ·
[Input Audio Policy](../services/input_audio_policy/README.md) ·
[LED](../services/led/README.md) ·
[LED Layout](../services/led_layout/README.md) ·
[LED Modes](../services/led_modes/README.md) ·
[LED Source](../services/led_source/README.md) ·
[Live Sync](../services/live_sync/README.md) ·
[Remote Control](../services/remote_control/README.md) ·
[Performance Monitor](../services/performance_monitor/README.md) ·
[Link](../services/link/README.md) ·
[Microphone](../services/microphone/README.md) ·
[Messages](../services/messages/README.md) ·
[Orientation](../services/orientation/README.md) ·
[Paint](../services/paint/README.md) ·
[Pong](../services/pong/README.md) ·
[Power](../services/power/README.md) ·
[Scores](../services/scores/README.md) ·
[Settings](../services/settings/README.md) ·
[Show Package](../services/show_package/README.md) ·
[Show Renderer](../services/show_renderer/README.md) ·
[Show Runtime](../services/show_runtime/README.md) ·
[Snake Game](../services/snake_game/README.md) ·
[State](../services/state/README.md) ·
[Suit Geometry](../services/suit_geometry/README.md) ·
[Synth](../services/synth_service/README.md) ·
[Telemetry](../services/telemetry/README.md) ·
[Tetris Game](../services/tetris_game/README.md) ·
[2048 Game](../services/twenty48_game/README.md)

## Drivers

[Audio I2S](../drivers/audio_i2s/README.md) ·
[Board Hardware](../drivers/hw/README.md) ·
[Input](../drivers/input/README.md) ·
[LED Strip](../drivers/led_strip/README.md) ·
[MPU6500](../drivers/mpu6500/README.md) ·
[OLED](../drivers/oled/README.md) ·
[SD Storage](../drivers/storage_sd/README.md)

## Review

An explanation awaiting approval begins with:

    Pending review

Remove that first line to validate the explanation. If behavior changes enough
to require another review, restore the line and record why in `PROGRESS.md`.
Approval covers the explanation; hardware acceptance remains a separate record.

The generated [file inspection checklist](documentation/file-checklist.md)
tracks exact source revisions that have been read and described. Its checkmarks
are independent of the approval marker. Writing should focus on decisions,
state, ownership and failure behavior; implementation details stay in source and
tests unless they explain an important constraint.
