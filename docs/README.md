# Documentation

Design explanations live in each app, service or driver's own README.md.

## Review

An explanation awaiting your approval begins with:

    Pending review

Remove that first line to validate it. That edit is the approval record; there is
no separate status table to maintain. If behaviour changes enough to require a
new review, restore the line and describe the reason in PROGRESS.md. Approval
covers the explanation; hardware test results remain recorded separately.

## Current explanations

### Apps

- [Paint](../apps/paint_app/README.md)
- [Gyro calibration](../apps/gyro_calibration/README.md)
- [UI drawing](../apps/ui_shell/README.md)
- [Calculator](../apps/calculator/README.md)
- [Arm Pose](../apps/arm_pose/README.md)
- [3D view](../apps/threedee/README.md)
- [On-screen keyboard](../apps/keyboard/README.md)
- [Snake screen](../apps/snake/README.md)
- [2048 screen](../apps/twenty48/README.md)
- [Tetris screen](../apps/tetris/README.md)
- [Flappy screen](../apps/flappy/README.md)

### Services

- [Orientation](../services/orientation/README.md)
- [Paint](../services/paint/README.md)
- [Suit geometry](../services/suit_geometry/README.md)
- [Input audio policy](../services/input_audio_policy/README.md)
- [Audio resource policy](../services/audio_policy/README.md)
- [Battery monitor](../services/power/README.md)
- [Calculator math](../services/calculator_math/README.md)
- [Shared status](../services/state/README.md)
- [Microphone recording](../services/microphone/README.md)
- [Game high scores](../services/scores/README.md)
- [Snake game](../services/snake_game/README.md)
- [2048 rules](../services/twenty48_game/README.md)
- [Tetris rules](../services/tetris_game/README.md)
- [Flappy rules](../services/flappy_game/README.md)

### Drivers

- [Input](../drivers/input/README.md)
- [OLED](../drivers/oled/README.md)
- [Board support](../drivers/hw/README.md)
- [MPU acquisition](../drivers/mpu6500/README.md)
- [SD access](../drivers/storage_sd/README.md)

Further module explanations are still needed. This list is not full coverage.

## Project-wide material

- [Architecture](architecture-proposal.md)
- [Work queue](../TODO.md) and [progress](../PROGRESS.md)
- [File inspection checklist](documentation/file-checklist.md)
- Historical material: reference/. Earlier technical reviews are archived under
  reference/technical/ and may describe old code.

Writing focuses on the module's decisions and behaviour. Explain each concept
once and link to its owner. Move history and pending work into the progress/work
queue rather than repeating them in each explanation.
