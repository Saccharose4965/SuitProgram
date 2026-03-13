SuitProgram (ESP32 wearable shell)
==================================

This repository is an ESP-IDF firmware project for an ESP32 wearable platform
with OLED, audio, Bluetooth, IMU/orientation, LEDs, SD card, and menu-driven
apps.

The runtime is centered around the shell in `main/app_shell.c`.


Build and flash
---------------
Environment:
- ESP-IDF from `/home/anotherone/Documents/suit/esp-idf`

Typical commands:
- `source /home/anotherone/Documents/suit/esp-idf/export.sh`
- `idf.py -p /dev/ttyUSB0 -b 115200 flash monitor`


Current runtime architecture
----------------------------
Entry point:
- `main/app_main.c`
  - Creates `shell_task` pinned to core 1
  - Task config: stack `12288`, priority `5`
  - `shell_task` calls `app_shell_start()`

Shell ownership (`main/app_shell.c`):
- App registry (`s_builtin_apps`)
- App switching queue/state
- Global input handling
- Frame composition (HUD/content/legend)
- OLED submit path

Shell frame cadence:
- `shell_run_loop()` targets `33 ms` period (~30 FPS)


Startup sequence (exact current flow)
-------------------------------------
`app_shell_start()` does:
1. `shell_init_hw_and_display()`
   - `hw_spi2_init_once()`
   - `hw_gpio_init()`
   - `oled_init()`
   - `oled_clear()`
   - starts `oled_task`
   - plays startup OLED logo animation (`anim_logo()`)
   - `shell_audio_init_if_needed()`
   - starts async startup tone task (`audio_play_tone(440, 1000)`)
   - `orientation_service_start()`
2. `system_state_init()`
3. `app_settings_init()`
4. `shell_setup_link()`
   - `link_init(...)`
   - if successful: `link_set_frame_rx(...)` and `link_start_info_broadcast(1000)`
5. `shell_seed_initial_system_state()`
6. `shell_init_input_and_apps()`
7. `shell_profiler_init()`
8. `shell_run_loop()`

Intentionally commented out in startup:
- `led_modes_start()`
- `power_monitor_start()`
- `gps_services_start(9600)`

Important:
- `shell_setup_link()` does not mount SD and does not start `audio_rx`.
- File reception is now on-demand through the `file_rx` app.


Input model
-----------
Input source:
- `components/input/input.c`
- Events: `INPUT_EVENT_PRESS`, `INPUT_EVENT_LONG_PRESS`
- No release events

Button decode levels (mV targets):
- `A`: 825
- `AB combo`: 1100
- `B`: 1650
- `BC combo`: 2200
- `C`: 2475
- `D`: 3300

Timing constants:
- Debounce: `25 ms`
- Long press: `1200 ms`
- Level tolerance: `+/-220 mV`

Global shell actions (pre-app):
- Long `AB combo` -> switch to `menu`
- Long `BC combo` -> restart (`esp_restart()`)


App registry (current)
----------------------
From `main/app_shell.c` (`s_builtin_apps`):
- `title`
- `menu`
- `status`
- `volume`
- `preferences`
- `master_control`
- `service_restart`
- `adc_debug`
- `calculator`
- `leds_audio`
- `leds_custom`
- `leds_layout`
- `manual_bpm`
- `fft_sync`
- `music`
- `bt`
- `message`
- `file_rx`
- `keyboard`
- `fft`
- `fluid`
- `stickman`
- `threedee`
- `bad_apple`
- `pong`
- `snake`
- `flappy`
- `t2048`
- `tetris`

Boot app:
- Shell boots directly to `menu`.

Flag note:
- No app currently sets `SHELL_APP_FLAG_EXTERNAL`.


Menu map (current)
------------------
From `main/app_menu.c`:

Root:
- Settings
- Games
- Simulations
- Comm
- LEDs
- Music
- Misc

Settings:
- Back
- Status
- ADC Debug
- Bluetooth
- Volume
- Preferences
- Master Control
- Svc Restart
- Restart

Simulations:
- Back
- GPS
- FFT
- Fluid
- Arm Pose
- 3D Render

Games:
- Back
- Flappy
- 2048
- Tetris
- Pong
- Snake

Comm:
- Back
- Bluetooth
- Message
- Call
- File RX

LED:
- Back
- Audio Reactive
- Manual BPM
- FFT Sync
- Custom
- Layout Edit

Misc:
- Back
- Keyboard
- Calculator
- Bad Apple

Current menu stubs (not registered as apps):
- `gps`
- `call`

Selecting these logs an unknown app warning and remains in menu.


Per-app controls (current wiring)
---------------------------------
Menu (`main/app_menu.c`):
- `A`: up
- `B`: down
- `D`: select

Status (`main/app_status.c`):
- Read-only status view

Volume (`main/app_volume.c`):
- `A`: volume +0.1
- `B`: volume -0.1
- `D`: mute toggle

Preferences (`main/app_preferences.c`):
- `A`: choose shell direction icons
- `B`: choose 2048 direction icons
- `C`: toggle direction icon family

LED Audio / LED Custom (`main/app_leds.c`):
- `A`: up in list
- `B`: down in list
- `D`:
  - on `Back`: return to `menu`
  - on a new mode item: apply the mapped mode for the current page
  - on the current `plane` or `ring` row: cycle to the next variant
- LED Audio and LED Custom use smooth menu scrolling for selection changes
- Both pages expose the same animation catalog; the Audio page runs the beat/audio-reactive mapping and the Custom page runs the continuous renderer mapping
- `plane` variants:
  - Audio: `plane:sweep`, `plane:pair`, `plane:fan`, `plane:sweep+bg`, `plane:pair+bg`, `plane:fan+bg`
  - Custom: `plane:sweep`, `plane:mirror`, `plane:prism`, `plane:sweep+bg`, `plane:pair+bg`, `plane:fan+bg`
  - `+bg` uses the secondary color as a dim static background and renders the plane brighter on top
- `ring` variants:
  - Audio: `ring:pulse`, `ring:train`
  - Custom: `ring:pulse`, `ring:contour`
- Shared config exposed on both pages:
  - color preset / style / highlight
  - primary + secondary RGB values
  - `audio:*`
  - `bright:*`
- Audio `random` rotates through the beat-reactive animation list about every 10 seconds
- Custom page config:
  - `speed:%` controls free-running custom animation speed (`10%` to `250%`, step `10%`)
  - `fill` is the full-strip/full-layout coverage mode using the active selected color source
  - `random` rotates through the custom animation list about every 10 seconds

LED Layout (`main/app_led_layout.c`):
- `A`: previous field
- `B`: next field
- `C`: decrement current field
- `D`: increment/apply current field
- Long press `C/D`: coarse decrement/increment
- Fields:
  - Back
  - Side (`left` / `right`)
  - Part index within selected side
  - Length
  - Reversed
  - Preview (`off` / section / runner)
  - Save
  - Reload
  - Reset default

Manual BPM (`main/app_manual_bpm.c`):
- `A/B`: BPM down/up
- `C/D`: trigger offset back/forward
- Progress bar shows cycle position
- Marker shows the beat trigger offset within the cycle

Master Control (`main/app_master_control.c`):
- `A/B`: move up/down through the settings rows
- `C/D`: change the selected row value
- Rows:
  - `ratio`: `match`, `1/2`, `2/1`
  - `phase`: `0`, `offbeat`
  - `color`: `tron`, `match`, `free`
  - `control`: `off`, `on`
- When `control` is `on`, the device becomes the timing master and keeps syncing in the background
- It mirrors the real active beat source (`Manual BPM` or `FFT Sync`) instead of owning BPM or animation itself
- `tron` forces master blue and slave orange
- `match` sends the current beat primary/secondary colors
- `free` leaves colors local on each device while still syncing mode/timing

FFT Sync (`main/app_fft_sync.c`):
- `A/B`: shift FFT beat trigger phase backward/forward
- `C`: lock/unlock current BPM
- `D`: enable/disable FFT beat export
- Shows effective BPM, detected BPM, confidence, current cycle position, and trigger marker

Calculator (`main/app_calculator.c` / `components/calculator/calculator.c`):
- `A`: move left
- `B`: move down
- `C`: move right
- `D`: press selected key
- After an error, the next insert key starts a fresh expression

Tetris (`components/tetris/tetris.c`):
- `A`: move left
- `D`: move right
- `B`: rotate left
- `C`: rotate right
- Rotation now uses explicit tetromino cells plus small wall kicks

Music (`main/app_music.c`):
- `A/B`: list navigation
- `C`: play/pause (or start selected if inactive)
- `D` press: restart selected
- `D` long press: stop playback

Bluetooth (`components/bt_audio/bt_audio_app.c`):
- `A/B`: select discovered device
- `C`: scan
- `D`: connect/disconnect based on selection and state

File RX (`main/app_file_rx.c`):
- `C`: start/stop `audio_rx`
- On start, mounts SD if needed, then calls `audio_rx_start(AUDIO_RX_DEFAULT_PORT)`

FFT (`main/app_fft.c`):
- `A`: previous FFT view
- `B`: next FFT view

3D Render (`components/threedee/threedee.c`):
- `D`: recenter orientation
- Draws a simple animated wireframe scene with multiple rotating shapes
- If IMU orientation is not ready yet, it still renders the neutral scene instead of showing a text prompt

Arm Pose (`main/app_stickman.c`):
- `A/B`: rotate the figure view left/right
- `C`: toggle tracked arm left/right
- `D`: recenter orientation
- Renders a simple projected stick figure with the selected arm driven by IMU orientation

Message (`main/app_message.c`):
- `A`: start/stop mic recording
- `B`: play/stop the current saved message
- `D`: send the current saved message
- Recording is forced to the left mic slot
- Records save to `/sdcard/messages/outbox`; incoming files go to `/sdcard/messages/inbox`

Keyboard/Fluid/Pong/Snake/Flappy/2048/Tetris:
- Shell forwards input/tick/draw to their app/component handlers.


Link and file reception
-----------------------
Link service (`components/link` via `shell_setup_link()`):
- Initialized once at startup
- Starts periodic info broadcast when init succeeds

Audio RX service (`components/audio_rx`):
- Not started at boot
- Controlled by `file_rx` app only
- Default TCP port: `5000`
- Output path pattern: `/sdcard/rx_rec_XXXXX.wav`


Hardware map (from `components/hw/include/hw.h`)
-------------------------------------------------
SPI2:
- MOSI `23`, MISO `19`, CLK `18`
- CS: OLED `5`, SD `21`, IMU1 `14`, IMU2 `26`, IMU3 `27`
- OLED DC `13`, OLED RST `NC`

I2S:
- BCLK `4`, LRCK `22`, DOUT `25`, DIN `36`

ADC:
- Hall A `34`, Hall B `35`, buttons ladder `39`

LED strips:
- GPIO `32`, GPIO `33`

GPS UART:
- UART0 TX `1`, RX `3`

SD mount point:
- `/sdcard`


LED layout system (current)
---------------------------
The LED runtime is now driven by a persisted spatial layout model rather than
hard-coded strip assumptions.

Current storage/runtime behavior:
- Layout file path: `/sdcard/led_layout.txt`
- `led_layout_init()` loads it at runtime
- If the file is missing, a default chestplate layout is created in memory and
  written to SD when possible
- Layout state includes:
  - profile name
  - strip names / strip lengths
  - per-section strip assignment
  - per-section LED count
  - reverse flag
  - sampled torso-space geometry (polyline / arc)

Current chestplate topology:
- 2 physical strips / 2 GPIO outputs
- `left` strip has 9 sections:
  - `front_left_top`
  - `front_left_ring`
  - `front_left_rib`
  - `front_left_abs`
  - `front_left_belt`
  - `back_left_belt`
  - `back_left_vertebra`
  - `back_left_rib`
  - `back_left_top`
- `right` strip has 8 sections:
  - `front_right_top`
  - `front_right_rib`
  - `front_right_abs`
  - `front_right_belt`
  - `back_right_belt`
  - `back_right_vertebra`
  - `back_right_rib`
  - `back_right_top`

Why this matters:
- Logical LED order is now mapped onto two physical strips using the layout
- Animations can query per-LED torso coordinates
- Per-suit variants can keep the same geometry while changing per-section LED
  counts on SD
- The layout editor shows indices per strip, so both strips start at `0` for
  hardware/debug purposes

Layout authoring / debug workflow:
- On device: `LEDs -> Layout Edit`
  - change side, section length, reverse flag, preview mode
  - save / reload / reset default
  - preview the selected section live on the strips
- On host: Python tools under `tools/`
  - `tools/layout_model.py`: normalized chestplate geometry and sampling model
  - `tools/render_front_layout.py`: SVG preview renderer
  - `tools/generate_layout_sd.py`: export layout text file for SD

Animation model notes:
- Custom geometry-aware modes and several audio-reactive modes now use the
  sampled layout coordinates
- Audio mode `audio_energy` is a continuous FFT-driven equivalent of the custom
  `energy` pattern rather than a simple beat flash
- Color handling is now explicit:
  - `mono`
  - `duo`
  - `palette`
  - optional white highlights via highlight mode


Repository layout
-----------------
`main/`:
- Shell and app wrappers listed in `main/CMakeLists.txt`

`components/`:
- Reusable modules: hardware, oled, input, audio, bt_audio, fft, link, gps,
  orientation, storage, games, etc.

`tools/`:
- Layout preview and SD-export helpers for the chestplate geometry

Legacy entrypoint files still present in `main/` but not in build list:
- `main/reciever.c`
- `main/sender.c`


Known caveats (current code)
----------------------------
- `gps`, `message`, and `call` menu entries are stubs (unregistered app IDs).
- Global long `BC combo` restart is reserved by shell and preempts app-local handling.
- `led_modes_start()`, `power_monitor_start()`, and `gps_services_start(9600)` are still commented out in shell startup.


Legacy notes from original README
---------------------------------
This section intentionally preserves useful project context from the original notes.
Treat this as a design/backlog appendix and validate against code before relying on details.

Legacy hardware context:
- Target board: ESP32-WROVER-E (16 MB), flashed over USB-UART at 115200.
- Known historical MACs: `24:d7:eb:6b:26:40` and `24:d7:eb:6b:26:c8`.
- SD conventions: mount at `/sdcard`, hidden boot counter file `.rec_counter`.
- Original parts checklist (historical): ESP32 module, IMUs, microSD, SSD1309 OLED, NEO-6M, INMP441, MAX98357A, WS2815 + 74HCT level shifter, ACS712 + divider, 3S pack + BMS, regulators, hall sensor, ladder buttons, speaker, harness/connectors.

Extended module inventory (components in tree, not all shell-wired):
- Storage: SDSPI mount retries at 20/10/5/2/0.4 MHz; directory listing helper; boot counter helper.
- Audio core: shared I2S TX/RX, sample-rate retime, WAV/tone/embedded playback, Q1.15 gain path.
- Audio capture: PSRAM-backed ring capture path and latest-window reads.
- Audio player + BT: plays via A2DP when connected, local fallback otherwise; BT scan/connect UI in shell.
- Microphone recorder: WAV recorder component exists for `/sdcard/rec_XXXXX.wav` workflows.
- FFT: beat/BPM + multi-view OLED rendering + novelty-hole helper for button noise suppression.
- Power: divider-based pack voltage monitor with `POWER_*` menuconfig thresholds.
- Telemetry: Wi-Fi UDP sender/receiver utilities (standalone module).
- Link: framed ESP-NOW/Wi-Fi path with optional ACK and periodic info broadcast.
- Audio RX: TCP server to `/sdcard/rx_rec_XXXXX.wav` with persisted `rx_rec_index.txt` and `rx_rec_last.txt`.
- Standalone comms/call: UDP voice modules (`components/comms`, `components/call`) exist but are not part of shell app registry.
- IMU/Stick and logo helpers: `components/mpu6500`, `components/stick`, `components/logo` remain available.

Power / thermal notes (legacy hardware sizing):
- WS2815 original estimate: ~15 W idle, ~24 W pulse, ~40 W full-white for ~5 m at 144 LED/m.
- 4-hour sizing estimate in original notes: roughly ~160 Wh.
- Keep LED data level-shifted (74HCT family) and grounds shared between 12 V and logic rails.

Open TODO / wishlist (legacy backlog)
-------------------------------------

- Menu/UI: replace placeholder icons
- games sfx, conflict with audio player playing sfx ?
- improve fft confidence and stop_blinking mechanic (we want to keep blinking under noisy music, but stop under musical noise : not easy!)
- try to optimize fluid further without impacting on behavior, (we already managed to get it to run smooth on one core!)
- fix and bring back GPS time and link quality.
- fix GPS/message/call menu items to real apps
- Link/comms: define structured bundles (leaderboard/GPS/messages/audio headers), add per-type ACK/retry, and integrate link frames into games/remote LED paint tools.
- fix Pong host election/ACK UI
-> test w/ F.U.Y.A. music
- score com between costumes
- accelerometer synth (theremin like? idk)
- log of novely for fft?
- autosync with single phase peak musics ? that's work !

- do spatial aware led animations based on limb orientation
- Power: integrate current reading and store every now and then in sd card

Extras:
- wide putin
- create audio file that when displayed on the spectrogram, spell out words, as hidden messages
- port more titles (tron/doom/pacman/etc.)
- asteroids game !
