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
- `leds_audio`
- `leds_custom`
- `music`
- `bt`
- `file_rx`
- `keyboard`
- `fft`
- `fluid`
- `threedee`
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

Settings:
- Back
- Status
- Bluetooth
- Volume
- Keyboard
- Restart

Simulations:
- Back
- GPS
- FFT
- Fluid
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
- Custom

Current menu stubs (not registered as apps):
- `gps`
- `message`
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

LED Audio / LED Custom (`main/app_leds.c`):
- `A`: up in list
- `B`: down in list
- `D`:
  - on `Back`: return to `menu`
  - on mode item: apply selected mode

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
- `D`: return to `menu`
- On start, mounts SD if needed, then calls `audio_rx_start(AUDIO_RX_DEFAULT_PORT)`

FFT (`main/app_fft.c`):
- `A`: previous FFT view
- `B`: next FFT view

Keyboard/Fluid/3D/Pong/Snake/Flappy/2048/Tetris:
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


Repository layout
-----------------
`main/`:
- Shell and app wrappers listed in `main/CMakeLists.txt`

`components/`:
- Reusable modules: hardware, oled, input, audio, bt_audio, fft, link, gps,
  orientation, storage, games, etc.

Legacy entrypoint files still present in `main/` but not in build list:
- `main/old2.c`
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
- Transfer script note: original README referenced `transfer.sh`, but that script is not present in this repo snapshot.

Power / thermal notes (legacy hardware sizing):
- WS2815 original estimate: ~15 W idle, ~24 W pulse, ~40 W full-white for ~5 m at 144 LED/m.
- 4-hour sizing estimate in original notes: roughly ~160 Wh.
- Keep LED data level-shifted (74HCT family) and grounds shared between 12 V and logic rails.

Open TODO / wishlist (legacy backlog)
-------------------------------------
- measure real value adc to check ladder expected values
- don't blink when not confident
- zero out audio data upon button press and release to remove parasitic spikes
- implement blink phase tune in
- fix fluid sim (make it as close to goatedfluidsim.txt as possible)
- would bluetooth and fft be able to run at the same time ?
- fix led menu, have two sub menus : music synced and not.
- do spatial aware led animations based on limb orientation
- add button press to reset fft on all nearby costumes, need to think of where and when.
- allow to stop fft from running, first think how and where this should be triggered.
- check for completion of the todos that come after this one.
- increase and fix led framerate (might be a cabling issue, first try to do proper cabling.)
- separate the volume settings between volume for the imbeded speaker and the bluetooth volume
- games sfx
- Menu/UI: replace placeholder icons, measure real ladder voltages and tighten combo thresholds; optionally show GPS time status flag and link quality.
- Shell plumbing: wire GPS/message/call menu items to real apps or hide them; surface audio_rx status/controls.
- LED/FFT: replace `led_layout.c` with measured coordinates; add richer patterns (planes tied to IMUs, waves from chest ring, randomizer presets), improve FFT noise handling/tempo fallback.
- Link/comms: define structured bundles (leaderboard/GPS/messages/audio headers), add per-type ACK/retry, and integrate link frames into games/remote LED paint tools.
- Power: measure divider ratio and per-cell thresholds, add current sensing/export + low-battery guardrails.
- Games/content: polish Pong host election/ACK UI, add scores/levels to Snake, port more titles (tron/doom/pacman/etc.), add calculator and lightweight 3D renderer/virtual-plane LED effect.
- Extras: Tournai map on OLED, IMU fusion for limb pose to drive LEDs/stick, task priority/core pinning guidance for FreeRTOS.
- fix audio recording and calls
- create audio file that when displayed on the spectrogram, spell out words, as hidden messages

Task core pinning / priority notes (legacy tuning reference)
------------------------------------------------------------
Core 0 (Wi-Fi/BT friendly, lower contention):
- `audio_rx_wr` writer task (historical note said prio 3).
- BT/A2DP and many unpinned networking tasks often end up here.
- Telemetry RX, call/comms, LED modes, OLED/UI, GPS tasks may float here when unpinned.

Core 1 (heavy DSP/render):
- `audio_rx_task` (historical note said prio 1).
- `fft_task` visualizer (historical note said prio 5).
- `audio_capture_task` and IMU/orientation sampling were historically treated as DSP-side work.

Validation note:
- Pinning/priority details can drift over time; always confirm with current task creation sites before tuning.
