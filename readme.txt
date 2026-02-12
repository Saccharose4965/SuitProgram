SuitProgram (ESP32 wearable shell)
==================================

This repository is an ESP-IDF firmware project for a wearable OLED + audio + LED platform.
The app is organized around a shell (`main/app_shell.c`) that owns startup, input polling,
frame composition, and app switching.

What this README covers
-----------------------
- Current runtime architecture (as implemented now)
- Startup sequence and service ownership
- Input model and global combos
- App registry and menu mapping
- Hardware mapping and shared buses
- Central files to read first
- Known caveats and next cleanup targets


Build and flash
---------------
Environment:
- ESP-IDF (same toolchain used by your workspace)

Typical commands:
- `source /home/anotherone/Documents/suit/esp-idf/export.sh`
- `idf.py -p /dev/ttyUSB0 -b 115200 flash monitor`

Notes:
- `app_main()` launches the shell task pinned to CPU1.
- UART0 is shared with GPS in this project. If GPS is enabled, console output on UART0 must be disabled.


Current runtime architecture
----------------------------
Entry point:
- `main/app_main.c`
  - Creates `shell_task` on core 1 (priority 5)
  - `shell_task` calls `app_shell_start()`

Shell ownership:
- `main/app_shell.c`
  - Owns app registry (`s_builtin_apps`)
  - Owns current app state and switch queue
  - Polls input and dispatches events
  - Draws HUD + content + legend (for non-external apps)
  - Submits frames to OLED worker task

Startup sequence in `app_shell_start()`:
1. `shell_init_hw_and_display()`
   - `hw_spi2_init_once()`
   - `hw_gpio_init()`
   - `oled_init()` + `oled_clear()`
   - starts `oled_task`
   - `shell_audio_init_if_needed()`
   - `orientation_service_start()`
2. `system_state_init()`
3. `app_settings_init()`
4. `shell_setup_link()`
   - `link_init(...)`
   - `link_set_frame_rx(...)`
   - `link_start_info_broadcast(1000)`
   - `storage_mount_sd()`
   - `audio_rx_start(AUDIO_RX_DEFAULT_PORT)`
5. `shell_seed_initial_system_state()`
6. `shell_init_input_and_apps()`
7. `shell_run_loop()` (33 ms frame cadence)

Intentionally disabled in startup right now (commented out):
- `led_modes_start()`
- `power_monitor_start()`
- `gps_services_start(9600)`

Important nuance:
- LED service still starts on demand when entering the LEDs app (`main/app_leds.c` calls `led_modes_start()` in `leds_app_init`).


Framebuffer and OLED flow
-------------------------
Shell-side frame buffers in `main/app_shell.c`:
- `s_fb`: shell render buffer (apps draw here)
- `s_fb_oled_a` / `s_fb_oled_b`: staging buffers for OLED task handoff

Why two OLED staging buffers:
- Shell can keep rendering into `s_fb` while OLED task is transmitting previous staged frame.
- `oled_submit_frame()` copies `s_fb` into alternating A/B buffers and notifies `oled_task`.

OLED driver behavior (`components/oled/oled.c`):
- Uses shared SPI2 bus
- `oled_blit_full()` converts framebuffer layout and performs per-page diff updates
- If a page changed heavily, sends whole page; otherwise sends changed runs only
- This reduces SPI traffic versus always-full-page writes


Input model (single ADC ladder)
-------------------------------
Input component:
- `components/input/input.c`
- Events emitted: `INPUT_EVENT_PRESS`, `INPUT_EVENT_LONG_PRESS`
- No release events

Decoded logical buttons:
- `A`, `B`, `C`, `D`
- `AB combo` (around 1.1V)
- `BC combo` (around 2.2V)

Defaults in code:
- Debounce: 25 ms
- Long press: 1200 ms
- Ladder tolerance: +/- 220 mV

Global shell actions (handled before app handlers):
- Long `AB combo` -> switch to `menu`
- Long `BC combo` -> `esp_restart()`

Implication:
- App-local long-press handling for `BC combo` will not run, because shell consumes it globally.


App registry and menu mapping
-----------------------------
Registered shell apps (`main/app_shell.c` -> `s_builtin_apps`):
- `title` (blank)
- `menu`
- `status`
- `volume`
- `leds`
- `music`
- `bt`
- `keyboard`
- `fft` (external)
- `fluid`
- `threedee`
- `pong`
- `snake`
- `flappy` (external)
- `t2048` (external)
- `tetris` (external)

Boot behavior:
- Shell boots directly into `menu` (not `title`).

Menu structure (`main/app_menu.c`):
- Root: Settings, Games, Simulations, Comm, LEDs, Music
- Settings: Status, Bluetooth, Volume, Keyboard, Restart
- Simulations: GPS, FFT, Fluid, 3D Render
- Games: Flappy, 2048, Tetris, Pong, Snake
- Comm: Message, Call
- LED: LED Modes

Current stubs:
- `gps`, `message`, `call` are present in menu but not registered apps.
- Selecting them issues a switch request to unknown app id (logged warning) and stays in menu.


Per-app control summary (current wiring)
----------------------------------------
Menu:
- `A` up, `B` down, `D` select

Status:
- Read-only view of `system_state`

Volume:
- `A` louder, `B` quieter, `D` mute toggle
- Persisted to `/sdcard/suit_settings.txt`
- Applies to both local audio and BT percent mapping

LEDs:
- Menu-driven LED mode selection (audio-reactive and custom pages)
- Starts LED service on entry

Music:
- Scans `/sdcard` for `.wav`
- `A/B` move selection
- `C` play/pause (or start selected)
- `D` restart selected track
- long `D` stop

Bluetooth app:
- `A/B` select device
- `C` scan
- `D` connect/disconnect logic on current selection

FFT app:
- `A` previous view, `B` next view
- Views cycle through `FFT_VIEW_COUNT`

3D app:
- `D` recenter orientation

Snake:
- `A` left, `B` up, `C` right, `D` down

Pong:
- `A` pause toggle
- `B/C` paddle movement
- `D` toggle host/client role

External apps:
- `flappy`, `t2048`, `tetris` run their own task/loop and draw directly
- Shell skips framebuffer draw path while external app active


Hardware map (from `components/hw/include/hw.h`)
-------------------------------------------------
SPI2 shared bus:
- MOSI 23, MISO 19, CLK 18
- CS: OLED 5, SD 21, IMU1 14, IMU2 26, IMU3 27
- OLED DC 13, RST NC

I2S:
- BCLK 4, LRCK 22, DOUT 25, DIN 36

ADC:
- Hall A 34, Hall B 35, Buttons ladder 39

LED strips:
- GPIO32 / GPIO33

GPS UART:
- UART0 TX 1, RX 3

SD mount point:
- `/sdcard`


Shared buses and service boundaries
-----------------------------------
SPI2:
- Shared by OLED, SD, and MPU6500 devices
- Arbitration is through ESP-IDF SPI bus/device handles
- OLED transfers are staged via OLED task; SD and IMU still compete for bus time

I2S/audio:
- Single audio subsystem initialized once through `shell_audio_init_if_needed()`
- Used by local playback, BT source path, FFT sampling, and audio-related components

System state authority (`components/state`):
- Shell and services publish to `system_state`
- HUD/status read snapshots each frame


Link and networking
-------------------
Link module (`components/link`):
- Tries ESP-NOW first
- Also tries Wi-Fi telemetry path (SSID/pass/IP/port from `main/Kconfig`)
- Supports framed messages and optional ACK on ESP-NOW
- Broadcasts compact system-state info frames periodically

Audio RX service (`components/audio_rx`):
- TCP listener (default port 5000)
- Writes received streams to `/sdcard/rx_rec_XXXXX.wav`
- Uses ring buffer + writer task to decouple socket read and SD writes


Sensors and orientation
-----------------------
Orientation service (`components/orientation/orientation_service.c`):
- Starts IMU on SPI2 (CS IMU1)
- Runs sampler + orientation update task (~200 Hz)
- Exposes shared orientation context via `orientation_service_ctx()`

Used by:
- `threedee` app (view orientation)
- `fluid` app (acceleration-driven gravity in sim)

GPS component (`components/gps`):
- Has service APIs (`gps_service_start`, `gps_services_start`, time bridge)
- Not started by shell right now (startup call commented)


Central files (read first)
--------------------------
If you need to understand behavior quickly, start here in order:
1. `main/app_main.c` - shell task bootstrap
2. `main/app_shell.c` - startup, registry, input dispatch, frame loop
3. `main/app_menu.c` - menu tree and navigation
4. `components/input/input.c` - ladder decode and event model
5. `components/oled/oled.c` - OLED SPI write strategy (diff/page runs)
6. `components/hw/include/hw.h` - canonical pin map
7. `components/link/link.c` - ESP-NOW/Wi-Fi framing
8. `components/audio/audio.c` + `main/shell_audio.c` - shared audio bus init/use
9. `components/orientation/orientation_service.c` - IMU orientation service
10. `components/gps/gps.c` - GPS parsing/service APIs (currently not started)

App wrappers and shell app APIs:
- `main/shell_apps.h`
- `main/app_*.c`


Repository layout at a glance
-----------------------------
- `main/`:
  - Shell orchestration and app wrappers
  - Includes active app files listed in `main/CMakeLists.txt`
- `components/`:
  - Reusable modules (hw, oled, input, audio, bt_audio, fft, link, gps, orientation, games, etc.)
- `main/old.c`, `main/old2.c`, `main/sender.c`, `main/reciever.c`:
  - Legacy/experimental entrypoints, not in active main build list


Known caveats (current code state)
----------------------------------
- `gps`, `message`, `call` menu entries are stubs (not registered shell apps).
- Global long `BC combo` restart preempts any app-local long `BC` behavior.
- LED/power/GPS boot services are intentionally disabled in shell start (commented), so startup state is partly seeded/default.
- Heavy OLED updates plus SD/IMU activity can still produce contention and timing pressure despite OLED diff writes.


Short-term cleanup suggestions
------------------------------
- Register real apps or hide stub menu items (`gps`, `message`, `call`).
- Decide whether global long `BC` restart should remain hard-reserved.
- Move commented startup toggles to explicit runtime config or a single service init table.
- Consider a small bus/service dashboard in status view (SPI load hints, audio_rx state, GPS state).





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
