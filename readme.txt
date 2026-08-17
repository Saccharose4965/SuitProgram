SuitProgram (ESP32 wearable shell)
==================================

This repository is an ESP-IDF firmware project for an ESP32 wearable platform
with OLED, audio, Bluetooth, IMU/orientation, LEDs, SD card, and menu-driven
apps.

The runtime is centered around the shell in `main/app_shell.c`.


Project intent and handoff summary
----------------------------------
This is a multi-suit TRON-style wearable platform, not a single-purpose LED
sketch. The intended end state is several suits that can run spatial LED
animations, react to live audio, play authored song shows in sync, exchange
state/files, and expose utilities and games through a small OLED shell.

Keep the existing high-level separation:
- `main/` is the composition root: shell runtime plus thin app adapters.
- `components/` owns reusable hardware, media, analysis, communication, layout,
  rendering, and game engines.
- `tools/` owns host-side layout and show-authoring utilities.
- `shows/` contains editable source projects; exported runtime packages belong
  on SD rather than in the firmware image.

The shell app contract (`init`, `deinit`, `tick`, `handle_input`, `draw`) and
the spatial LED layout are strong foundations. Prefer extending those contracts
over adding alternate entry points or direct hardware access from apps.

The project is an advanced work in progress. The main areas that still need
stabilization are communication security, audio/shared-resource ownership,
FFT/BPM reliability, show audio integration, real power/GPS integration, and
reproducible builds.
See `Current TODO / roadmap` near the end of this file before making broad
changes.


Build and flash
---------------
Environment:
- ESP-IDF from `/home/anotherone/Documents/suit/esp-idf`

Typical commands:
- `source /home/anotherone/Documents/suit/esp-idf/export.sh`
- `idf.py -p /dev/ttyUSB0 -b 115200 flash monitor`

Reproducibility caveat:
- The repository currently relies on that local ESP-IDF checkout and does not
  pin/bootstrap the exact IDF and Python environment.
- The last inspected local IDF Python environment was incomplete after a Python
  version change, so a clean full build could not be reproduced from the README
  alone. Fix the toolchain/bootstrap before treating build failures as firmware
  regressions.
- Do not commit real Wi-Fi credentials through `main/Kconfig`, `sdkconfig`, or
  backup configuration files. Rotate any credentials that were committed and
  move local values to an untracked/provisioned configuration path.


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
- `shell_run_loop()` targets `16667 us` period (~60 FPS)


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
4. `led_layout_init()`
5. `show_runtime_init()`
6. `shell_setup_link()`
   - `link_init(...)`
   - if successful: `link_set_frame_rx(...)` and `link_start_info_broadcast(1000)`
7. `shell_seed_initial_system_state()`
8. `shell_init_input_and_apps()`
9. `shell_profiler_init()`
10. `shell_run_loop()`

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
- `leds_animations`
- `leds_source`
- `leds_color`
- `leds_layout`
- `manual_bpm`
- `fft_sync`
- `music`
- `show_player`
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
- LEDs
- Settings
- Games
- Simulations
- Comm
- Music
- Shows
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

LED Animations (`main/app_leds.c`):
- `A/B`: move through the combined animation list
- `C/D`: select a beat-triggered (`beat:*`) or continuous synchronized
  (`sync:*`) animation; on configuration rows they decrement/increment or
  toggle the value
- Configuration rows currently include custom speed plus plane/ring backgrounds
- This app chooses animation behavior only; source/timing and color are separate

LED Source (`main/app_led_source.c`):
- Chooses `manual` or `fft` as the beat/synchronized timeline source
- Links to the Manual BPM and FFT Sync detail pages
- A slave controlled by Master Control ignores the local source and follows the
  received master clock

LED Color (`main/app_led_color.c`):
- Configures presets, `mono` / `duo` / `palette`, peak highlights, both RGB
  colors, audio-driven brightness range, and global brightness
- `A/B`: move through fields
- `C/D`: decrement/increment or cycle the selected field

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


FFT/BPM pipeline and local benchmark
------------------------------------
The firmware FFT component is a live microphone pipeline. It captures audio in
the background, computes spectral/energy views, maintains novelty history, runs
tempo/phase analysis in a worker, and exposes a beat clock to the LED source and
Master Control. Current shared configuration is 16 kHz input, FFT size 1024,
256-sample hop, and an 80..159 BPM search range.

Treat its BPM confidence and phase output as experimental. It is useful and
feature-rich, but not reliable enough yet to be the final synchronization source.

The sibling directory `../localFFT` is the PC-side research workbench for
improving this pipeline. It is not part of the ESP-IDF build. In particular,
`../localFFT/benchmark/bpm_bench.c` currently:
- reads a manifest of labeled songs and decoded mono f32le inputs
- evaluates tracks as one continuous stream so detector state crosses song
  boundaries like a live product
- uses 22.05 kHz input, FFT size 512, 32 comb bands, an 8-second analysis window,
  and an 80..159 BPM candidate range
- compares local, accumulated, and held/final BPM estimates
- writes summary, trace, and per-BPM score CSV files
- works with `benchmark/visualize_bpm.py` to plot BPM, stability, error, and
  score heatmaps

The current benchmark has produced very good tempo results, but remains WIP and
must be expanded before its strategy is ported back to the ESP32. Beat phase is
the unresolved part: many songs do not expose one unambiguous phase peak.

A likely product direction is explicit user correction rather than promising
perfect automatic phase. Candidate controls, still to be discussed, include a
phase choice such as `normal` / `offbeat` and tempo/subdivision choices such as
`1x` / `2x` / `3x`. Phase offset and tempo multiplier are different concepts
and should probably remain separate settings. Do not lock this API or UX without
first reviewing benchmark evidence and real suit behavior.


Multi-suit show authoring system
--------------------------------
`tools/show_editor/main_window.py` is a desktop PC application for designing
song-timed LED animation shows shared by multiple suits. It is not an on-device
ESP32 UI. The editor is a substantial but unfinished tool, currently centered
on one integrated Qt window.

The intended workflow is:
1. Import/reference a song and define BPM plus beat offset.
2. Author layered effect clips on global and per-role (`A`, `B`, `C`) lanes.
3. Target canonical suit sections/groups rather than hard-coded LED indices.
4. Preview the same timeline against 2D/3D suit layouts.
5. Save editable JSON source in `shows/`.
6. Export a compact `show.bin` package, plus audio when needed, for SD playback.
7. Have a master suit start/synchronize playback while each suit renders its
   assigned role.

Implemented editor pieces include timeline clip editing, snapping/beat guides,
audio transport and WAV waveform support, layered role lanes, presets/inspector,
undo/redo, autosaved sessions, 2D/3D LED preview, and binary export. The three
current JSON projects export successfully.

Important limitations:
- This is WIP authoring software; effect-specific controls and workflow still
  need refinement.
- `main_window.py` has grown very large and should eventually be split into
  timeline, preview, inspector/edit-history, audio transport, and window/controller
  modules without changing the saved-project format casually.
- The firmware now validates `show.bin`, loads one role into PSRAM/RAM, resolves
  canonical targets against the local layout, and renders the compiled effect
  set from the absolute timeline.
- `show_player` can choose role A/B/C, start a show as authority, pause/resume,
  and stop. Followers auto-load the matching local package and catch up from
  periodic transport packets. `M`/`F` identify authority/follower and `*`
  indicates calibrated clock sync.
- Playback deliberately sends transport state, not LED frames. Every suit must
  have the same exported show directory on SD; the content-derived show UID and
  package CRC reject stale or corrupt copies.
- `track.wav` playback, an audio-output lease, audio-delay calibration, seek UI,
  control authentication, and hardware multi-suit rehearsal remain unfinished.

Related design documents:
- `docs/show_system_requirements.md`
- `docs/show_system_spec.md`
- `docs/show_editor_ux_plan.md`
- `tools/show_editor/README.md`


Engineering notes from the current review
-----------------------------------------
Strong foundations worth preserving:
- shell-owned app lifecycle and framebuffer composition
- asynchronous/dirty OLED updates
- mutex-protected system-state snapshots
- centralized pin/shared-SPI definitions
- spatial LED layout and per-suit persisted section lengths
- separate FFT/BPM worker and built-in shell profiler
- desktop compilation of rich show projects into a simpler runtime format

Highest-risk implementation debt:
- Telemetry sender and receiver currently disagree on upload-header offsets.
  File names, total sizes, sequence/source identity, timeouts, and all packet
  bounds need validation; received names must not permit path traversal.
- UDP fallback now feeds framed packets through the normal decoder. ACK waiting
  is armed before send and matches sequence/source. ESP-NOW control frames are
  still unauthenticated.
- The shared SPI registry can retain an IMU handle after the orientation service
  removes that device for a lower-speed retry.
- GPS uses UART0 console pins and globally disables logging from its task.
- Link startup state can be overwritten back to `CONNECTING` after successful
  initialization. HUD battery percentages are placeholders.
- Show playback now takes an exclusive LED-output lease and restores the prior
  continuous/beat/audio-brightness settings on stop. Audio, SD, OLED, and
  service task stopping still need consistent ownership/lifecycle rules.
- LED output now sends each RMT channel at its configured physical length and
  caches logical-to-physical mapping until the layout changes.
- Several large files now mix responsibilities. Split internally rather than
  creating a component for every small helper.
- No automated tests or CI currently protect protocols, parsers, layout mapping,
  show packages, menu/registry consistency, or service state machines.


Repository layout
-----------------
`main/`:
- Shell and app wrappers listed in `main/CMakeLists.txt`

`components/`:
- Reusable modules: hardware, oled, input, audio, bt_audio, fft, link, gps,
  orientation, storage, games, etc.

`tools/`:
- Layout preview and SD-export helpers for the chestplate geometry
- Qt multi-suit song/show editor, preview model, and binary exporter

`shows/`:
- Editable JSON source projects for song-timed, per-role LED shows

Sibling research directory:
- `../localFFT/`: PC FFT/BPM experiments and offline benchmark; not part of
  the ESP-IDF component graph

Legacy entrypoint files still present in `main/` but not in build list:
- `main/reciever.c`
- `main/sender.c`


Known caveats (current code)
----------------------------
- `gps` and `call` are menu stubs with no registered shell app. `message` is
  registered and partially implemented.
- Global long `BC combo` restart is reserved by shell and preempts app-local
  handling.
- `led_modes_start()`, `power_monitor_start()`, and `gps_services_start(9600)`
  are not started during shell boot. LED modes start on demand from the LED app;
  power and GPS remain unwired.
- The show player performs silent timeline playback and multi-suit transport;
  synchronized `track.wav` audio is not wired yet.
- Current battery percentages are placeholders, not trustworthy measurements.


Current TODO / roadmap
----------------------
Stabilize security, protocols, and hardware first:
- [ ] Rotate/remove committed Wi-Fi credentials and establish an untracked or
  provisioned secrets workflow.
- [ ] Pin the ESP-IDF/toolchain version, add a repeatable bootstrap/environment
  check, and move stable configuration into `sdkconfig.defaults`.
- [ ] Replace telemetry packet offset logic with one versioned encode/decode
  implementation used by both sender and receiver.
- [ ] Validate upload size, bounds, filename, source, sequence, timeout, and
  completion state; reject traversal and malformed packets.
- [x] Fix framed UDP receive, the send-before-ACK-clear race, and ACK matching.
- [ ] Define authentication/encryption expectations for suit control traffic.
- [x] Queue show-player network callback work into its service queue instead of
  loading/rendering inside Wi-Fi/ESP-NOW callbacks. Apply the same boundary to
  remaining callback-driven app protocols.
- [ ] Fix the shared-SPI stale device handle during IMU speed fallback.
- [ ] Move GPS away from UART0 console pins and remove the global log shutdown.
- [ ] Fix startup connection-state ordering and distinguish placeholders from
  live status values.

Make ownership and lifecycle explicit:
- [x] Establish exclusive LED output ownership between base beat/continuous
  renderers and authored show playback. Add a compositor only if simultaneous
  base+show blending becomes a real requirement.
- [ ] Add an audio arbiter/lease model for FFT capture, microphone recording,
  local playback, Bluetooth playback, calls, and sound effects.
- [ ] Add synchronized/refcounted SD mounting and prevent remount/unmount while
  files are in use.
- [ ] Make complete OLED command/data transfers atomic, ideally with the shell
  as the only display owner.
- [ ] Replace cross-core `volatile` stop/state flags with task notifications,
  event groups, locks, or atomics as appropriate.
- [ ] Give services consistent `start` / `stop` / `status` behavior and avoid
  forgetting task handles before a task has actually exited.
- [ ] Make app descriptors the routing source of truth and validate/derive menu
  IDs so dead entries cannot drift from the registry.

Optimize measured hot paths:
- [x] Send each RMT channel only its configured physical strip length instead of
  721 pixels on both channels.
- [x] Precompute logical LED -> strip/physical mappings when the layout changes.
- [ ] Use `shell_profiler` measurements before doing general CPU/memory cleanup.
- [ ] Revisit the 16 MB flash partition table for OTA A/B images and useful
  asset/storage space.

FFT/BPM research:
- [ ] Treat the live firmware FFT/BPM/phase pipeline as provisional and use
  `../localFFT/benchmark` as the decision environment.
- [ ] Expand the labeled corpus across genres, intros, breakdowns, silence,
  tempo changes, noisy live input, ambiguous half/double tempo, and transitions
  between songs.
- [ ] Add reproducible per-track and transition metrics: convergence time,
  strict BPM error, false switches, confidence calibration, CPU time, and memory.
- [ ] Make benchmark parameters/configurations explicit so candidate strategies
  share inputs, preprocessing, windows, and scoring.
- [ ] Compare tempo strategies against the current accumulated normalized comb
  score and hold/switch policy; preserve plots/traces for failed cases.
- [ ] Investigate phase separately from BPM: onset/novelty phase stability,
  ambiguity scoring, drift, and tracks with multiple plausible peaks.
- [ ] Discuss the intended manual correction UI before implementation. Current
  ideas are `normal` / `offbeat` phase plus separate `1x` / `2x` / `3x`
  tempo or subdivision selection; naming and semantics are not decided.
- [ ] Decide when automatic phase is trustworthy, when to keep the previous
  phase, and when to ask the wearer for input.
- [ ] Only port a benchmark strategy after measuring ESP32 real-time CPU, PSRAM,
  latency, task interaction, and live-microphone behavior.

Multi-suit show system:
- [ ] Continue the PC editor as the authoring source of truth; do not move its
  editing complexity into firmware.
- [ ] Split `tools/show_editor/main_window.py` by responsibility while
  preserving project/session behavior and export compatibility.
- [ ] Refine effect-specific editing, selection, audio transport, timeline UX,
  preview accuracy, presets, and schema migration/versioning.
- [x] Complete package bounds validation and CRC checking on firmware.
- [x] Implement bucketed, allocation-conscious show clip evaluation and spatial
  LED rendering for the local suit role.
- [x] Implement suit role selection, master start, ping/pong clock offsets,
  periodic play/pause/stop transport, late catch-up, and standalone degraded
  playback when peers disappear.
- [ ] Add seek controls, reusable shared clock-sync extraction, authenticated
  control traffic, reconnect stress tests, and measured drift correction over
  full-length songs.
- [ ] Integrate `track.wav` ownership/playback and configurable audio delay.
- [x] Add exporter tests for v1.1 offsets, CRC, timing/color suffixes, and
  content-derived UID changes.
- [ ] Add firmware-parser golden/corrupted package tests and deterministic
  preview/runtime effect parity tests.

Structure, tests, and unfinished features:
- [ ] Add host tests for link/telemetry codecs, LED layout mapping, show packages,
  menu/registry consistency, NMEA parsing, and audio/service state machines.
- [ ] Add CI, a formatter configuration, and a warning baseline.
- [ ] Split large mixed-responsibility files internally: shell runtime/registry/
  boot/link bridge; LED driver/mapping/color/effects; Master Control protocol/
  controller/view; Message storage/capture/transport/UI; show-editor widgets and
  controllers.
- [ ] Audit and then archive/remove excluded backups, copied C files, old
  entrypoints, Python bytecode, and stale sdkconfig copies.
- [ ] Replace placeholder menu icons and complete real GPS, power logging,
  message/call, Pong synchronization, game sound, and service-restart behavior.
- [ ] Add real battery voltage/current measurements before broadcasting suit
  telemetry.
- [x] Implement the first synchronized handcrafted-song runtime through the show
  system rather than one-off firmware code.
- [ ] Rehearse it on all physical suits, tune role spacing/effects against the
  real layouts, and finish synchronized audio.


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

- general optimizations and better cpu ressource management
- code cleanup: what are apps doing in main ? why are there other patches of code, bad structure
- fix or change fft method (confidence, autosync with single phase peak musics, ...)
- Menu/UI: replace placeholder icons
- games sfx, conflict with audio player playing sfx ?
- fix message and call, esp communication in general
- fix Pong host election/ACK UI
- handcrafted led animations on specific music

Not yet wired:
- GPS time and position
- Power: integrate current reading and store every now and then in sd card
- accelerometer synth (theremin like? idk)
- legs
- gesture menu selection ?

Extras:
- do spatial aware led animations based on limb orientation
- wide putin
- create audio file that when displayed on the spectrogram, spell out words, as hidden messages
- tron
- doom
- pacman
- asteroids game !
