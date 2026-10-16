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
- `../ShowEditor/` owns desktop-only layout and show-authoring utilities; nothing there
  is compiled or flashed by ESP-IDF. Launch with `../ShowEditor/show-editor`.
- `../ShowEditor/shows/` contains editable source projects; exported runtime packages belong
  on SD rather than in the firmware image.

The shell app contract (`init`, `deinit`, `tick`, `handle_input`, `draw`) and
the spatial LED layout are strong foundations. Prefer extending those contracts
over adding alternate entry points or direct hardware access from apps.

The project is an advanced work in progress. FFT tracking and show playback
are now reported working well on hardware. Preserve that baseline while
completing physical leg mapping, validating multi-suit behavior, and addressing
security, power calibration and reproducible builds deliberately.
See `Current status and scoped backlog` before making broad changes.


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
- The local IDF Python 3.13 environment was repaired for the FFT port, and the
  full firmware builds in `build/fft-port`. A fresh checkout still needs the
  matching IDF/toolchain environment; it is not bootstrapped by this repository.
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

Call and message are registered shell apps; validate their individual audio
and transport routes on hardware before assuming all combinations work.


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
- `A/B`: select a control; `C`: apply it; `D`: reset the detector for a new song
- Controls: phase lock (also soft-locks BPM), forced BPM, 1x/2x/3x pulse ratio,
  offbeat, beat output pause, and earlier/later phase trim
- Shows held/base BPM, output BPM, candidate, decision confidence, evidence
  quality (threshold 1), current cycle position, and trigger marker
- Long presses do not toggle a control twice; global shell shortcuts are unchanged

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
- On host: Python tools under `../ShowEditor/show_editor/layout/`
  - `../ShowEditor/show_editor/layout/layout_model.py`: normalized chestplate geometry and sampling model
  - `../ShowEditor/show_editor/layout/render_front_layout.py`: SVG preview renderer
  - `../ShowEditor/show_editor/layout/generate_layout_sd.py`: export layout text file for SD

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
the background and exposes the same beat clock/audio-level APIs to LED modes
and Master Control. It now ports the PC `../localFFT/LatestFFT.c` treble-weighted
direct variance-comb detector: 44.1 kHz, 1024-sample non-overlapping FFT frames,
32 bands, four seconds of audio history, and an 80..159 integer BPM range.

`components/fft/fft_detector.c` is the portable math/state machine;
`components/fft/fft.c` owns I2S, task scheduling, controls and beat/LED export;
`fft_render.c` remains display-only. Audio capture runs at about 43 frames/sec;
the expensive comb is requested every 22 frames (~0.51 sec), not every frame.
The low-priority worker yields between BPM batches and skips new jobs if busy.
Do not lower capture rate or change band definitions to save CPU without a new
PC comparison: that changes the feature on which the tracker was evaluated.

See `docs/fft-detector.md` for exact parameters, controls, port differences,
host tests, profiling logs and the board acceptance checklist. The port builds
but still needs on-board latency/load, I2S, LED, playback and radio testing.
Confidence/evidence are threshold-readiness measures, not accuracy probabilities.

The sibling directory `../localFFT` is the PC-side research workbench for
improving this pipeline. It is not part of the ESP-IDF build. In particular,
`../localFFT/benchmark/bpm_bench.c` currently:
- reads a manifest of labeled songs and decoded mono f32le inputs
- evaluates tracks as one continuous stream so detector state crosses song
  boundaries like a live product
- supports direct, treble-weighted direct and onset-enhanced feature variants;
  the port uses the selected four-second treble-weighted direct configuration,
  not every alternative explored in the benchmark
- can mix repeatable speech/babble and irregular high-crest-factor click noise
- uses curated full songs plus approved excerpts, explicit label corrections,
  octave-folded targets and a separate holdout; no blues, jazz or classical
- excludes the first five seconds per song from scoring without telling the
  detector where songs start; sliding-BPM songs are separate bonus evaluation
- compares local, accumulated, and held/final BPM estimates
- writes per-track and overall accuracy/error, stability, feature cost, and BPM
  scoring cost; detailed trace and per-BPM score CSVs are optional
- works with `benchmark/visualize_bpm.py` to plot BPM, stability, error, and
  score heatmaps

`../localFFT/benchmark/run_sweep.py` is the normal entry point. It builds the C
runner, caches decoded audio, runs the selected window/noise matrix, and resets
`benchmark/output/index.html` for each run. Results appear progressively there;
audio listening and detailed feature exports are available with `--evidence`.
Source songs stay visible in `benchmark/audio_pool/`; generated cache is separate.
See `../localFFT/benchmark/README.md` and `EXPERIMENTS.md`. The user runs sweeps;
do not automatically start another tuning/search campaign during the port.

Phase stays automatic by default. Phase lock, offbeat and subdivision controls
are now implemented, but fractional-BPM drift correction is NOT implemented.
Following one spectral phase peak is not sufficient: a song can move its strong
energy from beat to offbeat without actually changing tempo. Keep that research
separate from this embedded port. The 30-second lost-pulse stop is independent
of volume and still needs non-music/crowd testing on the physical microphone.


Multi-suit show authoring system
--------------------------------
`../ShowEditor/show_editor/main_window.py` is a desktop PC application for designing
song-timed LED animation shows shared by multiple suits. It is not an on-device
ESP32 UI. The editor is a substantial but unfinished tool, currently centered
on one integrated Qt window.

The intended workflow is:
1. Import/reference a song and define BPM plus beat offset.
2. Author layered effect clips on global and per-role (`A`, `B`, `C`) lanes.
3. Target canonical suit sections/groups rather than hard-coded LED indices.
4. Preview the same timeline against 2D/3D suit layouts.
5. Save editable JSON source in `../ShowEditor/shows/`.
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
- Timeline and preview/worker widgets now live in `timeline_widget.py` and
  `preview_widget.py`; their method bodies were preserved during extraction.
  `main_window.py` still needs a later inspector/history/audio-transport split.
  Do not change the saved-project or exported firmware format casually.
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
- `../ShowEditor/docs/show_editor_ux_plan.md`
- `../ShowEditor/README.md`


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
- Link startup state can be overwritten back to `CONNECTING` after successful
  initialization. HUD battery percentages are placeholders.
- Show playback now takes an exclusive LED-output lease and restores the prior
  continuous/beat/audio-brightness settings on stop. Audio, SD, OLED, and
  service task stopping still need consistent ownership/lifecycle rules.
- LED output now sends each RMT channel at its configured physical length and
  caches logical-to-physical mapping until the layout changes.
- Several large files now mix responsibilities. Split internally rather than
  creating a component for every small helper.
- Host regression tests cover the FFT detector and show exporter. There is no
  CI yet, and protocols, layout mapping, firmware show parsing, menu/registry
  consistency and service state machines still lack automated coverage.


Repository layout
-----------------
`main/`:
- Shell and app wrappers listed in `main/CMakeLists.txt`
- App-local headers stay alongside their sources; this is the composition
  root, not a reusable component API. Do not make components depend on it.

`components/`:
- Reusable modules: hardware, oled, input, audio, bt_audio, fft, link,
  orientation, storage, games, etc.
- Convention: implementation `.c` files and `CMakeLists.txt` at component root;
  first-party `.h` files in `include/`; tests in `tests/` where present.
- Include headers by name (`#include "fft.h"`), with paths supplied by CMake,
  not relative paths into another component's source directory.
- Internal helper headers also live in `include/`, but are not application
  APIs: FFT clients use `fft.h`, not `fft_detector.h` or `fft_render.h`;
  `bt_audio_internal.h` is only for Bluetooth implementation files.
- List firmware sources explicitly in CMake. Do not glob in tests, generators,
  prototypes or backups. Use public dependencies for exposed header types and
  private dependencies for implementation details.
- Existing vendor exception: Bad Apple's bundled Heatshrink headers/sources
  retain their layout; its license lives in `third_party/`. Keep vendor changes
  separate from first-party header moves.

`../ShowEditor/` (separate sibling desktop project, outside this Git repository):
- `show-editor`: launcher; uses its local `.venv` if installed, otherwise system Python
- `show_editor/`: Qt editor, preview, exporter and tests
- `show_editor/layout/`: geometry model, SVG previews and SD-layout export tools
- `shows/`: editable JSON source projects; `assets/`: bundled audio/preview
- `output/`: ignored generated exports; `.venv/`: ignored Python environment
- `README.md`: PC setup, commands and test instructions
- The ESP32 keeps only `components/show/` and `main/app_show_player.c`; exported
  packages are copied to SD, never linked into firmware.

Sibling research directory:
- `../localFFT/`: PC FFT/BPM experiments and offline benchmark; not part of
  the ESP-IDF component graph

Legacy entrypoint files still present in `main/` but not in build list:
- `main/reciever.c`
- `main/sender.c`
- `main/old2.c`

Structure audit (after the FFT header move):
- All 37 registered local components have existing declared source/include
  paths. All first-party component headers are now under `include/`.
- `components/audio_capture/` has an empty CMakeLists.txt; `components/led2/`
  has none. Neither implementation is compiled. Do not silently enable them.
- `components/fft/temp.c` is a PC/ALSA/SDL prototype, not firmware.
  `components/audio_player/audio_player.c.bak` and the tracked
  `components/twenty48/__pycache__/gen_sprites.cpython-313.pyc` are cleanup debt.
  Local untracked FFT copies/config edits were left untouched.
- Circular build dependencies remain: `audio_player <-> bt_audio`, plus
  `fft -> audio_player -> bt_audio -> input -> fft`. Fix ownership/callback
  boundaries in a separate behavior-tested refactor, not by deleting REQUIRES.
- Bluetooth contains app UI and still declares a private include path into
  `main/`; that crosses the intended composition/component boundary.
- Large responsibility mixes remain, especially `../ShowEditor/show_editor/main_window.py`
  (~4,200 lines after widget extraction) and `components/led/led.c` (~2,700). Split by responsibility
  when working on those modules; more folders alone will not fix the coupling.


Current status and scoped backlog (2026-10-16)
---------------------------------------------
This replaces the old speculative roadmap. An unchecked idea is not proof of a
bug. Preserve working behavior; require a reproducer, measured bottleneck or
explicit feature request before changing architecture.

Working baseline:
- FFT/BPM/phase: user reports the latest ESP implementation works very well.
  PC/ESP host replays cover hold/loss and phase/clock behavior. Do not restart
  detector research or tune thresholds without a concrete failing recording.
- Shows: user reports playback works very well. Audio ownership, track.wav,
  configurable delay, role coordination and late join already exist.
- Call/message are registered apps, not menu stubs. Their hardware behavior
  still needs explicit testing before claiming every route works.
- Pong multiplayer is implemented and passes simulated loss/jitter tests;
  physical two-suit validation is still pending.
- Package validation, runtime/role tests, editor export tests and PC/C spatial
  parity tests exist. Do not list them as entirely unimplemented.

Next: legs in authored shows
- [ ] Complete the physical leg layout. Show section IDs/groups and editor leg
  geometry exist, but firmware defaults and saved jonas/tom layouts only cover
  chestplate/arms. LED_LAYOUT_MAX_SECTIONS is 24; the full geometry has 33.
- [ ] Confirm leg LED counts, strip order, direction and output connections
  before updating mappings. Never append guessed counts to a working layout.
- [ ] Check the total pixel budget (currently 721), section limits and stack
  allocations together; increasing a struct limit alone is not a safe fix.
- [ ] Test all leg targets and spatial coordinates against the editor, then
  validate on one suit before rolling out to all three.
- [ ] Correct the show specification's stale "reserved future" leg wording once
  the full physical mapping is supported and verified.

Focused verification, not rewrites:
- [ ] Two-suit Pong: pairing order, disconnect/rejoin, collision correction and
  opposite-side controls on hardware.
- [ ] Three-suit shows: occupied-role swap, joining mid-song, full-song sync.
  Existing code/tests are not a substitute for this physical rehearsal.
- [ ] Preserve real failing logs/recordings when issues recur. Profile the
  failing path before changing task priorities, Bluetooth or LED scheduling.
- [ ] Verify power sensing against actual wiring/calibration before trusting
  battery telemetry. Current power code uses a placeholder divider ratio and
  repurposes a Hall ADC input; do not enable it blindly.

Security and reproducibility (separate, deliberate work):
- [ ] Remove tracked credentials through a provisioning/local-config workflow.
  Credential rotation requires the owner's coordination; do not silently change
  network access or rewrite Git history.
- [ ] Record the exact working ESP-IDF revision and local SDK patches so another
  build reproduces the tested firmware. Do not upgrade the SDK as cleanup.
- [ ] Decide the trusted-network model. ESP-NOW peers currently use
  encrypt=false; authentication is a real design question, not an unrequested
  protocol rewrite.
- [ ] Audit file-transfer limits/peer validation with targeted tests. Filename
  sanitization already exists in audio_rx; the old blanket "add validation"
  task was too vague to justify replacing a working transfer path.
- [ ] Back up/version the sibling ShowEditor project before committing the
  firmware-side editor deletions. Those removals remain deliberately unstaged.

Deferred ideas, not defects or prerequisites:
- Audio arbiter/leases, global service-lifecycle standardization, replacing all
  volatile flags, SD refcounting, OLED ownership rewrites, packet-codec
  consolidation and splitting large modules: only when a specific problem
  requires it, with regression tests for the affected boundary.
- OTA repartitioning: only if OTA becomes a requested feature.
- FFT corpus expansion, alternate algorithms and phase research: optional
  experiments, not required work on the now-working detector.
- Editor refactoring/presets, show seek UI, extra games/icons and gesture
  controls: feature requests, not reliability fixes.
- Previously asserted IMU stale-handle and startup-order bugs need a fresh
  code trace/reproducer before being treated as confirmed current failures.

Legacy notes from original README
---------------------------------
This section intentionally preserves useful project context from the original notes.
Treat this as a design/backlog appendix and validate against code before relying on details.

Legacy hardware context:
- Target board: ESP32-WROVER-E (16 MB), flashed over USB-UART at 115200.
- Known historical MACs: `24:d7:eb:6b:26:40` and `24:d7:eb:6b:26:c8`.
- SD conventions: mount at `/sdcard`, hidden boot counter file `.rec_counter`.
- Original parts checklist (historical): ESP32 module, IMUs, microSD, SSD1309 OLED, INMP441, MAX98357A, WS2815 + 74HCT level shifter, ACS712 + divider, 3S pack + BMS, regulators, hall sensor, ladder buttons, speaker, harness/connectors.

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

Open TODO / wishlist
---------------------

- Menu/UI: replace placeholder icons with ascii-art
- games sfx, conflict with audio player playing sfx ?

- accelerometer synth (theremin like? idk) !
- gesture menu selection ? (trigger things with limb position/movement)

- do spatial aware led animations based on limb orientation
- tron
- pacman
- asteroids game !
