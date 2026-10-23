# Progress

## Call app and voice service

- Extracted the half-duplex ESP-NOW voice protocol, queue, I2S worker and audio
  resource transition into services/call. apps/call_app now contains only lifecycle,
  button handling and status drawing.
- Kept integrated-speaker playback, direction flushing and worker-owned FFT
  exclusion. Hardened the receive boundary against a null payload and retained
  its focused production-code packet test.
- Added local Pending review explanations. Firmware and hardware verification
  follow the checks listed in the service document.

## Show package, renderer, runtime and player

- Split the former mixed Show component into services/show_package,
  services/show_renderer, services/show_runtime and apps/show_player. Removed
  the old component directory and moved each focused regression beside its
  owner.
- Package code now forms the file-validation/loading boundary; rendering owns
  geometry/effects only; runtime owns roles, leadership, audio clock, sync and
  LED arbitration; the app owns SD discovery, controls and OLED presentation.
- Added four local Pending review explanations and retired the dated monolithic
  implementation diary. Renderer, runtime and three-suit role tests pass; the
  complete ESP-IDF reconfigure/build passes.

## FFT service and viewer

- Moved microphone FFT, BPM hold, phase tracking, beat clock, audio levels and
  audio-clock diagnostic history into services/fft. Removed the uncompiled
  SDL/ALSA prototype; the maintained desktop workbench remains in localFFT.
- Moved monochrome graph rendering and the FFT screen into apps/fft_view. The
  app pulls immutable history snapshots, so opening/closing it no longer writes
  display state into the detector service or affects background analysis.
- Replaced the dated global FFT essay with focused Pending review explanations
  beside the service and app. Desktop parity, detector, phase and display host
  regressions pass; the complete ESP-IDF build passes.

## LED output service and strip driver

- Moved logical frame arbitration, layout mapping, cadence/coalescing, test
  patterns and beat effects into services/led. Extracted WS2815 encoding, two
  RMT channels, completion waits and bounded timeout recovery into
  drivers/led_strip.
- Preserved the separate pending, TX-shadow and per-strip physical buffers so
  producers can replace a pending frame without touching in-flight memory.
  Updated the sanitizer regression to exercise that boundary.
- Added Pending review explanations of the physical protocol, mapping,
  ownership, scheduling, audio brightness and beat renderer. Firmware build and
  the focused host test pass; hardware visual acceptance remains pending.

## Audio service and I2S driver

- Split the shared I2S peripheral, clock selection, DMA channels, amplifier pin
  and overflow/revision diagnostics into drivers/audio_i2s. FFT, microphone and
  Call now state their raw-hardware dependency directly.
- Moved local WAV/tone/generated playback, volume, output ownership and Show
  progress into services/audio. File playback now uses one mono/stereo streaming
  path instead of retaining a second mono-only implementation.
- Fixed clock failures leaving the reported sample rate permanently at zero,
  and made failed TX enable/prime attempts return the speaker to a muted stopped
  state. Removed three unbuilt legacy audio entrypoints from main.
- Added local Pending review design explanations. Firmware reconfigure/build
  passes; local speaker/microphone/Bluetooth transition testing remains on
  hardware.

## Retired standalone Call and Comms prototypes

- Removed the unreferenced blocking UDP Call and Comms components. They created
  their own Wi-Fi/audio/OLED/button control loops and duplicated the maintained
  shell Call, Messages, Telemetry and microphone paths.
- The live Call remains the ESP-NOW half-duplex implementation currently under
  the shell. Removing the prototypes eliminates competing I2S ownership and a
  misleading second call protocol before that live app is split.

## Pong app and multiplayer service

- Split matchmaking, packet validation, distributed collision authority,
  simulation and visual correction into services/pong. apps/pong_app owns mode
  selection, input, shell navigation and OLED drawing from a service snapshot.
- Preserved the first-waiter side assignment and independent-simulation model.
  The sanitizer test still covers two clocks, loss, scoring, malformed traffic,
  smoothing, disconnect recovery and AI pause.

## Suit link service

- Moved ESP-NOW framing, optional Wi-Fi/UDP fallback, ACK handling, receive
  tasks and status broadcast into services/link.
- Fixed false Wi-Fi success when ESP-NOW failed without a configured fallback,
  checked synchronization/task allocations, made sequence assignment atomic
  and returned UDP receive-task creation failures.
- Added a Pending review explanation of path selection, framing, ACK semantics,
  callback context, information broadcasts and current protocol limitations.

## Standalone LED animation service

- Moved the standalone catalogue, clocks, spatial caches, effect renderers and
  persistent render task into services/led_modes. It contains shared animation
  logic and submits completed frames through the LED output boundary.
- Documented free/synchronized timing, deterministic shuffle, geometry caching,
  palette/background/highlight behaviour and render ownership. Visual logic is
  unchanged pending hardware comparison.

## LED layout service and editor

- Moved persisted wiring, canonical section ordering, raw measurements and
  logical/physical mapping into services/led_layout. The matching physical
  preview and edit screen now lives in apps/led_layout_app.
- Consolidated the former leg-layout note into the service's Pending review
  explanation. Zero-length leg insertion continues to preserve all existing
  chestplate addresses and `suit_geometry` remains the posed-coordinate owner.
- Hardened layout parsing against integer wrap, unsupported strip/point counts,
  non-finite floats and non-finite live ring angles. Extended the sanitizer
  regression to cover malformed persisted values.

## Bluetooth screen and A2DP service

- apps/bluetooth now owns list state, controls, rendering and FFT screen policy.
  services/bt_audio owns Classic BT, discovery, connections, media commands,
  resampling and PCM delivery.
- Removed Bluetooth's private include dependency on main and the main forwarding
  adapter. Stream-control logic and its Audio Player relationship are unchanged.
- Added local Pending review explanations, including the remaining disconnect
  callback inversion and private Bluedroid dependency.

## Audio player service

- Moved asynchronous WAV/output orchestration, playback diagnostics and their
  focused host tests into services/audio_player.
- Preserved the stabilized single-owner A2DP monitor: it observes stalls, stops
  once after drain and avoids automatic suspend/start recovery races.
- Documented Bluetooth/local fallback, show delay, pause/abort hooks, task-stack
  allocation and current cyclic dependencies. No stream-control logic changed.

## File receiver layers

- services/audio_rx now owns the TCP server, filename policy, buffering, SD
  writer and lifecycle. apps/file_rx owns the toggle, resource preparation and
  screen. The receiver remains active after leaving its screen.
- Fixed an immediate start/stop race by publishing the stop state before task
  creation and keeping a stop request visible at task entry.
- Removed the stale tracked audio_player.c.bak snapshot; current history remains
  in Git. Added detailed Pending review explanations for both live layers.

## Retired audio-capture prototype

- Removed the unbuilt and unreferenced audio_capture ring-buffer prototype.
  Shared audio acquisition and recording already live in audio and the
  microphone service.
- The prototype also had unsafe ownership: early task exits left a non-null
  handle, stop could remain blocked in an infinite I2S read, and the handle was
  cleared without confirming termination while readers copied an unlocked ring.
  None of that dead path has been merged into the maintained pipeline.

## Bad Apple playback split

- apps/bad_apple now owns SD/media discovery, controls, audio offset, error UI
  and direct full-screen presentation.
- services/bad_apple_decoder owns file state, raw/RLE/heatshrink decoding,
  looping, frame cadence and status. The vendored decoder and license stay with
  that implementation.
- Removed the main-layer app file and old mixed component folder. Detailed
  Pending review explanations record format, catch-up and synchronization
  choices without changing the media protocol or one-second start offset.

## Removed hardware-demo remnants

- Removed the unbuilt led2 experiment. It defined a second `app_main`, fixed
  GPIO32/GPIO33 and two 500-pixel buffers, and independently implemented the
  same RMT strip protocol owned by the live LED driver.
- Removed the already-empty canceled GPS directory. No GPS source or references
  remained to migrate or document.

## Telemetry service ownership

- Moved the shared Wi-Fi/UDP session and file transport into services/telemetry.
  Its framing stays unchanged for Link, Messages, Call and Comms compatibility.
- Removed unused hard-coded suit MAC constants. Runtime sender identity remains
  the station MAC.
- Event-handler registration failure now frees the newly allocated event group
  and reaches callers instead of leaving a half-initialized service.
- Added a Pending review explanation of network startup, transfer framing,
  receiver validation, delivery semantics and current concurrency limits.

## Retired Stick prototype

- Removed the unreferenced blocking three-IMU Stick prototype. It had no caller
  and duplicated MPU register access, complementary orientation fusion, direct
  button polling and direct OLED ownership inside one file.
- The maintained Arm Pose app, shared orientation service, MPU driver and shell
  already own those responsibilities. Keeping the prototype would create a
  second uncalibrated sensor pipeline and could register the same SPI devices
  again if accidentally called.

## Fluid simulation and screen

- Moved the particle model, gravity sampling, adaptive worker and synchronized
  mask publication into services/fluid_sim. apps/fluid now contains only shell
  lifecycle and integer-scaled OLED rendering.
- Removed the unreferenced standalone runner and its duplicate direct button/
  OLED loop. The shell remains the single presentation path.
- Preserved particle constants, axis mapping, adaptive one/two-substep policy,
  worker placement and orientation fallback. Added local Pending review design
  explanations for both layers.

## Flappy rules and presentation

- services/flappy_game owns physics, pipes, pixel collision, score and death
  progression. apps/flappy groups splash, input handoff, rendering and worker.
- Shared bird mask belongs with the collision shape and is reused for drawing.
  Removed unused overwritten placeholder rendering and main forwarding adapter.
- Existing timing and gameplay are retained. Local explanations record splash
  cancellation, worker ownership and repeated-score limitations for follow-up.
- Temporary host checks passed for flap/gravity, death phases, settling and
  scoring. Build caught an extraction typo in rendering state access, corrected
  before final verification.

## Tetris rules and presentation

- services/tetris_game owns shapes, collision, rotation kicks, line clearing,
  spawning and scoring. apps/tetris owns input timing, worker and rendering.
- Consolidated duplicated piece-spawn transitions. Removed the old component
  and main adapter; retained the existing controls, cadence and gameplay.
- Added local Pending review explanations and recorded lifecycle limitations.
- Temporary host check passed for movement, rotation, row clearing, score
  updates and game-over reset.
- Final firmware build, stale-reference search, checklist and diff checks pass.
  Hardware gameplay checks remain deferred; nothing flashed.

## 2048 rules and presentation

- Extracted board movement, merges, random tile insertion and score persistence
  into services/twenty48_game. App owns polling, animation and rendering.
- Consolidated worker lifecycle with the screen under apps/twenty48; sprites
  and their generator live there too. Removed obsolete renderer declarations,
  the main adapter and old component folder.
- Preserved polling cadence, animation increment, merge rules and save timing.
  Both local explanations identify the remaining input/concurrency limitations.
- Temporary host checks cover horizontal/vertical merges, scores, no-op moves
  and board commit. Firmware reconfigure/build passed. No flash.

## Snake responsibility split

- Game state, stepping, collision, food and score persistence now live in
  services/snake_game. App callbacks, button mapping and rendering live together
  in apps/snake. Removed the old component and main forwarding adapter.
- Removed unused navigation callback state. No new task or gameplay changes.
- Added both local design explanations with Pending review, including existing
  tail-collision, multiple-turn, food-fallback and synchronous-save behaviours.
- Focused temporary host check covers reset, reverse guard, movement, food,
  scoring, collision and restart; passed without keeping extra test scaffolding.
- Final firmware build, stale-reference and diff checks pass. Physical gameplay
  remains deferred with the other hardware checks; nothing flashed.

## Keyboard app ownership

- Reviewed keyboard state, editing, pointer gestures, rendering and shell input.
  Consolidated the screen adapter into apps/keyboard/keyboard.c; removed its
  old component folder and main adapter/header.
- Public widget and shell declarations share the module header. No extra
  processing service is introduced for presentation and editing state.
- Documented exact existing controls and unfinished text-entry behaviour,
  including Enter, UTF-8 and pointer/button differences. No UI redesign.
- Widget and adapter source comparison confirms unchanged logic. Firmware
  reconfigure/build, stale-reference search, checklist and diff checks pass.
  Physical keyboard navigation/rendering remains in the deferred hardware pass.

## Startup logo consolidation

- Startup logo belongs to apps/ui_shell alongside shared drawing and scroll
  helpers. Removed the standalone logo component and its build dependency.
- Kept frame generation, delays, skip callback and shell task ownership intact.
- Added its design explanation to the owning module's Pending review README.
- Combined scores/logo firmware reconfigure/build passes. Source comparison
  confirms preserved implementation logic; checklist and diff checks pass.
  Hardware checks remain deferred; nothing flashed.

## High-score service

- Reviewed and moved scores into services/scores; removed the empty old folder.
  Existing game API, comparison policy and file format are unchanged.
- Added local Pending review explanation covering loading, writes, ownership
  and current limits. FAT replacement and malformed-file handling need follow-up.

## Recording ownership

- Moved microphone recording into services/microphone; the old component folder
  is removed. Existing component name, API and task scheduling are retained.
- Extracted recording-number persistence from the SD driver into a private
  recording-service source. Counter path ownership also leaves the board header.
- The caller now uses recording_counter_next; the four-byte file format and
  fallback naming behaviour are preserved.
- Added a local design explanation, including existing persistence and cleanup
  limits. Updated driver documentation and pending work.
- Envelope host check, firmware reconfigure/build and checklist pass. Hardware
  check deferred: record, stop and replay, including an automatic filename.

## Audit of migrated folders

- Added missing local explanations for Arm Pose, 3D, shared status, board
  support, MPU acquisition and SD access. All start with Pending review.
- Consolidated 3D lifecycle/input/rendering into one implementation. Removed
  forwarding source/header and unused setters; updated shell callback names.
- Corrected the inaccurate atomic-counter comment in SD code and header.
- Identified remaining legacy responsibilities and initialization limitations
  in moved drivers. These are tracked in TODO; migration is not completion.
- Every migrated module folder now has a local explanation; links and checklist
  pass. Paint regression and firmware build pass. No hardware test or flash.

## Corrected processing boundaries

- Calculator evaluation moved to services/calculator_math. Its screen now owns
  shell callbacks directly; removed the forwarding file and redundant header.
- Reviewed migrated Paint, gyro calibration, Arm Pose, 3D and UI infrastructure.
  Paint and gyro processing already use services. Arm Pose's upper-arm inference
  moved into orientation; camera projection, layout and rendering stay in apps.
- Updated explanations at each owner; user review remains pending. This
  supersedes the previous decision to keep the evaluator private to the app.
- Calculator, orientation-reference and Paint host checks pass; firmware
  reconfigure/build passes. No flash; hardware checks remain grouped for later.

## Calculator and battery monitor

- Calculator moved to apps/calculator, including its shell adapter and build
  ownership. Its expression evaluator is separated from input/drawing as a
  private app module. Arithmetic and UI behaviour are preserved.
- Parser comparison against the original source matches after the API rename;
  a focused host check covers precedence, functions and invalid expressions.
- Battery monitor moved to services/power with its Power sensing configuration.
  Reviewed acquisition through the board driver, estimation, publication and
  task ownership; removed unused include and redundant core-selection branch.
  Configuration values, cadence, smoothing and state slots are unchanged.
- Both old component folders were removed after migration. Per-module design
  explanations added with Pending review.
- Firmware reconfigure/build passed. Hardware checks are deferred together:
  calculator controls/live results and battery status after startup.

## App build ownership and audio policy

- Relocated Paint, Arm Pose and gyro calibration compile in the apps component;
  the 3D component also compiles its shell adapter. Main no longer compiles
  sources from sibling app directories.
- The shell lifecycle interface belongs to UI infrastructure. FFT arbitration
  and lazy board audio initialization belong to services/audio_policy.
- Renamed the policy API to reflect its scope; updated all callers. Request
  masks, blockers, ownership rules, warning intervals and scheduling are unchanged.
- Audio policy explanation added with Pending review. Shared UI explanation
  extended for its lifecycle interface and remains pending review.
- Two focused regressions moved to tests/ and passed. Firmware reconfigure/build
  passed; no flash. Existing legacy resource boundaries remain to be cleaned up.
- Hardware check: enter/leave Paint and 3D with FFT lighting enabled, verify
  background analysis resumes, then check Bluetooth playback.

## Documentation organization

- Current module explanations live beside their code as `README.md` under
  apps, services and drivers. The documentation index links to them.
- Removing the first `Pending review` line records user approval. Source-file
  inspection receipts remain separate from that approval.
- Earlier technical notes and the old review workflow are archived under
  `docs/reference/`; the superseded `docs/design/` directory is removed.
- Links and inspection receipts now use the new paths. Documentation coverage
  is still incomplete.

## OLED responsibility boundary

- OLED driver retains panel setup, framebuffer packing and SPI transfers.
- App UI infrastructure owns text drawing, fonts and the scroll curve.
- Callers explicitly depend on the drawing interface. Differential host tests
  compare drawing with the checkpoint; panel and UI regressions pass.
- Firmware build passed. Physical screen verification remains pending.

## Input responsibility boundary

- Driver owns ADC decoding, debounce and press/hold events; it reports activity
  with timing through a callback. Its FFT dependency is removed.
- `services/input_audio_policy` owns the existing suppression guard/tail rule.
- Shell wires the callback before polling. Input moved to drivers and its empty
  old directories were removed.
- Production host tests cover event timing and suppression arguments; Pong's
  integration test passes. Design explanations added for user review.
- Remaining cleanup: OLED drawing separation, app build ownership and the broader
  resource policy. Those boundaries are still incomplete.

## Foundation component moves

- Board/bus support: `drivers/hw/`.
- OLED transport and existing low-level drawing helpers: `drivers/oled/`.
- SD access: `drivers/storage_sd/`.
- Shared screen composition: `apps/ui_shell/`.
- Shared system state: `services/state/`.
- Original directories removed after moving all contents. Component names and
  APIs retained; no scheduling or rendering changes in this batch.
- Input remains in components pending extraction of its FFT noise-suppression
  callback. OLED drawing primitives also need a later driver/app boundary review.
- UI redesign remains deferred. These moves do not mark modules as documented
  or reviewed; file review receipts require revalidation at the new paths.

## Gyro calibration and folder cleanup

- Removed empty legacy orientation, MPU and 3D component directories.
- Settings now offers gyro calibration: settle, sample, review, save.
- Bias is saved on SD, loaded at startup and applied to all three gyro axes.
- Previous valid reference retained as backup; failed measurement/save leaves
  the active bias unchanged.
- Host tests cover motion rejection, cancellation, persistence, invalid files,
  backup loading and bias subtraction. Physical sensor/SD validation pending.

- `224f2cd` pushed: message improvements and documentation checkpoint.
- Message UI, notification, direct-delivery, recording-envelope and checklist
  tests passed. No hardware verification implied.
- Existing editor deletions, SDK settings and unrelated scratch files excluded.
- Architecture layers agreed; explanation style accepted.
- README shortened, original preserved; backlog and reviews separated.

## Orientation pilot

- Shared gravity-preserving calibration implemented for Paint, Arm Pose, Synth
  and legacy filter recenter. Reference values stay local to feature sessions.
- 3D view reads coherent snapshots, has private calibration and holds its last
  camera view if sensor data goes stale.
- Heading/tilt/roll, invalid input and reference-preservation regression added.
- Orientation, Paint and Synth host checks pass; firmware build passes.
- No firmware flashed.

## First physical architecture slice

- Orientation moved to `services/orientation/`.
- MPU acquisition moved to `drivers/mpu6500/` (component name preserved).
- Arm Pose and 3D visual code moved to `apps/arm_pose/` and `apps/threedee/`.
- Build discovery explicitly registers migrated components; shell adapters
  temporarily remain compiled by main, without depending back on it.
- Host test source paths updated. No algorithm, rendering or task-policy changes
  in these moves. Earlier calibration changes remain a separate behaviour change.
- Firmware reconfigure/build and orientation, Paint, Synth, BT-synth, FFT-gate
  and checklist host tests pass. ESP-IDF reports a transitional warning because
  main compiles the relocated 3D shell adapter; remove that ownership mismatch
  when extracting the shared shell contract. Existing BT dependency warnings remain.

Next: Paint service extraction and shared geometry. UI redesign/helpers wait
until visual code has moved into apps. Hardware calibration verification pending.

## Paint extraction

- `apps/paint_app`: menu selection, buttons, screen, error wording and shell lifecycle.
- `services/paint`: session, calibration requests, protocol, queue, brush/fade,
  geometry cache and LED ownership; explicit commands/status API.
- Shell routes paint frames to the service rather than through an app receiver.
- Existing regression now exercises both files; boundary and status checks added.
- Firmware reconfigure/build and Paint, orientation and FFT-gate checks pass.
- No changes to wire format, cadence, fading, coordinates or tasks intended.
- FFT/resource policy remains in the existing app support path. UI redesign
  stays deferred. Geometry extraction is completed in the following batch.

## Canonical suit geometry

- Geometry/section identities extracted into `services/suit_geometry`.
- Paint now depends on geometry directly, not Shows.
- Normal spatial animations use posed coordinates and posed ring measurements.
- Shows no longer select flat coordinates for pre-v1.2 packages; this and the
  normal-animation correction intentionally change some visuals.
- Current show pose, Paint mapping, wiring and LED counts are unchanged.
- Geometry lookup, Paint, show runtime/roles and uniform render host tests pass.
- Firmware reconfigure/build passes. No firmware flashed or new commits pushed.
- Physical effect comparison and old-show acceptance remain pending.
