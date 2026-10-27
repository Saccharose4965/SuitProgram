# Progress

## Remote suit control

- Added a dedicated Remote Control app and service. The controller selects a
  recently heard suit, both navigation contexts begin at the main menu, and
  A/B/C/D presses execute only in the target shell.
- Standard menus and grids publish bounded semantic snapshots which the
  controller redraws with shared widgets. Custom applications continue running
  only on the target and appear as a controller-side placeholder.
- Target physical input is leased for the session. Either wearer ends the lease
  only by holding A+B for three seconds; peer timeout also returns both local
  states safely to the main menu. Corner marks identify both roles.
- Start, input and end commands use acknowledged ESP-NOW delivery. Views and
  heartbeats stay best-effort. The full firmware build passes with 12% of the
  application partition free; two-suit hardware acceptance remains required.

## Shared choice navigation and continuous Pong controls

- Choice screens now own their scrolling state and reuse the main-menu sprite
  plus framed-name block. Lists show three vertically scrolling choices.
- The LED animation catalogue remains two-dimensional: three entries per
  column, A/C across columns, B down within a column and D to select. Column
  changes animate horizontally.
- Pong reads the cached debounced held button and moves its paddle continuously
  with acceleration and deceleration. A full traversal takes approximately one
  second.
- Paint and Pong pickers use the shared list renderer. LED source, FFT control,
  Preferences and Service Management now reserve D for selection/action.

## Pong controls, AI and collision authority

- The mode menu and paddle now use A/B for up/down. D confirms a mode and only
  pauses AI matches; C and the app-specific back action are unused. The legend
  changes with the active screen.
- The AI picks a stable error for each incoming ball, eases towards it, reacts
  after 180 ms and moves at a capped speed. It remains smooth while allowing
  misses.
- The real Pong service regression confirms that only the suit owning the
  approached paddle resolves its collision. The peer accepts that trajectory
  and visually interpolates onto it by midfield. Packet loss, scoring,
  malformed traffic and disconnect recovery continue to pass.

## Snake playfield, turns and lifecycle

- Added ready, playing and results states with a consumed start press and a
  two-second delay before the D/retry icon appears.
- Added a two-turn service queue validated against the last queued direction.
  Each movement step consumes one turn, preserving rapid legal corner input.
- Resized the grid to 29 by 11 four-pixel cells and framed it with a two-pixel
  border between the score row and button legend. Removed the shell HUD from
  this game so its own score row remains visible.
- Scores display apples eaten, derived from the stored length record. Food now
  uses a compact apple silhouette while the snake retains solid cells.

## Tetris lifecycle and game over

- Replaced direct input sampling and the private worker with shell callbacks.
  Any game button dismisses the ready overlay and is consumed.
- Changed the rules service to preserve the board and score when a new piece is
  blocked above the playfield. Finished runs ignore further rule commands.
- Added an immediate result overlay and a two-second lockout before the D/retry
  icon appears. D initializes the next run directly.
- Preserved the 450 ms drop cadence, 140 ms action cooldown and controls. The
  next-piece preview now sits below the score, and drawing targets the shell
  framebuffer directly.

## 2048 lifecycle and game over

- Replaced direct button polling and the private worker with shell input, tick
  and draw callbacks. Rendering now targets the shell framebuffer directly.
- Added ready, playing and results states. Any direction dismisses the ready
  overlay and is consumed; the following press performs the first move.
- Added committed-board game-over detection. The result overlay appears at once,
  then the D/retry legend icon appears after a two-second input lockout.
- Preserved direction mapping, cosine movement and score persistence while
  expressing animation speed in elapsed time.

## Flappy lifecycle, overlays and scoring

- Replaced the private Flappy worker and polling splash with shell-owned ready,
  playing and results states. D starts/retries and every button flaps in play.
- Kept 30 Hz game physics through a bounded fixed-step accumulator while drawing
  at the shell rate. App exit is now immediate and cannot leave an overlapping
  worker behind.
- Added a shared modal card renderer and used it for start/result information;
  Flappy no longer reserves or displays a button legend.
- Made scoring one-shot per physical pipe and reset that state on recycling.
  Successful record saves now clear their dirty flag.

## Master control and shell ownership

- Split master-control synchronization into `services/master_control` and its
  four-row OLED screen into `apps/master_control_app`. Protocol, peer clocks,
  role transitions and LED enforcement remain in the service.
- Moved foreground registry, menus, status composition, lifecycle and frame loop
  into `apps/shell`.
- Extracted runtime/task load logging into `services/performance_monitor` and
  renamed its API around monitoring rather than the old source filename.
- `main` now contains only `app_main.c`, ESP-IDF component configuration and
  project Kconfig files.
- Firmware build passes after both splits. Multi-suit master-control hardware
  acceptance remains required.

## LED controls and timing source

- Split the live LED configuration screens into `apps/led_control` and the
  FFT/manual clock generator into `services/led_source`.
- Source timing state and APIs no longer live in the shell's app registry
  header. Master control reads the source service directly; the source service
  reads the shared system snapshot instead of depending on a UI module.
- The animation navigation regression moved beside its owning app and passes.
- Hardware acceptance remains: animation/colour edits, manual BPM, FFT reset and
  force controls, isolated offset presses and the double-press 2x shortcut.

## Documentation cleanup, Messages and Arm Synth

- Replaced the old documentation archive, migration proposal and implementation
  diaries with a current architecture overview, show-system map and complete
  module index. Removed duplicated technical reviews after preserving the useful
  design decisions in each module's README.
- Moved the Message UI into `apps/message_app` and the direct transfer protocol
  plus arrival tracking into `services/messages`. Moved their focused host tests
  beside the new owners.
- Split Arm Synth into a thin `apps/synth` screen and `services/synth_service`,
  which now owns calibration, gesture mapping, oscillator, route fallback,
  worker lifecycle and FFT/audio ownership.
- Message and Synth host regressions pass. A complete ESP-IDF reconfigure/build
  passes. Hardware behavior is intentionally unchanged and remains to be checked
  when the next firmware is flashed.

## Settings, Music and playback diagnostics

- Settings persistence now lives in `services/settings`; its Settings-facing
  screens are grouped in `apps/settings_app`.
- Music browsing now lives in `apps/music`. Its SD catalogue is loaded on app
  entry, keeping filesystem work out of draw and scroll callbacks.
- End-of-playback heap tracing used during the Bluetooth investigation was
  removed. Actual stream-stall health logging and lightweight heap totals remain.
- Orientation, suit-geometry and input/audio-policy regressions now live beside
  their owning services. The remaining animation-control regression stays with
  its still-unmigrated LED app.
- Firmware reconfigure/build and the focused host regressions pass. Hardware
  checks remain: Settings persistence/recovery, Music entry/scroll/playback and
  a complete Show ending without the former allocation-backtrace dump.

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

## Bad Apple shared-clock synchronization

- Replaced the fixed one-second video delay with Audio's submitted-frame
  audible timeline, shared with Show Runtime. Local DMA depth and the adjustable
  Bluetooth delay are now applied by one clock implementation.
- Bad Apple starts video on the first audio progress sample, follows absolute
  playhead time, freezes on pause/stall and restarts audio when changing video
  encoding. Sequential decoding catches up in bounded four-frame batches.
- Missing WAV and video files are reported separately. The firmware build and
  Show clock regressions pass; local and Bluetooth timing still need hardware
  validation.

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
- Superseded technical reviews, migration diaries and the old review workflow
  were removed after current module READMEs became the canonical explanations.
  Cross-system design now lives directly under `docs/`; module explanations
  live beside their app, service or driver.
- Stale source-inspection receipts were cleared during that cleanup. Documentation
  coverage remains visible in the regenerated checklist.

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

## Animation catalogue boundary

- Split the former mixed animation grid into Beat and Ambient apps. Beat lists
  the trigger-driven renderer; Ambient lists the continuously rendered clock
  renderer and its speed control.
- Opening either catalogue no longer changes the active animation. A mode only
  changes when D selects it; selection still disables the other renderer.
- Preserved all effect implementations and numeric mode indices. Similar-looking
  modes remain pending hardware/user review rather than being guessed duplicate.
- Classified Audio Energy as Ambient because its render path consumes live band
  levels and its beat-trigger branch intentionally performs no state change.

## Recoverable game scores

- Score files now require one valid unsigned 32-bit decimal value; malformed,
  signed, overflowing and trailing data is reported instead of becoming zero.
- Updates sync a temporary copy, move a valid primary to backup and then install
  the new primary without relying on FAT replacement rename behavior.
- Loading considers primary, backup and interrupted temporary copies, keeps the
  highest valid value and repairs the primary. Host recovery tests and the full
  firmware build pass; physical interrupted-write testing remains documented.

## Recording persistence and startup cleanup

- Automatic recording numbers now use recoverable primary, backup and temporary
  counter files. Existing WAV and partial names are skipped with exclusive
  creation instead of being overwritten or replaced by a tick-count fallback.
- PCM is written to `.part`; the checked and synced final WAV header is installed
  under the requested name only after the recording closes successfully.
- Startup failures remove their empty partial file and release I2S RX, including
  allocation, handle and task-creation failures.

## Coherent LED renderer settings

- Beat and standalone renderers now keep related colour, style, background,
  brightness and mode-specific settings in protected configurations.
- Each worker takes one immutable settings copy before constructing a frame.
  Colour-app presets and synchronized master-control updates publish complete
  configurations, preventing a frame from mixing old and new values.
- Existing narrow setters remain for isolated changes. Effect equations,
  catalogue indices and defaults are unchanged.
- The full firmware build passes with 13% of the application partition free.
  Hardware review of every beat/standalone effect and synchronized shuffle is
  still required before changing effect visuals.

## Bluetooth event boundary

- Removed the Music-specific disconnect callback and the Bluetooth screen's
  draw-time connection transition handling.
- Bluedroid callbacks now enqueue disconnect and AVRCP play/pause events in a
  fixed static queue; the shell applies them in normal task context.
- Removed Bluetooth's build dependency on Audio Player and Music's dependency
  on Bluetooth. Audio Player still independently polls streaming status, so a
  delayed event cannot hide a disconnect.
- PCM buffering, feeder cadence, A2DP command sequencing, discovery/reconnect,
  task placement and the no-sniff policy are unchanged. Host playback tests and
  the full firmware build pass; extended hardware playback remains pending.

## Grid carousel cue

- The shared three-row grid now centres a narrower active column and leaves
  cropped portions of the available left and right columns visible.
- Navigation, selection order and animation timing are unchanged; only the
  horizontal card geometry changed.

## Board-support ownership cleanup

- Removed unused fixed-rate I2S constructors and their private channel handles
  from `drivers/hw`. The shared audio I2S driver is now the only channel owner.
- Removed the unused voltage-to-button decoder from board support. The input
  driver remains the only decoder/debounce owner.
- Physical I2S pins, ADC channels and shared calibrated ADC reads remain in
  board support.

## Serialized SD lifecycle

- Replaced the volatile mount-in-progress flag and one-second competing retry
  path with one statically allocated lifecycle mutex.
- Mount retries and unmount now execute exclusively. Concurrent mount callers
  wait, then reuse the mounted card produced by the first operation.
- Frequency fallback, SPI wiring setup, FAT options and caller-owned file
  lifetime rules are unchanged.

## Explicit MPU sampler stack size

- Renamed the MPU sampler stack argument to state that ESP-IDF measures it in
  bytes.
- Corrected the zero-value default from an accidental 1024 bytes to the intended
  4096 bytes. Both current callers select this default.
- Sampling rate, priority, core placement and sensor configuration are unchanged.

## Serialized Orientation startup

- Added one static lifecycle mutex around MPU device, filter-task and sampler
  creation. Concurrent callers now wait instead of creating duplicate state.
- Readiness becomes visible only after both tasks exist. Failed attempts remove
  their filter task and SPI device before a later retry.
- Snapshot readiness and raw-sample availability now read the synchronized
  published startup state.

## Single-owner Fluid orientation

- Removed Fluid's private MPU device lookup, initialization and sampler fallback.
- Fluid now obtains acceleration only through the shared Orientation service.
  Missing or stale sensor data produces neutral gravity while simulation and
  rendering continue.
- Removed the resulting board-support and MPU-driver component dependencies.
  Fluid worker scheduling and stop behavior are unchanged.

## Cooperative Fluid shutdown

- Replaced Fluid's timed polling and forced task deletion with a static worker
  completion signal.
- Shutdown now lets the worker finish its current bounded step and delete itself
  before particle and mask buffers are reset.
- Simulation timing, adaptive substeps and task placement are unchanged.

## Recoverable LED layout saves

- Version-3 layout files carry an end marker so truncated writes are rejected.
- Saves sync and parse the temporary candidate before retaining the prior valid
  primary as backup and promoting the new file.
- Startup recovers in newest-safe order from a complete temporary file, primary
  or backup. Legacy layouts remain readable and upgrade on their next save.

## Reboot-resistant LED choices

- Added the selected Beat/Ambient renderer, animation, ambient speed,
  backgrounds and complete colour profile to the SD-backed Settings model.
- Boot applies the colour and animation snapshots after suit geometry is ready,
  then seeds the HUD from the renderer that was restored.
- Mode, background and colour edits save immediately. Ambient speed previews
  without SD traffic on every step and saves when editing ends or the app exits.

## FFT and Bluetooth screen boundaries

- The FFT viewer now requests analysis through Audio Policy and releases only
  its own request on exit. It no longer initializes the audio bus or starts the
  detector directly.
- Pending viewer demand survives Bluetooth or other temporary audio ownership;
  the screen reports the blocking state and resumes through normal policy ticks.
- The Bluetooth audit confirmed that its app contains selection and drawing,
  while discovery, connection, deferred reconnect and streaming remain owned by
  the Bluetooth service.

## Live Sync LED source

- Replaced the standalone Master Control screen and its role/ratio/colour
  policies with FFT, Manual and Follow A/B/C choices in LED Source.
- Reused Show's convergent A/B/C identity coordinator while keeping its playback
  leadership, transport and LED ownership inactive outside Show.
- Live Sync copies the complete active renderer configuration and distributes
  future beat anchors using measured peer clock offsets. Followers continue
  locally through packet loss and expose a lost state after 2.5 seconds.
- A follower's previous local renderer state is restored when it leaves Follow.
  Full firmware build passes with 13% of the app partition free; multi-suit
  hardware acceptance remains.

## Orientation calibration ownership

- Removed the unused global recenter state from the orientation filter. The
  service now publishes calibrated absolute sensor pose.
- Added a shared heading-reference object that captures horizontal heading,
  preserves gravity-derived tilt and computes relative pose. Arm Pose, Paint,
  Synth and 3D each own one instance, so feature calibration is isolated.
- Centralized the 250 ms freshness check in Orientation and migrated every
  quaternion consumer. Gyro bias remains the persistent per-sensor correction;
  Fluid continues to consume raw acceleration deliberately.
- Orientation, gyro calibration, Paint and Synth host tests pass. The complete
  firmware builds with 13% of the app partition free.

## Shared information-screen layout

- Added a width- and height-clipped title/status-page primitive alongside the
  existing shared list, grid, scroll, overlay, HUD and legend renderers.
- Migrated File RX, direct Call, shell Status and Gyro Calibration without
  changing their controls or service work. Removed their direct OLED component
  dependencies where the shared UI is now the presentation boundary.
- Kept live transport lists, graphs, games, media frames and geometry custom.
  Text-page bounds, shared drawing, Paint UI and Show UI tests pass; the full
  firmware builds with 13% of the app partition free.
