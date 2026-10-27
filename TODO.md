# Work queue

## Agreed rework

- [ ] Hardware-check Remote Control with two suits: peer discovery, initial
  main-menu state, list/grid/settings navigation, target-only app startup,
  custom-view placeholder, ignored target buttons, three-second A+B exit from
  either suit and three-second peer-loss recovery.

- [ ] Hardware-check the split audio pipeline: FFT acquisition, recording,
  half-duplex Call, startup tone, local mono/stereo WAV, generated synth and
  repeated local/Bluetooth transitions. Confirm 44.1/48 kHz clock changes,
  amplifier pop suppression and recovery after an injected playback failure.

- [ ] Study suit-side show authoring after the Show runtime architecture is
  documented and cleaned. Compare two scopes:
  - a focused live workshop that selects effects/sections, edits a small set of
    parameters, previews immediately, records button-tapped clip timing and
    saves a valid show package;
  - a complete timeline editor equivalent to ShowEditor.
  Evaluate OLED navigation, four-button input, RAM, SD write safety, package
  validation, undo/recovery and whether synchronized suits can participate in
  preview. Prefer sharing the package model and validation code with the PC
  editor. Keep dense timeline arrangement, waveform work and bulk parameter
  automation on PC unless a concrete suit workflow benefits from them.

- [x] Split the live ESP-NOW Call implementation into services/call and
  apps/call_app. Packet validation, worker-owned FFT exclusion, integrated-speaker
  choice and half-duplex I2S direction switching are preserved. Hardware
  acceptance remains listed in services/call/README.md.

- [x] Split Pong into a network/simulation service and a presentation app.
  The service should expose explicit start-AI, start-human, paddle, pause,
  reset and immutable view-snapshot operations; the app should own AI/Human
  selection, shell navigation and OLED drawing. Preserve the existing
  first-waiter roles, collision authority, packet-loss convergence and
  midfield correction test while removing input/OLED dependencies from the
  service.

- [ ] Version and authenticate Link frames, define byte order and sender
  identity across ESP-NOW/UDP, and introduce a proper subscriber dispatcher.
  Review every Show, Live Sync, Paint, Pong, Message and Call packet
  together; add runtime shutdown/reinitialization only with concurrency tests.

- [x] Publish LED beat and standalone animation settings as coherent snapshots.
  Complete presets now become visible atomically and each worker uses one
  immutable settings copy per frame. Effect maths and defaults are unchanged;
  hardware visual review remains before any effect tuning.

- [ ] Hardware-review the LED layout editor across both strips: walk every
  section and ring start, change/revert a length and direction, save, reboot and
  confirm the physical mapping. Primary, synced temporary and backup files now
  provide tested interrupted-save recovery; physical SD fault injection remains.

- [x] Replace Bluetooth's Music-specific disconnect callback with a static
  service event queue consumed by the shell. Audio Player/Bluetooth no longer
  form a build cycle; PCM delivery and the no-sniff policy are unchanged.
  Long-duration play, pause, drain, disconnect and rapid-track hardware tests
  remain required.

- [ ] Discuss fallback file transfer through a server before changing File RX.
  Define what must be transferable, who initiates it, whether the PC relays
  between suits, how recipients are selected and how it coexists with direct
  ESP communication. If retained, redesign it around shared resource policy,
  authenticated/versioned framing, declared size, checksum, progress and safe
  partial-file recovery. Preserve the current receiver only as a prototype
  until that workflow is agreed.

- [ ] Validate Bad Apple audio/video timing on local speaker and Bluetooth.
  The implementation now uses Audio's shared audible clock, and source cycling
  restarts both streams; confirm the 100 ms Bluetooth calibration on hardware.

- [ ] Redesign Telemetry session/transfer concurrency with Link, Messages, Call
  and Comms together: serialize socket ownership and ACK routing, version the
  native-endian protocol, define buffer delivery semantics and add receiver
  shutdown/multiple-sender handling.

- [x] Give Fluid explicit sensor and worker ownership. It uses only the shared
  calibrated Orientation stream, falls back to neutral gravity, and waits for
  its worker's completion signal before resetting simulation buffers.

- [ ] Before reusing keyboard as an overlay: define accept/cancel without losing
  text, handle UTF-8 editing, connect held input, align popup hit-testing and
  respect the content rectangle. Existing standalone behaviour is preserved.

- [x] Validate high-score text strictly and preserve valid primary, backup or
  temporary copies through FAT-safe replacement. Host recovery tests pass;
  interrupted-write testing on the physical SD card remains in the Scores
  service hardware checklist.

- [x] Finish cleanup within migrated drivers: removed the unused fixed-rate I2S
  constructors and duplicate button decoder from `drivers/hw`; audio I2S and
  input are now the sole owners of those behaviours.
- [x] Make recording names collision-safe, recover the automatic counter from
  backup/temporary copies, finalize through `.part`, check WAV headers and
  release RX on every task-start failure. Physical fault injection remains in
  the Microphone service hardware checklist.
- [x] Serialize SD mount/unmount and Orientation startup. The MPU sampler names
  its stack size in bytes and uses its intended 4096-byte default. Orientation
  publishes readiness only after both worker tasks exist and cleans failed SPI
  attempts before permitting a retry.

- [x] Separate OLED transport from app drawing helpers; separate input events
  from FFT suppression policy. Host checks pass; hardware verification pending.

- [x] Pilot apps/services/drivers separation with Orientation and Paint. Their
  UI, shared logic and MPU/LED hardware boundaries now follow the architecture.
- [x] Centralize gravity-preserving calibration in Orientation. The filter
  publishes absolute pose, freshness policy is shared, and Arm Pose, Paint,
  Synth and 3D own independent heading references through one reference type.
- [x] Extract Paint protocol/session/rendering from its UI, preserving timing and ownership (host tests/build pass; hardware acceptance pending).
- [x] Share posed suit geometry across normal animations, Paint and Shows (hardware visual acceptance pending).
- [ ] Extract existing resource policy and packet dispatch without changing scheduling.
  FFT/audio policy is now in services/audio_policy; packet dispatch and broader
  resource coordination remain.
- [x] Separate FFT/BT screens from background logic. The FFT viewer now owns
  only a policy request and snapshot rendering; leaving releases its request
  without stopping other clients. Bluetooth UI owns selection and commands,
  while discovery, connection and streaming remain in the service.
- [x] Compare screens and centralize list, grid, scroll, title/status-page and
  legend presentation in UI Shell. Apps own motion and wording; dynamic
  transport lists, graphs, games and geometric renderers remain specialized.
- [ ] Separate lighting effects from hardware transmission; audit audio/BT boundaries before changing them.
- [ ] Write one design explanation per app/service/driver. Validate each by
  removing its first `Pending review` line; see [documentation](docs/README.md).
- [ ] Build/test each migration and request targeted hardware verification.
- [ ] Confirm the physical battery divider ratio and whether three battery
  slots should eventually have independent sensors. Current display mirrors
  one measurement; see services/power/README.md.

## Preserved earlier wishlist

Copied from the earlier project notes; not revalidated. Some items are complete,
partial or stale. Reconcile each item against its owning subsystem before acting.

Open TODO / wishlist
---------------------
- [ ] Document the entire ESP32/suit system in English, down to logic choices
  and their rationale. Track every live file in docs/documentation/file-checklist.md.
  A tick requires an explicit file-level review, not merely an overview mention;
  source changes invalidate that tick. Review rules and regeneration commands:
  docs/documentation/README.md. Include APIs, state/timing, task and resource
  ownership, errors/recovery, hardware assumptions, tradeoffs and test limits.
  Add separate inventories for ShowEditor, localFFT, relay/server utilities,
  SD packages, wiring and ESP-IDF local patches before declaring system coverage.
  Never reproduce credentials. Unknown rationale must be labelled unknown.
Progress review (2026-10-19; keep the requests below as the design backlog):
- Message UI first pass: explicit New/Outbox/Inbox navigation, deterministic
  cached browsing, real play/pause, no draw-time SD calls or hidden deletion.
  Recording names avoid overwriting incomplete files. UI bounds/control tests
  and direct-transfer tests pass. Still TODO: named recipient selection,
  per-suit sent history and persistent unread state. Recording now has a live
  peak-envelope graph fed by captured PCM, with ring/window tests; it freezes
  after stopping. No extra microphone reader or draw-time file access.
  Main/Comm arrival badges now exist for this boot, with tests for successful
  delivery, failed checksums, ACK retries and scan-time arrival races.
  New-message A/B are temporarily back/send, not recipient selection.
- Button consistency first pass: animation grid now uses A left/B down/C right/
  D select; Light Paint's list uses A/B and D with an explicit Back row.
  Speed still adjusts both ways in a D-entered edit mode. Host controls tests
  cover column bounds/wrap, selection and speed limits. Other apps still need
  individual review; gameplay, recording and playback shortcuts are unchanged.
- Implemented, hardware acceptance pending: Light Paint + shared arm math,
  heading-only Paint calibration, 1.5 m waist-centred target; Arm Synth with
  local/Bluetooth output. See [Paint](services/paint/README.md),
  [Orientation](services/orientation/README.md) and
  [Arm Synth](apps/synth/README.md).
- OLED artefact experiment: full-frame refresh is now the default. Every call
  sends all 1024 panel bytes, even for unchanged frames. Existing app frame
  rates are unchanged. To compare, menuconfig -> Suit OLED -> changed-region
  updates; CONFIG_SUIT_OLED_PARTIAL_UPDATES defaults to off. Test scrolling,
  app transitions and long BT shows before deciding which mode to keep.
  Host panel-emulation tests cover full/partial updates, clears and stale pixels.
- Pong audit: authoritative paddle impacts are computed by that paddle's suit;
  both suits advance the ball locally. The remote side also predicts a bounce
  for smooth display, then reconciles to the owner's event by midfield (late
  events use an 80 ms correction). Event snapshots are retransmitted in state
  packets for packet-loss recovery, not sent only once. Host tests pass;
  two-suit feel/latency still needs testing. No Pong logic changed in this pass.
- Implemented, multi-suit hardware check pending: selecting a show and pressing
  D on a follower starts it as the new playback leader; the old leader stops
  its audio and follows. A/B/C costume roles do not change. Starting the same
  show on a follower restarts it; D on the current leader's same show still
  pauses/resumes. Transport carries an increasing leadership term; concurrent
  starts tie-break by MAC, older terms cannot regain control, and release
  packets must match the term. Tests cover takeover, duplicates and stale terms.
  Update ALL participating suits (show sync protocol 0x53). Radio delivery and
  audio startup are asynchronous: brief overlap during handoff is possible;
  this is not a crossfade or a simultaneous hard audio cut.
- Defer broad rewrites until each has a bounded test: Live Sync,
  message navigation/notifications, animation catalogue and button consistency.
  Game SFX need audio ownership/mixing first; do not interrupt music or calls.

- fft still has this issue where it's not able to lock in on the right phase when its bpm is close to what it should be. i don't know how to describe it and my interpretation is probably wrong so don't try to correct for a problem that doesn't exist but i observe that sometimes a reset is required for it to work when i would want it not to be required. either the found bpm was slightly off for some reason ? or the phase was constentely being adjusted where it shouldn't be ? i dont know. tell me what you think it might be, we want a discussion here, no changes until proof.
 -- thinking about it more, it think this is actually what we built in, the fact that we don't want to change the phase lock once we lock in, so that we don't go to a different phase and would need to reajust. what i think we could do is to allow for sliding of the phase if we are in the neigbourhood of the max phase, not too close to the offbeat because that would mean we could switch to it which we don't want. the phase slide is only allowed in specific case which we have to discuss. i'm not entirely sure what's the current implementation as well, so what i said still stands we should discuss. we might need an additional time stability condition ? again i'm speculating here. one thing at a time.

- light painting with accelerometer as the brush on other suits. (default position as horizontal like a folded arm and target in front so first thing is to unfold arm.) along with that we can fix arm pose app, and make a common math pipeline if appropriate.
: almost there, but the armpose and paint app are not handling orientation correctly, the accelerometer y component is towards the fingers, x is towards the body when the arm is horizontal at the reference and while Z is pointing up. the screen has its x axis towards the mpu Y and y axis towards the mpu's negative x. when we point we want the opposite rotation behavior as the threedee environment (the same as its camera) and the target should be towards the negative x when we calibrate.rolling the upper arm on itself triggers a Y axis rotation which on the screen should be seen as pointing up or down.shaking someone hand is a Z rotation and giving someone a palm in the face is an X rotation, idk if that makes any sense :p
- accelerometer synth (theremin like? idk) !
- role lead/follow switching if someone else starts a song.
- Menu/UI: replace placeholder icons with ascii-art
- games sfx, conflict with audio player playing sfx ?
- add effect that goes round along the idk which axis it is but the one that goes when the person inclines forward
- gesture menu selection ? (trigger things with limb position/movement)

- do spatial aware led animations based on limb orientation or motion amount (new app)
- tron
- pacman
- asteroids game !
-fix message : we want to have main menu notification that a new one arrived, on the message app we want to be able to select either send new message, view sent list or listen to recieved message (find appropriate not too long names) on the view app : we have a history of costume specific messages, which we can navigate with A and B, listen to with C (play pause) and a shortcut to new message with D; on new message app we have a visual audio graph that starts drawing when we press record, stops when we press again (button start stop C), the button D is play pause to listen to it and AB select target ([name1],[name2] or everyone) which means we need names for each suits, maybe we got that already but if not, add it (same name used for game leaderboards)
on the listen app, we get a list of recieved message, scroll through with A/B listen to with D.
any suggestions are welcome, tell me if you think any other features could be great to include.
- [ ] Review the animation catalogues visually. Beat-triggered and continuous
  Ambient modes now have separate apps without changing effect maths or stable
  mode indices. Decide with hardware feedback which similar-looking modes are
  genuine duplicates and whether any Ambient mode belongs in Beat before
  removing or converting it.

- maybe the keyboard could find a use when choosing names for messages for example, maybe we make it an overlay.
- global UI rework, prettify, normalize, animate. there are artefacts because of our screen optimization, i don't know if i like that, well i don't. but sometimes it might be required to avoid clutter on the SPI. this needs testing but i would like to try to disable it for now so that i can complain if it's actually a problem.
