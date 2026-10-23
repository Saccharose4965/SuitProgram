# Work queue

## Agreed rework

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
  Review every Show, master-control, Paint, Pong, Message and Call packet
  together; add runtime shutdown/reinitialization only with concurrency tests.

- [ ] Publish LED beat and standalone animation settings as coherent snapshots.
  Current colour/style/background scalars can be observed across separate
  writes for one rendered frame. Hardware-test every beat/standalone mode and
  synchronized shuffle before changing effect maths.

- [ ] Hardware-review the LED layout editor across both strips: walk every
  section and ring start, change/revert a length and direction, save, reboot and
  confirm the physical mapping. Define crash-safe recovery for the FAT
  remove-then-rename fallback before treating SD writes as transactional.

- [ ] Replace Bluetooth's Music-specific disconnect callback with a service
  event consumed outside draw callbacks. Break the Audio Player/Bluetooth build
  cycle only alongside playback regression tests; retain the no-sniff policy
  until long-duration hardware tests prove an alternative.

- [ ] Move File RX's FFT/Bluetooth interruption decision into shared resource
  policy. Add authenticated/versioned framing, byte count/checksum and receiver
  progress before treating TCP close as a verified file transfer.

- [ ] Validate Bad Apple audio/video timing on local speaker and Bluetooth;
  replace the fixed one-second offset only with measured or shared-clock timing.
  Decide whether source cycling should restart matching audio.

- [ ] Redesign Telemetry session/transfer concurrency with Link, Messages, Call
  and Comms together: serialize socket ownership and ACK routing, version the
  native-endian protocol, define buffer delivery semantics and add receiver
  shutdown/multiple-sender handling.

- [ ] Once shared orientation startup is guaranteed, remove Fluid's direct MPU
  recovery path so every orientation consumer uses the same calibration. Review
  the 300 ms force-delete fallback before changing its worker lifecycle.

- [ ] Fix Flappy exit during splash and preserve worker ownership until actual
  termination; prevent overlapping sessions. Review repeat-counted pipe scores.

- [ ] Review 2048's direct legacy button polling, framebuffer publication and
  worker-stop timeout before changing lifecycle/input behaviour.
  Tetris also needs framebuffer publication, stop-timeout ownership and
  task-creation error handling reviewed.

- [ ] Before reusing keyboard as an overlay: define accept/cancel without losing
  text, handle UTF-8 editing, connect held input, align popup hit-testing and
  respect the content rectangle. Existing standalone behaviour is preserved.

- [ ] Verify high-score replacement on FAT and tighten malformed-file handling;
  preserve existing scores during any persistence fix.

- [ ] Finish cleanup within migrated drivers: audit legacy I2S/button helpers in hw.
- [ ] Review recording persistence and failure cleanup: collision-safe automatic
  names, checked writes, counter recovery and RX release on task-start failure.
- [ ] Review SD mount serialization, shared-state initialization ordering and
  MPU sampler stack-size naming/default. Documented limitations remain open.

- [x] Separate OLED transport from app drawing helpers; separate input events
  from FFT suppression policy. Host checks pass; hardware verification pending.

- [ ] Pilot apps/services/drivers separation with orientation and Paint.
- [ ] Centralize gravity-preserving calibration in orientation; audit every consumer, including Arm Pose, Synth and 3D Render. Keep reference ownership explicit so one app does not silently recalibrate another.
- [x] Extract Paint protocol/session/rendering from its UI, preserving timing and ownership (host tests/build pass; hardware acceptance pending).
- [x] Share posed suit geometry across normal animations, Paint and Shows (hardware visual acceptance pending).
- [ ] Extract existing resource policy and packet dispatch without changing scheduling.
  FFT/audio policy is now in services/audio_policy; packet dispatch and broader
  resource coordination remain.
- [ ] Separate FFT/BT screens from background logic.
- [ ] Compare screens, then introduce shared list/scroll/title/status/legend helpers under app infrastructure. Keep custom graph/game drawing. No separate display app.
- [ ] Separate lighting effects from hardware transmission; audit audio/BT boundaries before changing them.
- [ ] Write one design explanation per app/service/driver. Validate each by
  removing its first `Pending review` line; see [documentation](docs/README.md).
- [ ] Build/test each migration and request targeted hardware verification.
- [ ] Confirm the physical battery divider ratio and whether three battery
  slots should eventually have independent sensors. Current display mirrors
  one measurement; see services/power/README.md.

## Preserved earlier wishlist

Copied from the old README; not revalidated. Some items are complete, partial or
stale. Reconcile per subsystem, never assume every item is a defect. Earlier
status sections remain in [the archive](docs/reference/previous-readme.txt).

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
  local/Bluetooth output. See controls above and docs/reference/technical/light-painting-guide.md.
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
- Defer broad rewrites until each has a bounded test: master-control sync,
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
- confirm for pong that the trajectory is computed only once upon collision, by the person where the collision happens and sent only at that point to the other suit for it to correct it's own trajectory given the data it recieved. because the collision might be interpreded differently as the position might not have been the same on the two, we do interpolation of the trajectory by starting from initial thought to be the correct trajectory and slowly merge towards the other one, smoothly. so that means we don't get lag.
- uniformity with the buttons : the last one is always the select button (if there needs one) if we have a 2d navigation, we do left and right with A and C, B does the down, if we only have a simple list 1D, we do A and B up and down, or left and right.
-fix message : we want to have main menu notification that a new one arrived, on the message app we want to be able to select either send new message, view sent list or listen to recieved message (find appropriate not too long names) on the view app : we have a history of costume specific messages, which we can navigate with A and B, listen to with C (play pause) and a shortcut to new message with D; on new message app we have a visual audio graph that starts drawing when we press record, stops when we press again (button start stop C), the button D is play pause to listen to it and AB select target ([name1],[name2] or everyone) which means we need names for each suits, maybe we got that already but if not, add it (same name used for game leaderboards)
on the listen app, we get a list of recieved message, scroll through with A/B listen to with D.
any suggestions are welcome, tell me if you think any other features could be great to include.
- fix master control, it should have the same sync method as the show, that one worked very well(master control is broken), settings changes should sync as well any changes are accepted, from both costumes, master control can be taken from the other or deactivated.
- fix the absolute disaster mess that the animations app is. we want all those animations to be bpm controlled, some of them are not, some of them are dupliacted, add an app for non bpm syncable animations.

- maybe the keyboard could find a use when choosing names for messages for example, maybe we make it an overlay.
- global UI rework, prettify, normalize, animate. there are artefacts because of our screen optimization, i don't know if i like that, well i don't. but sometimes it might be required to avoid clutter on the SPI. this needs testing but i would like to try to disable it for now so that i can complain if it's actually a problem.
