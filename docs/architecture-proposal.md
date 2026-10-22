# Architecture: apps, services, drivers

Discussion draft, not an implemented migration. The agreed layers are apps,
services (background work **and** shared logic), and drivers. There is no fourth
shared-logic layer. The purpose is to make ownership obvious, not to add wrappers.

## What the current dependencies tell us

This is a first map from component build declarations, includes and selected
call sites. It is not a complete call graph or a full review of every component.
An arrow means “uses”; a build dependency alone does not prove an active call.

| Feature | Current path | Boundary problem |
| --- | --- | --- |
| Arm input | MPU driver → orientation service → arm maths → Paint/Arm Pose/Synth | Mostly separated already; 3D Render still reads mutable filter state directly. |
| Painting | `main/app_light_paint.c` → orientation, link, show geometry, LED output | UI, protocol handling, brush rendering and ownership live in one app file. |
| Beat tracking | FFT component → audio, microphone, player, LEDs and OLED | Processing and display belong to different layers but share a component. |
| Bluetooth playback | player ↔ Bluetooth component → audio/ESP-IDF | Both build and source dependencies run both ways; BT remote play/pause calls the player. The BT component also contains its screen. |
| Lighting | show/modes → LED component → RMT/hardware/layout | `led.c` contains effects and audio brightness as well as transmission. It is not just a driver. |
| Shows | show runtime → rendering/package/layout, link and playback | A reasonable service grouping, but reusable suit geometry is named/owned as show-specific. |
| Network reception | link → shell dispatch → call app, Paint app, messages, show, Pong | The shell understands packet consumers instead of just screens. |
| Button noise handling | input driver → FFT suppression | Hardware input knows about a particular consumer's analysis policy. |
| Resource conflicts | apps → `app_led_support.c` → FFT/audio/BT/show state | System-wide policy is housed and named like LED UI support. |

Evidence anchors: `main/app_shell.c` (`link_frame_handler`, shell loop),
`main/app_led_support.c`, `components/input/input.c`,
`components/bt_audio/bt_audio.c` (remote playback command), and component
`CMakeLists.txt` files. `components/led/led.c` contains both RMT transmission
and beat-colour/brightness helpers. These are concrete seams, not inferred bugs.

There is also an important scheduling distinction: some services have their own
tasks, while show/source coordination is ticked by the shell loop. Moving code
into `services/` must not silently move that work onto new tasks or change its rate.

## Rules for the new structure

**Apps express intent.** A screen asks to start painting, play a show, or reset
tracking; it displays status and handles buttons. It should not know which DMA
buffer to release or which other screen happens to use the microphone.

**Services implement features.** They own feature state, coordinate resources,
and hold shared calculations. A service can run in a task, be ticked, or simply
be a function called by another service. “Service” does not mean “always running”.
Painting can remain opt-in and stop when its app closes; FFT can remain active
without its display being open.

**Drivers operate hardware.** They acquire samples or transmit data, expose
errors and respect ownership. They do not select effects, navigate screens or
decide that a button click should suppress beat evidence.

Apps may use simple display/input drivers directly; forcing every text draw
through a service adds no useful boundary. Services may depend on other services
explicitly. Drivers should not depend upward on apps or feature policy.

The shell is app infrastructure: navigation, shared screen layout and lifecycle.
Boot is the small place that assembles dependencies and starts them. Neither
needs to become a fourth functional layer.

## Proposed physical layout

Names below are a target for discussion, not instructions to move whole existing
components unchanged. Mixed files must first be separated by responsibility.

```text
SuitProgram/
  main/                       boot and dependency wiring only
  apps/
    shell/                    navigation, common UI, app registry
    paint/  synth/  arm_pose/  fft/  bluetooth/  shows/
    messages/  call/  settings/  games/ ...
  services/
    orientation/              fusion, coherent snapshots, calibrated arm maths
    beat_tracking/            capture coordination, BPM and phase logic
    audio/                    playback, recording, routing, synth generation
    connectivity/             suit transport and peer state
    messages/  call/           delivery and conversation state
    lighting/                 animations, output ownership, suit layout/geometry
    shows/                    packages, timeline, synchronization
    paint/                    brush projection, strokes, fading, protocol
    resources/                explicit conflict rules and release/resume policy
  drivers/
    board/  imu/  audio_io/  bluetooth/  leds/  display/  input/  storage/
  docs/
```

ESP-IDF components remain the build units. Explicitly register component
directories with the build; do not assume arbitrary nested folders are discovered.
Keep component names initially where possible, so path cleanup does not also
become a public-API rename. Board setup and shared buses belong under drivers.
ESP-IDF itself stays outside these three project-owned folders.

Bluetooth's hardware-facing stack adapter belongs under drivers; file playback,
routing and fallback belong under services; discovery/connection UI belongs under
apps. The exact split of its existing stream buffer/feeder is a later focused
review, not permission to rewrite a delicate working pipeline.

Suit geometry should be shared by shows and painting, not duplicated or loaded
through an app. Group shared logic beside the feature that owns it; avoid a
generic `utils/` dumping ground. Small app-local game rules need not become a
service just because they contain maths.

## The changes that matter more than folder names

1. **One explicit resource policy.** Extract the existing conflict rules first,
   preserving them. Owners request/release capabilities; the policy decides
   whether FFT must pause and when pending demand may resume. Start with concrete
   rules, not a general dependency scheduler. Preserve button-noise suppression;
   route the input event to that policy instead of deleting the behaviour.
2. **Data flows down; events report up.** Services expose status snapshots and
   accept commands. Drivers report events without calling app logic. BT remote
   controls can be wired to playback by a small callback at startup, breaking
   the reverse dependency without changing what the button does. No global
   event-bus framework is needed for a handful of known consumers.
3. **Separate screen lifetime from feature lifetime.** Closing an FFT graph is
   not stopping the detector. Leaving Canvas does release painting. Write these
   rules explicitly per feature instead of applying one universal lifecycle.
4. **Keep the existing timing model during extraction.** Thread/core placement,
   queues, allocation timing, wire formats, file formats and numerical results
   are compatibility constraints, even when internal names change.

## Order of work, after agreement

First use orientation/painting as the pilot: split UI from brush/session logic,
give both Paint and Shows a shared geometry boundary, and retain the existing
coordinate regression test. This is small enough to review as a complete flow.

Next extract resource policy and shell packet dispatch, preserving the present
call order and timing. Then separate FFT/BT display code from their services.
Defer LED transmission and audio/BT internals until their interfaces and regression
checks are clear; those have the greatest hardware timing risk.

Make moves and behavioural edits separate changes. Each batch needs a build,
relevant host tests and a short hardware check appropriate to the feature. Keep
a reversible checkpoint before migration. Do not delete backups, legacy-looking
components or wishlist items until their use is checked, rather than guessed.

## Documentation standard for this cleanup

Explain a feature as a chain of decisions: problem → information available →
chosen model → state transitions → failure behaviour → consequences. Mention
exact constants only when they affect understanding or tuning. Link source and
tests for implementation details instead of narrating C line by line.

Separate three kinds of statement: current behaviour visible in code, design
intent supported by our decisions/comments, and a proposed improvement. When a
historical reason is unknown, say so. Existing technical reviews remain reference
notes; their checkmarks mean source was inspected, not that this new explanation
standard is complete. The pilot is [How light painting works](design/light-painting.md).
