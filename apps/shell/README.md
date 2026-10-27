Pending review

# Application shell

The shell is the foreground application host. It initializes shared hardware and
services, owns the app registry, polls buttons, performs app transitions and
composes the HUD, content area and button legend into each OLED frame.

## Startup

The ESP entry point creates the shell task on CPU1. The shell initializes the
OLED, audio bus, orientation service, shared state, saved settings, SD-backed
layout and gyro bias, Link routing and the built-in app registry. Its startup
logo runs in a separate temporary task and can be skipped by input.

LED preferences load with the other settings, then apply after the LED layout
has established suit geometry. The restored renderer and mode seed the HUD so
its first label describes the animation that is actually running.

Initialization order matters because later services consume the shared buses and
saved configuration established earlier. Failures are logged and the shell
continues where the affected subsystem supports degraded operation.

## Frame loop

Each frame polls input, queues and applies at most one app switch, advances
background Show, LED Source and Live Sync services, then calls the active app's
tick and draw callbacks. Show ownership suppresses the normal LED clock. A
Follow source suppresses manual-BPM app ticks while remote timing owns LEDs.

Before app work, the shell drains Bluetooth's small service-event queue.
Disconnect events notify Audio Player, while AVRCP play/pause requests use the
normal Audio Player API. This keeps service and app work out of Bluedroid and
OLED draw callbacks.

The shell snapshots system state for the HUD, renders normal apps into its shared
framebuffer and submits one OLED frame. Apps marked external own their display
path and skip shell composition. Frame pacing targets 60 Hz and passes timing
samples to the performance monitor.

App changes run the outgoing cleanup callback before the incoming initializer.
Requests are queued so callbacks cannot replace the active app midway through a
frame.

## Remote-control lease

Remote Control is a shell-level lease because it changes who owns foreground
input. Starting a session sends the target to the main menu. The controller
stays in its Remote Control host app and forwards A/B/C/D presses; the target
dispatches them to its real active app from the shell task. This starts workers
and hardware only on the target.

While leased, the target ignores its physical buttons and the controller cannot
trigger its own global shell shortcuts. Both sides still sample the debounced
A+B level. Holding it continuously for three seconds ends the session and sends
both shells to the main menu. Link timeout performs the same cleanup so neither
suit can remain trapped after its peer disappears.

After composing each target frame, the shell publishes the standard widget's
semantic capture. The controller renders that state locally. Apps with custom
drawing still run on the target and accept remote buttons, while the controller
shows a placeholder instead of duplicating their visual state or computation.

## Registry and menus

`shell_app_registry.h` is an internal declaration bridge for apps whose public
headers have not yet been normalized. It is not a shared service API. The root
and category menus map stable app IDs to labels; the status screen reads the
frame snapshot and active Link path.

Menu pages use the shared three-visible-item block renderer. Each page owns its
scroll position; optional per-entry sprites currently use placeholder artwork.
The Status screen uses the shared clipped text-page layout.

The shell contains presentation and orchestration only. Feature algorithms,
protocols, persistence and device access live in their service or driver owners.
