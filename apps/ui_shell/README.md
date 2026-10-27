Pending review

# Shared UI

This module provides screen composition and drawing for app screens. The shell
uses its HUD/content/legend layout, while applications draw their own content.
The HUD shows local FFT state and the Live Sync source. `sync:loc` means local
timing; `sync:A/B/C` identifies a Follow source and `*` marks a current link.

The modal overlay helper clears and frames a caller-selected rectangle, then
centres a title, two information lines and a prompt. It draws over existing app
content and owns presentation only; the calling app decides when the overlay is
visible and what input changes its state.

Choice screens share the main-menu visual language: a 9-by-9 sprite slot beside
a horizontal framed name block, with three entries visible. One-dimensional
lists scroll vertically. Two-dimensional grids contain three rows per column.
The active column occupies three-quarters of the content width while cropped
parts of its horizontal neighbours remain visible at the edges. Columns animate
horizontally as the selection changes. Each screen owns its
motion state, so another screen cannot alter its animation. Sprite pointers are
optional; missing artwork uses the common placeholder.

Standard menu and grid draws also record a bounded semantic description of the
current frame: layout kind, title, total and selected indices, nearby labels and
the button legend. The shell resets this capture before every frame. Remote
Control uses it to reproduce navigation without transmitting OLED pixels or
starting the mirrored app. A screen that draws custom content leaves the capture
empty. Small corner markers distinguish controller and target leases without
claiming content space.

Simple information screens use the shared text-page renderer. It places a title
and fixed eight-pixel-spaced 3x5 lines inside the supplied content rectangle,
clips each line to its width and stops before its height. File RX, Call, Status
and Gyro Calibration use this layout. Their apps own all wording and state
formatting. Bluetooth, Music, Show and Service Management keep specialized
lists because their rows combine live transport state or footer controls.
Graphs, games, media frames and geometric previews keep custom renderers.

app_shell.h owns the common app lifecycle contract: initialization, cleanup,
tick, input and drawing callbacks, plus the state snapshot and navigation
request callback. The shell implementation and registry live in `apps/shell`.

## Text and animation helpers

The drawing API is declared in ui_draw.h. Tiny text sets pixels in a supplied
framebuffer with panel-bound clipping. It uses a compact 3-by-5 font and fixed
character spacing. Fonts are owned here alongside their rendering logic.

The three-line helper clears its own framebuffer, centres three strings using
the larger font and submits the result to the [OLED driver](../../drivers/oled/README.md).
Its static buffer requires serialized calls.

The logo slide curve gives a small initial rightward movement followed by a
damped movement left. It returns a pixel offset for the caller's image placement.

A differential host test compares glyph pixels, clipping, three-line output and
scroll offsets against checkpoint 224f2cd. A focused text-page test checks line
truncation and vertical bounds. The differential test requires the checkpoint
in local Git history.

## Startup logo

startup_logo.c builds a monochrome 64-by-64 image from discs, lines and arcs.
Overlapping stages grow the centre, arms, tips, rings and connecting links.
Drawing these shapes and their easing curves is presentation logic owned here.

Playback draws 160 frames, holds for 250 ms, then slides the final image away
over 60 steps using the shared scroll curve. Each frame waits approximately
1/30 second after rendering, so rendering cost adds to the total duration.
The buffer is reused during the slide.

The caller supplies a skip callback. Playback checks it before frames and in
10 ms wait increments; a skip clears the display and returns. The shell owns
the startup task and button policy. The animation creates no task and requires
exclusive display use while playing. Its static buffer/callback support one
playback at a time. OLED initialization and transfer failures are not propagated
by this legacy void API.

## Remaining review

Text/curve extraction is verified. Final sprites and broader visual polish
remain in the work queue.
