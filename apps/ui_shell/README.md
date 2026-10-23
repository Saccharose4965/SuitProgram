Pending review

# Shared UI

This module provides screen composition and drawing for app screens. The shell
uses its HUD/content/legend layout, while applications draw their own content.

app_shell.h owns the common app lifecycle contract: initialization, cleanup,
tick, input and drawing callbacks, plus the state snapshot and navigation
request callback. The shell implementation and registry remain in main during
migration.

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
scroll offsets against checkpoint 224f2cd. That test requires the checkpoint in
local Git history.

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

Text/curve extraction is verified. The existing shell composition code still
needs a full design review; its detailed explanation is pending. Layout redesign
remains in the work queue.
