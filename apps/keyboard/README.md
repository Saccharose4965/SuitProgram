Pending review

# On-screen keyboard

This module owns the keyboard's layout, selection, text editing and drawing.
Its screen adapter lives in the same implementation. These operations describe
UI interaction; it performs no background processing or hardware acquisition.
The reusable state/API remains public for a future text-entry overlay.

## Pages and text

The grid has ten columns and five rows, with lowercase and uppercase AZERTY
pages and two symbol pages. Shift cycles through single uppercase entry,
caps lock and lowercase. On symbol pages it switches symbol sets.
The spacebar occupies four adjacent grid cells but draws as one key.

Each instance owns its selection, pointer state and text. It uses an internal
96-byte text buffer by default, or a caller buffer whose capacity excludes the
terminator: the allocation must therefore provide capacity plus one byte.
Initialization clears the text. Ordinary input appends; backspace deletes the
last byte. There is no cursor within the text.

Current Enter handling clears the buffer and hides the keyboard. The standalone
screen makes it visible again on the next tick. It does not submit text to a
consumer, so an overlay needs an explicit acceptance contract before reuse.

## Pointer and suit controls

The widget selects a key by position and commits on release over that key.
Holding can expose accented variants after 0.4 seconds or repeat backspace
at 10 Hz. Leaving the active key resets the hold state. Variant selection uses
horizontal pointer position.

The suit screen maps A/C to left/right, B to down and D to an immediate
press/release pair. Navigation wraps. Moving right across the spacebar jumps
to its end. B+C also moves down; its long-press event deletes one byte.
Ticks send an unpressed hover event. Consequently, this adapter does not provide
the sustained pointer presses needed for accent popups or held-key repeat.

## Rendering and current limits

The app uses 10-pixel cells and positions the keyboard relative to HUD/legend
heights. It draws text at the panel's top-left and keys in panel coordinates;
the supplied content rectangle is currently ignored. Key interiors invert for
selection while borders stay fixed. Popup placement is clamped to the panel.

Text operations and label widths count bytes. Accented UTF-8 glyphs can be split
by truncation or backspace, and popup hit-testing does not account for the
renderer's edge repositioning. These existing limitations require a deliberate
input/UI fix. This migration preserves controls and rendering.

The module creates no tasks and persists no text. One static instance backs
the standalone screen; callers of the widget API can provide their own state.
