Pending review

# 2048 screen

This app owns input polling, slide animation, tile sprites, drawing and the
worker lifecycle. Board updates and scores come from the
[game service](../../services/twenty48_game/README.md).

## Input and timing

The existing worker runs at priority 5 with a 4096-byte stack on core 1
(core 0 on a single-core build). Each iteration waits 12 ms after its work.
It polls the legacy board button decoder directly; shell input callbacks
remain unused. A transition from no button to a button requests a move:
A left, B up, C right, D down. Input arriving during animation is ignored.
The remembered button persists across app sessions, as before.

Changed boards animate by advancing a phase by 0.2 each iteration. Cosine
easing turns the game's cell displacements into pixel offsets. At pi, the app
commits the proposed board and redraws at rest. Animation speed therefore
depends on worker cadence, not measured elapsed time.

## Drawing and lifecycle

The board uses 15-pixel sprites with a 14-pixel stride. A cached grid background
is restored each frame; moving old tiles are overlaid, then score labels appear
to the left. The new random tile appears when the move commits.
Sprite generation is kept beside its assets; gen_sprites.py reads the project's
2048.png and writes t48_sprites.h.

The shell copies the worker framebuffer for display. That copy is unsynchronized
and can catch a partially rendered frame. Layout uses panel coordinates rather
than the supplied content rectangle.

Exit requests stop and waits up to 500 ms while the worker saves the best score.
The existing timeout then clears the app's task handle even if the worker has
not finished. This and input-decoder consolidation need a focused follow-up;
their timing has not been changed during migration.
