Pending review

# 2048 screen

The app presents the [2048 rules service](../../services/twenty48_game/README.md)
through ready, playing and results states. Any directional button dismisses the
ready overlay. That press is consumed, and the next direction performs the first
move. Controls remain A left, B up, C right and D down.

The shell supplies button events and elapsed time. A legal move starts a cosine-
eased animation lasting approximately 190 ms. Input is accepted again after the
proposed board is committed. The tick callback detects a finished committed
board and enters results.

Results immediately overlays the final score. Direction icons disappear during
a two-second lockout, after which a refresh icon appears in the D position. D
then initializes a new board and enters play directly.

The app draws the cached grid, moving tile sprites, score and record directly
into the shell framebuffer. Tile sprites use a 15-pixel shape on a 14-pixel
stride. `gen_sprites.py` regenerates `t48_sprites.h` from the project image.

Entering loads the saved record and creates the preview board. Leaving saves a
new record through the rules service. The shell callback path serializes input,
rules updates and rendering.
