Pending review

# Tetris screen

The app owns the worker, input timing and drawing. Board state and collision
rules come from the [game service](../../services/tetris_game/README.md).

The worker uses core 1 on dual-core builds, core 0 otherwise, priority 5 and a
4096-byte stack. It samples input directly each iteration: A/D move left/right,
B/C rotate left/right. A changed button is accepted only after 140 ms since
the previous accepted action. Holding a button does not repeat; an edge inside
the cooldown is discarded. The shell legend remains blank.

Every 450 ms it requests one downward step. Delays do not generate multiple
catch-up drops. After input, falling and drawing, the worker waits 30 ms.
These polling and timing rules preserve the original game.

Drawing centres four-pixel cells below an eight-pixel top band. Settled and
falling blocks use different fill sizes. A preview and scores occupy the top.
Shape cells come from the service; screen placement and clipping stay here.
The renderer uses panel coordinates and ignores the shell content rectangle.

The shell copies the worker's framebuffer without synchronization, so a copy
can catch a partial frame. Exit requests stop and waits up to 500 ms; the worker
saves the best score before exiting. The app still clears its task handle after
the timeout even if the worker remains busy. Task creation errors are not
reported to the UI. These existing lifecycle limits need a focused follow-up.

Lifecycle, polling and rendering are grouped in tetris.c, with no forwarding
adapter or separate drawing task.
