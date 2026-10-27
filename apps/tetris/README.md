Pending review

# Tetris screen

The app presents the [Tetris rules service](../../services/tetris_game/README.md)
through ready, playing and results states. Any game button dismisses the ready
overlay and is consumed. During play A/D move left/right and B/C rotate left/
right.

The shell supplies button events and elapsed time. Accepted actions start a
140 ms cooldown. The falling piece advances every 450 ms; long display frames
are capped before entering the accumulator. When a drop reports game over, the
app preserves the final state and enters results.

Results immediately displays the final score. Control icons disappear for two
seconds, then a refresh icon appears in the D position. D initializes a new run
and enters play directly.

Drawing targets the shell framebuffer. The 10-by-12 board uses four-pixel cells
below the score row. Settled blocks and the falling piece use distinct fill
sizes. The next-piece preview begins at y=10 on the right, below the record.

Entering creates a preview run and loads the saved record. Leaving finishes the
active service state so a new record is persisted. The shell callback path
serializes controls, falling, rules state and presentation.
