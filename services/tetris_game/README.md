Pending review

# Tetris rules

The service owns a 10-by-12 board, falling piece, next piece and scores.
Seven tetrominoes each have four explicit cell layouts. The same layouts are
exposed to rendering so collision and drawing agree on occupied cells.
Coordinates are board cells; screen pixels belong to the app.

## Movement and rotation

Horizontal moves are accepted when every occupied cell stays within the side
walls and above the floor, without touching settled blocks. Cells above the
board are allowed while a piece enters.

Rotation tries the requested orientation at the current location, then offsets
of one left, one right, two left, two right and one up. The first valid attempt
wins. These are the existing simple kicks, not a full competitive Tetris ruleset.

Each drop request moves down one cell if possible. Otherwise a piece whose
origin is still above the board resets the board and current score. A piece
inside the board locks into the occupied cells and complete rows are removed,
shifting higher rows down. Each cleared row awards 100 points.

Both lock and game-over reset spawn the queued piece at the top and choose a
new random preview. Shapes use independent random selection, without a bag.
The shared spawn path keeps these two transitions consistent.

## State and persistence

Initialization clears the board and score and loads the saved best through the
[scores service](../scores/README.md). A new record marks it dirty. Finish saves
that record, logging failures. Resetting after game over retains the best.

The app owns timing and calls move, rotate and drop synchronously on its worker.
The service creates no task, reads no buttons and draws nothing. Its const state
view is for that worker; it is not a synchronized snapshot for other tasks.
