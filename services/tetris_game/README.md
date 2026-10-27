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

Each drop request moves down one cell if possible. A blocked piece whose origin
is still above the board marks the run finished and preserves the final board
and score. A blocked piece inside the board locks into the occupied cells and
complete rows are removed, shifting higher rows down. Each cleared row awards
100 points.

After a normal lock, the queued piece spawns at the top and a new random preview
is selected. Shapes use independent random selection, without a bag. Movement,
rotation and drop calls leave a finished run unchanged until initialization
starts the next run.

## State and persistence

Initialization clears the board, score and game-over state and loads the saved best through the
[scores service](../scores/README.md). A new record marks it dirty. Finish saves
that record, logging failures; a successful save clears the dirty flag.

The app owns timing and calls move, rotate and drop synchronously from the shell
callback path. Its const state view supplies the same path with board, piece,
score, preview and game-over data.
