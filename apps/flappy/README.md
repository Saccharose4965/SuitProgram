Pending review

# Flappy screen

The app presents the [Flappy game service](../../services/flappy_game/README.md)
through three states: ready, playing and results. Ready shows the saved record;
D starts a run. Every button press flaps during play. Results shows the run score
and record; D starts a new run. The app has no bottom legend.

The shell owns execution. Its elapsed time feeds a 30 Hz fixed-step accumulator,
which keeps the game rules independent from display rate. Long frames are capped
at 250 ms and at eight recovery steps, bounding the work performed in one shell
frame. Multiple presses before the next simulation step become one flap.

Drawing happens directly into the shell framebuffer. It renders the pipes,
ground, shared bird mask, current score and record. The bird flips after impact
and blinks during the death sequence. Ready and results use the shared modal
overlay helper over the playfield.

Entering initializes one game state and loads the record. Leaving an active run
calls finish so a new record is persisted even when the player exits mid-run.
The shell's single callback path owns all mutable presentation state and makes
exit immediate.
