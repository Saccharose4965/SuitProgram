Pending review

# Snake screen

The app presents the [Snake rules service](../../services/snake_game/README.md)
through ready, playing and results states. Any direction dismisses the ready
overlay and is consumed. During play A/B/C/D request left/up/right/down turns.

The shell supplies button events and elapsed time. The rules service advances
at six cells per second and buffers up to two legal turns. Collision enters
results immediately. Control icons then disappear for two seconds before a
refresh icon appears in the D position; D starts a new run.

The playfield contains 29 by 11 four-pixel cells. A two-pixel border surrounds
it between the eight-pixel score row and bottom control legend. The score and
record count apples eaten, derived from the service's current and best lengths
after subtracting the four starting cells.

Snake cells remain solid squares. Food uses a small four-by-four apple silhouette
with a raised stem, rounded body and narrow base. Ready and results use the
shared modal overlay inside the playfield.

The app draws directly into the shell framebuffer. The shell callback path
serializes input, simulation and presentation, while reset and turn validation
remain in the rules service.
