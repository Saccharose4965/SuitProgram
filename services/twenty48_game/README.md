Pending review

# 2048 rules

The service owns the old board, proposed board, movement distances, move
direction and scores. Tile values are exponents: 1 represents 2, 2 represents
4. Board coordinates and cell displacement are independent of screen pixels.

Initialization clears the board, loads the best score and adds two tiles.
Each new tile chooses an empty cell uniformly and is 2 with 90% probability,
otherwise 4. Random calls retain their previous order.

A directional move scans each line from its destination edge. Empty cells
increase the shift distance; equal adjacent tiles merge once, then the merge
candidate is cleared so the resulting tile cannot merge again in that move.
The merged tile's value adds to the score and updates the in-memory best.

A move computes the proposed board and per-origin displacement. If it changes
the board, a new random tile is added and the app animates the old tiles.
Commit replaces the old board after that animation. A no-op creates no tile.
Directions are right/down/left/up numbered 0–3; invalid requests are rejected.
The app serializes moves and commits through the shell callback path.

Game-over detection examines the committed board. Play can continue when an
empty cell exists or a horizontal or vertical neighbour has the same value.
A full board without either condition is finished.

Deinitialization saves a dirty best score through the
[scores service](../scores/README.md). Failure is logged. The service exposes a
const state view to the shell-owned app lifecycle. Board persistence remains
outside the current game design.

The existing exponent/score arithmetic is retained, including its finite
integer limits. Display sprites impose their own maximum visible tile.
