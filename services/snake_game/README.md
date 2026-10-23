Pending review

# Snake game

The service owns a 32-by-14 board, a body of up to 120 cells, direction, food,
timing and best length. The app supplies directional intents and elapsed time.
It reads a const view of state on the same thread; that view is not a concurrent
snapshot.

## Starting and advancing

Initialization loads the high score, with a minimum displayed best of four,
then starts a four-cell snake facing right at the board centre. Food placement
tries 64 random positions outside the body before falling back to (0,0).
The fallback can overlap the body; it is unchanged.

Elapsed time accumulates and advances in one-sixth-second steps. A delayed tick
can run multiple steps. Direction changes reject the immediate opposite of the
current direction. Multiple turns before a movement step remain possible.

Each step computes a new head and checks walls and all existing body cells
before shifting the body. This includes the tail that would otherwise move.
Food grows the body by duplicating its final shifted cell and then respawns.
The capacity limit prevents further growth but still allows eating/respawning.

## Death and scores

Collision marks the game dead. Later steps leave it still. Any press restarts
a dead game, retaining the current best length. While alive, invalid direction
intents are ignored.

A new best length is persisted through the
[score service](../scores/README.md) when food is eaten. Saving blocks that
tick. Failure keeps the new best in memory and logs a warning; there is no
automatic retry until a later new record.

All rules and random-number calls preserve the former implementation. The
service has no display, button types, navigation callback or task of its own.
