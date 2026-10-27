Pending review

# Snake game

The service owns a 29-by-11 board, a body of up to 120 cells, direction, food,
timing and best length. The app supplies directional intents and elapsed time.
It reads a const view of state on the same thread; that view is not a concurrent
snapshot.

## Starting and advancing

Initialization loads the high score, with a minimum displayed best of four,
then starts a four-cell snake facing right at the board centre. Reset starts a
new run while retaining the loaded best. Food placement
tries 64 random positions outside the body before falling back to (0,0).
The fallback can overlap the body; it is unchanged.

Elapsed time accumulates and advances in one-sixth-second steps. A delayed tick
can run multiple steps. Direction requests enter a two-item queue. Each request
is checked against the most recently queued direction, or the active direction
when the queue is empty. Duplicates and immediate opposites are rejected. Each
movement step consumes at most one queued turn before moving the head.

Each step computes a new head and checks walls and all existing body cells
before shifting the body. This includes the tail that would otherwise move.
Food grows the body by duplicating its final shifted cell and then respawns.
The capacity limit prevents further growth but still allows eating/respawning.

## Death and scores

Collision marks the game dead. Later steps and direction requests leave it
still until the app requests reset.

A new best length is persisted through the
[score service](../scores/README.md) when food is eaten. Saving blocks that
tick. Failure keeps the new best in memory and logs a warning; there is no
automatic retry until a later new record.

The service is synchronous and exposes its state to the shell-owned app path.
