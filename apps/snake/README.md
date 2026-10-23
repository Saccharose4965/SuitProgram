Pending review

# Snake screen

The app translates button presses into directions and draws the state supplied
by the [game service](../../services/snake_game/README.md).
A/B/C/D mean left/up/right/down. Other press events carry no direction but
still restart a dead game, preserving the existing press-any behaviour.

Initialization starts a game. Shell ticks advance it; drawing only reads it.
The app and service run on the shell thread without a new task.

Cells are four pixels square, below an eight-pixel score band. Body and food
use solid squares. The top line shows current length and best length; death
adds a restart prompt. Rendering currently uses the full panel and ignores
the supplied content rectangle. That existing layout is preserved.

Shell callbacks, button mapping and drawing live together in snake.c. No
navigation callback is stored because the game never used it.
