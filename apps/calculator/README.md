Pending review

# Calculator

The app keeps one expression and a selected key in a 5-by-5 grid. Opening it
clears both. A/C move left/right, B moves down, and D presses the selected key;
movement wraps around. The existing B+C event also moves down.

## Editing

The expression holds up to 63 characters. Insertions check the preceding token
so the keypad can explain incomplete input immediately: a second decimal point,
unmatched closing parenthesis or missing multiplication produces a short error.
Multiplication must be explicit. Entering a new operator after an operator
replaces the previous one. DEL removes one character.

After equals, an operator continues from the result; a number or function starts
a fresh expression. After an error, the next insertion starts fresh too. Results
use eight significant digits, which also determines precision when continuing
from a displayed result.

Evaluation is provided by the [calculator math service](../../services/calculator_math/README.md).

## Display

The left pane shows the expression's trailing characters, errors and a live
result whenever the expression is complete. The right pane draws the keypad,
custom mathematical icons and a selection border. Live evaluation still occurs
during drawing; this cleanup preserves the existing refresh behaviour.

calculator.c owns the shell callbacks, editing and drawing. There are no
background tasks or saved calculator sessions.
