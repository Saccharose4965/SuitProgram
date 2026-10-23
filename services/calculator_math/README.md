Pending review

# Calculator math

The service evaluates arithmetic expressions without UI state. Parsing walks the text
recursively: parentheses/functions and unary signs, then powers, then
multiplication/division/remainder, then addition/subtraction. Powers associate
to the right, so 2^3^2 means 2^(3^2). Existing unary precedence makes -2^2
evaluate to 4; that convention is preserved.

Supported names are pi, sqrt, ln and exp. The percent key means remainder;
the parser also accepts mod. Division or remainder by a magnitude at or below
1e-12 reports DIV0. Invalid syntax, invalid function domains and non-finite
results report ERR. A successful parse must consume the entire expression.

The evaluator has no screen, input or global expression state. The calculator
limits input to 63 characters. Recursion depth follows the supplied expression;
other callers must also bound their input.
