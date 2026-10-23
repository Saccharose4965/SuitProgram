"""Small check of the app's production expression evaluator."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class CalculatorTest(unittest.TestCase):
    def test_expressions(self):
        source = r'''
#include "calculator_math.h"
#include <assert.h>
#include <math.h>
static void value(const char *s, double expected) {
    double result; bool zero = true;
    assert(calculator_evaluate(s, &result, &zero));
    assert(!zero && fabs(result - expected) < 1e-9);
}
int main(void) {
    value("2+3*4", 14);
    value("(2+3)*4", 20);
    value("2^3^2", 512);
    value("-2^2", 4); /* Existing unary-before-power behaviour. */
    value("sqrt(9)+ln(exp(1))", 4);
    value("7%4", 3);
    value("7 mod 4", 3);
    double result = 99; bool zero = false;
    assert(!calculator_evaluate("1/0", &result, &zero) && zero);
    assert(!calculator_evaluate("sqrt(-1)", &result, &zero) && !zero);
    assert(!calculator_evaluate("2+", &result, &zero));
    assert(!calculator_evaluate("2 3", &result, &zero));
    assert(!calculator_evaluate("exp(10000)", &result, &zero));
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "calculator"
            subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-I", str(ROOT / "services/calculator_math/include"), "-x", "c", "-",
                 str(ROOT / "services/calculator_math/calculator_math.c"), "-lm",
                 "-o", str(binary)], input=source, text=True, check=True,
            )
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
