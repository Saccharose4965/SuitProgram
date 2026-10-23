#pragma once
#include <stdbool.h>

// Evaluate arithmetic independently of display and input.
bool calculator_evaluate(const char *expression, double *result, bool *division_by_zero);
