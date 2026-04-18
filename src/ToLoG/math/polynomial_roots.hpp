#pragma once

#include <vector>

namespace ToLoG
{

std::vector<double> solve_quadratic(double a, double b, double c);

std::vector<double> solve_cubic(double a3, double a2, double a1, double a0);

std::vector<double> solve_quartic(double a4, double a3, double a2, double a1, double a0);

}
