#include "epsilon.hpp"
#include <ToLoG/math/polynomial_roots.hpp>
#include <gtest/gtest.h>

namespace ToLoG
{

TEST(PolynomialRootsTest, QuadraticTest)
{
    double a, b, c;
    std::vector<double> x;

    a = 1, b = -2, c = 1;
    x = solve_quadratic(a, b, c);
    EXPECT_EQ(x.size(), 1);
    EXPECT_DOUBLE_EQ(x[0], 1.0);

    a = 3, b = -4, c = 1;
    x = solve_quadratic(a, b, c);
    EXPECT_EQ(x.size(), 2);
    EXPECT_DOUBLE_EQ(x[0], 1.0/3.0);
    EXPECT_DOUBLE_EQ(x[1], 1.0);

    a = 2, b = -3, c = 4;
    x = solve_quadratic(a, b, c);
    EXPECT_EQ(x.size(), 0);

    a = 0, b = 1, c = -5;
    x = solve_quadratic(a, b, c);
    EXPECT_EQ(x.size(), 1);
    EXPECT_DOUBLE_EQ(x[0], 5.0);
}

TEST(PolynomialRootsTest, CubicTest)
{
    double a, b, c, d;
    std::vector<double> x;

    a = 1, b = -6, c = 11, d = -6;
    x = solve_cubic(a, b, c, d);
    EXPECT_EQ(x.size(), 3);
    EXPECT_DOUBLE_EQ(x[0], 1);
    EXPECT_DOUBLE_EQ(x[1], 2);
    EXPECT_DOUBLE_EQ(x[2], 3);

    a = 1, b = -5, c = 8, d = -4;
    x = solve_cubic(a, b, c, d);
    EXPECT_GE(x.size(), 2);
    EXPECT_NEAR(x[0], 1, epsilon);
    EXPECT_NEAR(x[1], 2, epsilon);
    if (x.size()==3) {
        EXPECT_NEAR(x[2], 2, epsilon);
    }
}

TEST(PolynomialRootsTest, QuarticTest)
{
    double a, b, c, d, e;
    std::vector<double> x;
}

}
