#include <gtest/gtest.h>
#include <ToLoG/optimization/SimplexSolver.hpp>

namespace ToLoG
{

TEST(SimplexTest, OptBookP331Test)
{
    SparseVector<double> c(2);
    c.set(1, -1);

    LinearProgram lp(c);

    SparseVector<double> coeffs(2);

    // 3x0 + 2x1 <= 6
    coeffs.set(0, 3);
    coeffs.set(1, 2);
    lp.add_constraint(coeffs, 6, Constraint::Type::LEQ);

    // -3x0 + 2x1 <= 0
    coeffs.set(0, -3);
    coeffs.set(1, 2);
    lp.add_constraint(coeffs, 0, Constraint::Type::LEQ);

    auto solver = SimplexSolver(lp);
    solver.solve();

    EXPECT_EQ(SimplexSolver::Status::OPTIMAL, solver.status());

    EXPECT_NEAR(1, solver.solution().at(0), 1e-12);
    EXPECT_NEAR(1.5, solver.solution().at(1), 1e-12);
}

TEST(SimplexTest, UnboundedTest)
{
    SparseVector<double> c(2);
    c.set(0, -1);
    c.set(1, -1);

    LinearProgram lp(c);

    SparseVector<double> coeffs(2);

    // x0 - x1 <= 0
    coeffs.set(0, 1);
    coeffs.set(1, -1);
    lp.add_constraint(coeffs, 0, Constraint::Type::LEQ);

    auto solver = SimplexSolver(lp);
    solver.solve();
    EXPECT_EQ(SimplexSolver::Status::UNBOUNDED, solver.status());

}

TEST(SimplexTest, BasicLPTest1)
{
    // Min -5x0 + x1 + x2
    SparseVector<double> c(3);
    c.set(0, -5);
    c.set(1, 1);
    c.set(2, 1);
    LinearProgram lp(c);

    SparseVector<double> coeffs(3);

    // 3x0 - x1 - x2 <= -1
    coeffs.set_zero();
    coeffs.set(0, 3);
    coeffs.set(1, -1);
    coeffs.set(2, -1);
    lp.add_constraint(coeffs, -1, Constraint::Type::LEQ);

    // x0 + 2x1 - x2 <= -2
    coeffs.set_zero();
    coeffs.set(0, 1);
    coeffs.set(1, 2);
    coeffs.set(2, -1);
    lp.add_constraint(coeffs, -2, Constraint::Type::LEQ);

    // 2x0 + x1 <= 2
    coeffs.set_zero();
    coeffs.set(0, 2);
    coeffs.set(1, 1);
    lp.add_constraint(coeffs, 2, Constraint::Type::LEQ);

    // x0 + x1 = 1
    coeffs.set_zero();
    coeffs.set(0, 1);
    coeffs.set(1, 1);
    lp.add_constraint(coeffs, 1, Constraint::Type::EQ);

    SimplexSolver solver(lp);
    solver.solve();

    EXPECT_EQ(SimplexSolver::Status::OPTIMAL, solver.status());
    EXPECT_NEAR(1.0, solver.solution().at(0), 1e-12);
    EXPECT_NEAR(0.0, solver.solution().at(1), 1e-12);
    EXPECT_NEAR(4.0, solver.solution().at(2), 1e-12);
}

TEST(SimplexTest, BasicLPTest)
{
    // Minimize x1 + x2
    // s.t. x1 + x2 = 2

    SparseVector<double> c(2);
    c.set(0, 1.0);
    c.set(1, 1.0);

    LinearProgram lp(c);

    SparseVector<double> coeffs(2);
    coeffs.set(0, 1);
    coeffs.set(1, 1);
    lp.add_constraint(coeffs, 2, Constraint::Type::EQ);

    SimplexSolver solver(lp);
    solver.solve();

    auto x = solver.solution();

    EXPECT_EQ(SimplexSolver::Status::OPTIMAL, solver.status());

    EXPECT_NEAR(x.at(0) + x.at(1), 2.0, 1e-12);
    EXPECT_GE(x.at(0), 0.0);
    EXPECT_GE(x.at(1), 0.0);
}

TEST(SimplexSolverTest, Phase1AndPhase2)
{
    // Objective
    SparseVector<double> c(3);
    c.set(0, 1);
    c.set(1, 10);
    c.set(2, 1);

    LinearProgram lp(c);

    // x0 + x1 = 2
    SparseVector<double> coeffs(3);
    coeffs.set(0, 1);
    coeffs.set(1, 1);
    lp.add_constraint(coeffs, 2, Constraint::Type::EQ);

    // x1 + x2 = 3
    coeffs.set_zero();
    coeffs.set(1, 1);
    coeffs.set(2, 1);
    lp.add_constraint(coeffs, 3, Constraint::Type::EQ);

    SimplexSolver solver(lp);
    solver.solve();

    EXPECT_EQ(solver.status(), SimplexSolver::Status::OPTIMAL);

    const auto& x = solver.solution();
    EXPECT_NEAR(x.at(0), 2.0, 1e-12);
    EXPECT_NEAR(x.at(1), 0.0, 1e-12);
    EXPECT_NEAR(x.at(2), 3.0, 1e-12);
}

TEST(SimplexTest, SingletonTest)
{
    double a = 7.2;

    // min x s.t. x = a
    SparseVector<double> c(1);
    SparseVector<double> coeffs(1);
    coeffs.set(0, 1);
    LinearProgram lp(c);
    lp.add_constraint(coeffs, a, Constraint::Type::EQ);

    auto solver = SimplexSolver(lp);
    solver.solve();

    EXPECT_EQ(solver.status(), SimplexSolver::Status::OPTIMAL);
    EXPECT_NEAR(solver.solution().at(0), a, 1e-12);
}

TEST(SimplexTest, SingleGeqZero)
{
    // min x s.t. x >= 0
    SparseVector<double> c(1);
    c.set(0, 1);
    LinearProgram lp(c);

    SparseVector<double> coeffs(1);
    coeffs.set(0, 1);
    lp.add_constraint(coeffs, 0, Constraint::Type::GEQ);

    auto solver = SimplexSolver(lp);
    solver.solve();

    EXPECT_EQ(solver.status(), SimplexSolver::Status::OPTIMAL);
    EXPECT_EQ(0.0, solver.solution().at(0));
    EXPECT_EQ(0.0, lp(solver.solution()));
}

TEST(SimplexTest, UnconstrainedProblemUnboundedTest)
{
    // min -5x s.t. {}
    SparseVector<double> c(1);
    c.set(0, -5);
    LinearProgram lp(c);

    auto solver = SimplexSolver(lp);
    solver.solve();

    EXPECT_EQ(solver.status(), SimplexSolver::Status::UNBOUNDED);
}


TEST(SimplexTest, ConflictingEqualityConstraintsTest)
{
    // Objective
    SparseVector<double> c(1);
    c.set(0, 1);

    LinearProgram lp(c);

    // x = 3
    SparseVector<double> coeffs(1);
    coeffs.set(0, 1);
    lp.add_constraint(coeffs, 3, Constraint::Type::EQ);

    // x = 5
    coeffs.set_zero();
    coeffs.set(0, 1);
    lp.add_constraint(coeffs, 5, Constraint::Type::EQ);

    SimplexSolver solver(lp);
    solver.solve();

    EXPECT_EQ(solver.status(), SimplexSolver::Status::INFEASIBLE);
}

TEST(SimplexTest, SimpleSumTest)
{
    size_t n = 100;

    // min Sum_i x_i
    SparseVector<double> c(n);
    for (int i = 0; i < n; ++i) {c.set(i, 1);}

    LinearProgram lp(c);

    // x_0 >= 1
    SparseVector<double> coeffs(n);
    coeffs.set(0, 1);
    lp.add_constraint(coeffs, 1, Constraint::Type::GEQ);

    // x_{i+1} >= x_i + 1
    for (int i = 0; i < n-1; ++i) {
        coeffs.set_zero();
        coeffs.set(i+1, 1);
        coeffs.set(i, -1);
        lp.add_constraint(coeffs, 1, Constraint::Type::GEQ);
    }

    auto solver = SimplexSolver(lp);
    solver.solve();

    // x = [1, 2, 3, 4, ...]
    EXPECT_EQ(solver.status(), SimplexSolver::Status::OPTIMAL);
    for (int i = 0; i < n; ++i) {
        EXPECT_NEAR(i+1, solver.solution().at(i), 1e-12);
    }
    EXPECT_NEAR(0.5*n*(n+1), lp(solver.solution()), 1e-12);
}


}
