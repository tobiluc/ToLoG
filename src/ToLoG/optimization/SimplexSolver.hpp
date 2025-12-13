#pragma once

#include <ToLoG/optimization/LinearProgram.hpp>
#include <cassert>
#include <iostream>
#include <ostream>

namespace ToLoG
{

/// The Simplex Solver implicitly assumes x >= 0
class SimplexSolver
{
public:
    enum class Status {
        NONOPTIMAL=0,
        OPTIMAL=1,
        UNBOUNDED=2,
        INFEASIBLE=3
    };

private:

    LinearProgram& lp_;
    std::vector<double> x_; // current solution

    /// The Simplex Tableau
    /// ---------------------
    /// objective      | 0
    /// ---------------------
    /// rows           | rhs
    SparseMatrix<double,MatrixMajor::ROW> tableau_;
    std::vector<int> basis_;
    uint n_slack_ = 0;
    uint n_artificial_;
    Status status_;

public:

    explicit SimplexSolver(LinearProgram& _lp) :
        tableau_(0,0),
        x_(_lp.n_unknowns()),
        lp_(_lp),
        n_artificial_(0),
        n_slack_(0),
        status_(Status::NONOPTIMAL)
    {
    }

    void solve();

    inline const std::vector<double>& solution() const {
        return x_;
    }

    inline Status status() const {
        return status_;
    }

    std::optional<Constraint> generate_gomory_mixed_integer_cut(
        int _int_var, const std::vector<int>& _integers);

private:
    enum class Phase {ONE, TWO};

    void setup_phase_1();

    bool is_feasible_phase_1();

    void setup_phase_2();

    bool run_simplex(Phase _phase);

    // Remove artificial variables from the tableau and basis
    void remove_artificial_variables();

    // Populate x_ from tableau and current basis
    void populate_solution()
    {
        std::fill(x_.begin(), x_.end(), 0.0);
        int rhs_col = tableau_.n_cols() - 1;
        for (int i = 0; i < basis_.size(); ++i) {
            int b = basis_[i];
            if (b >= 0 && b < lp_.n_unknowns()) {
                x_[b] = tableau_.at(i+1, rhs_col);
            }
        }
    }

    void pick_pivot(int& col, int& row);

    // row = leaving variable row
    // col = entering variable column
    void pivot(int row, int col);

    /**
     * Returns a slack variable s_j expressed as sum c_ij x_i + constTerm
     */
    void getSlackVarCoeffs(const int& j, SparseVector<double>& varCoeffs, double& constTerm);

    friend std::ostream& operator<< (std::ostream& out, const Status& s) {
        if (s == Status::NONOPTIMAL) return out << "Nonoptimal";
        if (s == Status::OPTIMAL) return out << "Optimal";
        if (s == Status::UNBOUNDED) return out << "Unbounded";
        if (s == Status::INFEASIBLE) return out << "Infeasible";
        return out;
    }

    // Print tableau rows (for debugging)
    void debug_print_tableau() {
        std::cerr << "TABLEAU (" << tableau_.n_rows() << "x" << tableau_.n_cols() << ")\n";
        for (int i = 0; i < tableau_.n_rows(); ++i) {
            std::cerr << "row " << i << " [";
            for (int j = 0; j < tableau_.n_cols(); ++j) {
                std::cerr << tableau_.at(i,j);
                if (j+1 < tableau_.n_cols()) std::cerr << ", ";
            }
            std::cerr << "]\n";
        }
        std::cerr << "BASIS: [";
        for (auto b : basis_) std::cerr << b << ", ";
        std::cerr << "]\n";
    }

};

}
