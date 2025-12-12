#pragma once

#include <ToLoG/unstable/LinearProgram.hpp>
#include <cassert>
#include <iostream>
#include <ostream>

namespace ToLoG
{

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

    ///
    /// \brief generateGomoryMixedIntegerCut
    /// \param x Current optimal solution
    /// \param Ai lhs coefficient of cut
    /// \param bi rhs constant of cut
    /// \return true iff cut was successfully generated
    ///
    // bool generateGomoryMixedIntegerCut(const Vecd& x, Vecd& Ai, double& bi)
    // {
    //     uint n = problem.dimension();
    //     uint n1 = problem.nIntegerConstraints();
    //     uint n2 = n-n1;

    //     // Find Integer violation (column)
    //     int col = -1;
    //     for (auto i = 0u; i < n1; ++i) {
    //         assert(problem.isIntegerConstrained(i));
    //         if (!isInt(x[i])) {
    //             col = i;
    //             break;
    //         }
    //     }
    //     if (col == -1) {return false;}

    //     //std::cout << "Variable " << col << " violates Integer Constraint!" << std::endl;

    //     // Find corresponding Basis row
    //     int row = -1;
    //     for (uint r = 1; r <= basis.size(); ++r) {
    //         if (basis[r-1] == col) {
    //             row = r;
    //             break;
    //         }
    //     }
    //     assert(row != -1);
    //     assert(T(row,col)==1);
    //     //std::cout << "In row " << row << std::endl;

    //     // Get row and rhs
    //     bi = br(row);
    //     double fi = getFrac(bi);
    //     assert(fi > 0 && fi < 1);
    //     bi = floor(bi);
    //     //Vecd varCoeffs(n);

    //     // Mixed Integer Rounding
    //     for (uint j = 0; j < n1; ++j) {
    //         double aij = T(row,j);
    //         double fij = getFrac(aij);
    //         if (fij <= fi) { // N1<=
    //             Ai[j] = floor(aij);
    //         } else { // N1>
    //             Ai[j] = floor(aij) + ((fij - fi) / (1. - fi));
    //         }
    //     }
    //     for (uint j = n1; j < n1+n2+nSlackVars; ++j) {
    //         double aij = T(row,j);
    //         if (aij >= 0) {continue;}

    //         double coeff = aij / (1. - fi);

    //         if (j < n) { // N2- problem var
    //             Ai[j] = coeff;
    //         } else { // N2- slack var
    //             Vecd varCoeffs(n); double constTerm;
    //             getSlackVarCoeffs(j, varCoeffs, constTerm); // resubstitute
    //             //std::cout << "VarCoeffs for slack " << j << " are " << varCoeffs.transpose() << " + " << constTerm << std::endl;
    //             Ai += coeff * varCoeffs;
    //             bi -= coeff * constTerm;
    //         }
    //     }

    //     for (uint j = 0; j < Ai.size(); ++j) {if (isInt(Ai[j])) {Ai[j] = std::round(Ai[j]);}}
    //     if (isInt(bi)) {bi = std::round(bi);}

    //     return true;
    // }

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
