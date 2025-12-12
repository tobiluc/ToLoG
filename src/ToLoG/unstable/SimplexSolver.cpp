#include <ToLoG/unstable/SimplexSolver.hpp>

namespace ToLoG
{

void SimplexSolver::setup_phase_1()
{
    std::fill(x_.begin(), x_.end(), 0.0);

    // Count slacks/artificials
    struct RowInfo {ConstraintType type; double rhs; bool flip_coeffs;};
    std::vector<RowInfo> rows;
    rows.reserve(lp_.n_constraints());
    n_slack_ = n_artificial_ = 0;
    for (int i = 0; i < lp_.n_constraints(); ++i) {
        // ax <= b -> ax + s = b
        // ax >= b -> ax - s = b

        // Ensure rhs >= 0
        ConstraintType type = lp_.constraint(i).type();
        double rhs = lp_.constraint(i).rhs();
        bool flip_coeffs = rhs < 0.0;
        if (flip_coeffs) {
            rhs = -rhs;
            if (type == ConstraintType::LEQ) {type = ConstraintType::GEQ;}
            else if (type == ConstraintType::GEQ) {type = ConstraintType::LEQ;}
        }

        // Add row info
        rows.push_back(RowInfo{type, rhs, flip_coeffs});

        // Count variables
        switch (type) {
        case ConstraintType::LEQ: ++n_slack_; break;
        case ConstraintType::GEQ: ++n_slack_; ++n_artificial_; break;
        case ConstraintType::EQ: ++n_artificial_; break;
        default: break;
        }
    }

    // Setup tableau
    // (m+1) x (n+ns+na+1)
    tableau_.set_zero();
    tableau_.resize(
        1 + lp_.n_constraints(),
        lp_.n_unknowns() + n_slack_ + n_artificial_ + 1
    );

    // The initial objective function for phase 1
    // is the sum of artificial variables
    for (int j = lp_.n_unknowns()+n_slack_; j < tableau_.n_cols()-1; ++j) {
        tableau_.set(0, j, 1.0);
    }

    // Fill constraint rows
    int n = lp_.n_unknowns();
    int slack_start = n;
    int art_start = n + n_slack_;
    int rhs_col = tableau_.n_cols() - 1;

    int slack_col = slack_start;
    int art_col   = art_start;
    basis_.resize(lp_.n_constraints());
    for (int i = 0; i < lp_.n_constraints(); ++i) {
        const auto& c = lp_.constraint(i);
        const auto& row = rows[i];

        int row_idx = 1+i;

        // Original vars
        for (auto it = c.coeffs().cbegin(); it != c.coeffs().cend(); ++it) {
            double v = row.flip_coeffs ? -it.value() : it.value();
            tableau_.set(row_idx, it.key(), v);
        }

        // Slack/Artifical vars
        if (row.type == ConstraintType::LEQ) {
            // ax + s = b
            tableau_.set(row_idx, slack_col, 1.0);
            basis_[i] = slack_col++;
        }
        else if (row.type == ConstraintType::GEQ) {
            // ax - s + a = b
            tableau_.set(row_idx, slack_col, -1.0);
            ++slack_col;
            tableau_.set(row_idx, art_col, 1.0);
            basis_[i] = art_col++;
        }
        else {
            // ax + a = b
            tableau_.set(row_idx, art_col, 1.0);
            basis_[i] = art_col++;
        }

        // Set rhs
        tableau_.set(row_idx, rhs_col, row.rhs);
    }
    assert(slack_col == slack_start + n_slack_);
    assert(art_col == art_start + n_artificial_);

    for (int i = 0; i < lp_.n_constraints(); ++i) {
        int b = basis_[i];
        if (b >= art_start && b < rhs_col) {
            tableau_.row(0) -= tableau_.row(i+1);
        }
    }

#ifndef NDEBUG
    // Each basis column must be unit vector
    for (int b : basis_) {
        if (b >= 0) {
            bool has_1 = false;
            for (int i = 0; i < lp_.n_constraints(); ++i) {
                double v = tableau_.at(i+1, b);
                if (is_approx_zero(v - 1.0)) {
                    assert(!has_1);
                    has_1 = true;
                } else {assert(is_approx_zero(v));}
            }
        }
    }
#endif
}

bool SimplexSolver::is_feasible_phase_1()
{
    return tableau_.at(0, tableau_.n_cols()-1) <= 1e-16;
}

void SimplexSolver::setup_phase_2()
{
    // Set objective row for Phase 2
    // Minimize c^T x
    tableau_.row(0).set_zero();
    tableau_.row(0).set_segment(0, lp_.objective());

    // Subtract multiples of basic rows to put in canonical form
    // so that coefficients of basic variables in row 0 are zero
    for (int i = 0; i < lp_.n_constraints(); ++i) {
        int b = basis_[i];
        if (b >= 0 && b < lp_.n_unknowns()) {
            double coeff = tableau_.at(0, b);
            tableau_.row(0) -= tableau_.row(1+i) * coeff;
        }
    }

}

bool SimplexSolver::run_simplex(Phase _phase)
{
#ifndef NDEBUG
    if (_phase == Phase::ONE) {
        // Check rhs >= 0
        for (int i = 1; i < tableau_.n_rows(); ++i) {
            assert(tableau_.at(i, tableau_.n_cols()-1) >= 0.0);
        }
    }
#endif

    while (true) {
        int entering = -1;
        int leaving_row = -1;
        pick_pivot(entering, leaving_row);

        if (entering == -1) {break;} // optimal

        if (leaving_row == -1) {
            status_ = Status::UNBOUNDED;
            std::cerr << "Simplex: unbounded!\n";
            return false;
        }

        pivot(leaving_row, entering);
    }

    return true;
}

void SimplexSolver::pick_pivot(int& col, int& row)
{
    // Pick Pivot Column
    // If all coeffs in obj. are nonpos, sol is optimal, return
    col = -1;
    double min = INFINITY;
    for (int j = 0; j < tableau_.n_cols()-1; ++j) {
        double t = tableau_.at(0,j);
        if (t < 0 && t < min) {
            min = t;
            col = j;
        }
    }
    if (col == -1) {return;} // optimal

    // Pick Pivot Row
    // If all entries in col are nonpos, sol is unbounded, return
    row = -1;
    min = INFINITY;
    for (int i = 1; i < tableau_.n_rows(); ++i) {
        double t = tableau_.at(i,col);
        if (t > 0.0) {
            double b = tableau_.at(i, tableau_.n_cols()-1);
            t = b / t;
            if (t < min) {
                min = t;
                row = i;
            }
        }
    }
    if (row == -1) {return;} // unbounded
}

void SimplexSolver::pivot(int row, int col)
{
    double pivot_val = tableau_.at(row, col);

    // divide leaving row by pivot to make 1
    tableau_.row(row) /= pivot_val;

    // eliminate entering column in other rows
    for (int i = 0; i < tableau_.n_rows(); ++i) {
        if (i == row) {continue;}
        double factor = tableau_.at(i, col);
        tableau_.row(i) -= tableau_.row(row) * factor;
    }

    basis_[row-1] = col;
}

void SimplexSolver::remove_artificial_variables()
{
    if (n_artificial_ == 0) {return;}

    int m = lp_.n_constraints();
    int n = lp_.n_unknowns();
    int rhs_col = tableau_.n_cols() - 1;

    int slack_start = n;
    int art_start   = n + n_slack_;
    int art_end     = art_start + n_artificial_;

    // Remove artificial basis
    for (int i = 0; i < m; ++i) {
        int b = basis_[i];
        if (b < art_start || b >= art_end) {
            continue; // not artificial
        }

        int row = i + 1;
        bool replaced = false;

        // Try to pivot in an original or slack variable
        for (int col = 0; col < art_start; ++col) {
            double aij = tableau_.at(row, col);
            if (std::abs(aij) < 1e-12) {continue;}

            // Pivot col into the basis
            pivot(row, col);
            basis_[i] = col;
            replaced = true;
            break;
        }

        if (!replaced) {
            // No valid pivot → row has no basic var
            basis_[i] = -1;
        }
    }

    // Remove artificial columns
    std::vector<double> rhs(tableau_.n_rows());
    for (int i = 0; i < tableau_.n_rows(); ++i) {
        rhs[i] = tableau_.at(i, rhs_col);
    }
    tableau_.resize(tableau_.n_rows(), art_start + 1);
    for (int i = 0; i < tableau_.n_rows(); ++i) {
        tableau_.set(i, tableau_.n_cols()-1, rhs[i]);
    }

    n_artificial_ = 0;
}


void SimplexSolver::solve()
{
    status_ = Status::NONOPTIMAL;

    setup_phase_1();

    if (!run_simplex(Phase::ONE)) {return;}

    if (!is_feasible_phase_1()) {
        status_ = Status::INFEASIBLE;
        std::cerr << "Simplex: infeasible!\n";
        return;
    }

    remove_artificial_variables();

    setup_phase_2();

    if (!run_simplex(Phase::TWO)) {return;}

    populate_solution();

    status_ = Status::OPTIMAL;
}

}
