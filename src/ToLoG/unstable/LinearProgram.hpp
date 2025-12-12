#pragma once
#include <ToLoG/unstable/Sparse.hpp>

namespace ToLoG
{

enum class ConstraintType {
    LEQ, GEQ, EQ
};

class Constraint {
public:
    Constraint(
        const SparseVector<double>& _coeffs,
        const double& _rhs, const ConstraintType _type) :
    coeffs_(_coeffs), rhs_(_rhs), type_(_type)
    {
    }

    inline const SparseVector<double>& coeffs() const {
        return coeffs_;
    }

    inline double rhs() const {
        return rhs_;
    }

    inline ConstraintType type() const {
        return type_;
    }

    inline bool is_feasible(const SparseVector<double>& _x) const {
        double dot(coeffs_.dot(_x));
        switch(type_) {
            case ConstraintType::EQ:
                return is_approx_zero(dot - rhs_);
            case ConstraintType::LEQ:
                return dot <= rhs_;
            case ConstraintType::GEQ:
                return dot >= rhs_;
            default: return false;
        }
    }

private:
    SparseVector<double> coeffs_;
    double rhs_;
    ConstraintType type_;
};

/// min_x c^T * x
class LinearProgram
{
public:
    LinearProgram(
        const SparseVector<double>& _objective
        ) :
        objective_(_objective)
    {
    }

    inline void add_constraint(const SparseVector<double>& _coeffs, const double& _rhs, const ConstraintType _type) {
        constraints_.emplace_back(_coeffs, _rhs, _type);
    }

    inline int n_unknowns() const {
        return objective_.size();
    }

    inline int n_constraints() const {
        return constraints_.size();
    }

    inline const Constraint& constraint(int i) const {
        return constraints_[i];
    }

    inline const SparseVector<double>& objective() const {
        return objective_;
    }

    inline double operator()(const SparseVector<double>& _x) const {
        return _x.dot(objective_);
    }

    inline bool is_feasible(const SparseVector<double>& _x) const {
        for (int i = 0; i < n_constraints(); ++i) {
            if (!constraints_[i].is_feasible(_x)) {return false;}
        }
        return true;
    }

private:
    SparseVector<double> objective_;
    std::vector<Constraint> constraints_;
};

// /// min_x c^T * x
// /// s.t. Ax = b, x >= 0
// template<typename FT>
// class SimplexLinearProgram
// {
// public:
//     SimplexLinearProgram(
//         const SparseVector<FT>& _c,
//         const SparseMatrix<FT,MatrixMajor::ROW>& _A,
//         const SparseVector<FT>& _rhs
//         ) : objective_(_c), A_(_A), rhs_(_rhs), n_slack_(0)
//     {
//     }

//     // SimplexLinearProgram(
//     //     const LinearProgram<FT>& _lp
//     //     ) :
//     //     objective_(_lp.objective()),
//     //     n_slack_(0),
//     //     A_(0,_lp.n_unknowns()),
//     //     rhs_(0)
//     // {
//     //     // TODO: Remove dependend constraints

//     //     // Count how many slack variables we need
//     //     n_slack_ = 0;
//     //     for (int i = 0; i < _lp.n_constraints(); ++i) {
//     //         FT l = _lp.lower().at(i);
//     //         FT u = _lp.upper().at(i);
//     //         if (l == u) {continue;}
//     //         bool linf = _lp.is_upper_unbounded(i);
//     //         bool uinf = _lp.is_lower_unbounded(i);
//     //         if (!linf) {++n_slack_;}
//     //         if (!uinf) {++n_slack_;}
//     //     }
//     //     objective_.resize(objective_.size() + n_slack_);

//     //     int slack_idx = A_.n_cols();
//     //     rhs_.resize(A_.n_cols() + n_slack_);
//     //     A_.resize(0, rhs_.size());
//     //     for (int i = 0; i < _lp.n_constraints(); ++i) {
//     //         FT l = _lp.lower().at(i);
//     //         FT u = _lp.upper().at(i);

//     //         bool linf = _lp.is_upper_unbounded(i);
//     //         bool uinf = _lp.is_lower_unbounded(i);

//     //         // l <= ax <= l -> ax = l
//     //         // ax <= u -> ax + sl = u (slack su >= 0)
//     //         // l <= ax -> ax - sl = l (slack sl >= 0)
//     //         enum class Type {NoSlack, AddSlack, SubSlack};
//     //         auto add_constraint = [&](Type _type, FT _rhs) {
//     //             int r = A_.n_rows();
//     //             A_.resize(r+1, A_.n_cols());
//     //             A_.row(r).set_segment(0, _lp.matrix().n_cols(), _lp.matrix().row(i));
//     //             if (_type != Type::NoSlack) {
//     //                 A_.set(r, slack_idx, (_type==Type::AddSlack)? 1.0 : -1.0);
//     //                 ++slack_idx;
//     //             }
//     //             rhs_.set(r, _rhs);
//     //         };

//     //         if (l == u) {add_constraint(Type::NoSlack , l); continue;}
//     //         if (!uinf) {add_constraint(Type::AddSlack, u);}
//     //         if (!linf) {add_constraint(Type::SubSlack, l);}
//     //     }

//     // }

//     inline int n_slack() const {
//         return n_slack_;
//     }

//     /// N Unknowns including slacks
//     inline int n_unknowns() const {
//         return A_.n_cols();
//     }

//     inline int n_constraints() const {
//         return A_.n_rows();
//     }

//     inline FT operator()(const SparseVector<FT>& _x) const {
//         assert(_x.size() <= n_unknowns() || _x.size() == n_unknowns()-n_slack());
//         return _x.dot(objective_);
//     }

//     inline bool is_feasible(const SparseVector<FT>& _x) const {
//         assert(_x.size() == n_unknowns());
//         for (int i = 0; i < n_constraints(); ++i) {
//             FT res = A_.row(i).dot(_x);
//             if (res != rhs_.at(i)) {return false;}
//         }
//         return _x.all([](const FT& _val) {return _val >= 0;});
//     }

//     inline const SparseVector<FT>& objective() const {
//         return objective_;
//     }

//     inline const SparseMatrix<FT,MatrixMajor::ROW>& A() const {
//         return A_;
//     }

//     inline const SparseVector<FT>& rhs() const {
//         return rhs_;
//     }

//     inline void prune() {
//         objective_.prune();
//         A_.prune();
//         rhs_.prune();
//     }

// private:
//     SparseVector<FT> objective_;
//     SparseMatrix<FT,MatrixMajor::ROW> A_;
//     SparseVector<FT> rhs_;
//     int n_slack_ = 0;
// };

}
