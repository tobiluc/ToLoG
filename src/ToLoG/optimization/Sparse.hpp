#pragma once
#include <ToLoG/HashMap.hpp>
#include <cassert>

namespace ToLoG
{

template<typename FT>
inline bool is_approx_zero(const FT& _val) {
    return _val > -1e-16 && _val < 1e-16;
}

template<typename FT>
class SparseVector {
public:
    using Predicate = const std::function<bool(const FT& _val)>;

    SparseVector(int _size) :
        size_(_size)
    {}

    inline HashMap<int,FT>::const_iterator cbegin() const {
        return entries_.cbegin();
    }

    inline HashMap<int,FT>::const_iterator cend() const {
        return entries_.cend();
    }

    inline int size() const {
        return size_;
    }

    inline bool all(Predicate _pred) const {
        for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
            if (!_pred(it.value())) {
                return false;
            }
        }
        return full() || _pred(static_cast<FT>(0));
    }

    inline bool any(Predicate _pred) const {
        return !all([&](const FT& _val) {return !_pred(_val);});
    }

    inline bool is_zero() const {
        return all([](const FT& _val) {
            return _val == static_cast<FT>(0);
        });
    }

    inline void resize(int _size) {
        if (_size < size_) {
            // Shrinking
            entries_.erase_if([&](int _i, const FT& _val) -> bool {
                return _i >= _size;
            });
        }
        size_ = _size;
    }

    inline FT& operator[](int _i) {
        assert(_i >= 0 && _i < size_);
        return entries_[_i];
    }

    inline FT at(int _i) const {
        assert(_i >= 0 && _i < size_);
        return entries_.get_or_default(_i, static_cast<FT>(0));
    }

    inline void set(int _i, FT _val) {
        if (_i < 0 || _i >= size_) {
            throw std::logic_error("SparseVector.set(index): index outside valid range");
        }
        if (is_approx_zero(_val)) {entries_.erase(_i);}
        else {entries_.insert(_i, _val);}
    }

    inline void set(const std::vector<std::pair<int,FT>>& _data) {
        for (auto [i, val] : _data) {
            set(i, val);
        }
    }

    /// Sets the segment from _start to _start + _x.size() to _x
    inline void set_segment(int _start, const SparseVector<FT>& _x) {
        // Set range to 0
        entries_.erase_if([&](int _i, const FT& _val) {
            return (_i >= _start && _i < _start+_x.size_);
        });
        // Copy x to range
        for (auto it = _x.entries_.cbegin(); it != _x.entries_.cend(); ++it) {
            int i = _start + it.key();
            if (i >= size_) {continue;}
            set(i, it.value());
        }
    }

    inline SparseVector<FT> get_segment(int _start, int _end) const {
        SparseVector<FT> seg(_end-_start);
        for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
            seg.set(it.key()-_start, it.value());
        }
        return seg;
    }

    inline void set_zero() {
        entries_.clear();
    }

    inline FT dot(const SparseVector<FT>& _rhs) const {
        if (size_ != _rhs.size_) {throw std::logic_error("dot product requires vectors of same sizes");}
        FT res(0);
        for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
            res += it.value() * _rhs.at(it.key());
        }
        return res;
    }

    inline SparseVector<FT> operator-() const {
        SparseVector<FT> res = *this;
        res.entries_.apply([](int _i, FT& _e) {_e = -_e;});
        return res;
    }

    inline SparseVector<FT>& operator+=(const SparseVector<FT>& _rhs) {
        if (size_ != _rhs.size_) {throw std::logic_error("Can't add sparse vectors of different sizes");}
        for (auto it = _rhs.entries_.cbegin(); it != _rhs.entries_.cend(); ++it) {
            if (is_approx_zero(entries_[it.key()] += it.value())) {
                entries_.erase(it.key());
            }
        }
        return *this;
    }

    inline SparseVector<FT> operator+(const SparseVector<FT>& _rhs) const {
        SparseVector<FT> res = *this;
        res += _rhs;
        return res;
    }

    inline SparseVector<FT>& operator*=(const SparseVector<FT>& _rhs) {
        if (size_ != _rhs.size_) {throw std::logic_error("Can't multiply sparse vectors of different sizes");}
        for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
            if (is_approx_zero(entries_[it.key()] *= _rhs.at(it.key()))) {
                entries_.erase(it.key());
            }
        }
        return *this;
    }

    inline SparseVector<FT> operator*(const SparseVector<FT>& _rhs) const {
        SparseVector<FT> res = *this;
        res *= _rhs;
        return res;
    }

    inline SparseVector<FT>& operator-=(const SparseVector<FT>& _rhs) {
        if (size_ != _rhs.size_) {throw std::logic_error("Can't subtract sparse vectors of different sizes");}
        for (auto it = _rhs.entries_.cbegin(); it != _rhs.entries_.cend(); ++it) {
            if (is_approx_zero(entries_[it.key()] -= it.value())) {
                entries_.erase(it.key());
            }
        }
        return *this;
    }

    inline SparseVector<FT> operator-(const SparseVector<FT>& _rhs) const {
        SparseVector<FT> res = *this;
        res -= _rhs;
        return res;
    }

    inline SparseVector<FT>& operator/=(const FT& _rhs) {
        if (_rhs == static_cast<FT>(0)) {throw std::logic_error("Divide by 0");}
        for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
            entries_[it.key()] /= _rhs;
        }
        return *this;
    }

    inline SparseVector<FT> operator/(const FT& _rhs) {
        SparseVector<FT> res = *this;
        res /= _rhs;
        return res;
    }

    inline SparseVector<FT>& operator*=(const FT& _rhs) {
        if (is_approx_zero(_rhs)) {set_zero(); return *this;}
        for (auto it = entries_.cbegin(); it != entries_.cend(); ++it) {
            entries_[it.key()] *= _rhs;
        }
        return *this;
    }

    inline SparseVector<FT> operator*(const FT& _rhs) {
        SparseVector<FT> res = *this;
        res *= _rhs;
        return res;
    }

    inline bool operator==(const SparseVector<FT>& _rhs) const {
        if (size_ != _rhs.size_) {return false;}
        for (auto it1 = entries_.cbegin(),
                it2 = _rhs.entries_.cbegin();
             it1 != entries_.cend(); ++it1, ++it2) {
            if (it1.value() != _rhs.at(it1.key()) ||
                it2.value() != at(it2.key())) {
                return false;
            }
        }
        return true;
    }

    inline bool operator!=(const SparseVector<FT>& _rhs) const {
        return !(this->operator==(_rhs));
    }

    inline void prune() {
        entries_.erase_if([&](const int& _i, const FT& _val) -> bool {
            return is_approx_zero(_val);
        });
    }

    inline friend std::ostream& operator<<(std::ostream& _os, const SparseVector<FT>& _x) {
        return _os << _x.entries_;
    }

private:
    HashMap<int,FT> entries_;
    int size_ = 0;

    inline bool empty() const {
        return entries_.empty();
    }

    inline bool full() const {
        assert(entries_.size() <= size());
        return entries_.size() == size();
    }
};

enum class MatrixMajor {
    COL, ROW
};

template<typename FT, MatrixMajor M>
class SparseMatrix
{
public:
    SparseMatrix(int _rows, int _cols) :
        n_rows_(0),
        n_cols_(0)
    {
        resize(_rows, _cols);
    }

    inline FT at(int _row, int _col) const {
        if constexpr (M == MatrixMajor::COL) {
            return data_[_col].at(_row);
        } else {
            return data_[_row].at(_col);
        }
    }

    inline void set(int _row, int _col, FT _val) {
        if constexpr (M == MatrixMajor::COL) {
            data_[_col].set(_row, _val);
        } else {
            data_[_row].set(_col, _val);
        }
    }

    inline void set_zero() {
        for (auto& d : data_) {d.set_zero();}
    }

    inline const SparseVector<FT>& row(int _row) const {
        if constexpr (M == MatrixMajor::COL) {
            throw std::logic_error("row() is currently only supported for row major");
        } else {
            return data_[_row];
        }
    }

    inline SparseVector<FT>& row(int _row) {
        if constexpr (M == MatrixMajor::COL) {
            throw std::logic_error("row() is currently only supported for row major");
        } else {
            return data_[_row];
        }
    }

    inline const SparseVector<FT>& col(int _col) const {
        if constexpr (M == MatrixMajor::COL) {
            return data_[_col];
        } else {
            throw std::logic_error("col() is currently only supported for column major");
        }
    }

    inline SparseVector<FT>& col(int _col) {
        if constexpr (M == MatrixMajor::COL) {
            return data_[_col];
        } else {
            throw std::logic_error("col() is currently only supported for column major");
        }
    }

    inline int n_rows() const {
        return n_rows_;
    }

    inline int n_cols() const {
        return n_cols_;
    }

    inline void resize(int _rows, int _cols) {
        if constexpr (M == MatrixMajor::COL) {
            data_.reserve(_cols);
            // Add new columns
            for (int i = 0; i < _cols-n_cols_; ++i) {
                data_.emplace_back(_rows);
            }
            // Resize old columns
            if (_rows != n_rows_) {
                for (int col = 0; col < n_cols_; ++col) {
                    data_[col].resize(_rows);
                }
            }
        } else {
            data_.reserve(_rows);
            // Add new rows
            for (int i = 0; i < _rows-n_rows_; ++i) {
                data_.emplace_back(_cols);
            }
            // Resize old rows
            if (_cols != n_cols_) {
                for (int row = 0; row < n_rows_; ++row) {
                    data_[row].resize(_cols);
                }
            }
        }
        n_rows_ = _rows;
        n_cols_ = _cols;
    }

    /// x is interpreted as column vector
    inline SparseVector<FT> operator*(const SparseVector<FT>& _x) const {
        if (n_cols_ != _x.size()) {throw std::logic_error("dimension mismatch for matrix multiplication");}
        SparseVector<FT> res(n_rows_);
        if constexpr (M == MatrixMajor::COL) {
            for (int col = 0; col < n_cols_; ++col) {
                for (auto r_it = data_[col].cbegin(); r_it != data_[col].cend(); ++r_it) {
                    res[r_it.key()] += r_it.value() * _x.at(col);
                }
            }
        } else {
            for (int row = 0; row < n_rows_; ++row) {
                res[row] = data_[row].dot(_x);
            }
        }
        res.prune();
        return res;
    }

    // template<MatrixMajor MT>
    // inline SparseMatrix<FT,M> operator*(const SparseMatrix<FT,MT>& _rhs) {
    //     if (n_cols_ != _rhs.n_rows_) {throw std::logic_error("dimension mismatch for matrix multiplication");}
    //     SparseMatrix<FT,M> res(n_rows_, _rhs.n_cols_);
    //     if constexpr (M == MatrixMajor::COL) {
    //         for (int col = 0; col < n_cols_; ++col) {
    //             for (auto r_it = data_.cbegin(); r_it != data_.cend(); ++r_it) {

    //             }
    //         }
    //     } else {
    //         for (int row = 0; row < n_rows_; ++row) {
    //             for (int col = 0;)
    //             for (auto c_it = data_.cbegin(); c_it != data_.cend(); ++c_it) {
    //                 res
    //             }
    //         }
    //     }
    // }

    inline void prune() {
        for (auto& d : data_) {d.prune();}
    }

private:
    std::vector<SparseVector<FT>> data_;
    int n_rows_ = 0;
    int n_cols_ = 0;
};

}
