#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <algorithm>

namespace ToLoG
{

template<typename FT, int DIM> requires(DIM > 0)
class Point
{
public:
    Point() {
        for (int i=0;i<DIM;++i) {
            data_[i] = FT(0);
        }
    }
    Point(const FT* _data) {
        for (int i = 0; i < DIM; ++i) {
            data_[i] = _data[i];
        }
    }
    template<typename... Args,
             typename = std::enable_if_t<sizeof...(Args) == DIM>>
    Point(Args&&... args)
        : data_{ static_cast<FT>(args)... }
    {}
    inline constexpr size_t size() const noexcept {
        return static_cast<size_t>(DIM);
    }
    inline FT& operator[](const int& _i) {
        return data_[_i];
    }
    inline const FT& operator[](const int& _i) const {
        return data_[_i];
    }
    inline Point<FT,DIM> operator-(const Point<FT,DIM>& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] -= _rhs.data_[i];
        }
        return res;
    }
    inline Point<FT,DIM> operator+(const Point<FT,DIM>& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] += _rhs.data_[i];
        }
        return res;
    }
    inline Point<FT,DIM> operator*(const FT& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] *= _rhs;
        }
        return res;
    }
    inline Point<FT,DIM> operator/(const FT& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] /= _rhs;
        }
        return res;
    }
    inline const FT* data() const {
        return data_;
    }
    inline bool operator==(const Point<FT,DIM>& _p) const {
        for (int i = 0; i < DIM; ++i) {
            if (data_[i] != _p.data_[i]) {
                return false;
            }
        }
        return true;
    }
    inline bool operator<(const Point& _rhs) const {
        return std::lexicographical_compare(
            data_, data_+DIM,
            _rhs.data_, _rhs.data_+DIM
            );
    }
    friend inline std::ostream& operator<<(std::ostream& _os, const Point& _p) {
        if constexpr(DIM == 0) {return _os;}
        for (int i = 0; i < DIM-1; ++i) {_os << _p[i] << " ";}
        return _os << _p[DIM-1];
    }
private:
    FT data_[DIM];
};

template<typename FT, int DIM>
struct Traits<Point<FT,DIM>>
{
    using value_type = FT;
    using vector_type = Point<FT,DIM>;
    constexpr static int dim = DIM;
};

}
