#pragma once
#include <ToLoG/vector_concepts.hpp>
#include <algorithm>

namespace ToLoG
{

template<typename FT, int DIM> requires(DIM > 0)
class Point
{
private:
    template<typename P>
    static constexpr bool assignable =
        requires(const P& p, int i) {
            {p[i]} -> std::convertible_to<FT>;
        } && requires {
            typename Traits<P>;
            Traits<P>::dim == DIM;
        };
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
    template<typename... Args>
        requires (sizeof...(Args) == DIM) &&
            (std::convertible_to<Args, FT> && ...)
    Point(Args&&... args)
        : data_{static_cast<FT>(args)...}
    {}
    template<typename P> requires(assignable<P>)
    Point(const P& _p) {
        for (int i = 0; i < DIM; ++i) {
            data_[i] = static_cast<FT>(_p[i]);
        }
    }
    inline constexpr size_t size() const noexcept {
        return static_cast<size_t>(DIM);
    }
    inline FT& operator[](const int& _i) {
        return data_[_i];
    }
    inline const FT& operator[](const int& _i) const {
        return data_[_i];
    }
    template<typename P> requires(assignable<P>)
    inline Point<FT,DIM> operator-(const P& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] -= _rhs[i];
        }
        return res;
    }
    template<typename P> requires(assignable<P>)
    inline Point<FT,DIM> operator+(const P& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] += _rhs[i];
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
    template<typename P> requires(assignable<P>)
    inline Point<FT,DIM>& operator=(const P& _rhs) {
        for (int i = 0; i < DIM; ++i) {
            data_[i] = static_cast<FT>(_rhs[i]);
        }
        return *this;
    }
    template<typename P> requires(assignable<P>)
    inline bool operator==(const P& _p) const {
        for (int i = 0; i < DIM; ++i) {
            if (data_[i] != _p[i]) {
                return false;
            }
        }
        return true;
    }
    template<typename P> requires(assignable<P>)
    inline bool operator<(const P& _rhs) const {
        for (int i = 0; i < DIM; ++i) {
            if (data_[i] < _rhs[i]) {return true;}
            if (_rhs[i] < data_[i]) {return false;}
        }
        return false;
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
